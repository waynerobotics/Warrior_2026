#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace warrior::hardware {

// One open USB-CDC SLCAN port to a single REV SPARK MAX, with its own
// tx/rx threads. Mirrors `SparkSession` in
// warrior_serial/warrior_serial/nudge_sparks.py — which is the working
// Python reference.
//
// Lifecycle:
//   1. open(port)              opens raw, exclusive; starts tx + rx threads.
//   2. Heartbeat (mode-bitmask + enable) is emitted at ~50 Hz from open.
//      Without it the controller stops streaming Status 0/2.
//   3. The rx thread parses inbound SLCAN frames. The lower 6 bits of the
//      first valid frame's arbitration ID become this session's
//      `device_id()`. Status 0 (applied/faults) and Status 2 (position)
//      payloads populate the accessors.
//   4. Once device_id is known, narrow the mode bitmask to (1 << device_id).
//   5. Call set_target_position() + enable() to start sending setpoints.
//      disable() returns the tx loop to bare heartbeat.
//   6. close() (or dtor) cleanly joins threads.
//
// Thread safety: every public method is safe to call from any thread.
class SparkMaxSession
{
public:
    SparkMaxSession() = default;
    ~SparkMaxSession();

    SparkMaxSession(const SparkMaxSession &) = delete;
    SparkMaxSession & operator=(const SparkMaxSession &) = delete;

    // Open port at 115200, raw + exclusive, spawn tx & rx threads. Returns
    // false on failure (port already open by another process, etc.).
    bool open(const std::string & port);
    void close();
    bool is_open() const { return fd_ >= 0; }

    const std::string & port() const { return port_; }

    // -1 until the rx thread parses the first inbound frame.
    int device_id() const { return device_id_.load(); }
    // Force a fallback device_id when the controller stays silent during
    // discovery. Used only by the manager as a last-resort pairing.
    void force_device_id(int device_id);

    // Latest Status 2 position (motor rotations). NaN until first frame.
    float position_rotations() const { return position_rot_.load(); }
    // Latest Status 0 applied output as percent (-100..100).
    float applied_output_percent() const { return applied_pct_.load(); }
    // Latest Status 0 fault bitmask.
    uint16_t faults() const { return faults_.load(); }

    // Diagnostic frame counters — useful for "is anything streaming at all"
    // checks during bring-up.
    uint64_t status_0_count() const { return status_0_count_.load(); }
    uint64_t status_2_count() const { return status_2_count_.load(); }
    uint64_t other_frame_count() const { return other_frame_count_.load(); }
    uint64_t tx_count() const { return tx_count_.load(); }

    // Set the position setpoint (in motor rotations). Takes effect on the
    // next tx tick. No-op while disabled (heartbeat still flows).
    void set_target_position(float rotations);
    void enable();
    void disable();

private:
    void tx_loop();
    void rx_loop();
    void consume_line(const std::string & line);
    bool write_all_fd(const std::string & s);

    int fd_ = -1;
    std::string port_;

    std::atomic<int>      device_id_{-1};
    std::atomic<float>    position_rot_;
    std::atomic<float>    applied_pct_{0.0f};
    std::atomic<uint16_t> faults_{0};

    std::atomic<uint64_t> status_0_count_{0};
    std::atomic<uint64_t> status_2_count_{0};
    std::atomic<uint64_t> other_frame_count_{0};
    std::atomic<uint64_t> tx_count_{0};

    std::mutex cmd_mutex_;
    float      target_position_rot_ = 0.0f;
    bool       enabled_             = false;

    std::atomic<bool> running_{false};
    std::thread       tx_thread_;
    std::thread       rx_thread_;
    std::string       rx_buf_;  // only touched by rx_thread_
};

// Enumerate /dev/ttyACM* whose USB ancestor has VID:PID 0483:A30E (REV SPARK
// MAX). Mirrors `_list_spark_ports()` in nudge_sparks.py.
std::vector<std::string> list_sparkmax_ports();

}  // namespace warrior::hardware

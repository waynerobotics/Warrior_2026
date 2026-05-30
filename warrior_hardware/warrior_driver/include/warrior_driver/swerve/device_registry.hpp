#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "warrior_driver/arduino/arduino_serial_device.hpp"
#include "warrior_driver/sparkmax/sparkmax_session.hpp"

namespace warrior::driver {

// Owns the hardware-side serial connections for the manager:
//   - One ArduinoSerialDevice per drive Arduino (matched by <NAME,...> name)
//   - One SparkMaxSession per REV SPARK MAX (USB-CDC, one port per controller —
//     each SPARK MAX *is* its own SLCAN endpoint; there is no shared bus)
//
// A background thread scans /dev/ttyACM* periodically:
//   - Arduinos: probed with <WHO> → <NAME,...> handshake
//   - SPARK MAXes: identified by USB VID:PID 0483:A30E, then each opened
//     session passively scans for the controller's device_id (low 6 bits
//     of any inbound CAN frame). Heartbeat (enable + mode-bitmask) is
//     emitted from session-open so the controller streams Status 0/2.
//
// Public API is thread-safe.
class DeviceRegistry
{
public:
    struct ArduinoConfig
    {
        std::vector<std::string> wanted_names;
        int baud_rate = 115200;
    };

    struct SparkConfig
    {
        // CAN device_ids we want to claim (matches each module's spark_can_id).
        // Sessions whose discovered device_id is not in this list are closed.
        std::vector<int> wanted_can_ids;
    };

    DeviceRegistry(rclcpp::Logger logger,
                   ArduinoConfig arduino_cfg,
                   SparkConfig spark_cfg,
                   std::chrono::duration<double> discovery_period);
    ~DeviceRegistry();

    DeviceRegistry(const DeviceRegistry &) = delete;
    DeviceRegistry & operator=(const DeviceRegistry &) = delete;

    void start();
    void stop();

    // ── Arduino API ────────────────────────────────────────────────────────
    bool is_arduino_connected(const std::string & name);
    std::vector<std::string> connected_arduino_names();
    bool send_drive_percent(const std::string & name, int percent);
    std::vector<std::pair<std::string, std::string>> drain_arduino_messages();
    void send_zero_to_all_arduinos();

    // ── SPARK MAX API ──────────────────────────────────────────────────────
    bool is_spark_connected(int can_id);
    std::vector<int> connected_spark_can_ids();
    std::string spark_port(int can_id);  // empty if not connected

    // Send a position setpoint (motor rotations) to the SPARK at can_id.
    // Implicitly enables setpoint transmission. Returns false if the spark
    // is not connected.
    bool send_steer_position(int can_id, float motor_rotations);

    // Stop sending setpoints to this spark (heartbeat keeps flowing so
    // status frames continue). Use when no command is active.
    void release_steer(int can_id);

    // Latest decoded telemetry. nullopt if controller not connected or no
    // status frame received yet.
    std::optional<float> spark_position_rot(int can_id);
    std::optional<float> spark_applied_pct(int can_id);

    // Seconds since the last Status 2 (position) frame for this controller.
    // Negative if never seen / not connected.
    double seconds_since_spark_position(int can_id, rclcpp::Time now);

private:
    void discovery_loop();
    std::vector<std::string> missing_arduinos_locked() const;
    std::vector<int>         missing_sparks_locked() const;
    void scan_once();
    std::vector<std::string> snapshot_claimed_ports_locked() const;
    std::optional<std::string> probe_arduino(const std::string & port,
                                             ArduinoSerialDevice & dev);
    bool wait_arduino_reset();

    // Open SparkMaxSession on `port`, wait ≤ spark_discover_timeout for it
    // to learn its device_id. Returns it (or nullptr on failure / timeout
    // / unwanted device_id). The returned session is fully running.
    std::unique_ptr<SparkMaxSession> probe_spark(const std::string & port);

    rclcpp::Logger logger_;
    ArduinoConfig arduino_cfg_;
    SparkConfig spark_cfg_;
    std::chrono::duration<double> discovery_period_;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::unique_ptr<ArduinoSerialDevice>> arduinos_;
    // Keyed by SPARK MAX CAN device_id (1..62). Discovered passively.
    std::unordered_map<int, std::unique_ptr<SparkMaxSession>> sparks_;
    // Last time we saw a Status 2 frame from this controller, keyed by can_id.
    std::unordered_map<int, rclcpp::Time> last_spark_status_;

    std::atomic<bool> stop_flag_{false};
    std::condition_variable stop_cv_;
    std::mutex stop_mutex_;
    std::thread discovery_thread_;
};

}  // namespace warrior::driver

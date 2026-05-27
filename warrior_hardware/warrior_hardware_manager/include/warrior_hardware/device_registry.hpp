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

#include "warrior_hardware/arduino_serial_device.hpp"
#include "warrior_hardware/sparkmax_slcan_device.hpp"

namespace warrior::hardware {

// Owns the hardware-side serial connections for the manager:
//   - One ArduinoSerialDevice per drive Arduino (matched by <NAME,...> name)
//   - One SparkMaxSlcanDevice for the shared SLCAN CAN adapter
//
// A background thread scans /dev/ttyACM* and /dev/ttyUSB* periodically:
//   - probes unclaimed ports for an Arduino (WHO/NAME) first
//   - then probes any remaining unclaimed ports for an SLCAN adapter (V)
// Ports already claimed by either side are skipped.
//
// The public API is thread-safe.
class DeviceRegistry
{
public:
    struct ArduinoConfig
    {
        std::vector<std::string> wanted_names;
        int baud_rate = 115200;
    };

    struct SlcanConfig
    {
        // "auto" = scan unclaimed ports; otherwise = use this exact path.
        std::string port = "auto";
        char bitrate_code = '8';  // 1 Mbps — FRC / SPARK MAX default
    };

    DeviceRegistry(rclcpp::Logger logger,
                   ArduinoConfig arduino_cfg,
                   SlcanConfig slcan_cfg,
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

    // ── SLCAN API ──────────────────────────────────────────────────────────
    bool is_slcan_connected();
    std::string slcan_port();  // empty if not connected
    bool send_steer_position(int can_id, float motor_rotations);
    std::vector<sparkmax::CanFrame> drain_slcan_frames();

private:
    void discovery_loop();
    std::vector<std::string> missing_arduinos_locked() const;
    void scan_once();
    std::vector<std::string> snapshot_claimed_ports_locked() const;
    std::optional<std::string> probe_arduino(const std::string & port,
                                             ArduinoSerialDevice & dev);
    bool wait_arduino_reset();

    rclcpp::Logger logger_;
    ArduinoConfig arduino_cfg_;
    SlcanConfig slcan_cfg_;
    std::chrono::duration<double> discovery_period_;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::unique_ptr<ArduinoSerialDevice>> arduinos_;
    std::unique_ptr<SparkMaxSlcanDevice> slcan_;

    std::atomic<bool> stop_flag_{false};
    std::condition_variable stop_cv_;
    std::mutex stop_mutex_;
    std::thread discovery_thread_;
};

}  // namespace warrior::hardware

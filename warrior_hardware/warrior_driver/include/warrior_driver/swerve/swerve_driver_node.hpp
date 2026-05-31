#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <warrior_msgs/msg/swerve_cmd.hpp>
#include <warrior_msgs/msg/swerve_state.hpp>

#include "warrior_driver/swerve/device_registry.hpp"
#include "warrior_driver/swerve/swerve_config.hpp"

namespace warrior::driver {

// ════════════════════════════════════════════════════════════════════════
//  Per-module runtime state.
//  Everything that changes during operation for a single swerve module:
//  the latest commands, the latest feedback, and timing bookkeeping.
// ════════════════════════════════════════════════════════════════════════
struct SwerveModuleState
{
    // ── Latest command (received on /warrior_swerve_command) ──
    double cmd_steer_position_rad   = 0.0;   // desired steer angle (algorithm coords, rad)
    double cmd_drive_velocity_rad_s = 0.0;   // desired drive wheel velocity (rad/s)
    bool   have_command             = false; // true once at least one command has arrived
    rclcpp::Time last_command_time;          // arrival time of the most recent command (timeout source)

    // ── Latest feedback from the SPARK MAX (over SLCAN) ──
    double fb_steer_position_rad   = 0.0;    // measured steer angle (algorithm coords, rad)
    double fb_steer_velocity_rad_s = 0.0;    // measured steer angular velocity (rad/s)
    double fb_drive_position_rad   = 0.0;    // open-loop drive position, integrated from commanded velocity
    rclcpp::Time last_steer_pos_time;        // last time fresh steer position was received (zero = never)
    rclcpp::Time last_steer_vel_time;        // last time fresh steer velocity was received (zero = never)

    rclcpp::Time last_update_time;           // last time update() processed this module (for dt)
};

// ════════════════════════════════════════════════════════════════════════
//  One swerve module = immutable config + mutable runtime state +
//  startup-calibration progress.
// ════════════════════════════════════════════════════════════════════════
struct SwerveModule
{
    SwerveModuleConfig config;               // immutable config loaded from YAML
    SwerveModuleState  state;                // mutable runtime state

    // ── Auto-calibration progress (used only during the startup phase) ──
    std::vector<double> calib_buffer;        // raw encoder readings collected for averaging (motor rotations)
    bool calib_recorded = false;             // true once this module's averaged forward offset is computed
};

// ════════════════════════════════════════════════════════════════════════
//  SwerveDriverNode
//
//  Bridges the upper-layer swerve controller and the low-level hardware:
//    - Subscribes to steer/drive commands (warrior_msgs::msg::SwerveCmd).
//    - Maps algorithm-coordinate commands to hardware encoder targets (steer)
//      and drive percentages, then sends them to the SPARK MAX and Arduino
//      through the DeviceRegistry.
//    - Reads the SPARK MAX absolute encoder, maps it back to algorithm
//      coordinates, and publishes module state (warrior_msgs::msg::SwerveState).
//    - Publishes diagnostics describing connection / feedback health.
//
//  Startup auto-calibration:
//    Before accepting any commands, the node samples each module's absolute
//    encoder while the operator holds the wheels physically pointing
//    "forward". It averages the readings over a window and stores the result
//    as encoder_pos_forward — the encoder value that corresponds to the
//    algorithm's zero (forward) heading. The result is also written to
//    steer_calibration.yaml for the record. If any module fails to connect
//    within calib_timeout_s_, the node shuts itself down.
// ════════════════════════════════════════════════════════════════════════
class SwerveDriverNode : public rclcpp::Node
{
public:
    SwerveDriverNode();
    ~SwerveDriverNode() override;

    // Called by main() before rclcpp::shutdown so the final safe-stop serial
    // write (drive 0% to every Arduino) actually goes out.
    void send_safe_stop();

private:
    // ── Initialization ──
    // Read module_names and each modules.<name>.* parameter from YAML and
    // populate modules_ and module_index_.
    void load_modules();

    // ── ROS callbacks ──
    // Store the latest steer/drive command for the addressed module and stamp
    // its arrival time. Ignores commands for unknown swerve_ids.
    void on_command(const warrior_msgs::msg::SwerveCmd::SharedPtr msg);

    // Main periodic loop (runs at update_rate_hz_). While calibration is not
    // finished it runs the calibration gate and returns early; afterwards it
    // maps and sends commands, reads feedback, and publishes module state.
    void update();

    // Periodic diagnostics publisher (runs at diagnostics_rate_hz_).
    void publish_diagnostics();

    // Read and log any pending lines (ACK/ERR/other) from connected Arduinos.
    void drain_and_log_arduino_messages();

    // ── Helpers ──
    // Look up a module by name; returns nullptr if not present.
    SwerveModule * find_module(const std::string & name);

    // ── Auto-calibration ──
    // One calibration tick. For each not-yet-recorded module: wait for the
    // SPARK MAX to connect and produce fresh encoder data, then accumulate
    // samples. Once calib_samples_ have been gathered, average them, store the
    // result in config.encoder_pos_forward, and mark the module recorded.
    // When all modules are recorded, write the YAML and set calib_done_.
    // If calib_timeout_s_ elapses before all modules connect, log FATAL and
    // call rclcpp::shutdown() to kill the whole program.
    void auto_calibrate();

    // Write the calibrated encoder_pos_forward values to the file at
    // calib_write_path_, formatted so it can be layered on top of
    // warrior_driver.yaml to override encoder_pos_forward. Includes a
    // timestamp comment. No-op (with a warning) if calib_write_path_ is empty.
    void write_calibration_yaml();

    // ── Module storage ──
    std::vector<SwerveModule> modules_;                       // all swerve modules
    std::unordered_map<std::string, std::size_t> module_index_; // name -> index into modules_

    // ── Configuration parameters (from YAML) ──
    std::string command_topic_;              // topic subscribed for incoming commands
    std::string state_topic_;                // topic published with per-module state
    double update_rate_hz_      = 50.0;      // update() frequency
    double command_timeout_s_   = 0.5;       // commands older than this are treated as stale
    double steer_stale_after_s_ = 0.5;       // encoder feedback older than this is "stale"
    int    baud_rate_           = 115200;    // Arduino serial baud rate
    double discovery_period_s_  = 2.0;       // how often the registry rescans for missing devices
    double diagnostics_rate_hz_ = 1.0;       // publish_diagnostics() frequency

    // ── ROS interfaces ──
    rclcpp::Subscription<warrior_msgs::msg::SwerveCmd>::SharedPtr cmd_sub_;
    rclcpp::Publisher<warrior_msgs::msg::SwerveState>::SharedPtr state_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // ── Hardware abstraction (owns Arduino + SPARK MAX sessions) ──
    std::unique_ptr<DeviceRegistry> registry_;

    // ── Auto-calibration parameters and state ──
    bool   calib_done_      = false;         // global gate: false until every module is recorded
    double calib_timeout_s_ = 60.0;          // hard deadline for all modules to connect; FATAL+shutdown on miss
    int    calib_samples_   = 100;           // number of encoder samples averaged per module
    std::string calib_write_path_;           // output path for steer_calibration.yaml ("" = don't write)
    rclcpp::Time calib_start_time_;          // when calibration started (for the timeout check)
};

}  // namespace warrior::driver
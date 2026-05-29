#pragma once

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "warrior_driver/swerve/swerve_config.hpp"

namespace warrior::driver {

/**
 * @brief SteerCalibrationNode — standalone steer-offset calibrator.
 *
 * DOES NOT require warrior_driver to be running. It opens each SPARK MAX
 * USB-SLCAN port directly, sends only broadcast heartbeats (no setpoints),
 * waits for Status 2 position frames, then computes per-module
 * steer_offset_rad values that make the current encoder positions read as 0
 * (straight forward).
 *
 * Procedure:
 *   1. Power OFF the robot.
 *   2. Physically rotate ALL wheels to point straight forward.
 *   3. Power ON the SPARK MAXes (they boot in neutral — wheels stay free).
 *   4. Run this node (warrior_driver must NOT be running):
 *        ros2 run warrior_driver steer_calibration_node \
 *          --ros-args --params-file .../steer_calibration.yaml
 *   5. Call the service:
 *        ros2 service call /steer_calibration/calibrate std_srvs/srv/Trigger "{}"
 *   6. Paste the printed steer_offset_rad values into warrior_driver.yaml.
 *
 * Why warrior_driver must be off:
 *   warrior_driver sends enable + mode + setpoint heartbeats at ~50 Hz from
 *   the moment it starts. Once those arrive, the SPARK MAX locks to its last
 *   setpoint (0 on first boot) and the wheel can no longer be pushed by hand.
 *   This node sends heartbeats WITHOUT setpoints, so the SPARK MAX streams
 *   its encoder position but never drives.
 */
class SteerCalibrationNode : public rclcpp::Node
{
public:
    SteerCalibrationNode();

private:
    void load_modules();

    void on_calibrate(
        const std_srvs::srv::Trigger::Request::SharedPtr  request,
        std_srvs::srv::Trigger::Response::SharedPtr       response);

    // ── per-port SLCAN state (used only during on_calibrate) ─────────────────
    struct PortState
    {
        int         fd                = -1;
        std::string path;
        std::string rx_buf;
        int         can_id            = -1;  ///< learned from first inbound Status frame
        float       position_rot      = std::numeric_limits<float>::quiet_NaN();
        bool        telemetry_sent    = false;
    };

    bool  open_slcan_port(PortState & ps) const;
    void  close_slcan_port(PortState & ps) const;
    bool  write_port(PortState & ps, const std::string & data) const;
    void  drain_port(PortState & ps) const;  ///< read + parse one burst of bytes

    // ── module registry ───────────────────────────────────────────────────────
    struct CalibModule
    {
        SwerveModuleConfig config;
    };

    std::vector<CalibModule>                modules_;
    std::unordered_map<int, std::size_t>    can_id_to_module_;  ///< spark_can_id → index

    // ── ROS interfaces ────────────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;

    // ── parameters ────────────────────────────────────────────────────────────
    char        bitrate_code_    = '8';  ///< SLCAN 'S' command code (8 = 1 Mbit/s)
    double      read_timeout_s_  = 5.0; ///< How long to wait for all Status 2 frames
    std::string output_dir_;
};

}  // namespace warrior::driver

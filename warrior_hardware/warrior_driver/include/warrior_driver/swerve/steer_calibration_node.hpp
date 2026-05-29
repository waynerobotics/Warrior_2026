#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <warrior_msgs/msg/swerve_state.hpp>

#include "warrior_driver/swerve/swerve_config.hpp"

namespace warrior::driver {

/// Holds the live state received from /warrior_swerve_state for one module.
struct CalibModuleState
{
    double steer_position_rad = 0.0;
    bool   have_feedback      = false;
    rclcpp::Time last_feedback_time;
};

/**
 * @brief SteerCalibrationNode
 *
 * Subscribes to /warrior_swerve_state and, when the ~/calibrate service is
 * called, captures the current steer position of every module and computes the
 * steer_offset_rad that would make that position read as 0 rad (straight
 * forward).
 *
 * The computed offsets are:
 *   new_steer_offset_rad = current_steer_position_rad * steer_sign + old_steer_offset_rad
 *
 * Results are printed to the ROS logger and written as a ready-to-paste
 * YAML snippet to a timestamped file in /tmp/.
 *
 * Usage:
 *   1. Physically align all wheels to point straight forward.
 *   2. Start warrior_driver_node so SPARK MAX feedback is live.
 *   3. Start this node (or use the launch file).
 *   4. Call the service:
 *        ros2 service call /steer_calibration/calibrate std_srvs/srv/Trigger "{}"
 *   5. Copy the printed YAML offsets into warrior_driver.yaml.
 */
class SteerCalibrationNode : public rclcpp::Node
{
public:
    SteerCalibrationNode();

private:
    void load_modules();
    void on_state(const warrior_msgs::msg::SwerveState::SharedPtr msg);
    void on_calibrate(
        const std_srvs::srv::Trigger::Request::SharedPtr  request,
        std_srvs::srv::Trigger::Response::SharedPtr       response);

    // Module configuration (mirrors warrior_driver parameters)
    struct CalibModule
    {
        SwerveModuleConfig  config;
        CalibModuleState    state;
    };

    std::vector<CalibModule>                     modules_;
    std::unordered_map<std::string, std::size_t> module_index_;

    // ROS interfaces
    rclcpp::Subscription<warrior_msgs::msg::SwerveState>::SharedPtr state_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              calibrate_srv_;

    std::string state_topic_;
    double      feedback_timeout_s_ = 2.0;   ///< Max age of feedback before refusing to calibrate
    std::string output_dir_;                  ///< Directory to write the YAML snippet
};

}  // namespace warrior::driver

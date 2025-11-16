#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <geometry_msgs/msg/twist.hpp>
#include <controller_interface/controller_interface.hpp>

namespace warrior::control {

class SwerveDriveController final : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    // controller_interface::CallbackReturn on_deactivate(
    //     const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // ROS2 Subscription for cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Desired velocities
    double vx_cmd_ = 0.0;
    double vy_cmd_ = 0.0;
    double wz_cmd_ = 0.0;

    std::vector<std::string> steer_joint_names_;
    std::vector<std::string> drive_joint_names_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steer_cmd_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> drive_cmd_;

    // Command interfaces
    std::vector<hardware_interface::LoanedCommandInterface> wheel_position_interfaces_;
    std::vector<hardware_interface::LoanedCommandInterface> wheel_velocity_interfaces_;
    std::vector<hardware_interface::LoanedCommandInterface> steering_position_interfaces_;

    // Swerve kinematics parameters
    std::unordered_map<std::string, std::pair<double, double>> wheel_dist_from_center_ = {
        {"front_dist", {0.34, 0.}},
        {"left_dist",  {-0.17, 0.294}},
        {"right_dist", {-0.17, -0.294}},
    };

    void computeJointCommand(double vx, double vy, double wz);
};

    
}  // namespace warrior::control
#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

    // Odom estimation variables
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    // ROS odometry publisher and TF broadcaster
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string odom_frame_;
    std::string base_frame_;

    void updateOdometry(double vx, double vy, double wz, double dt);

    // Joint names
    std::vector<std::string> steer_joint_names_;
    std::vector<std::string> drive_joint_names_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steer_cmd_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> drive_cmd_;

    // Command interfaces
    std::vector<hardware_interface::LoanedCommandInterface> wheel_position_interfaces_;
    std::vector<hardware_interface::LoanedCommandInterface> wheel_velocity_interfaces_;
    std::vector<hardware_interface::LoanedCommandInterface> steering_position_interfaces_;

    // Swerve kinematics parameters
    double wheel_radius_;
    std::unordered_map<std::string, std::pair<double, double>> wheel_dist_from_center_;

    void computeJointCommand(double vx, double vy, double wz);
};

    
}  // namespace warrior::control
#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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
    // Node namespace
    std::string controller_namespace_ = "swerve_drive_controller";

    // ROS2 Subscription for cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

    // Desired velocities
    double vx_cmd_ = 0.0;
    double vy_cmd_ = 0.0;
    double wz_cmd_ = 0.0;

    // Odom estimation variables
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    // wheel angular speeds (read from state)
    double front_wheel_w_ = 0.0;
    double left_wheel_w_ = 0.0;
    double right_wheel_w_ = 0.0;

    // steering angles (computed from kinematics) rad
    double front_steer_angle_ = 0.0;
    double left_steer_angle_ = 0.0;
    double right_steer_angle_ = 0.0;

    // ROS odometry publisher and TF broadcaster
    std::string odom_frame_;
    std::string base_frame_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void updateOdometry(double dt);

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
    std::unordered_map<std::string, double> wheel_to_center_;

    double alpha_front_;
    double alpha_left_;
    double alpha_right_;

    void computeJointCommand(double vx, double vy, double wz);
    void readWheelAngularVel();
};

    
}  // namespace warrior::control
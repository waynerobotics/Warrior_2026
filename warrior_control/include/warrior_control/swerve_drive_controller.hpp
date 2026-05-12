#pragma once

#include <array>
#include <eigen3/Eigen/Dense>
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

    // Command velocity limits
    double vx_limit_min_ = -0.3;
    double vx_limit_max_ = 0.3;
    double vy_limit_min_ = -0.3;
    double vy_limit_max_ = 0.3;
    double wz_limit_min_ = -0.5;
    double wz_limit_max_ = 0.5;

    // Odom estimation variables
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double base_link_height_ = 0.0;
    double base_link_height_offset_ = 0.0;

    Eigen::Matrix<double, 6, 1> kf_state_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> kf_cov_ = Eigen::Matrix<double, 6, 6>::Identity();
    bool kf_initialized_ = false;

    double process_noise_position_ = 1e-3;
    double process_noise_yaw_ = 1e-3;
    double process_noise_velocity_ = 1e-2;
    double measurement_noise_linear_ = 5e-2;
    double measurement_noise_angular_ = 5e-2;

    // desired wheel linear speeds (computed from kinematics) m/s
    double desired_front_wheel_speed_ = 0.0;
    double desired_left_wheel_speed_ = 0.0;
    double desired_right_wheel_speed_ = 0.0;

    // wheel angular speeds (read from state interfaces)
    double front_wheel_w_ = 0.0;
    double left_wheel_w_ = 0.0;
    double right_wheel_w_ = 0.0;

    // desired steering angles (computed from kinematics) rad
    double desired_front_steer_angle_ = 0.0;
    double desired_left_steer_angle_ = 0.0;
    double desired_right_steer_angle_ = 0.0;

    // steering angles (read from state interfaces)
    double front_steer_angle_ = 0.0;
    double left_steer_angle_ = 0.0;
    double right_steer_angle_ = 0.0;

    // ROS odometry publisher and TF broadcaster
    std::string odom_frame_;
    std::string base_frame_;
    std::string edge_state_topic_;
    std::string tracking_error_topic_;
    std::string edge_state_frame_id_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_gt_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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
    std::unordered_map<std::string, double> alpha_;

    void applyCmdVelLimits();

    double computeSteerAngle(double desired_angle, double current_angle, double& desired_linear_speed);
    double computeDriveSpeed(double desired_angle, double current_angle, double desired_linear_speed);

    void computeJointCommand(double vx, double vy, double wz);
    void readWheelAngularVel();
    void readSteeringAngles();
    Eigen::Vector2d modulePosition(const std::string & module_name) const;
    bool estimateBodyTwist(Eigen::Vector3d & body_twist) const;
    void updateOdometry(double dt, const Eigen::Vector3d & body_twist);
    void kalmanPredict(double dt);
    void kalmanCorrect(const Eigen::Vector3d & body_twist);
    static double wrap2Pi(double angle);
};

    
}  // namespace warrior::control
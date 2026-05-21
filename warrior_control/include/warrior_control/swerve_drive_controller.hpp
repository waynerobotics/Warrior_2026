#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <functional>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <eigen3/Eigen/Dense>

#include "warrior_control/swerve_ik.hpp"

namespace warrior::control {

class SwerveDriveController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // ── Joint names ──────────────────────────────────────────────────────────
    std::vector<std::string> steer_joint_names_;
    std::vector<std::string> drive_joint_names_;

    // ── Command / state handles ───────────────────────────────────────────────
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steer_cmd_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> drive_cmd_;

    // ── Robot geometry ────────────────────────────────────────────────────────
    double wheel_radius_ = 0.1;
    std::unordered_map<std::string, double> wheel_to_center_;
    std::unordered_map<std::string, double> alpha_;

    // ── Velocity commands (smoothed, sent to IK) ──────────────────────────────
    double vx_cmd_ = 0.0;
    double vy_cmd_ = 0.0;
    double wz_cmd_ = 0.0;

    // ── Velocity targets (raw, from /cmd_vel topic) ───────────────────────────
    double vx_target_ = 0.0;
    double vy_target_ = 0.0;
    double wz_target_ = 0.0;

    // ── Velocity limits ───────────────────────────────────────────────────────
    double vx_limit_min_ = -0.3, vx_limit_max_ = 0.3;
    double vy_limit_min_ = -0.3, vy_limit_max_ = 0.3;
    double wz_limit_min_ = -0.5, wz_limit_max_ = 0.5;

    // ── Velocity smoother parameters ──────────────────────────────────────────
    double max_accel_linear_  = 1.0;   // m/s²   — normal acceleration limit
    double max_accel_angular_ = 2.0;   // rad/s² — normal angular acceleration limit
    double max_decel_linear_  = 3.0;   // m/s²   — emergency stop deceleration
    double max_decel_angular_ = 5.0;   // rad/s² — emergency stop angular decel

    // Target is considered "stop" when its magnitude is below this threshold
    double stop_threshold_linear_  = 0.01;  // m/s
    double stop_threshold_angular_ = 0.01;  // rad/s

    // ── Wheel states ──────────────────────────────────────────────────────────
    double front_wheel_w_    = 0.0;
    double left_wheel_w_     = 0.0;
    double right_wheel_w_    = 0.0;
    double front_steer_angle_ = 0.0;
    double left_steer_angle_  = 0.0;
    double right_steer_angle_ = 0.0;

    // ── Odometry & TF ────────────────────────────────────────────────────────
    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    double base_link_height_offset_ = 0.0;
    double base_link_height_        = 0.0;
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;

    // ── Kalman filter ─────────────────────────────────────────────────────────
    double process_noise_position_  = 1e-3;
    double process_noise_yaw_       = 1e-3;
    double process_noise_velocity_  = 1e-2;
    double measurement_noise_linear_  = 5e-2;
    double measurement_noise_angular_ = 5e-2;

    Eigen::Matrix<double, 6, 1> kf_state_  = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> kf_cov_    = Eigen::Matrix<double, 6, 6>::Identity();
    bool kf_initialized_ = false;

    // ── ROS interfaces ────────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_gt_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── Swerve IK ─────────────────────────────────────────────────────────────
    std::unique_ptr<SwerveIK> swerve_ik_;

    // ── Private methods ───────────────────────────────────────────────────────
    void applyCmdVelLimits();
    void smoothCmdVel(double dt);
    void readWheelAngularVel();
    void readSteeringAngles();
    bool estimateBodyTwist(Eigen::Vector3d & body_twist) const;
    void kalmanPredict(double dt);
    void kalmanCorrect(const Eigen::Vector3d & body_twist);
    void updateOdometry(double dt, const Eigen::Vector3d & body_twist);
    void computeJointCommand(double vx, double vy, double wz);
    double wrap2Pi(double angle);
};

}  // namespace warrior::control
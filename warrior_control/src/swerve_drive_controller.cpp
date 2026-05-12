
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <utility>
#include "warrior_control/swerve_drive_controller.hpp"

namespace warrior::control {

controller_interface::InterfaceConfiguration SwerveDriveController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cmd_config;
    cmd_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cmd_config.names.reserve(steer_joint_names_.size() + drive_joint_names_.size());

    for (const auto & name : steer_joint_names_) {
        cmd_config.names.push_back(name + "/position");
        RCLCPP_INFO(get_node()->get_logger(), "Command Interface added: %s", (name + "/position").c_str());
    }

    for (const auto & name : drive_joint_names_) {
        cmd_config.names.push_back(name + "/velocity");
        RCLCPP_INFO(get_node()->get_logger(), "Command Interface added: %s", (name + "/velocity").c_str());
    }
    return cmd_config;
}

controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_config;
    state_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_config.names.reserve(steer_joint_names_.size() + drive_joint_names_.size());

    for (const auto & name : steer_joint_names_) {
        state_config.names.push_back(name + "/position");
        state_config.names.push_back(name + "/velocity");
        state_config.names.push_back(name + "/effort");
    }   

    for (const auto & name : drive_joint_names_) {
        state_config.names.push_back(name + "/position");
        state_config.names.push_back(name + "/velocity");
        state_config.names.push_back(name + "/effort");
    }
    return state_config;
}

controller_interface::CallbackReturn SwerveDriveController::on_init() {
    auto logger = get_node()->get_logger();

    RCLCPP_INFO(logger, "Initializing SwerveDriveController...");

    wheel_radius_ = auto_declare<double>("wheel_radius", 0.1);
    steer_joint_names_ = auto_declare<std::vector<std::string>>("steer_joint_names", {});
    drive_joint_names_ = auto_declare<std::vector<std::string>>("drive_joint_names", {});

    odom_frame_ = auto_declare<std::string>("odom_frame_id", "odom");
    base_frame_ = auto_declare<std::string>("base_frame_id", "base_footprint");
    base_link_height_offset_ = auto_declare<double>("base_link_height_offset", 0.0);

    process_noise_position_ = auto_declare<double>("kf.process_noise_position", 1e-3);
    process_noise_yaw_ = auto_declare<double>("kf.process_noise_yaw", 1e-3);
    process_noise_velocity_ = auto_declare<double>("kf.process_noise_velocity", 1e-2);
    measurement_noise_linear_ = auto_declare<double>("kf.measurement_noise_linear", 5e-2);
    measurement_noise_angular_ = auto_declare<double>("kf.measurement_noise_angular", 5e-2);

    wheel_to_center_["front"] = auto_declare<double>("wheel_to_center.front", 0.0);
    wheel_to_center_["left"]  = auto_declare<double>("wheel_to_center.left", 0.0);
    wheel_to_center_["right"] = auto_declare<double>("wheel_to_center.right", 0.0);

    alpha_["front"] = auto_declare<double>("alpha.front", 0.0);
    alpha_["left"]  = auto_declare<double>("alpha.left", 0.0);
    alpha_["right"] = auto_declare<double>("alpha.right", 0.0);

    const auto vx_limit = auto_declare<std::vector<double>>("cmd_vel_limit.vx_limit", {-0.3, 0.3});
    const auto vy_limit = auto_declare<std::vector<double>>("cmd_vel_limit.vy_limit", {-0.3, 0.3});
    const auto wz_limit = auto_declare<std::vector<double>>("cmd_vel_limit.wz_limit", {-0.5, 0.5});

    if (vx_limit.size() >= 2) {
        vx_limit_min_ = vx_limit[0];
        vx_limit_max_ = vx_limit[1];
    }
    if (vy_limit.size() >= 2) {
        vy_limit_min_ = vy_limit[0];
        vy_limit_max_ = vy_limit[1];
    }
    if (wz_limit.size() >= 2) {
        wz_limit_min_ = wz_limit[0];
        wz_limit_max_ = wz_limit[1];
    }

    RCLCPP_INFO(logger, "Cmd vel limits loaded: vx=[%.2f, %.2f], vy=[%.2f, %.2f], wz=[%.2f, %.2f]",
                vx_limit_min_, vx_limit_max_, vy_limit_min_, vy_limit_max_, wz_limit_min_, wz_limit_max_);

    RCLCPP_INFO(logger, "SwerveDriveController initialized with %zu steer joints and %zu drive joints.",
                steer_joint_names_.size(), drive_joint_names_.size());
    for (const auto & name : steer_joint_names_) {
        RCLCPP_INFO(logger, "Steer Joint: %s", name.c_str());
    }
    for (const auto & name : drive_joint_names_) {
        RCLCPP_INFO(logger, "Drive Joint: %s", name.c_str());
    }

    if(steer_joint_names_.size() != 3 || drive_joint_names_.size() != 3) {
        RCLCPP_ERROR(logger, "Expected 3 steer and 3 drive joints, got %zu steer and %zu drive.",
                     steer_joint_names_.size(), drive_joint_names_.size());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::CallbackReturn SwerveDriveController::on_configure(
    const rclcpp_lifecycle::State & previous_state) {
    // Implement configuration logic
    (void)previous_state;
    auto logger = get_node()->get_logger();

    steer_cmd_.reserve(steer_joint_names_.size());
    drive_cmd_.reserve(drive_joint_names_.size());

    RCLCPP_INFO(logger, "Configuring SwerveDriveController...");

    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
            vx_cmd_ = msg->twist.linear.x;
            vy_cmd_ = msg->twist.linear.y;
            wz_cmd_ = msg->twist.angular.z;        // Invert angular command to match the errors
            applyCmdVelLimits();
        });

    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom_est", 10);
    // edge_state_pub_ = get_node()->create_publisher<edge_msgs::msg::EdgeState>(edge_state_topic_, 10);
    // tracking_error_pub_ = get_node()->create_publisher<edge_msgs::msg::EdgeState>(tracking_error_topic_, 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

    // edge_action_sub_ = get_node()->create_subscription<edge_msgs::msg::EdgeAction>(
    //     "/edge/command_action", 10,
    //     [this](const edge_msgs::msg::EdgeAction::SharedPtr msg)
    //     {
    //         if (msg->action_type == edge_msgs::msg::EdgeAction::CMD_VEL) {
    //             vx_cmd_ = msg->action_value[0];
    //             vy_cmd_ = msg->action_value[1];
    //             wz_cmd_ = -msg->action_value[2];  // Invert angular command to match the errors
    //             applyCmdVelLimits();
    //         }
    //     });

    // Subscribe to simulation ground-truth odometry for TF broadcasting and base_link height.
    odom_gt_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_gt", 10,  // Subscribe to ground truth odometry
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        nav_msgs::msg::Odometry compensated = *msg;
        compensated.pose.pose.position.z += base_link_height_offset_;
        base_link_height_ = compensated.pose.pose.position.z;
        odom_pub_->publish(compensated);  // Publish to /odom_est

        // Broadcast TF with the compensated values
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = compensated.header;
        tf_msg.child_frame_id = compensated.child_frame_id;
        tf_msg.transform.translation.x = compensated.pose.pose.position.x;
        tf_msg.transform.translation.y = compensated.pose.pose.position.y;
        tf_msg.transform.translation.z = compensated.pose.pose.position.z;
        tf_msg.transform.rotation = compensated.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    });

    RCLCPP_INFO(logger, "SwerveDriveController configured.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {
    (void)previous_state;
    auto logger = get_node()->get_logger();
    RCLCPP_INFO(logger, "Activating SwerveDriveController...");

    steer_cmd_.clear();
    drive_cmd_.clear();

    RCLCPP_INFO(logger, "The size of command_interfaces_: %zu", command_interfaces_.size());

    for(auto & cmd_interface : command_interfaces_) {
        std::string interface_name = cmd_interface.get_name();
        for (const auto & steer_name : steer_joint_names_) {
            if (interface_name == steer_name + "/position") {
                steer_cmd_.emplace_back(std::ref(cmd_interface));
            }
            RCLCPP_INFO(logger, "Steer Interface name: %s", interface_name.c_str());
        }
        for (const auto & drive_name : drive_joint_names_) {
            if (interface_name == drive_name + "/velocity") {
                drive_cmd_.emplace_back(std::ref(cmd_interface));
            }
            RCLCPP_INFO(logger, "Drive Interface name: %s", interface_name.c_str());
        }
    }

    RCLCPP_INFO(logger, "SwerveDriveController activated.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type SwerveDriveController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) {
    
    (void)time;
    double dt = period.seconds();

    applyCmdVelLimits();

    readWheelAngularVel();  // Update wheel angular velocities from state interfaces
    readSteeringAngles();

    Eigen::Vector3d body_twist = Eigen::Vector3d::Zero();
    if (!estimateBodyTwist(body_twist)) {
        body_twist << vx_cmd_, vy_cmd_, wz_cmd_;
    }

    if (!kf_initialized_) {
        kf_state_.setZero();
        kf_state_.segment<3>(3) = body_twist;
        kf_initialized_ = true;
    }

    kalmanPredict(dt);
    kalmanCorrect(body_twist);

    // RCLCPP_INFO(get_node()->get_logger(), "Updating SwerveDriveController with vx: %.2f, vy: %.2f, wz: %.2f",
    //                 vx_cmd_, vy_cmd_, wz_cmd_);

    computeJointCommand(vx_cmd_, vy_cmd_, wz_cmd_);   // Compute and set joint commands

    // updateOdometry(dt, body_twist);  // Update odometry based on filtered estimate

    return controller_interface::return_type::OK;
}

void SwerveDriveController::applyCmdVelLimits() {
    vx_cmd_ = std::clamp(vx_cmd_, vx_limit_min_, vx_limit_max_);
    vy_cmd_ = std::clamp(vy_cmd_, vy_limit_min_, vy_limit_max_);
    wz_cmd_ = std::clamp(wz_cmd_, wz_limit_min_, wz_limit_max_);
}


void SwerveDriveController::readWheelAngularVel() {
    // Read wheel velocities from state interfaces
    for (const auto & state_interface : state_interfaces_) {
        std::string interface_name = state_interface.get_name();
        for (const auto & drive_name : drive_joint_names_) {
            if (interface_name == drive_name + "/velocity") {
                if (drive_name.find("front") != std::string::npos) {
                    front_wheel_w_ = state_interface.get_value();
                } else if (drive_name.find("left") != std::string::npos) {
                    left_wheel_w_ = state_interface.get_value();
                } else if (drive_name.find("right") != std::string::npos) {
                    right_wheel_w_ = state_interface.get_value();
                }
            }
        }
    }
}

void SwerveDriveController::readSteeringAngles() {
    for (const auto & state_interface : state_interfaces_) {
        std::string interface_name = state_interface.get_name();
        for (const auto & steer_name : steer_joint_names_) {
            if (interface_name == steer_name + "/position") {
                if (steer_name.find("front") != std::string::npos) {
                    front_steer_angle_ = state_interface.get_value();
                } else if (steer_name.find("left") != std::string::npos) {
                    left_steer_angle_ = state_interface.get_value();
                } else if (steer_name.find("right") != std::string::npos) {
                    right_steer_angle_ = state_interface.get_value();
                }
            }
        }
    }
}

bool SwerveDriveController::estimateBodyTwist(Eigen::Vector3d & body_twist) const {
    Eigen::Matrix<double, 6, 3> A;
    Eigen::Matrix<double, 6, 1> b;
    A.setZero();
    b.setZero();

    const double df = wheel_to_center_.at("front");
    const double dl = wheel_to_center_.at("left");
    const double dr = wheel_to_center_.at("right");
    const double alpha_front_ = alpha_.at("front");
    const double alpha_left_  = alpha_.at("left");
    const double alpha_right_ = alpha_.at("right");

    const double front_x = df * std::sin(alpha_front_);
    const double front_y = df * std::cos(alpha_front_);
    const double left_x = dl * std::cos(alpha_left_);
    const double left_y = dl * std::sin(alpha_left_);
    const double right_x = dr * std::cos(alpha_right_);
    const double right_y = dr * std::sin(alpha_right_);

    const std::array<double, 3> steer_angles = {
        front_steer_angle_, left_steer_angle_, right_steer_angle_};
    const std::array<double, 3> wheel_speeds = {
        front_wheel_w_, left_wheel_w_, right_wheel_w_};
    const std::array<std::pair<double, double>, 3> module_positions = {
        std::make_pair(front_x, front_y),
        std::make_pair(left_x, left_y),
        std::make_pair(right_x, right_y)};

    for (size_t i = 0; i < 3; ++i) {
        const double wheel_linear = wheel_speeds[i] * wheel_radius_;
        const double theta = steer_angles[i];
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        const double x = module_positions[i].first;
        const double y = module_positions[i].second;

        A(2 * i, 0) = 1.0;
        A(2 * i, 2) = -y;
        b(2 * i) = wheel_linear * c;

        A(2 * i + 1, 1) = 1.0;
        A(2 * i + 1, 2) = x;
        b(2 * i + 1) = wheel_linear * s;
    }

    const Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);
    if (!solution.allFinite()) {
        return false;
    }

    body_twist = solution;
    return true;
}

void SwerveDriveController::kalmanPredict(double dt) {
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q(0, 0) = process_noise_position_ * dt * dt;
    Q(1, 1) = process_noise_position_ * dt * dt;
    Q(2, 2) = process_noise_yaw_ * dt * dt;
    Q(3, 3) = process_noise_velocity_ * dt;
    Q(4, 4) = process_noise_velocity_ * dt;
    Q(5, 5) = process_noise_velocity_ * dt;

    kf_state_ = F * kf_state_;
    kf_state_(2) = wrap2Pi(kf_state_(2));
    kf_cov_ = F * kf_cov_ * F.transpose() + Q;
}

void SwerveDriveController::kalmanCorrect(const Eigen::Vector3d & body_twist) {
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;
    H(2, 5) = 1.0;

    Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();
    R(0, 0) = measurement_noise_linear_;
    R(1, 1) = measurement_noise_linear_;
    R(2, 2) = measurement_noise_angular_;

    const Eigen::Matrix<double, 3, 1> z = body_twist;
    const Eigen::Matrix<double, 3, 1> innovation = z - H * kf_state_;
    const Eigen::Matrix<double, 3, 3> S = H * kf_cov_ * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = kf_cov_ * H.transpose() * S.inverse();

    kf_state_ += K * innovation;
    kf_state_(2) = wrap2Pi(kf_state_(2));

    const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    kf_cov_ = (I - K * H) * kf_cov_;
}

double SwerveDriveController::wrap2Pi(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

void SwerveDriveController::updateOdometry(double dt, const Eigen::Vector3d & body_twist)
{
    (void)dt;
    (void)body_twist;

    x_ = kf_state_(0);
    y_ = kf_state_(1);
    yaw_ = kf_state_(2);

    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);
    const double body_vx = kf_state_(3);
    const double body_vy = kf_state_(4);
    const double body_wz = kf_state_(5);
    const double global_vx = body_vx * cy - body_vy * sy;
    const double global_vy = body_vx * sy + body_vy * cy;

    // === Publish Odometry ===
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = get_node()->now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = global_vx;
    odom.twist.twist.linear.y = global_vy;
    odom.twist.twist.angular.z = body_wz;

    odom.pose.covariance[0] = kf_cov_(0, 0);
    odom.pose.covariance[7] = kf_cov_(1, 1);
    odom.pose.covariance[35] = kf_cov_(2, 2);
    odom.twist.covariance[0] = kf_cov_(3, 3);
    odom.twist.covariance[7] = kf_cov_(4, 4);
    odom.twist.covariance[35] = kf_cov_(5, 5);

    // odom_pub_->publish(odom);
    // RCLCPP_INFO(get_node()->get_logger(), "Published odometry: x=%.2f, y=%.2f, yaw=%.2f",
                // x_, y_, yaw_);

    // === Publish TF ===
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
}

// Compute optimal steering angle and adjust linear speed if reversing is needed
double SwerveDriveController::computeSteerAngle(double desired_angle, double current_angle, double& desired_linear_speed) {
    double delta = wrap2Pi(desired_angle - current_angle);
    
    if (std::fabs(delta) > M_PI_2) {
        desired_angle += M_PI;
        desired_linear_speed *= -1.0;
    }
    
    return wrap2Pi(desired_angle);
}

// Scale drive speed based on steering angle error to prevent wheel slippage
double SwerveDriveController::computeDriveSpeed(double desired_angle, double current_angle, double desired_linear_speed) {
    double angle_error = wrap2Pi(desired_angle - current_angle);
    
    if (std::fabs(angle_error) > M_PI_2) {
        return 0.0;
    }
    
    double scale = 1.0 - (std::fabs(angle_error) / M_PI_2);
    return desired_linear_speed * scale;
}

void SwerveDriveController::computeJointCommand(double vx, double vy, double wz) {

    auto rb1_ = wheel_to_center_["front"];
    auto rb2_ = wheel_to_center_["left"];
    auto rb3_ = wheel_to_center_["right"];

    // Assuming the vehicle body coordinate system is placed at the center of the robot, 
    // and the wheels are located at 120 degree intervals around the center.
    double alpha1 = alpha_["front"];
    double alpha2 = alpha_["left"];
    double alpha3 = alpha_["right"];

    // Get the position vector of each wheel relative to the robot center
    double rb1_x_ = rb1_ * std::cos(alpha1);
    double rb1_y_ = rb1_ * std::sin(alpha1);
    double rb2_x_ = rb2_ * std::cos(alpha2);
    double rb2_y_ = rb2_ * std::sin(alpha2);
    double rb3_x_ = rb3_ * std::cos(alpha3);
    double rb3_y_ = rb3_ * std::sin(alpha3);

    // Implement inverse swerve kinematics calculations
    double x1 = vx - rb1_y_ * wz;
    double y1 = vy + rb1_x_ * wz;
    double x2 = vx - rb2_y_ * wz;
    double y2 = vy + rb2_x_ * wz;
    double x3 = vx - rb3_y_ * wz;
    double y3 = vy + rb3_x_ * wz;

    // Calculate desired wheel angles for steering
    auto d_front_steer_angle = atan2(y1, x1);
    auto d_left_steer_angle = atan2(y2, x2);
    auto d_right_steer_angle = atan2(y3, x3);

    // Calculate desired wheel linear speeds for driving
    auto d_front_wheel_speed = sqrt(pow(x1, 2) + pow(y1, 2));
    auto d_left_wheel_speed  = sqrt(pow(x2, 2) + pow(y2, 2));
    auto d_right_wheel_speed = sqrt(pow(x3, 2) + pow(y3, 2));

    // Compute optimal steering angles and adjust speeds to minimize rotation and prevent slippage
    desired_front_steer_angle_ = computeSteerAngle(d_front_steer_angle, front_steer_angle_, d_front_wheel_speed);
    desired_left_steer_angle_  = computeSteerAngle(d_left_steer_angle,  left_steer_angle_,  d_left_wheel_speed);
    desired_right_steer_angle_ = computeSteerAngle(d_right_steer_angle, right_steer_angle_, d_right_wheel_speed);

    // Compute final wheel linear speeds after adjusting for steering angles
    desired_front_wheel_speed_ = computeDriveSpeed(desired_front_steer_angle_, front_steer_angle_, d_front_wheel_speed);
    desired_left_wheel_speed_  = computeDriveSpeed(desired_left_steer_angle_,  left_steer_angle_,  d_left_wheel_speed);
    desired_right_wheel_speed_ = computeDriveSpeed(desired_right_steer_angle_, right_steer_angle_, d_right_wheel_speed);
    
    // Update wheel angular speed commands
    const double front_wheel_w_cmd = desired_front_wheel_speed_ / wheel_radius_;
    const double left_wheel_w_cmd  = desired_left_wheel_speed_ / wheel_radius_;
    const double right_wheel_w_cmd = desired_right_wheel_speed_ / wheel_radius_;

    RCLCPP_INFO(get_node()->get_logger(), "Computed wheel steering angles (radians): Front: %.2f, Left: %.2f, Right: %.2f",
                desired_front_steer_angle_, desired_left_steer_angle_, desired_right_steer_angle_);
    RCLCPP_INFO(get_node()->get_logger(), "Steering angles (degrees): Front: %.2f, Left: %.2f, Right: %.2f",
                desired_front_steer_angle_ * 180.0 / M_PI, desired_left_steer_angle_ * 180.0 / M_PI, desired_right_steer_angle_ * 180.0 / M_PI);
    RCLCPP_INFO(get_node()->get_logger(), "Computed wheel linear velocities (m/s): Front: %.2f, Left: %.2f, Right: %.2f",
                desired_front_wheel_speed_, desired_left_wheel_speed_, desired_right_wheel_speed_);
    RCLCPP_INFO(get_node()->get_logger(), "Computed wheel angular velocities (rad/s): Front: %.2f, Left: %.2f, Right: %.2f",
                front_wheel_w_cmd, left_wheel_w_cmd, right_wheel_w_cmd);

    // Set commands to interfaces
    for (size_t i = 0; i < drive_cmd_.size(); ++i) {
        if (drive_cmd_[i].get().get_prefix_name().find("front") != std::string::npos) {
            drive_cmd_[i].get().set_value(front_wheel_w_cmd);
        } else if (drive_cmd_[i].get().get_prefix_name().find("left") != std::string::npos) {
            drive_cmd_[i].get().set_value(left_wheel_w_cmd);
        } else if (drive_cmd_[i].get().get_prefix_name().find("right") != std::string::npos) {
            drive_cmd_[i].get().set_value(right_wheel_w_cmd);
        }
    }

    for (size_t i = 0; i < steer_cmd_.size(); ++i) {
        if (steer_cmd_[i].get().get_prefix_name().find("front") != std::string::npos) {
            steer_cmd_[i].get().set_value(desired_front_steer_angle_);
        } else if (steer_cmd_[i].get().get_prefix_name().find("left") != std::string::npos) {
            steer_cmd_[i].get().set_value(desired_left_steer_angle_);
        } else if (steer_cmd_[i].get().get_prefix_name().find("right") != std::string::npos) {
            steer_cmd_[i].get().set_value(desired_right_steer_angle_);
        }
    }
    
}   


}  // namespace warrior::control


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(warrior::control::SwerveDriveController, controller_interface::ControllerInterface)
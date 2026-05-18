
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
    odom_topic_ = auto_declare<std::string>("odom_topic", "/odom_est");
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

    if (vx_limit.size() >= 2) { vx_limit_min_ = vx_limit[0]; vx_limit_max_ = vx_limit[1]; }
    if (vy_limit.size() >= 2) { vy_limit_min_ = vy_limit[0]; vy_limit_max_ = vy_limit[1]; }
    if (wz_limit.size() >= 2) { wz_limit_min_ = wz_limit[0]; wz_limit_max_ = wz_limit[1]; }

    // Initialize SwerveIK
    try {
        swerve_ik_ = std::make_unique<SwerveIK>(wheel_to_center_, alpha_, wheel_radius_);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger, "Failed to initialize SwerveIK: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
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

    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

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

    RCLCPP_INFO(logger, "SwerveDriveController activated with %zu steer and %zu drive interfaces.",
                steer_cmd_.size(), drive_cmd_.size());
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

void SwerveDriveController::computeJointCommand(double vx, double vy, double wz) {
    const std::array<double, 3> current_angles = {
        front_steer_angle_, left_steer_angle_, right_steer_angle_};

    const auto cmds = swerve_ik_->computeSwerveCommand(vx, vy, wz, current_angles);

    RCLCPP_INFO(get_node()->get_logger(), "Steering angles (deg): Front: %.2f, Left: %.2f, Right: %.2f",
        cmds[0].steering_angle * 180.0 / M_PI,
        cmds[1].steering_angle * 180.0 / M_PI,
        cmds[2].steering_angle * 180.0 / M_PI
    );
    RCLCPP_INFO(get_node()->get_logger(), "Driving speeds (rad/s): Front: %.2f, Left: %.2f, Right: %.2f",
        cmds[0].driving_speed, 
        cmds[1].driving_speed, 
        cmds[2].driving_speed
    );

    for (size_t i = 0; i < drive_cmd_.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            if (drive_cmd_[i].get().get_prefix_name().find(SwerveIK::WHEEL_NAMES[j]) != std::string::npos) {
                drive_cmd_[i].get().set_value(cmds[j].driving_speed);
            }
        }
    }

    for (size_t i = 0; i < steer_cmd_.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            if (steer_cmd_[i].get().get_prefix_name().find(SwerveIK::WHEEL_NAMES[j]) != std::string::npos) {
                steer_cmd_[i].get().set_value(cmds[j].steering_angle);
            }
        }
    }
}


}  // namespace warrior::control


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(warrior::control::SwerveDriveController, controller_interface::ControllerInterface)
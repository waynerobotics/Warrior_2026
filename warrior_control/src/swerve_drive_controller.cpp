
#include <cmath>
#include <eigen3/Eigen/Dense>
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

    wheel_to_center_["front"] = auto_declare<double>("wheel_to_center.front", 0.0);
    wheel_to_center_["left"]  = auto_declare<double>("wheel_to_center.left", 0.0);
    wheel_to_center_["right"] = auto_declare<double>("wheel_to_center.right", 0.0);

    // Angles to wheels
    alpha_front_ = auto_declare<double>("alpha.front", 0.0);
    alpha_left_  = auto_declare<double>("alpha.left", 0.0);
    alpha_right_ = auto_declare<double>("alpha.right", 0.0);

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
    auto logger = get_node()->get_logger();

    steer_cmd_.reserve(steer_joint_names_.size());
    drive_cmd_.reserve(drive_joint_names_.size());

    RCLCPP_INFO(logger, "Configuring SwerveDriveController...");

    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/cmd_vel", 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
            vx_cmd_ = msg->twist.linear.x;
            vy_cmd_ = msg->twist.linear.y;
            wz_cmd_ = msg->twist.angular.z;
        });

    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

    RCLCPP_INFO(logger, "SwerveDriveController configured.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {
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
    
    double dt = period.seconds();

    readWheelAngularVel();  // Update wheel angular velocities from state interfaces

    RCLCPP_INFO(get_node()->get_logger(), "Updating SwerveDriveController with vx: %.2f, vy: %.2f, wz: %.2f",
                    vx_cmd_, vy_cmd_, wz_cmd_);

    computeJointCommand(vx_cmd_, vy_cmd_, wz_cmd_);   // Compute and set joint commands

    updateOdometry(dt);  // Update odometry based on commanded velocities

    return controller_interface::return_type::OK;
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

void SwerveDriveController::updateOdometry(double dt)
{
    // === Integrate pose ===
    yaw_ += wz_cmd_ * dt;

    double cy = cos(yaw_);
    double sy = sin(yaw_);

    double global_vx = vx_cmd_ * cy - vy_cmd_ * sy;
    double global_vy = vx_cmd_ * sy + vy_cmd_ * cy;

    x_ += global_vx * dt;
    y_ += global_vy * dt;

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

    odom.twist.twist.linear.x = vx_cmd_;
    odom.twist.twist.linear.y = vy_cmd_;
    odom.twist.twist.angular.z = wz_cmd_;

    odom_pub_->publish(odom);

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

    auto df_ = wheel_to_center_["front"];
    auto dl_ = wheel_to_center_["left"];
    auto dr_ = wheel_to_center_["right"];

    // Implement swerve kinematics calculations
    double lf_x = df_ * sin(alpha_front_);
    double lf_y = df_ * cos(alpha_front_);

    double ll_x = dl_ * cos(alpha_left_);
    double ll_y = dl_ * sin(alpha_left_);   
    double lr_x = dr_ * cos(alpha_right_);
    double lr_y = dr_ * sin(alpha_right_);

    // Calculate wheel angles for steering
    double f_th = atan2(vy - wz * df_ * lf_y, vx + wz * lf_x);
    double l_th = atan2(vy + wz * dl_ * ll_y, vx + wz * ll_x);
    double r_th = atan2(vy + wz * dr_ * lr_y, vx - wz * lr_x);
    

    double front_wheel_v = sqrt(pow(vx + wz * lf_x, 2) + pow(vy - wz * lf_y, 2));
    double left_wheel_v  = sqrt(pow(vx + wz * ll_x, 2) + pow(vy + wz * ll_y, 2));
    double right_wheel_v = sqrt(pow(vx - wz * lr_x, 2) + pow(vy + wz * lr_y, 2));

    RCLCPP_INFO(get_node()->get_logger(), "Computed wheel steering angles (radians): Front: %.2f, Left: %.2f, Right: %.2f",
                f_th, l_th, r_th);
    RCLCPP_INFO(get_node()->get_logger(), "Computed wheel velocities: Front: %.2f, Left: %.2f, Right: %.2f",
                front_wheel_v, left_wheel_v, right_wheel_v);
    
    // Update wheel angular speeds
    front_wheel_w_ = front_wheel_v / wheel_radius_;
    left_wheel_w_  = left_wheel_v / wheel_radius_;
    right_wheel_w_ = right_wheel_v / wheel_radius_;

    // Set commands to interfaces
    for (size_t i = 0; i < drive_cmd_.size(); ++i) {
        if (drive_cmd_[i].get().get_prefix_name().find("front") != std::string::npos) {
            drive_cmd_[i].get().set_value(front_wheel_w_);
        } else if (drive_cmd_[i].get().get_prefix_name().find("left") != std::string::npos) {
            drive_cmd_[i].get().set_value(left_wheel_w_);
        } else if (drive_cmd_[i].get().get_prefix_name().find("right") != std::string::npos) {
            drive_cmd_[i].get().set_value(right_wheel_w_);
        }
    }

    for (size_t i = 0; i < steer_cmd_.size(); ++i) {
        if (steer_cmd_[i].get().get_prefix_name().find("front") != std::string::npos) {
            steer_cmd_[i].get().set_value(f_th);
        } else if (steer_cmd_[i].get().get_prefix_name().find("left") != std::string::npos) {
            steer_cmd_[i].get().set_value(l_th);
        } else if (steer_cmd_[i].get().get_prefix_name().find("right") != std::string::npos) {
            steer_cmd_[i].get().set_value(r_th);
        }
    }
    
}   


}  // namespace warrior::control


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(warrior::control::SwerveDriveController, controller_interface::ControllerInterface)
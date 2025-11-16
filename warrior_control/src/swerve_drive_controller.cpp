
#include "warrior_control/swerve_drive_controller.hpp"
#include <cmath>
// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

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

    steer_joint_names_ = auto_declare<std::vector<std::string>>("steer_joint_names", {});
    drive_joint_names_ = auto_declare<std::vector<std::string>>("drive_joint_names", {});

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

    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            vx_cmd_ = msg->linear.x;
            vy_cmd_ = msg->linear.y;
            wz_cmd_ = msg->angular.z;
        });

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
    
    RCLCPP_INFO(get_node()->get_logger(), "Updating SwerveDriveController with vx: %.2f, vy: %.2f, wz: %.2f",
                    vx_cmd_, vy_cmd_, wz_cmd_);
    computeJointCommand(vx_cmd_, vy_cmd_, wz_cmd_);

    return controller_interface::return_type::OK;
}


void SwerveDriveController::computeJointCommand(double vx, double vy, double wz) {
    // Implement swerve kinematics calculations
    double x1 = wheel_dist_from_center_["front_dist"].first;
    double y1 = wheel_dist_from_center_["front_dist"].second;
    double x2 = wheel_dist_from_center_["left_dist"].first;
    double y2 = wheel_dist_from_center_["left_dist"].second;
    double x3 = wheel_dist_from_center_["right_dist"].first;
    double y3 = wheel_dist_from_center_["right_dist"].second;   

    // std::pair<double, double> f_d = wheel_dist_from_center_["front_dist"];
    // std::pair<double, double> l_d = wheel_dist_from_center_["left_dist"];
    // std::pair<double, double> r_d = wheel_dist_from_center_["right_dist"];

    // Calculate wheel angles for steering
    double f_th = atan2(vy + wz * y1, vx - wz * x1);
    double l_th = atan2(vy + wz * y2, vx - wz * x2);
    double r_th = atan2(vy + wz * y3, vx - wz * x3);
    

    Eigen::Matrix3d  forward_matrix;
    forward_matrix << cos(f_th), sin(f_th), -y1 * cos(f_th) + x1 * sin(f_th),
                      cos(l_th), sin(l_th), -y2 * cos(l_th) + x2 * sin(l_th),
                      cos(r_th), sin(r_th), -y3 * cos(r_th) + x3 * sin(r_th);

    Eigen::Vector3d velocity_vector(vx, vy, wz);
    Eigen::Vector3d wheel_velocity = forward_matrix * velocity_vector;

    RCLCPP_INFO(get_node()->get_logger(), "Computed wheel steering angles (radians): Front: %.2f, Left: %.2f, Right: %.2f",
                f_th, l_th, r_th);
    RCLCPP_INFO(get_node()->get_logger(), "Computed wheel velocities: Front: %.2f, Left: %.2f, Right: %.2f",
                wheel_velocity(0), wheel_velocity(1), wheel_velocity(2));
    

    // Set commands to interfaces
    for (size_t i = 0; i < drive_cmd_.size(); ++i) {
        if (drive_cmd_[i].get().get_prefix_name().find("front") != std::string::npos) {
            drive_cmd_[i].get().set_value(wheel_velocity(0));
        } else if (drive_cmd_[i].get().get_prefix_name().find("left") != std::string::npos) {
            drive_cmd_[i].get().set_value(wheel_velocity(1));
        } else if (drive_cmd_[i].get().get_prefix_name().find("right") != std::string::npos) {
            drive_cmd_[i].get().set_value(wheel_velocity(2));
        }
        // RCLCPP_INFO(get_node()->get_logger(), "Wheel %s velocity command set to %.2f",
        //             drive_cmd_[i].get().get_prefix_name().c_str(),
        //             drive_cmd_[i].get().get_value());
    }

    for (size_t i = 0; i < steer_cmd_.size(); ++i) {
        if (steer_cmd_[i].get().get_prefix_name().find("front") != std::string::npos) {
            steer_cmd_[i].get().set_value(f_th);
        } else if (steer_cmd_[i].get().get_prefix_name().find("left") != std::string::npos) {
            steer_cmd_[i].get().set_value(l_th);
        } else if (steer_cmd_[i].get().get_prefix_name().find("right") != std::string::npos) {
            steer_cmd_[i].get().set_value(r_th);
        }
        // RCLCPP_INFO(get_node()->get_logger(), "Wheel %s steering angle command set to %.2f",
        //             steer_cmd_[i].get().get_prefix_name().c_str(),
        //             steer_cmd_[i].get().get_value());
    }
    
}   

}  // namespace warrior::control


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(warrior::control::SwerveDriveController, controller_interface::ControllerInterface)
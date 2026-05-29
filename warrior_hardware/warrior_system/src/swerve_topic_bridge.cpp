#include "warrior_system/swerve_topic_bridge.hpp"

#include <rclcpp/executors.hpp>

namespace warrior::system
{

CallbackReturn SwerveTopicBridge::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    for (const auto* name : MODULE_NAMES) {
        modules_[name] = ModuleScratch{};
    }

    const auto get_hardware_parameter = [this](const std::string& parameter_name,
                                               const std::string& default_value) {
        if (auto it = info_.hardware_parameters.find(parameter_name);
            it != info_.hardware_parameters.end())
        {
            return it->second;
        }
        return default_value;
    };

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=" + info_.name});
    bridge_node_ = rclcpp::Node::make_shared("_", options);

    const std::string command_topic = get_hardware_parameter(
        "swerve_command_topic", "/warrior_swerve_command");
    const std::string state_topic = get_hardware_parameter(
        "swerve_state_topic",   "/warrior_swerve_state");

    cmd_pub_ = bridge_node_->create_publisher<warrior_msgs::msg::SwerveCmd>(
        command_topic, rclcpp::QoS(1));

    state_sub_ = bridge_node_->create_subscription<warrior_msgs::msg::SwerveState>(
        state_topic, rclcpp::SensorDataQoS(),
        [this](const warrior_msgs::msg::SwerveState::SharedPtr msg) {
            auto it = modules_.find(msg->swerve_id);
            if (it == modules_.end()) return;
            it->second.state_steer_position_rad   = msg->steer_position_rad;
            it->second.state_drive_position_rad   = msg->drive_position_rad;
            it->second.state_drive_velocity_rad_s = msg->drive_velocity_rad_s;
        });

    return CallbackReturn::SUCCESS;
}

// Identify which module + interface a joint name refers to.
// e.g. "front_steer_joint" -> ("front", is_steer=true)
//      "left_drive_joint"  -> ("left",  is_steer=false)
static void identify_joint(const std::string& joint_name,
                           std::string& module_key,
                           bool& is_steer)
{
    const bool is_front = joint_name.find("front") != std::string::npos;
    const bool is_left  = joint_name.find("left")  != std::string::npos;
    const bool is_right = joint_name.find("right") != std::string::npos;

    if (static_cast<int>(is_front) + static_cast<int>(is_left) + static_cast<int>(is_right) != 1) {
        throw std::runtime_error("Cannot identify module for joint: " + joint_name);
    }

    const bool is_drive = joint_name.find("drive") != std::string::npos;
    is_steer            = joint_name.find("steer") != std::string::npos;

    if (static_cast<int>(is_steer) + static_cast<int>(is_drive) != 1) {
        throw std::runtime_error("Cannot identify type for joint: " + joint_name);
    }

    module_key = is_front ? "front" : (is_left ? "left" : "right");
}

std::vector<hardware_interface::StateInterface> SwerveTopicBridge::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (const auto& joint : info_.joints) {
        std::string module_key;
        bool is_steer;
        identify_joint(joint.name, module_key, is_steer);

        auto& scratch = modules_.at(module_key);
        for (const auto& interface : joint.state_interfaces) {
            if (is_steer && interface.name != hardware_interface::HW_IF_POSITION) {
                continue;
            }
            if (!is_steer && interface.name != hardware_interface::HW_IF_POSITION &&
                interface.name != hardware_interface::HW_IF_VELOCITY) {
                continue;
            }
            double* ptr = nullptr;
            if (is_steer) {
                ptr = &scratch.state_steer_position_rad;
            } else if (interface.name == hardware_interface::HW_IF_POSITION) {
                ptr = &scratch.state_drive_position_rad;
            } else {
                ptr = &scratch.state_drive_velocity_rad_s;
            }
            state_interfaces.emplace_back(joint.name, interface.name, ptr);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SwerveTopicBridge::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (const auto& joint : info_.joints) {
        std::string module_key;
        bool is_steer;
        identify_joint(joint.name, module_key, is_steer);

        auto& scratch = modules_.at(module_key);
        const std::string expected_iface = is_steer ? hardware_interface::HW_IF_POSITION
                                                    : hardware_interface::HW_IF_VELOCITY;

        for (const auto& interface : joint.command_interfaces) {
            if (interface.name != expected_iface) continue;
            double* ptr = is_steer ? &scratch.cmd_steer_position_rad
                                   : &scratch.cmd_drive_velocity_rad_s;
            command_interfaces.emplace_back(joint.name, interface.name, ptr);
        }
    }

    return command_interfaces;
}

hardware_interface::return_type SwerveTopicBridge::read(const rclcpp::Time& /*time*/,
                                                        const rclcpp::Duration& /*period*/)
{
    if (rclcpp::ok()) {
        rclcpp::spin_some(bridge_node_);
    }
    // State is written directly into modules_[*].state_* by the subscriber
    // callback, and export_state_interfaces() already points to those fields.
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SwerveTopicBridge::write(const rclcpp::Time& time,
                                                         const rclcpp::Duration& /*period*/)
{
    if (!rclcpp::ok()) return hardware_interface::return_type::OK;

    // Publish one SwerveCmd per module per write tick.
    for (const auto* name : MODULE_NAMES) {
        const auto& scratch = modules_.at(name);
        warrior_msgs::msg::SwerveCmd msg;
        msg.swerve_id            = name;
        msg.steer_position_rad   = scratch.cmd_steer_position_rad;
        msg.drive_velocity_rad_s = scratch.cmd_drive_velocity_rad_s;
        msg.stamp                = time;
        cmd_pub_->publish(msg);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace warrior::system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(warrior::system::SwerveTopicBridge, hardware_interface::SystemInterface)

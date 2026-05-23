#include <rclcpp/executors.hpp>
#include "warrior_system/swerve_topic_bridge.hpp"

namespace warrior::system
{

/***************************************************************************************/
/*******                                                                         *******/
/*******                             on_init                                     *******/
/*******                                                                         *******/
/***************************************************************************************/
CallbackReturn SwerveTopicBridge::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Initialize state and command maps for each swerve module
    for (const auto& [module_key, swerve_id] : swerve_id_map_)
    {
        auto swerve_state = warrior_msgs::msg::SwerveState{};
        swerve_state.swerve_id = swerve_id;
        latest_swerve_states_[swerve_id] = swerve_state;

        auto swerve_cmd = warrior_msgs::msg::SwerveCmd{};
        swerve_cmd.swerve_id = swerve_id;
        latest_swerve_commands_[swerve_id] = swerve_cmd;
    }

    // Helper to read hardware parameters with default values
    const auto get_hardware_parameter = [this](const std::string& parameter_name,
                                               const std::string& default_value) {
        if (auto it = info_.hardware_parameters.find(parameter_name);
            it != info_.hardware_parameters.end())
        {
            return it->second;
        }
        return default_value;
    };

    // Create node (random suffix to avoid duplicate publisher warnings)
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=" + info_.name});
    topic_bridge_node_ = rclcpp::Node::make_shared("_", options);
    

    // Swerve command publisher
    swerve_cmd_publisher_ = topic_bridge_node_->create_publisher<warrior_msgs::msg::SwerveCmd>(
        get_hardware_parameter("swerve_command_topic", "/warrior_swerve_command"), rclcpp::QoS(1));

    // Swerve state subscriber — store by swerve_id
    swerve_state_subscriber_ = topic_bridge_node_->create_subscription<warrior_msgs::msg::SwerveState>(
        get_hardware_parameter("swerve_state_topic", "/warrior_swerve_state"),
        rclcpp::SensorDataQoS(),
        [this](const warrior_msgs::msg::SwerveState::SharedPtr msg) {
            latest_swerve_states_[msg->swerve_id] = *msg;
        });

    return CallbackReturn::SUCCESS;
}

/***************************************************************************************/
/*******                                                                         *******/
/*******           Helper: identify module and type from joint name              *******/
/*******                                                                         *******/
/***************************************************************************************/
static void identify_joint(const std::string& joint_name,
                            std::string& module_key,
                            bool& is_steer)
{
    bool is_front = joint_name.find("front") != std::string::npos;
    bool is_left  = joint_name.find("left")  != std::string::npos;
    bool is_right = joint_name.find("right") != std::string::npos;

    if ((int)is_front + (int)is_left + (int)is_right != 1)
    {
        throw std::runtime_error("Cannot identify module for joint: " + joint_name);
    }

    bool is_drive = joint_name.find("drive") != std::string::npos;
    is_steer      = joint_name.find("steer") != std::string::npos;

    if ((int)is_steer + (int)is_drive != 1)
    {
        throw std::runtime_error("Cannot identify type for joint: " + joint_name);
    }

    module_key = is_front ? "front" : (is_left ? "left" : "right");
}

/***************************************************************************************/
/*******                                                                         *******/
/*******                export_state_interfaces (for controller)                 *******/
/*******                                                                         *******/
/***************************************************************************************/
std::vector<hardware_interface::StateInterface> SwerveTopicBridge::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (const auto& joint : info_.joints)
    {
        std::string module_key;
        bool is_steer;
        identify_joint(joint.name, module_key, is_steer);

        const std::string& swerve_id       = swerve_id_map_.at(module_key);
        std::string        expected_iface  = is_steer ? hardware_interface::HW_IF_POSITION
                                                      : hardware_interface::HW_IF_VELOCITY;

        for (const auto& interface : joint.state_interfaces)
        {
            if (interface.name != expected_iface) continue;

            if (is_steer)
            {
                state_interfaces.emplace_back(joint.name, interface.name,
                                              &latest_swerve_states_[swerve_id].steer_angle);
            }
            else
            {
                state_interfaces.emplace_back(joint.name, interface.name,
                                              &latest_swerve_states_[swerve_id].drive_speed);
            }
        }
    }

    return state_interfaces;
}

/***************************************************************************************/
/*******                                                                         *******/
/*******                export_command_interfaces (for controller)               *******/
/*******                                                                         *******/
/***************************************************************************************/
std::vector<hardware_interface::CommandInterface> SwerveTopicBridge::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (const auto& joint : info_.joints)
    {
        std::string module_key;
        bool is_steer;
        identify_joint(joint.name, module_key, is_steer);

        const std::string& swerve_id      = swerve_id_map_.at(module_key);
        std::string        expected_iface = is_steer ? hardware_interface::HW_IF_POSITION
                                                     : hardware_interface::HW_IF_VELOCITY;

        for (const auto& interface : joint.command_interfaces)
        {
            if (interface.name != expected_iface) continue;

            if (is_steer)
            {
                command_interfaces.emplace_back(joint.name, interface.name,
                                                &latest_swerve_commands_[swerve_id].spark);
            }
            else
            {
                command_interfaces.emplace_back(joint.name, interface.name,
                                                &latest_swerve_commands_[swerve_id].flipsky);
            }
        }
    }

    return command_interfaces;
}

/***************************************************************************************/
/*******                                                                         *******/
/*******                                 read                                    *******/
/*******                                                                         *******/
/***************************************************************************************/
hardware_interface::return_type SwerveTopicBridge::read(const rclcpp::Time& /*time*/,
                                                        const rclcpp::Duration& /*period*/)
{
    if (rclcpp::ok())
    {
        rclcpp::spin_some(topic_bridge_node_);
    }

    // State is written directly into latest_swerve_states_ via the subscriber callback,
    // and export_state_interfaces() already points to those fields — nothing else to do here.

    return hardware_interface::return_type::OK;
}

/***************************************************************************************/
/*******                                                                         *******/
/*******                                 write                                   *******/
/*******                                                                         *******/
/***************************************************************************************/
hardware_interface::return_type SwerveTopicBridge::write(const rclcpp::Time& /*time*/,
                                                         const rclcpp::Duration& /*period*/)
{
    if (!rclcpp::ok()) return hardware_interface::return_type::OK;

    // Publish one SwerveCmd per module.
    // Command values are written directly into latest_swerve_commands_ by the controller
    // via the pointers registered in export_command_interfaces().
    for (const auto& [module_key, swerve_id] : swerve_id_map_)
    {
        swerve_cmd_publisher_->publish(latest_swerve_commands_[swerve_id]);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace warrior::system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(warrior::system::SwerveTopicBridge, hardware_interface::SystemInterface)
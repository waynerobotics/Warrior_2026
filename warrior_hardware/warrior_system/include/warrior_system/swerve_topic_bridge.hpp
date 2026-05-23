#pragma once

#include <memory>
#include <string>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <warrior_msgs/msg/swerve_cmd.hpp>
#include <warrior_msgs/msg/swerve_state.hpp>

namespace warrior::system {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SwerveTopicBridge final : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

private:

    // const std::unordered_map<std::string, std::string> swerve_id_map_ = {
    //     {"front", warrior_msgs::msg::SwerveState::FRONT}, 
    //     {"left", warrior_msgs::msg::SwerveState::LEFT}, 
    //     {"right", warrior_msgs::msg::SwerveState::RIGHT}
    // };

    // Publisher and subscriber for the topic-bridge communication with the robot's joints
    rclcpp::Subscription<warrior_msgs::msg::SwerveState>::SharedPtr swerve_state_subscriber_;
    rclcpp::Publisher<warrior_msgs::msg::SwerveCmd>::SharedPtr swerve_cmd_publisher_;
    
    rclcpp::Node::SharedPtr topic_bridge_node_;
    std::unordered_map<std::string, warrior_msgs::msg::SwerveState> latest_swerve_states_;
    std::unordered_map<std::string, warrior_msgs::msg::SwerveCmd> latest_swerve_commands_;
    bool sum_wrapped_joint_states_{ false };

    /// The size of this vector is (standard_interfaces_.size() x nr_joints)
    // std::vector<std::vector<double>> joint_commands_;
    // std::vector<std::vector<double>> joint_states_;

    // If the difference between the current joint state and joint command is less than this value,
    // the joint command will not be published.
    // double trigger_joint_command_threshold_ = 1e-5;

    template <typename HandleType>
    bool getHWInterface(const std::string& name, const std::string& interface_name, const size_t vector_index,
                        std::vector<std::vector<double>>& values, std::vector<HandleType>& interfaces);
};

}  // namespace warrior::system
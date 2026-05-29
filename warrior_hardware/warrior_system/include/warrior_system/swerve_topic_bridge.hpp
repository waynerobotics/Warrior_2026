#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>

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
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    static constexpr std::array<const char*, 3> MODULE_NAMES = {"front", "left", "right"};

    // Per-module pointer-backed scratch. The controller hands out references
    // to these fields via the LoanedCommand/StateInterface; read() copies
    // latest state from /warrior_swerve_state into the state_* fields, and
    // write() copies cmd_* into one SwerveCmd per module on the wire.
    struct ModuleScratch
    {
        double state_steer_position_rad   = 0.0;
        double state_drive_position_rad   = 0.0;
        double state_drive_velocity_rad_s = 0.0;
        double cmd_steer_position_rad     = 0.0;
        double cmd_drive_velocity_rad_s   = 0.0;
    };

    rclcpp::Node::SharedPtr bridge_node_;
    rclcpp::Subscription<warrior_msgs::msg::SwerveState>::SharedPtr state_sub_;
    rclcpp::Publisher<warrior_msgs::msg::SwerveCmd>::SharedPtr cmd_pub_;

    std::unordered_map<std::string, ModuleScratch> modules_;
};

}  // namespace warrior::system

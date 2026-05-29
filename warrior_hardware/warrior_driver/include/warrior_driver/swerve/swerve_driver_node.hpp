#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <warrior_msgs/msg/swerve_cmd.hpp>
#include <warrior_msgs/msg/swerve_state.hpp>

#include "warrior_driver/swerve/device_registry.hpp"
#include "warrior_driver/utils/module_config.hpp"

namespace warrior::driver {

class SwerveDriverNode : public rclcpp::Node
{
public:
    SwerveDriverNode();
    ~SwerveDriverNode() override;

    // Called by main() before rclcpp::shutdown so the last serial write goes out.
    void send_safe_stop();

private:
    struct ModuleRuntimeState
    {
        // Latest command (from /warrior_swerve_command)
        double cmd_steer_position_rad   = 0.0;
        double cmd_drive_velocity_rad_s = 0.0;
        bool   have_command             = false;
        rclcpp::Time last_command_time;

        // Latest feedback from SPARK MAX (over SLCAN)
        double fb_steer_position_rad     = 0.0;
        double fb_steer_velocity_rad_s   = 0.0;
        rclcpp::Time last_steer_pos_time;  // zero if never received
        rclcpp::Time last_steer_vel_time;
    };

    struct Module
    {
        warrior::driver::ModuleConfig config;
        ModuleRuntimeState runtime;
    };

    void load_modules();
    void on_command(const warrior_msgs::msg::SwerveCmd::SharedPtr msg);
    void update();
    void publish_diagnostics();
    void drain_and_log_arduino_messages();
    Module * find_module(const std::string & name);

    std::vector<Module> modules_;
    std::unordered_map<std::string, std::size_t> module_index_;

    std::string command_topic_;
    std::string state_topic_;
    double update_rate_hz_      = 50.0;
    double command_timeout_s_   = 0.5;
    double steer_stale_after_s_ = 0.5;
    int    baud_rate_           = 115200;
    double discovery_period_s_  = 2.0;
    double diagnostics_rate_hz_ = 1.0;

    rclcpp::Subscription<warrior_msgs::msg::SwerveCmd>::SharedPtr cmd_sub_;
    rclcpp::Publisher<warrior_msgs::msg::SwerveState>::SharedPtr state_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    std::unique_ptr<DeviceRegistry> registry_;
};

}  // namespace warrior::driver

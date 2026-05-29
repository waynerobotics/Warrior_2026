#include <rclcpp/rclcpp.hpp>

#include "warrior_motor_manager/motor_manager_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<warrior::hardware::MotorManagerNode>();
    rclcpp::spin(node);
    node->send_safe_stop();  // last act before rclcpp::shutdown
    rclcpp::shutdown();
    return 0;
}

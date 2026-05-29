#include <rclcpp/rclcpp.hpp>

#include "warrior_driver/swerve/swerve_driver_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<warrior::driver::SwerveDriverNode>();
    rclcpp::spin(node);
    node->send_safe_stop();  // last act before rclcpp::shutdown
    rclcpp::shutdown();
    return 0;
}

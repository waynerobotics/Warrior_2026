#include <rclcpp/rclcpp.hpp>
#include "warrior_driver/swerve/steer_calibration_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<warrior::driver::SteerCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}

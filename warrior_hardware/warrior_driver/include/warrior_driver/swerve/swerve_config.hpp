#pragma once

#include <string>

namespace warrior::driver {

struct SwerveModuleConfig
{
    std::string name;

    std::string drive_device_name;
    std::string steer_device_name;

    int spark_can_id = 0;

    double steer_motor_rot_per_module_rot = 1.0;
    double steer_offset_rad               = 0.0;
    double steer_sign                     = 1.0;
    double encoder_pos_forward            = 0.0;
    double drive_sign                     = 1.0;
    double max_drive_rad_s                = 1.0;
};

}  // namespace warrior::driver

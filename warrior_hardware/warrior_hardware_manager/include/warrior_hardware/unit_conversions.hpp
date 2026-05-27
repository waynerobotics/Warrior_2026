#pragma once

#include <algorithm>
#include <cmath>

#include "warrior_hardware/module_config.hpp"

namespace warrior::hardware {

// Steer position (radians at the module) -> SPARK MAX motor rotations.
inline double steer_rad_to_motor_rotations(double steer_position_rad, const ModuleConfig & cfg)
{
    const double corrected_rad = cfg.steer_sign * steer_position_rad + cfg.steer_offset_rad;
    const double module_rotations = corrected_rad / (2.0 * M_PI);
    return module_rotations * cfg.steer_motor_rot_per_module_rot;
}

// Drive velocity (rad/s at the wheel) -> integer percent in [-100, 100].
inline int drive_rad_s_to_percent(double drive_velocity_rad_s, const ModuleConfig & cfg)
{
    const double max = cfg.max_drive_rad_s > 0.0 ? cfg.max_drive_rad_s : 1.0;
    double normalized = drive_velocity_rad_s / max;
    normalized = std::clamp(normalized, -1.0, 1.0);
    return static_cast<int>(std::round(normalized * 100.0 * cfg.drive_sign));
}

// Inverse: SPARK MAX motor rotations -> module steer position in radians.
inline double motor_rotations_to_steer_rad(double motor_rotations, const ModuleConfig & cfg)
{
    const double gear = cfg.steer_motor_rot_per_module_rot != 0.0
                          ? cfg.steer_motor_rot_per_module_rot : 1.0;
    const double sign = cfg.steer_sign != 0.0 ? cfg.steer_sign : 1.0;

    const double module_rotations = motor_rotations / gear;
    const double corrected_rad    = module_rotations * 2.0 * M_PI;
    return (corrected_rad - cfg.steer_offset_rad) / sign;
}

// Inverse: SPARK MAX motor RPM -> module steer velocity in rad/s.
// (Velocity offset is constant so it cancels.)
inline double motor_rpm_to_steer_rad_s(double motor_rpm, const ModuleConfig & cfg)
{
    const double gear = cfg.steer_motor_rot_per_module_rot != 0.0
                          ? cfg.steer_motor_rot_per_module_rot : 1.0;
    const double sign = cfg.steer_sign != 0.0 ? cfg.steer_sign : 1.0;

    const double motor_rot_per_s  = motor_rpm / 60.0;
    const double module_rot_per_s = motor_rot_per_s / gear;
    return (module_rot_per_s * 2.0 * M_PI) / sign;
}

}  // namespace warrior::hardware

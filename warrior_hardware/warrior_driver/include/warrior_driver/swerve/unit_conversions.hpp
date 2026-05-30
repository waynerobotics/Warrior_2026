#pragma once

#include <algorithm>
#include <cmath>

#include "warrior_driver/swerve/swerve_config.hpp"

namespace warrior::driver {

// ── Steer command from algorithm (algorithm coordinates)
//    -> SPARK MAX motor rotations (hardware coordinates) ──
//
// steer_motor_rot_per_module_rot: gear ratio
//   = motor rotations / module rotations
//   Example: 42.0 means motor must rotate 42 times for module to rotate once
inline double steer_rad_to_encoder_pos(double steer_position_rad, const SwerveModuleConfig & cfg)
{
    // Apply sign correction and offset
    const double corrected_rad = cfg.steer_sign * steer_position_rad;
    
    // Convert radians to module rotations (1 full rotation = 2π radians)
    const double delta_rotations = corrected_rad * cfg.steer_motor_rot_per_module_rot / (2.0 * M_PI);
    
    // Apply gear ratio: convert module rotations to motor rotations
    double encoder_position = delta_rotations + cfg.encoder_pos_forward;  // Add encoder offset before applying gear ratio
    
    // Wrap motor rotations to [0, gear_ratio) range (like wrapping to [0, 2π))
    if (cfg.steer_motor_rot_per_module_rot > 0.0) {
        encoder_position = std::fmod(encoder_position, cfg.steer_motor_rot_per_module_rot);
        if (encoder_position < 0.0) {
            encoder_position += cfg.steer_motor_rot_per_module_rot;
        }
    }
    
    return encoder_position;
}

// Drive velocity (rad/s at the wheel) -> integer percent in [-100, 100].
inline int drive_rad_s_to_percent(double drive_velocity_rad_s, const SwerveModuleConfig & cfg)
{
    const double max = cfg.max_drive_rad_s > 0.0 ? cfg.max_drive_rad_s : 1.0;
    double normalized = drive_velocity_rad_s / max;
    normalized = std::clamp(normalized, -1.0, 1.0);
    return static_cast<int>(std::round(normalized * 100.0 * cfg.drive_sign));
}

// ── SPARK MAX encoder position (hardware coordinates, rotations)
//    -> Steer position for upper layer (algorithm coordinates, radians) ──
inline double encoder_pos_to_steer_rad(double encoder_pos, const SwerveModuleConfig & cfg)
{
    const double gear = cfg.steer_motor_rot_per_module_rot != 0.0
                          ? cfg.steer_motor_rot_per_module_rot : 1.0;
    const double sign = cfg.steer_sign != 0.0 ? cfg.steer_sign : 1.0;

    // Subtract encoder offset (in motor rotations) to get delta rotations
    const double delta_rotations = encoder_pos - cfg.encoder_pos_forward;
    
    // Convert motor rotations to radians (divide by gear ratio, multiply by 2π)
    const double corrected_rad = delta_rotations / gear * (2.0 * M_PI);
    
    // Remove sign to convert back to algorithm coordinates
    return corrected_rad * sign;
}

// Inverse: SPARK MAX motor RPM -> module steer velocity in rad/s.
// (Velocity offset is constant so it cancels.)
inline double motor_rpm_to_steer_rad_s(double motor_rpm, const SwerveModuleConfig & cfg)
{
    const double gear = cfg.steer_motor_rot_per_module_rot != 0.0
                          ? cfg.steer_motor_rot_per_module_rot : 1.0;
    const double sign = cfg.steer_sign != 0.0 ? cfg.steer_sign : 1.0;

    const double motor_rot_per_s  = motor_rpm / 60.0;
    const double module_rot_per_s = motor_rot_per_s / gear;
    return (module_rot_per_s * 2.0 * M_PI) / sign;
}

}  // namespace warrior::driver
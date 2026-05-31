#pragma once

#include <algorithm>
#include <cmath>

#include "warrior_driver/swerve/swerve_config.hpp"

namespace warrior::driver {

// ── Steer command from algorithm (algorithm coordinates, radians)
//    -> SPARK MAX absolute encoder target (hardware coordinates, motor rotations) ──
//
// steer_motor_rot_per_module_rot: gear ratio = motor rotations / module rotations.
//   Example: 42.0 means the motor must rotate 42 times for the module to rotate once.
//
// The SPARK MAX uses a CUMULATIVE position controller: a target of 84 means
// "rotate to absolute position 84", and setting 0 afterwards makes it unwind
// 2 full motor turns. It does NOT know that 84, 42, and 0 point the same way.
//
// To avoid forcing the controller to spin a full extra turn whenever the
// algorithm angle crosses the ±π boundary, we snap the computed target to the
// equivalent position NEAREST the current encoder reading, so the move is
// always at most half a module turn.
inline double steer_rad_to_encoder_pos(double steer_position_rad,
                                        const SwerveModuleConfig & cfg,
                                        double current_encoder_pos)
{
    // Apply steer-direction sign to the algorithm angle.
    const double corrected_rad = cfg.steer_sign * steer_position_rad;

    // Convert radians to motor rotations:
    //   radians -> module rotations (/ 2π) -> motor rotations (* gear ratio)
    const double delta_rotations =
        corrected_rad * cfg.steer_motor_rot_per_module_rot / (2.0 * M_PI);

    // Add the calibrated forward offset (in motor rotations) to obtain the
    // absolute encoder target that corresponds to this commanded angle.
    double target = delta_rotations + cfg.encoder_pos_forward;

    // Snap the target to the equivalent position nearest the current reading.
    // period = motor rotations per one full module turn. Any target +/- N*period
    // points the wheel the same physical direction, so we pick the representative
    // within half a turn of where the encoder currently is. This keeps a
    // cumulative controller from taking the long way around (a full-turn jolt).
    const double period = cfg.steer_motor_rot_per_module_rot;
    if (period > 0.0) {
        double diff = std::fmod(target - current_encoder_pos, period);
        if (diff >  period / 2.0) diff -= period;
        if (diff < -period / 2.0) diff += period;
        target = current_encoder_pos + diff;
    }

    return target;
}

// ── Drive velocity (rad/s at the wheel) -> integer percent in [-100, 100] ──
//
// max_drive_rad_s maps to 100% duty. The command is normalized by this max,
// clamped to [-1, 1], scaled to a percentage, and given the drive-direction sign.
inline int drive_rad_s_to_percent(double drive_velocity_rad_s, const SwerveModuleConfig & cfg)
{
    const double max = cfg.max_drive_rad_s > 0.0 ? cfg.max_drive_rad_s : 1.0;
    double normalized = drive_velocity_rad_s / max;
    normalized = std::clamp(normalized, -1.0, 1.0);
    return static_cast<int>(std::round(normalized * 100.0 * cfg.drive_sign));
}

// ── SPARK MAX absolute encoder reading (hardware coordinates, motor rotations)
//    -> Steer position for the upper layer (algorithm coordinates, radians) ──
//
// Exact inverse of steer_rad_to_encoder_pos (minus the nearest-equivalent snap,
// which doesn't affect the reported angle): subtract the forward offset, convert
// motor rotations back to radians, then remove the steer sign.
inline double encoder_pos_to_steer_rad(double encoder_pos, const SwerveModuleConfig & cfg)
{
    const double gear = cfg.steer_motor_rot_per_module_rot != 0.0
                          ? cfg.steer_motor_rot_per_module_rot : 1.0;
    const double sign = cfg.steer_sign != 0.0 ? cfg.steer_sign : 1.0;

    // Subtract the calibrated forward offset to get a delta in motor rotations.
    const double delta_rotations = encoder_pos - cfg.encoder_pos_forward;

    // Convert motor rotations back to radians:
    //   motor rotations -> module rotations (/ gear) -> radians (* 2π)
    const double corrected_rad = delta_rotations / gear * (2.0 * M_PI);

    // Remove the steer sign to return to algorithm coordinates.
    // (sign is +1 or -1, so multiply and divide are equivalent.)
    return corrected_rad * sign;
}

// ── SPARK MAX motor RPM -> module steer velocity in rad/s ──
//
// The constant forward offset cancels in a velocity, so only gear ratio and
// sign matter here.
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
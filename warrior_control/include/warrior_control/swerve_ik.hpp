// swerve_ik.hpp

/**
 * @brief Inverse kinematics solver for a 3-wheeled swerve drive robot.
 *
 * Computes optimal steering angles and drive speeds for each wheel module
 * given a desired body twist (vx, vy, wz). The solver minimizes unnecessary
 * wheel rotation by flipping the drive direction when the required steering
 * change exceeds 90 degrees, and scales drive speed based on steering error
 * to prevent wheel slippage during reorientation.
 *
 * Assumptions:
 * - All wheels share the same radius.
 * - Each wheel module position is defined by its distance to the robot center
 *   and its angular offset (alpha) from the robot's x-axis.
 * - The robot body frame is centered at the geometric center of the wheel modules.
 *
 * @note Thread-safe for read-only computeSwerveCommand() calls after construction.
 */

#pragma once
#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <stdexcept>

namespace warrior::control {

struct SwerveCommand {
    double steering_angle = 0.0;  // rad
    double driving_speed  = 0.0;  // rad/s (wheel angular velocity)
};

class SwerveIK {
public:
    static constexpr std::array<const char*, 3> WHEEL_NAMES = {"front", "left", "right"};

    SwerveIK(
        const std::unordered_map<std::string, double>& wheel_to_center,
        const std::unordered_map<std::string, double>& alpha,
        double wheel_radius)
    : wheel_to_center_(wheel_to_center)
    , alpha_(alpha)
    , wheel_radius_(wheel_radius)
    {
        if (wheel_to_center_.size() != 3 || alpha_.size() != 3) {
            throw std::runtime_error(
                "SwerveIK: expected 3 wheels, got wheel_to_center=" +
                std::to_string(wheel_to_center_.size()) +
                " alpha=" + std::to_string(alpha_.size()));
        }
    }

    /**
     * @brief Compute wheel commands from desired body twist.
     *
     * @param vx              Desired linear velocity in x (m/s)
     * @param vy              Desired linear velocity in y (m/s)
     * @param wz              Desired angular velocity around z (rad/s)
     * @param current_angles  Current steering angles {front, left, right} (rad)
     * @return                Wheel commands {front, left, right}
     */
    std::array<SwerveCommand, 3> computeSwerveCommand(
        double vx, double vy, double wz,
        const std::array<double, 3>& current_angles) const
    {
        std::array<SwerveCommand, 3> cmds;

        // If the desired body velocity is very small, keep current angles and set speeds to zero
        const double speed_threshold = 0.01;  // m/s
        if (std::hypot(vx, vy) < speed_threshold && std::fabs(wz) < speed_threshold) {
            for (int i = 0; i < 3; ++i) {
                cmds[i].steering_angle = current_angles[i];  // Keep current angle
                cmds[i].driving_speed  = 0.0;
            }
            return cmds;
        }

        // Compute desired wheel speeds and angles based on inverse kinematics
        for (int i = 0; i < 3; ++i) {
            const double rbi     = wheel_to_center_.at(WHEEL_NAMES[i]);
            const double alpha_i = alpha_.at(WHEEL_NAMES[i]);
            const double rbi_x   = rbi * std::cos(alpha_i);
            const double rbi_y   = rbi * std::sin(alpha_i);

            const double xi = vx - rbi_y * wz;
            const double yi = vy + rbi_x * wz;

            double desired_angle = std::atan2(yi, xi);
            double desired_speed = std::hypot(xi, yi);

            desired_angle = computeSteerAngle(desired_angle, current_angles[i], desired_speed);
            desired_speed = computeDriveSpeed(desired_angle, current_angles[i], desired_speed);

            cmds[i].steering_angle = desired_angle;
            cmds[i].driving_speed  = desired_speed / wheel_radius_;
        }

        return cmds;
    }

private:
    static double wrap2Pi(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    static double computeSteerAngle(double desired_angle, double current_angle, double& desired_linear_speed)
    {
        const double delta = wrap2Pi(desired_angle - current_angle);
        if (std::fabs(delta) > M_PI_2) {
            desired_angle += M_PI;
            desired_linear_speed *= -1.0;
        }
        return wrap2Pi(desired_angle);
    }

    static double computeDriveSpeed(double desired_angle, double current_angle, double desired_linear_speed)
    {
        const double angle_error = std::fabs(wrap2Pi(desired_angle - current_angle));
        if (angle_error > M_PI_2) return 0.0;
        return desired_linear_speed * std::cos(angle_error);
    }

    std::unordered_map<std::string, double> wheel_to_center_;
    std::unordered_map<std::string, double> alpha_;
    double wheel_radius_;
};

}  // namespace warrior::control
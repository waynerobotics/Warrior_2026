// Copyright 2026 Wayne Robotics. Licensed under the Apache License, Version 2.0.
#ifndef WARRIOR_MTI680G_SIM__MODEL_HPP_
#define WARRIOR_MTI680G_SIM__MODEL_HPP_

#include <array>
#include <cmath>
#include <random>
#include <stdexcept>

namespace warrior_mti680g_sim
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

inline double radians(double degrees) {return degrees * kPi / 180.0;}

inline double sample_sigma(double density, double frequency)
{
  // One-sided white-noise density, ideal anti-alias bandwidth = sample rate/2.
  return density * std::sqrt(frequency / 2.0);
}

inline double horizontal_sigma(double cep, double baseline, double ppm)
{
  return (cep + baseline * ppm * 1e-6) / std::sqrt(2.0 * std::log(2.0));
}

inline std::array<double, 3> offset_lla(
  double latitude, double longitude, double altitude, double east, double north, double up)
{
  // WGS84 local tangent approximation, suitable for centimeter sensor errors.
  constexpr double a = 6378137.0;
  constexpr double e2 = 6.69437999014e-3;
  const double phi = radians(latitude);
  const double denominator = 1.0 - e2 * std::sin(phi) * std::sin(phi);
  const double rn = a / std::sqrt(denominator);
  const double rm = a * (1.0 - e2) / std::pow(denominator, 1.5);
  return {latitude + north / (rm + altitude) * 180.0 / kPi,
    longitude + east / ((rn + altitude) * std::cos(phi)) * 180.0 / kPi,
    altitude + up};
}

inline std::array<double, 4> perturb_orientation(
  const std::array<double, 4> & q, double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2.0), sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0), sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0), sy = std::sin(yaw / 2.0);
  const std::array<double, 4> d{
    sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
  std::array<double, 4> result{
    q[3] * d[0] + q[0] * d[3] + q[1] * d[2] - q[2] * d[1],
    q[3] * d[1] - q[0] * d[2] + q[1] * d[3] + q[2] * d[0],
    q[3] * d[2] + q[0] * d[1] - q[1] * d[0] + q[2] * d[3],
    q[3] * d[3] - q[0] * d[0] - q[1] * d[1] - q[2] * d[2]};
  double norm = 0.0;
  for (double value : result) {norm += value * value;}
  if (!std::isfinite(norm) || norm < 1e-12) {
    throw std::invalid_argument("Invalid raw IMU quaternion");
  }
  for (double & value : result) {value /= std::sqrt(norm);}
  return result;
}

template<std::size_t N>
inline std::array<double, N * N> diagonal(const std::array<double, N> & sigma)
{
  std::array<double, N * N> result{};
  for (std::size_t i = 0; i < N; ++i) {result[i * N + i] = sigma[i] * sigma[i];}
  return result;
}
}  // namespace warrior_mti680g_sim
#endif  // WARRIOR_MTI680G_SIM__MODEL_HPP_

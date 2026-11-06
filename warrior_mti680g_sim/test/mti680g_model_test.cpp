// Copyright 2026 Wayne Robotics. Licensed under the Apache License, Version 2.0.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "warrior_mti680g_sim/model.hpp"

void close(double actual, double expected, double tolerance)
{
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error("Model value outside expected tolerance");
  }
}

int main()
{
  using namespace warrior_mti680g_sim;
  close(sample_sigma(radians(0.007), 100), 0.0008638939, 1e-9);
  close(sample_sigma(60e-6 * kGravity, 100), 0.004160609, 1e-8);
  close(horizontal_sigma(0.01, 1000, 1.0), 0.0093425398, 1e-9);
  const auto lla = offset_lla(0, 0, 0, 1, 1, 2);
  close(lla[0], 1.0 / 110574.2758, 1e-12);
  close(lla[1], 1.0 / 111319.4908, 1e-12);
  close(lla[2], 2, 1e-12);
  const auto q = perturb_orientation({0, 0, std::sqrt(0.5), std::sqrt(0.5)}, 0, 0, kPi / 2);
  close(q[2], 1, 1e-12);
  close(q[3], 0, 1e-12);
  const auto cov = diagonal<3>({0.2, 0.5, 0.05});
  close(cov[0], 0.04, 1e-12);
  close(cov[4], 0.25, 1e-12);
  close(cov[8], 0.0025, 1e-12);
  close(cov[1], 0, 1e-12);
  bool rejected = false;
  try {perturb_orientation({0, 0, 0, 0}, 0, 0, 0);} catch (const std::invalid_argument &) {
    rejected = true;
  }
  if (!rejected) {throw std::runtime_error("Invalid quaternion was accepted");}
  std::cout << "MTi-680G units, CEP, ENU offsets, orientation and covariance checks passed\n";
}

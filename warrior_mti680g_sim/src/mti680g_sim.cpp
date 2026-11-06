// Copyright 2026 Wayne Robotics. Licensed under the Apache License, Version 2.0.
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>

#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/navsat.pb.h>
#include <ignition/transport/Node.hh>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "warrior_mti680g_sim/model.hpp"

namespace warrior_mti680g_sim
{
class Mti680gSim : public rclcpp::Node
{
public:
  Mti680gSim() : Node("mti680g_sim")
  {
    const double rate = nonnegative("imu_rate_hz", 100.0);
    if (rate <= 0.0) {throw std::invalid_argument("imu_rate_hz must be positive");}
    roll_pitch_sigma_ = radians(nonnegative("roll_pitch_stddev_deg", 0.2));
    yaw_sigma_ = radians(nonnegative("yaw_stddev_deg", 0.5));
    gyro_sigma_ = sample_sigma(radians(nonnegative("gyro_noise_density_deg_s", 0.007)), rate);
    accel_sigma_ = sample_sigma(
      nonnegative("accel_noise_density_micro_g", 60.0) * 1e-6 * kGravity, rate);
    position_sigma_ = horizontal_sigma(
      nonnegative("rtk_position_cep_m", 0.01), nonnegative("rtk_baseline_m", 1000.0),
      nonnegative("rtk_baseline_error_ppm", 1.0));
    vertical_sigma_ = nonnegative("vertical_position_stddev_m", 0.02);
    velocity_sigma_ = nonnegative("velocity_stddev_m_s", 0.05);
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_frame");
    antenna_frame_ = declare_parameter<std::string>("antenna_frame", "gnss_antenna_frame");
    velocity_frame_ = declare_parameter<std::string>("velocity_frame", "gnss_enu");
    const auto seed = declare_parameter<int>("noise_seed", 680);
    imu_rng_.seed(seed);
    gnss_rng_.seed(seed + 1);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gnss/fix", rclcpp::SensorDataQoS());
    velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/gnss/velocity", rclcpp::SensorDataQoS());
    if (!transport_.Subscribe("/mti680g/imu", &Mti680gSim::on_imu, this) ||
      !transport_.Subscribe("/mti680g/navsat", &Mti680gSim::on_navsat, this))
    {
      throw std::runtime_error("Could not subscribe to Gazebo MTi-680G sensor topics");
    }
    RCLCPP_INFO(get_logger(), "MTi-680G nominal RTK model: IMU + GNSS antenna ENU velocity");
  }

  ~Mti680gSim() override
  {
    transport_.Unsubscribe("/mti680g/imu");
    transport_.Unsubscribe("/mti680g/navsat");
  }

private:
  double nonnegative(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(name + " must be finite and nonnegative");
    }
    return value;
  }

  void on_imu(const ignition::msgs::IMU & raw)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    sensor_msgs::msg::Imu msg;
    msg.header.stamp.sec = raw.header().stamp().sec();
    msg.header.stamp.nanosec = raw.header().stamp().nsec();
    msg.header.frame_id = imu_frame_;
    std::array<double, 4> q;
    try {
      q = perturb_orientation(
        {raw.orientation().x(), raw.orientation().y(), raw.orientation().z(), raw.orientation().w()},
        imu_normal_(imu_rng_) * roll_pitch_sigma_, imu_normal_(imu_rng_) * roll_pitch_sigma_,
        imu_normal_(imu_rng_) * yaw_sigma_);
    } catch (const std::invalid_argument &) {return;}
    msg.orientation.x = q[0]; msg.orientation.y = q[1];
    msg.orientation.z = q[2]; msg.orientation.w = q[3];
    msg.angular_velocity.x = raw.angular_velocity().x() + imu_normal_(imu_rng_) * gyro_sigma_;
    msg.angular_velocity.y = raw.angular_velocity().y() + imu_normal_(imu_rng_) * gyro_sigma_;
    msg.angular_velocity.z = raw.angular_velocity().z() + imu_normal_(imu_rng_) * gyro_sigma_;
    msg.linear_acceleration.x = raw.linear_acceleration().x() + imu_normal_(imu_rng_) * accel_sigma_;
    msg.linear_acceleration.y = raw.linear_acceleration().y() + imu_normal_(imu_rng_) * accel_sigma_;
    msg.linear_acceleration.z = raw.linear_acceleration().z() + imu_normal_(imu_rng_) * accel_sigma_;
    msg.orientation_covariance = diagonal<3>({roll_pitch_sigma_, roll_pitch_sigma_, yaw_sigma_});
    msg.angular_velocity_covariance = diagonal<3>({gyro_sigma_, gyro_sigma_, gyro_sigma_});
    msg.linear_acceleration_covariance = diagonal<3>({accel_sigma_, accel_sigma_, accel_sigma_});
    imu_publisher_->publish(msg);
  }

  void on_navsat(const ignition::msgs::NavSat & raw)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(raw.latitude_deg()) || !std::isfinite(raw.longitude_deg()) ||
      !std::isfinite(raw.altitude()) || !std::isfinite(raw.velocity_east()) ||
      !std::isfinite(raw.velocity_north()) || !std::isfinite(raw.velocity_up()) ||
      std::abs(raw.latitude_deg()) >= 89.9) {return;}
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp.sec = raw.header().stamp().sec();
    fix.header.stamp.nanosec = raw.header().stamp().nsec();
    fix.header.frame_id = antenna_frame_;
    const auto lla = offset_lla(
      raw.latitude_deg(), raw.longitude_deg(), raw.altitude(),
      gnss_normal_(gnss_rng_) * position_sigma_, gnss_normal_(gnss_rng_) * position_sigma_,
      gnss_normal_(gnss_rng_) * vertical_sigma_);
    fix.latitude = lla[0]; fix.longitude = lla[1]; fix.altitude = lla[2];
    // NavSatStatus has no RTK-specific status; GBAS is not an RTK flag.
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
    fix.position_covariance = diagonal<3>({position_sigma_, position_sigma_, vertical_sigma_});
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_publisher_->publish(fix);

    geometry_msgs::msg::TwistWithCovarianceStamped velocity;
    velocity.header.stamp = fix.header.stamp;
    velocity.header.frame_id = velocity_frame_;
    // Native NavSat velocities are already ENU, at the antenna, in m/s.
    // Do not derive velocity from noisy positions or use /odom_gt.
    velocity.twist.twist.linear.x = raw.velocity_east() + gnss_normal_(gnss_rng_) * velocity_sigma_;
    velocity.twist.twist.linear.y = raw.velocity_north() + gnss_normal_(gnss_rng_) * velocity_sigma_;
    velocity.twist.twist.linear.z = raw.velocity_up() + gnss_normal_(gnss_rng_) * velocity_sigma_;
    velocity.twist.covariance = diagonal<6>(
      {velocity_sigma_, velocity_sigma_, velocity_sigma_, 1e3, 1e3, 1e3});
    velocity_publisher_->publish(velocity);
  }

  double roll_pitch_sigma_, yaw_sigma_, gyro_sigma_, accel_sigma_;
  double position_sigma_, vertical_sigma_, velocity_sigma_;
  std::string imu_frame_, antenna_frame_, velocity_frame_;
  std::mt19937 imu_rng_, gnss_rng_;
  std::normal_distribution<double> imu_normal_{0.0, 1.0}, gnss_normal_{0.0, 1.0};
  std::mutex mutex_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_publisher_;
  ignition::transport::Node transport_;
};
}  // namespace warrior_mti680g_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<warrior_mti680g_sim::Mti680gSim>());
  rclcpp::shutdown();
  return 0;
}

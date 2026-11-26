// Copyright 2026 Wayne Robotics. Apache-2.0.
//
// LaneLayer: white lane paint as a high but passable cost.
//
// Inputs are the two flat clouds from warrior_lane_detection, both stamped
// with the camera image time:
//   mark_topic  - lane paint cells            (marking)
//   clear_topic - all ground the camera sees  (clearing, includes the paint)
// Clouds of the same stamp are applied together, clear first, then mark, so
// the paint in view is never briefly erased. Ground the camera cannot see
// (its blind zone right in front of the robot) is never cleared, so lines
// remain remembered while the robot drives over them.
//
// Each marked cell is painted with lane_cost (< 253, so the planner and the
// controller's collision check may still cross it) decaying linearly to 0 at
// falloff_radius. Lane boundaries therefore repel the planner, but a line
// that must be crossed, such as the IGVC start, finish and speed-check
// lines, does not block the path the way a lethal ObstacleLayer mark does.
#pragma once

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "warrior_costmap_plugins/lane_grid.hpp"

namespace warrior_costmap_plugins
{

/// Apply @p transform to every (x, y, z) float point of @p cloud and keep
/// x, y. Safe on empty clouds (published whenever no paint is in view).
void transformCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const tf2::Transform & transform,
  std::vector<LanePoint> & out);

class LaneLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  LaneLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void matchSize() override;
  bool isClearable() override {return true;}

private:
  using Cloud = sensor_msgs::msg::PointCloud2;

  struct Pending
  {
    Cloud::ConstSharedPtr mark;
    Cloud::ConstSharedPtr clear;
    rclcpp::Time received;
  };

  void store(const Cloud::ConstSharedPtr & msg, bool is_mark);
  bool toGlobal(const Cloud & cloud, std::vector<LanePoint> & out);

  rclcpp::Subscription<Cloud>::SharedPtr mark_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr clear_sub_;
  std::mutex mutex_;
  std::map<int64_t, Pending> pending_;  // keyed by header stamp (ns)
  LaneGrid grid_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("LaneLayer")};
  std::string global_frame_;
  bool rolling_window_{false};
  bool reset_pending_{false};
  double clear_half_{0.141};
  double pair_timeout_{0.5};
  double max_age_{2.0};
};

}  // namespace warrior_costmap_plugins

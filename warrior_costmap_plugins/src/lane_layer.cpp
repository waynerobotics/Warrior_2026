// Copyright 2026 Wayne Robotics. Apache-2.0.
#include "warrior_costmap_plugins/lane_layer.hpp"

#include <algorithm>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace warrior_costmap_plugins
{

void LaneLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("LaneLayer: failed to lock node");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("mark_topic", rclcpp::ParameterValue(std::string("/perception/lane_points")));
  declareParameter(
    "clear_topic", rclcpp::ParameterValue(std::string("/perception/lane_free_points")));
  declareParameter("lane_cost", rclcpp::ParameterValue(250));
  declareParameter("falloff_radius", rclcpp::ParameterValue(0.9));
  declareParameter("lethal", rclcpp::ParameterValue(false));
  declareParameter("keepout_radius", rclcpp::ParameterValue(0.0));
  declareParameter("clear_resolution", rclcpp::ParameterValue(0.2));
  declareParameter("pair_timeout", rclcpp::ParameterValue(0.5));
  declareParameter("max_age", rclcpp::ParameterValue(2.0));

  std::string mark_topic, clear_topic;
  int lane_cost = 250;
  double falloff = 0.9, clear_resolution = 0.2, keepout = 0.0;
  bool lethal = false;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".mark_topic", mark_topic);
  node->get_parameter(name_ + ".clear_topic", clear_topic);
  node->get_parameter(name_ + ".lane_cost", lane_cost);
  node->get_parameter(name_ + ".falloff_radius", falloff);
  node->get_parameter(name_ + ".lethal", lethal);
  node->get_parameter(name_ + ".keepout_radius", keepout);
  node->get_parameter(name_ + ".clear_resolution", clear_resolution);
  node->get_parameter(name_ + ".pair_timeout", pair_timeout_);
  node->get_parameter(name_ + ".max_age", max_age_);

  if (lane_cost < 1 || lane_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    throw std::invalid_argument("LaneLayer: lane_cost must be in [1, 252]");
  }
  // Free points are cell centres of a clear_resolution grid in the robot
  // frame. Clear only the cell's inscribed disc: it stays inside the cell at
  // any yaw, so a free cell never erases a line in the next cell.
  clear_half_ = clear_resolution / 2.0;

  clock_ = node->get_clock();
  logger_ = node->get_logger();
  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  default_value_ = nav2_costmap_2d::FREE_SPACE;
  matchSize();
  grid_.configure(
    static_cast<unsigned char>(lane_cost), falloff,
    layered_costmap_->getCostmap()->getResolution(), lethal, keepout);

  mark_sub_ = node->create_subscription<Cloud>(
    mark_topic, rclcpp::SensorDataQoS(),
    [this](Cloud::ConstSharedPtr msg) {store(msg, true);});
  clear_sub_ = node->create_subscription<Cloud>(
    clear_topic, rclcpp::SensorDataQoS(),
    [this](Cloud::ConstSharedPtr msg) {store(msg, false);});
  current_ = true;
  if (lethal) {
    RCLCPP_INFO(
      logger_, "%s: lane paint LETHAL, keep-out %.2f m, then %d -> 0 over %.2f m "
      "(mark %s, clear %s)", name_.c_str(), keepout, lane_cost, falloff,
      mark_topic.c_str(), clear_topic.c_str());
  } else {
    RCLCPP_INFO(
      logger_, "%s: lane paint cost %d -> 0 over %.2f m (mark %s, clear %s)",
      name_.c_str(), lane_cost, falloff, mark_topic.c_str(), clear_topic.c_str());
  }
}

void LaneLayer::matchSize()
{
  CostmapLayer::matchSize();
}

void LaneLayer::reset()
{
  resetMaps();
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear();
  current_ = true;
}

void LaneLayer::store(const Cloud::ConstSharedPtr & msg, bool is_mark)
{
  const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);
  auto & entry = pending_[stamp];
  if (!entry.mark && !entry.clear) {
    entry.received = clock_->now();
  }
  (is_mark ? entry.mark : entry.clear) = msg;
}

void transformCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const tf2::Transform & transform,
  std::vector<LanePoint> & out)
{
  out.clear();
  // An empty cloud (no paint in view) is normal. PointCloud2ConstIterator
  // takes &data.front() and aborts on an empty buffer with STL assertions on.
  if (cloud.data.empty() || cloud.width * cloud.height == 0) {
    return;
  }
  out.reserve(cloud.width * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    const tf2::Vector3 p = transform * tf2::Vector3(*x, *y, *z);
    out.push_back({p.x(), p.y()});
  }
}

bool LaneLayer::toGlobal(const Cloud & cloud, std::vector<LanePoint> & out)
{
  out.clear();
  if (cloud.data.empty() || cloud.width * cloud.height == 0) {
    return true;
  }
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_->lookupTransform(
      global_frame_, cloud.header.frame_id, tf2_ros::fromMsg(cloud.header.stamp));
  } catch (const tf2::TransformException &) {
    return false;
  }
  tf2::Transform transform;
  tf2::fromMsg(t.transform, transform);
  transformCloud(cloud, transform, out);
  return true;
}

void LaneLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }

  // Take every stamp whose mark and clear clouds have both arrived, or whose
  // partner is overdue (then apply what we have).
  std::vector<Pending> ready;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time now = clock_->now();
    for (auto it = pending_.begin(); it != pending_.end(); ) {
      const double age = (now - it->second.received).seconds();
      if ((it->second.mark && it->second.clear) || age > pair_timeout_ || age < 0.0) {
        ready.push_back(std::move(it->second));
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::vector<LanePoint> points;
  for (auto & p : ready) {
    // A transform that is not available yet is retried on the next cycle.
    const Cloud & ref = p.clear ? *p.clear : *p.mark;
    if (!tf_->canTransform(global_frame_, ref.header.frame_id,
        tf2_ros::fromMsg(ref.header.stamp)))
    {
      const double age = (clock_->now() - p.received).seconds();
      if (age >= 0.0 && age < max_age_) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_[rclcpp::Time(ref.header.stamp).nanoseconds()] = std::move(p);
      } else {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000, "%s: dropping lane clouds, no TF %s <- %s",
          name_.c_str(), global_frame_.c_str(), ref.header.frame_id.c_str());
      }
      continue;
    }
    if (p.clear && toGlobal(*p.clear, points)) {
      grid_.clear(*this, points, clear_half_);
    }
    if (p.mark && toGlobal(*p.mark, points)) {
      grid_.mark(*this, points);
    }
  }

  double x0, y0, x1, y1;
  if (grid_.takeTouched(x0, y0, x1, y1)) {
    *min_x = std::min(*min_x, x0);
    *min_y = std::min(*min_y, y0);
    *max_x = std::max(*max_x, x1);
    *max_y = std::max(*max_y, y1);
  }
  current_ = true;
}

void LaneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  grid_.paint(*this, master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace warrior_costmap_plugins

PLUGINLIB_EXPORT_CLASS(warrior_costmap_plugins::LaneLayer, nav2_costmap_2d::Layer)

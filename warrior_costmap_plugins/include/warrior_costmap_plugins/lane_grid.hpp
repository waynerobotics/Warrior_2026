// Copyright 2026 Wayne Robotics. Apache-2.0.
//
// ROS-independent core of LaneLayer: a boolean "lane paint" grid that lives
// inside a Costmap2D (so it rolls with the costmap) plus the cost profile
// painted into the master grid around each marked cell: high but passable by
// default, or a hard boundary (lethal paint + inscribed keep-out band).
#pragma once

#include <cstdint>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace warrior_costmap_plugins
{

struct LanePoint
{
  double x;
  double y;
};

class LaneGrid
{
public:
  /// Value stored in the layer's own grid for a lane cell (internal only).
  static constexpr unsigned char MARK = 1;

  /// @param peak_cost      soft cost; must stay below 253
  ///                       (INSCRIBED_INFLATED_OBSTACLE). Passable mode: cost
  ///                       on the paint and out to keepout_radius. Lethal
  ///                       mode: cost at the edge of the keep-out band.
  /// @param falloff_radius the soft cost decays linearly from peak_cost to 0
  ///                       over this distance (m) past keepout_radius.
  /// @param resolution     costmap resolution (m/cell).
  /// @param lethal         paint is LETHAL_OBSTACLE and every cell within
  ///                       @p keepout_radius is INSCRIBED_INFLATED_OBSTACLE:
  ///                       the planner never puts the robot centre there and
  ///                       the controller's footprint check never lets the
  ///                       body touch the paint.
  /// @param keepout_radius the robot's half-width (m). Lethal mode: the
  ///                       inscribed band. Passable mode: a plateau of
  ///                       peak_cost, so the planner keeps the whole body off
  ///                       the paint when it can.
  void configure(
    unsigned char peak_cost, double falloff_radius, double resolution,
    bool lethal = false, double keepout_radius = 0.0);

  /// Erase marks whose cell centre lies within @p half of a point (a disc,
  /// so it stays inside the free cell at any yaw). Returns true if anything
  /// changed; grows the touched box.
  bool clear(nav2_costmap_2d::Costmap2D & grid, const std::vector<LanePoint> & points, double half);

  /// Mark the cell containing each point. Returns true if anything changed.
  bool mark(nav2_costmap_2d::Costmap2D & grid, const std::vector<LanePoint> & points);

  /// Paint the cost profile of every mark (searched within the window grown
  /// by the falloff radius) into @p master, restricted to [min_i,max_i) x
  /// [min_j,max_j). Combines with max; NO_INFORMATION cells are overwritten
  /// so lanes seen before the lidar are not hidden as unknown.
  void paint(
    const nav2_costmap_2d::Costmap2D & grid, nav2_costmap_2d::Costmap2D & master,
    int min_i, int min_j, int max_i, int max_j) const;

  /// Cost at @p distance metres from paint.
  unsigned char costAt(double distance) const;

  /// World-frame box touched since the last takeTouched(), padded by the
  /// reach of the cost profile. Returns false if nothing was touched.
  bool takeTouched(double & min_x, double & min_y, double & max_x, double & max_y);

  /// Distance from paint beyond which the profile is 0 (m).
  double reach() const {return keepout_radius_ + falloff_radius_;}

private:
  void touch(double x, double y);

  struct Offset
  {
    int di;
    int dj;
    unsigned char cost;
  };

  unsigned char peak_cost_{250};
  double falloff_radius_{0.9};
  bool lethal_{false};
  double keepout_radius_{0.0};
  int radius_cells_{0};
  std::vector<Offset> kernel_;
  bool touched_{false};
  double tmin_x_{0}, tmin_y_{0}, tmax_x_{0}, tmax_y_{0};
};

}  // namespace warrior_costmap_plugins

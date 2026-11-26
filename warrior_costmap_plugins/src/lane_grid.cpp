// Copyright 2026 Wayne Robotics. Apache-2.0.
#include "warrior_costmap_plugins/lane_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"

namespace warrior_costmap_plugins
{

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

void LaneGrid::configure(
  unsigned char peak_cost, double falloff_radius, double resolution,
  bool lethal, double keepout_radius)
{
  if (peak_cost >= INSCRIBED_INFLATED_OBSTACLE) {
    throw std::invalid_argument("lane_cost must be < 253");
  }
  if (resolution <= 0.0 || falloff_radius < 0.0 || keepout_radius < 0.0) {
    throw std::invalid_argument(
            "resolution must be > 0, falloff_radius and keepout_radius >= 0");
  }
  peak_cost_ = peak_cost;
  falloff_radius_ = falloff_radius;
  lethal_ = lethal;
  keepout_radius_ = keepout_radius;
  radius_cells_ = static_cast<int>(std::ceil(reach() / resolution));
  kernel_.clear();
  for (int dj = -radius_cells_; dj <= radius_cells_; ++dj) {
    for (int di = -radius_cells_; di <= radius_cells_; ++di) {
      const unsigned char cost = costAt(std::hypot(di, dj) * resolution);
      if (cost > 0) {
        kernel_.push_back({di, dj, cost});
      }
    }
  }
}

unsigned char LaneGrid::costAt(double distance) const
{
  if (lethal_) {
    if (distance <= 0.0) {
      return LETHAL_OBSTACLE;
    }
    if (distance <= keepout_radius_) {
      return INSCRIBED_INFLATED_OBSTACLE;
    }
  } else if (distance <= keepout_radius_) {
    return peak_cost_;
  }
  distance -= keepout_radius_;
  if (falloff_radius_ <= 0.0 || distance >= falloff_radius_) {
    return 0;
  }
  return static_cast<unsigned char>(std::lround(peak_cost_ * (1.0 - distance / falloff_radius_)));
}

void LaneGrid::touch(double x, double y)
{
  if (!touched_) {
    tmin_x_ = tmax_x_ = x;
    tmin_y_ = tmax_y_ = y;
    touched_ = true;
    return;
  }
  tmin_x_ = std::min(tmin_x_, x);
  tmin_y_ = std::min(tmin_y_, y);
  tmax_x_ = std::max(tmax_x_, x);
  tmax_y_ = std::max(tmax_y_, y);
}

bool LaneGrid::takeTouched(double & min_x, double & min_y, double & max_x, double & max_y)
{
  if (!touched_) {
    return false;
  }
  min_x = tmin_x_ - reach();
  min_y = tmin_y_ - reach();
  max_x = tmax_x_ + reach();
  max_y = tmax_y_ + reach();
  touched_ = false;
  return true;
}

bool LaneGrid::clear(
  nav2_costmap_2d::Costmap2D & grid, const std::vector<LanePoint> & points, double half)
{
  bool changed = false;
  unsigned char * data = grid.getCharMap();
  const int sx = static_cast<int>(grid.getSizeInCellsX());
  const int sy = static_cast<int>(grid.getSizeInCellsY());
  for (const auto & p : points) {
    int i0, j0, i1, j1;
    grid.worldToMapNoBounds(p.x - half, p.y - half, i0, j0);
    grid.worldToMapNoBounds(p.x + half, p.y + half, i1, j1);
    i0 = std::max(i0, 0);
    j0 = std::max(j0, 0);
    i1 = std::min(i1, sx - 1);
    j1 = std::min(j1, sy - 1);
    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        unsigned char & cell = data[grid.getIndex(i, j)];
        if (cell != MARK) {
          continue;
        }
        double wx, wy;
        grid.mapToWorld(i, j, wx, wy);
        // Only inside the free cell's inscribed circle (any yaw): a square
        // of this half-size reached a line in the neighbouring cell.
        if (std::hypot(wx - p.x, wy - p.y) <= half) {
          cell = nav2_costmap_2d::FREE_SPACE;
          changed = true;
          touch(wx, wy);
        }
      }
    }
  }
  return changed;
}

bool LaneGrid::mark(nav2_costmap_2d::Costmap2D & grid, const std::vector<LanePoint> & points)
{
  bool changed = false;
  unsigned char * data = grid.getCharMap();
  for (const auto & p : points) {
    unsigned int mx, my;
    if (!grid.worldToMap(p.x, p.y, mx, my)) {
      continue;
    }
    unsigned char & cell = data[grid.getIndex(mx, my)];
    if (cell != MARK) {
      cell = MARK;
      changed = true;
      touch(p.x, p.y);
    }
  }
  return changed;
}

void LaneGrid::paint(
  const nav2_costmap_2d::Costmap2D & grid, nav2_costmap_2d::Costmap2D & master,
  int min_i, int min_j, int max_i, int max_j) const
{
  const int sx = static_cast<int>(grid.getSizeInCellsX());
  const int sy = static_cast<int>(grid.getSizeInCellsY());
  const unsigned char * data = grid.getCharMap();
  unsigned char * out = master.getCharMap();
  // Marks just outside the window still contribute their falloff into it.
  const int si0 = std::max(0, min_i - radius_cells_);
  const int sj0 = std::max(0, min_j - radius_cells_);
  const int si1 = std::min(sx, max_i + radius_cells_);
  const int sj1 = std::min(sy, max_j + radius_cells_);
  for (int j = sj0; j < sj1; ++j) {
    for (int i = si0; i < si1; ++i) {
      if (data[grid.getIndex(i, j)] != MARK) {
        continue;
      }
      for (const auto & k : kernel_) {
        const int ti = i + k.di;
        const int tj = j + k.dj;
        if (ti < min_i || ti >= max_i || tj < min_j || tj >= max_j) {
          continue;
        }
        unsigned char & cell = out[master.getIndex(ti, tj)];
        if (cell == NO_INFORMATION || cell < k.cost) {
          cell = k.cost;
        }
      }
    }
  }
}

}  // namespace warrior_costmap_plugins

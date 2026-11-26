// Copyright 2026 Wayne Robotics. Apache-2.0.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "warrior_costmap_plugins/lane_grid.hpp"

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using warrior_costmap_plugins::LaneGrid;
using warrior_costmap_plugins::LanePoint;

namespace
{
constexpr double RES = 0.1;

// 10 m x 10 m grid, origin (0, 0).
Costmap2D grid(unsigned char fill = FREE_SPACE) {return Costmap2D(100, 100, RES, 0.0, 0.0, fill);}

unsigned char at(const Costmap2D & map, double x, double y)
{
  unsigned int mx, my;
  EXPECT_TRUE(map.worldToMap(x, y, mx, my));
  return map.getCost(mx, my);
}

// A lane line along y at x = 5.0 m, sampled every 5 cm like the camera cloud.
std::vector<LanePoint> line_at_x5()
{
  std::vector<LanePoint> pts;
  for (double y = 1.0; y <= 9.0; y += 0.05) {
    pts.push_back({5.05, y});
  }
  return pts;
}
}  // namespace

TEST(LaneGrid, RejectsNonPassablePeak)
{
  LaneGrid g;
  EXPECT_THROW(g.configure(INSCRIBED_INFLATED_OBSTACLE, 0.9, RES), std::invalid_argument);
  EXPECT_NO_THROW(g.configure(252, 0.9, RES));
}

TEST(LaneGrid, CostProfileDecaysLinearlyAndStaysPassable)
{
  LaneGrid g;
  g.configure(250, 0.9, RES);
  EXPECT_EQ(g.costAt(0.0), 250);
  EXPECT_NEAR(g.costAt(0.45), 125, 1);
  EXPECT_EQ(g.costAt(0.9), 0);
  EXPECT_EQ(g.costAt(2.0), 0);
  for (double d = 0.0; d < 1.0; d += 0.01) {
    EXPECT_LT(g.costAt(d), INSCRIBED_INFLATED_OBSTACLE);
  }
}

TEST(LaneGrid, PaintsHighButPassableCostAroundTheLine)
{
  LaneGrid g;
  g.configure(250, 0.9, RES);
  Costmap2D layer = grid(), master = grid();
  ASSERT_TRUE(g.mark(layer, line_at_x5()));
  g.paint(layer, master, 0, 0, 100, 100);
  EXPECT_EQ(at(master, 5.05, 5.0), 250);
  EXPECT_NEAR(at(master, 5.55, 5.0), 250 * (1 - 0.5 / 0.9), 2);
  EXPECT_EQ(at(master, 6.05, 5.0), FREE_SPACE);
  EXPECT_EQ(at(master, 4.05, 5.0), FREE_SPACE);
  for (unsigned int j = 0; j < 100; ++j) {
    for (unsigned int i = 0; i < 100; ++i) {
      ASSERT_LT(master.getCost(i, j), INSCRIBED_INFLATED_OBSTACLE);
    }
  }
}

TEST(LaneGrid, KeepsLethalAndFillsUnknown)
{
  LaneGrid g;
  g.configure(250, 0.9, RES);
  Costmap2D layer = grid(), master = grid(NO_INFORMATION);
  unsigned int mx, my;
  ASSERT_TRUE(master.worldToMap(5.05, 5.0, mx, my));
  master.setCost(mx, my, LETHAL_OBSTACLE);  // a barrel standing on the line
  g.mark(layer, line_at_x5());
  g.paint(layer, master, 0, 0, 100, 100);
  EXPECT_EQ(at(master, 5.05, 5.0), LETHAL_OBSTACLE);
  EXPECT_EQ(at(master, 5.05, 6.0), 250);      // was unknown, now lane
  EXPECT_EQ(at(master, 8.0, 5.0), NO_INFORMATION);  // untouched unknown stays
}

TEST(LaneGrid, ClearsOnlyWhereTheCameraSawGround)
{
  LaneGrid g;
  g.configure(250, 0.9, RES);
  Costmap2D layer = grid();
  g.mark(layer, line_at_x5());
  // Visible ground: y in [5, 9]; the line below y = 5 is in the blind zone.
  std::vector<LanePoint> free_pts;
  for (double y = 5.1; y < 9.0; y += 0.2) {
    for (double x = 4.1; x < 6.0; x += 0.2) {
      free_pts.push_back({x, y});
    }
  }
  ASSERT_TRUE(g.clear(layer, free_pts, 0.2 / std::sqrt(2.0)));
  Costmap2D master = grid();
  g.paint(layer, master, 0, 0, 100, 100);
  EXPECT_EQ(at(master, 5.05, 3.0), 250);         // remembered in the blind zone
  EXPECT_EQ(at(master, 5.05, 7.0), FREE_SPACE);  // seen without paint -> cleared
}

TEST(LaneGrid, FreeCellBesideALineDoesNotEraseIt)
{
  // Regression: an axis-aligned square of half res/sqrt(2) reached the line
  // in the neighbouring cell; once the line was in the blind zone nothing
  // marked it again.
  LaneGrid g;
  g.configure(250, 0.9, RES);
  Costmap2D layer = grid();
  g.mark(layer, line_at_x5());
  std::vector<LanePoint> beside;
  for (double y = 1.1; y < 9.0; y += 0.2) {
    beside.push_back({5.17, y});   // 0.12 m from the line cell centre
    beside.push_back({4.93, y});
  }
  EXPECT_FALSE(g.clear(layer, beside, 0.2 / 2.0));
  Costmap2D master = grid();
  g.paint(layer, master, 0, 0, 100, 100);
  EXPECT_EQ(at(master, 5.05, 5.0), 250);
}

TEST(LaneGrid, MarkOutsideWindowStillShadesIntoIt)
{
  LaneGrid g;
  g.configure(250, 0.9, RES);
  Costmap2D layer = grid(), master = grid();
  g.mark(layer, line_at_x5());
  // Window is x in [5.3, 10): excludes the line cells but within falloff.
  g.paint(layer, master, 53, 0, 100, 100);
  EXPECT_GT(at(master, 5.35, 5.0), 0);
  EXPECT_EQ(at(master, 4.95, 5.0), FREE_SPACE);  // outside window untouched
}

TEST(LaneGrid, TouchedBoxIsPaddedByFalloff)
{
  LaneGrid g;
  g.configure(250, 0.9, RES);
  Costmap2D layer = grid();
  double x0, y0, x1, y1;
  EXPECT_FALSE(g.takeTouched(x0, y0, x1, y1));
  g.mark(layer, {{5.05, 5.05}});
  ASSERT_TRUE(g.takeTouched(x0, y0, x1, y1));
  EXPECT_NEAR(x0, 5.05 - 0.9, 1e-9);
  EXPECT_NEAR(y1, 5.05 + 0.9, 1e-9);
  EXPECT_FALSE(g.takeTouched(x0, y0, x1, y1));
  EXPECT_FALSE(g.mark(layer, {{5.05, 5.05}}));  // already marked: no change
}

// ---- lethal mode: lane lines are a hard boundary ---------------------------
TEST(LaneGrid, LethalProfileHasKeepOutBandThenSoftFalloff)
{
  LaneGrid g;
  g.configure(200, 0.5, RES, true, 0.65);
  EXPECT_EQ(g.costAt(0.0), LETHAL_OBSTACLE);
  EXPECT_EQ(g.costAt(0.05), INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(g.costAt(0.65), INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_LE(g.costAt(0.66), 200);
  EXPECT_NEAR(g.costAt(0.65 + 0.25), 100, 1);
  EXPECT_EQ(g.costAt(1.15), 0);
  EXPECT_DOUBLE_EQ(g.reach(), 1.15);
}

TEST(LaneGrid, LethalLineBlocksPlannerCentreWithinHalfWidth)
{
  LaneGrid g;
  g.configure(200, 0.5, RES, true, 0.65);
  Costmap2D layer = grid();
  Costmap2D master = grid();
  g.mark(layer, line_at_x5());
  g.paint(layer, master, 0, 0, 100, 100);
  EXPECT_EQ(at(master, 5.05, 5.0), LETHAL_OBSTACLE);
  for (double d = 0.1; d <= 0.6; d += 0.1) {   // both sides of the line
    EXPECT_EQ(at(master, 5.05 + d, 5.0), INSCRIBED_INFLATED_OBSTACLE) << d;
    EXPECT_EQ(at(master, 5.05 - d, 5.0), INSCRIBED_INFLATED_OBSTACLE) << d;
  }
  EXPECT_LT(at(master, 5.05 + 0.8, 5.0), INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(at(master, 5.05 + 1.3, 5.0), FREE_SPACE);
}

TEST(LaneGrid, PassableModeKeepOutIsAPassablePlateau)
{
  LaneGrid g;
  g.configure(220, 0.4, RES, false, 0.6);
  EXPECT_EQ(g.costAt(0.0), 220);
  EXPECT_EQ(g.costAt(0.6), 220);
  EXPECT_NEAR(g.costAt(0.8), 110, 1);
  EXPECT_EQ(g.costAt(1.0), 0);
  EXPECT_DOUBLE_EQ(g.reach(), 1.0);
  for (double d = 0.0; d < 1.2; d += 0.01) {
    EXPECT_LT(g.costAt(d), INSCRIBED_INFLATED_OBSTACLE);
  }
}

TEST(LaneGrid, LethalTouchedBoxIsPaddedByTheWholeReach)
{
  LaneGrid g;
  g.configure(200, 0.5, RES, true, 0.65);
  Costmap2D layer = grid();
  g.mark(layer, {{5.05, 5.05}});
  double x0, y0, x1, y1;
  ASSERT_TRUE(g.takeTouched(x0, y0, x1, y1));
  EXPECT_NEAR(x0, 5.05 - 1.15, 1e-9);
  EXPECT_NEAR(y1, 5.05 + 1.15, 1e-9);
}

// ---- cloud handling (LaneLayer input) --------------------------------------
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "warrior_costmap_plugins/lane_layer.hpp"

namespace
{
sensor_msgs::msg::PointCloud2 xyz_cloud(const std::vector<std::array<float, 3>> & pts)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_footprint";
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  if (pts.empty()) {
    return cloud;  // iterators themselves assert on an empty buffer
  }
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto & p : pts) {
    *x = p[0]; *y = p[1]; *z = p[2];
    ++x; ++y; ++z;
  }
  return cloud;
}
}  // namespace

TEST(TransformCloud, EmptyCloudIsSafe)
{
  // Regression: an empty "no paint in view" cloud aborted planner_server and
  // controller_server (&data.front() on an empty vector, STL assertions on).
  std::vector<LanePoint> out{{1.0, 1.0}};
  warrior_costmap_plugins::transformCloud(xyz_cloud({}), tf2::Transform::getIdentity(), out);
  EXPECT_TRUE(out.empty());
  sensor_msgs::msg::PointCloud2 bare;  // no fields at all
  warrior_costmap_plugins::transformCloud(bare, tf2::Transform::getIdentity(), out);
  EXPECT_TRUE(out.empty());
}

TEST(TransformCloud, AppliesRigidTransform)
{
  tf2::Transform t(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI / 2), tf2::Vector3(10, 20, 0));
  std::vector<LanePoint> out;
  warrior_costmap_plugins::transformCloud(xyz_cloud({{1, 0, 0}, {0, 2, 0}}), t, out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0].x, 10.0, 1e-6);
  EXPECT_NEAR(out[0].y, 21.0, 1e-6);
  EXPECT_NEAR(out[1].x, 8.0, 1e-6);
  EXPECT_NEAR(out[1].y, 20.0, 1e-6);
}

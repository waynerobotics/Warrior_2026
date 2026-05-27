#
# This is unneeded and doesn't work
#
# Kept for reference: nav2_simple_commander.costmap_2d does not export a
# `CostmapLayer` Python base class. Implementing a custom costmap layer in
# ROS 2 Humble requires a C++ pluginlib plugin, not a Python subclass.
# This module will raise NameError on import; it is intentionally NOT
# registered as a console_script entry_point in setup.py.
#

import rclpy
from rclpy.node import Node
# from nav2_simple_commander.costmap_2d import CostmapLayer
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2

class PointCloudCostmapLayer(CostmapLayer):  # noqa: F821 - intentionally broken, see header
    def __init__(self):
        super().__init__('point_cloud_layer')

        # Declare parameters
        self.declare_parameter('point_cloud_topic', 'obstacle_pointcloud')
        self.declare_parameter('obstacle_height_min', 0.0)
        self.declare_parameter('obstacle_height_max', 2.0)
        self.declare_parameter('inflation_radius', 0.55)

        # Get parameters
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.obstacle_height_min = self.get_parameter('obstacle_height_min').value
        self.obstacle_height_max = self.get_parameter('obstacle_height_max').value
        self.inflation_radius = self.get_parameter('inflation_radius').value

        # Create subscription
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self.point_cloud_callback,
            10)

        self.points = []

    def point_cloud_callback(self, msg):
        self.points = []

        # Extract points from point cloud
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # Check height constraints
            if z < self.obstacle_height_min or z > self.obstacle_height_max:
                continue

            self.points.append((x, y))

    def update_costs(self, master_costmap):
        # Clear previous data
        self.costmap.fill(0)

        # Process points
        for x, y in self.points:
            # Convert to grid coordinates
            i, j = self.world_to_map(x, y)

            # Check bounds
            if not (0 <= i < self.size_x and 0 <= j < self.size_y):
                continue

            # Mark as obstacle
            self.costmap[i, j] = 254  # LETHAL_OBSTACLE

            # Inflate obstacle
            cells_radius = int(self.inflation_radius / self.resolution)
            for di in range(-cells_radius, cells_radius + 1):
                for dj in range(-cells_radius, cells_radius + 1):
                    ni, nj = i + di, j + dj

                    # Check bounds
                    if not (0 <= ni < self.size_x and 0 <= nj < self.size_y):
                        continue

                    # Calculate distance
                    dist = np.hypot(di, dj) * self.resolution
                    if dist <= self.inflation_radius:
                        cost = 254 - int((dist / self.inflation_radius) * 253)
                        if cost > self.costmap[ni, nj]:
                            self.costmap[ni, nj] = cost

        # Update master costmap
        np.maximum(master_costmap, self.costmap, out=master_costmap)

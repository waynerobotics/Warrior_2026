import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from rclpy.publisher import Publisher
from rclpy.time import Time
import tf2_ros

class MapRobotPoseNode(Node):
    def __init__(self):
        super().__init__('map_robot_pose_node')
        self.get_logger().info('Map Robot Pose Node started')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.robot_map_pose = self.create_publisher(PoseStamped, 'robot_map_pose', 10)                
        self.timer = self.create_timer(0.1, self.publish_robot_map_pose)

    def publish_robot_map_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = t.transform.translation.x
        ps.pose.position.y = t.transform.translation.y
        ps.pose.position.z = t.transform.translation.z
        ps.pose.orientation = t.transform.rotation
        self.robot_map_pose.publish(ps)

def main(args=None):
    rclpy.init(args=args)
    node = MapRobotPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from rclpy.publisher import Publisher
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
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose_msg = PoseStamped()

            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'

            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            
            self.robot_map_pose.publish(pose_msg)
        except Exception as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapRobotPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
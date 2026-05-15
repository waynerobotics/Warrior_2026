import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
import cv2
import numpy as np
from pupil_apriltags import Detector

class AprilTagGPSBridge(Node):
    def __init__(self):
        super().__init__('apriltag_gps_bridge')
        
        # Settings
        self.robot_name = 'shanti'
        self.shanti_tag_id = 0  # Robot tag
        self.master_tag_id = 1 # Reference tag
        self.tag_size = 0.1875  # meters
        
        # Camera settings (from your original code)
        self.camera_params = (514.8914923, 515.01073552, 320.13602236, 240.43150943)
        
        # Setup camera
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('Camera not found! Check connection.')
            return
        
        # Setup AprilTag detector
        self.detector = Detector(families="tag36h11")
        
        # Publisher for robot position
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/apriltag_pose',
            10
        )
        self.gps_publisher = self.create_publisher(
            NavSatFix,
            f'/{self.robot_name}/gps/fix',
            10
        )
        # Subscribe to GPS to display it
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps_reference',
            self.store_gps,
            10
        )
        
        # Subscribe to waypoint distance
        self.waypoint_distance_sub = self.create_subscription(
            Float32,
            f'/{self.robot_name}/distance_to_waypoint',
            self.store_waypoint_distance,
            10
        )
        
        # Subscribe to waypoint status
        self.waypoint_status_sub = self.create_subscription(
            String,
            f'/{self.robot_name}/waypoint_status',
            self.store_waypoint_status,
            10
        )
        
        self.latest_gps = None
        self.waypoint_distance = None
        self.waypoint_status = None
        self.current_coords = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Reference GPS point (master tag location) - can be set manually or from incoming GPS
        self.reference_gps = None
        
        # OPTION 1: Set a default reference GPS (replace with your actual coordinates)
        # Uncomment and set to your master tag location if GPS is not available
        # self.reference_gps = {
        #     'latitude': 42.3601,
        #     'longitude': -71.0589,
        #     'altitude': 10.0
        # }
        # self.get_logger().info(f'Reference GPS set to: {self.reference_gps}')
        
        # Timer to process camera
        self.timer = self.create_timer(0.033, self.process_frame)  # 30 FPS
        
        self.get_logger().info('AprilTag Bridge started! Show tags to camera.')
    
    def store_gps(self, msg):
        """Store latest GPS for display"""
        self.latest_gps = msg
        
        # Automatically set reference GPS if not already set
        if self.reference_gps is None:
            self.reference_gps = msg
            self.get_logger().info(f'Reference GPS auto-set from incoming data: Lat {msg.latitude:.7f}, Lon {msg.longitude:.7f}')
    
    def set_reference_gps(self, latitude, longitude, altitude):
        """
        Manually set the reference GPS point (master tag location).
        
        Parameters:
            latitude: GPS latitude
            longitude: GPS longitude
            altitude: Altitude in meters
        """
        self.reference_gps = {
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude
        }
        self.get_logger().info(f'Reference GPS set to: Lat {latitude:.7f}, Lon {longitude:.7f}, Alt {altitude:.2f}m')
    
    def store_waypoint_distance(self, msg):
        """Store distance to next waypoint"""
        self.waypoint_distance = msg.data
    
    def store_waypoint_status(self, msg):
        """Store waypoint status message"""
        self.waypoint_status = msg.data
    
    def convert_pose_to_gps(self, relative_x, relative_y, relative_z):
        """
        Convert relative apriltag pose to GPS coordinates.
        
        Parameters:
            relative_x: Forward/backward offset in meters
            relative_y: Left/right offset in meters
            relative_z: Up/down offset (altitude) in meters
            
        Returns:
            Tuple of (latitude, longitude, altitude) or None if reference GPS unavailable
        """
        # Use reference_gps if set, otherwise use latest incoming GPS
        ref_gps = self.reference_gps if self.reference_gps else self.latest_gps
        print ("reference gps: ", ref_gps)
        if ref_gps is None:
            return None
        
        # Get reference coordinates
        if isinstance(ref_gps, dict):
            # If reference_gps is a dict
            ref_lat = ref_gps['latitude']
            ref_lon = ref_gps['longitude']
            ref_alt = ref_gps['altitude']
        else:
            # If it's a NavSatFix message
            ref_lat = ref_gps.latitude
            ref_lon = ref_gps.longitude
            ref_alt = ref_gps.altitude
        
        # Conversion factors
        lat_to_meters = 111000  # meters per degree latitude
        lon_to_meters = 111000 * np.cos(np.radians(ref_lat))  # meters per degree longitude
        
        # Convert meters to degrees
        delta_lat = relative_x / lat_to_meters  # relative_x is forward/back (North/South)
        delta_lon = relative_y / lon_to_meters  # relative_y is left/right (East/West)
        
        # Calculate new GPS coordinates
        new_lat = ref_lat + delta_lat
        new_lon = ref_lon + delta_lon
        new_alt = ref_alt + relative_z  # relative_z is already in meters
        
        return new_lat, new_lon, new_alt
    
    def publish_gps_fix(self, relative_x, relative_y, relative_z):
        """
        Create and publish a NavSatFix message from apriltag pose.
        
        Parameters:
            relative_x: Forward/backward offset in meters
            relative_y: Left/right offset in meters
            relative_z: Up/down offset (altitude) in meters
        """
        gps_coords = self.convert_pose_to_gps(relative_x, relative_y, relative_z)
        
        if gps_coords is None:
            return  # No reference GPS available
        
        lat, lon, alt = gps_coords
        
        # Create NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = f'{self.robot_name}/base_link'
        
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        
        # Set covariance (can be adjusted based on AprilTag accuracy)
        gps_msg.position_covariance = [1.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        # Publish the GPS fix
        self.gps_publisher.publish(gps_msg)
        print(f"Published GPS Fix: Lat {lat:.7f}, Lon {lon:.7f}, Alt {alt:.2f} m")
    
    def process_frame(self):
        """Process one camera frame"""
        ret, frame = self.cap.read()
        if not ret:
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect tags
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )
        
        robot_tag = None
        master_tag = None
        
        # Find our tags
        for tag in tags:
            if tag.tag_id == self.shanti_tag_id:
                robot_tag = tag
                # Draw robot tag in cyan
                corners = tag.corners.astype(int)
                cv2.polylines(frame, [corners], True, (255, 255, 0), 2)
                cv2.putText(frame, "Shanti Tag 0", tuple(corners[0]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                # Draw corner dots
                for corner in corners:
                    cv2.circle(frame, tuple(corner), 4, (0, 255, 255), -1)
            
            elif tag.tag_id == self.master_tag_id:
                master_tag = tag
                # Draw master tag in green
                corners = tag.corners.astype(int)
                cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                cv2.putText(frame, "Master Tag 1", tuple(corners[0]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Draw corner dots
                for corner in corners:
                    cv2.circle(frame, tuple(corner), 4, (0, 255, 0), -1)
        
        # Calculate position if both tags found
        if robot_tag and master_tag:
            # Get positions
            robot_pos = robot_tag.pose_t
            master_pos = master_tag.pose_t
            
            # Calculate relative position in meters
            relative_x = robot_pos[0][0] - master_pos[0][0]
            relative_y = robot_pos[1][0] - master_pos[1][0]
            relative_z = robot_pos[2][0] - master_pos[2][0]
            
            # Store current coordinates
            self.current_coords = {
                'x': relative_x,
                'y': relative_y,
                'z': relative_z
            }
            
            # Send position to ROS
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = relative_x
            msg.pose.position.y = relative_y
            msg.pose.position.z = relative_z
            msg.pose.orientation.w = 1.0
            
            self.pose_publisher.publish(msg)
            
            # Publish GPS fix from apriltag pose
            self.publish_gps_fix(relative_x, relative_y, relative_z)
            print ("published GPS fix from apriltag pose: ", self.current_coords)
            
            # Draw line between tags
            robot_center = np.mean(robot_tag.corners, axis=0).astype(int)
            master_center = np.mean(master_tag.corners, axis=0).astype(int)
            cv2.line(frame, tuple(master_center), tuple(robot_center), (0, 0, 255), 2)
            
            # Display COORDINATES instead of distance
            # Convert to feet for display
            x_ft = relative_x * 3.28084
            y_ft = relative_y * 3.28084
            z_ft = relative_z * 3.28084
            
            # Display robot coordinates in a nice format
            cv2.putText(frame, "Shantis Coordinates (from Master):", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.putText(frame, f"X: {x_ft:+.2f} ft ({relative_x:+.3f} m)", (10, 55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.putText(frame, f"Y: {y_ft:+.2f} ft ({relative_y:+.3f} m)", (10, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.putText(frame, f"Z: {z_ft:+.2f} ft ({relative_z:+.3f} m)", (10, 105),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Calculate and show distance as reference
            distance = np.sqrt(relative_x**2 + relative_y**2 + relative_z**2)
            distance_ft = distance * 3.28084
            cv2.putText(frame, f"Distance: {distance_ft:.2f} ft", (10, 135),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # Display GPS coordinates if available
            if self.latest_gps:
                cv2.putText(frame, "GPS Position:", (10, 165),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"Lat: {self.latest_gps.latitude:.7f}", (10, 190),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"Lon: {self.latest_gps.longitude:.7f}", (10, 210),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"Alt: {self.latest_gps.altitude:.2f} m", (10, 230),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Display waypoint information
            y_offset = 260
            cv2.putText(frame, "WAYPOINT NAVIGATION:", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)  # Orange
            
            if self.waypoint_status:
                cv2.putText(frame, self.waypoint_status, (10, y_offset + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)  # Bright green
            
            if self.waypoint_distance is not None:
                distance_color = (0, 255, 0)  # Green
                if self.waypoint_distance < 5:  # Close to waypoint
                    distance_color = (0, 0, 255)  # Red - within tolerance
                
                cv2.putText(frame, f"Distance to Waypoint: {self.waypoint_distance:.2f} m", 
                           (10, y_offset + 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, distance_color, 2)
        
        else:
            cv2.putText(frame, "Looking for tags...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"Shantis tag (ID 0): {'Found' if robot_tag else 'Not found'}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0) if robot_tag else (100, 100, 100), 1)
            cv2.putText(frame, f"Master tag (ID 1): {'Found' if master_tag else 'Not found'}", (10, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if master_tag else (100, 100, 100), 1)
        
        # Add coordinate system reference at bottom
        cv2.putText(frame, "(x=forward/back, y=left/right, z=up/down)", (10, frame.shape[0] - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Show instructions
        cv2.putText(frame, "Press 'q' to quit", (10, frame.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("AprilTag GPS Tracker", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()
    
    def destroy_node(self):
        """Cleanup"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = AprilTagGPSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
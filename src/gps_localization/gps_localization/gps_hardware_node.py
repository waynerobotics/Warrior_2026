import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import pynmea2
import threading
import time
from pupil_apriltags import Detector

class GPSHardwareNode(Node):
    def __init__(self):
        super().__init__('gps_hardware_node')
        
        # Settings
        self.robot_name = 'shanti'
        self.burger_tag_id = 0  # Robot tag
        self.master_tag_id = 1  # Reference tag
        self.tag_size = 0.1875  # meters
        
        # Camera settings (from apriltag bridge)
        self.camera_params = (514.8914923, 515.01073552, 320.13602236, 240.43150943)
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')  # Common for GT-U7
        self.declare_parameter('baudrate', 9600)  # GT-U7 default
        self.declare_parameter('timeout', 1.0)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        
        # GPS data storage
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.fix_quality = 0
        self.num_satellites = 0
        self.hdop = 0.0
        self.last_fix_time = None
        
        # Setup camera
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('Camera not found! Check connection.')
            return
        
        # Setup AprilTag detector
        self.detector = Detector(families="tag36h11")
        
        # Publisher for robot position (PoseStamped for AprilTag coordinates)
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/apriltag_pose',
            10
        )
        
        # Publisher for GPS position
        self.gps_pub = self.create_publisher(
            NavSatFix,
            f'/{self.robot_name}/gps_position',
            10
        )
        
        self.current_coords = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Try to open serial port
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.get_logger().info(f'GPS Hardware Node started on {self.port} at {self.baudrate} baud')
            
            # Start reading thread
            self.running = True
            self.read_thread = threading.Thread(target=self.read_gps_data)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            # Timer to process camera (30 FPS)
            self.timer = self.create_timer(0.033, self.process_frame)
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open GPS port {self.port}: {e}')
            self.get_logger().info('Check connections and try these commands:')
            self.get_logger().info('  ls /dev/tty* | grep -E "USB|ACM"')
            self.get_logger().info('  sudo chmod 666 /dev/ttyUSB0')
            raise e
    
    def read_gps_data(self):
        """Continuously read data from GPS module"""
        buffer = ""
        
        while self.running:
            try:
                # Read data from serial port
                if self.serial_port.in_waiting:
                    num_bytes = self.serial_port.in_waiting
                    if num_bytes > 0:
                        try:
                            data = self.serial_port.read(num_bytes).decode('ascii', errors='ignore')
                            buffer += data
                        except Exception as e:
                            self.get_logger().debug(f'Serial read decode error: {e}')
                    
                    # Process complete lines
                    lines = buffer.split('\n')
                    buffer = lines[-1]  # Keep incomplete line in buffer
                    
                    for line in lines[:-1]:
                        line = line.strip()
                        if line.startswith('$'):
                            self.parse_nmea_sentence(line)
                
                time.sleep(0.01)  # Small delay to prevent CPU overload
                
            except Exception as e:
                self.get_logger().debug(f'GPS read error: {e}')
                continue
    
    def parse_nmea_sentence(self, sentence):
        """Parse NMEA sentences from GPS"""
        try:
            if sentence.startswith('$GPGGA') or sentence.startswith('$GNGGA'):
                # GGA - Global Positioning System Fix Data
                msg = pynmea2.parse(sentence)
                
                if msg.gps_qual and msg.gps_qual > 0:
                    self.current_lat = msg.latitude
                    self.current_lon = msg.longitude
                    self.current_alt = msg.altitude if msg.altitude else 0.0
                    self.fix_quality = msg.gps_qual
                    self.num_satellites = int(msg.num_sats) if msg.num_sats else 0
                    self.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 99.0
                    self.last_fix_time = self.get_clock().now()
                    self.get_logger().debug(
                        f'GPS Fix: {self.current_lat:.7f}, {self.current_lon:.7f}, '
                        f'Alt: {self.current_alt:.1f}m, Sats: {self.num_satellites}'
                    )
                else:
                    self.get_logger().warn("Waiting for GPS fix... Make sure GT-U7 has clear sky view")
            
            elif sentence.startswith('$GPRMC') or sentence.startswith('$GNRMC'):
                # RMC - Recommended Minimum Navigation Information
                msg = pynmea2.parse(sentence)
                
                if msg.status == 'A':  # A=Active (valid), V=Void (invalid)
                    if msg.latitude and msg.longitude:
                        self.current_lat = msg.latitude
                        self.current_lon = msg.longitude
                        self.last_fix_time = self.get_clock().now()
                        
        except pynmea2.ParseError as e:
            self.get_logger().debug(f'NMEA parse error: {e}')
        except Exception as e:
            self.get_logger().debug(f'GPS parsing error: {e}')
    
    def process_frame(self):
        """Process one camera frame for AprilTags and display data"""
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
            if tag.tag_id == self.burger_tag_id:
                robot_tag = tag
                # Draw robot tag in cyan
                corners = tag.corners.astype(int)
                cv2.polylines(frame, [corners], True, (255, 255, 0), 2)
                cv2.putText(frame, "Robot Tag 0", tuple(corners[0]), 
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
            
            # Send AprilTag position to ROS
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = relative_x
            msg.pose.position.y = relative_y
            msg.pose.position.z = relative_z
            msg.pose.orientation.w = 1.0
            
            self.pose_publisher.publish(msg)
            
            # Draw line between tags
            robot_center = np.mean(robot_tag.corners, axis=0).astype(int)
            master_center = np.mean(master_tag.corners, axis=0).astype(int)
            cv2.line(frame, tuple(master_center), tuple(robot_center), (0, 0, 255), 2)
            
            # Display AprilTag COORDINATES
            x_ft = relative_x * 3.28084
            y_ft = relative_y * 3.28084
            z_ft = relative_z * 3.28084
            
            # Display robot coordinates
            cv2.putText(frame, "AprilTag Coordinates (from Master):", (10, 30),
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
        else:
            cv2.putText(frame, "Looking for tags...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"Robot tag (ID 0): {'Found' if robot_tag else 'Not found'}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0) if robot_tag else (100, 100, 100), 1)
            cv2.putText(frame, f"Master tag (ID 1): {'Found' if master_tag else 'Not found'}", (10, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if master_tag else (100, 100, 100), 1)
        
        # Display GPS coordinates if available
        if self.last_fix_time is not None:
            time_since_fix = (self.get_clock().now() - self.last_fix_time).nanoseconds / 1e9
            
            if time_since_fix <= 2.0:
                cv2.putText(frame, "GPS Position:", (10, 165),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"Lat: {self.current_lat:.7f}", (10, 190),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"Lon: {self.current_lon:.7f}", (10, 210),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"Alt: {self.current_alt:.2f} m", (10, 230),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"Sats: {self.num_satellites} | HDOP: {self.hdop:.1f}", (10, 250),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Publish GPS data
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'gps'
                
                gps_msg.latitude = self.current_lat
                gps_msg.longitude = self.current_lon
                gps_msg.altitude = self.current_alt
                
                # Set status based on fix quality
                if self.fix_quality == 0:
                    gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
                elif self.fix_quality == 1:
                    gps_msg.status.status = NavSatStatus.STATUS_FIX
                elif self.fix_quality == 2:
                    gps_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
                else:
                    gps_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                
                gps_msg.status.service = NavSatStatus.SERVICE_GPS
                
                # Set covariance
                if self.num_satellites >= 6 and self.hdop < 2.0:
                    covariance = 0.01
                elif self.num_satellites >= 4 and self.hdop < 5.0:
                    covariance = 0.1
                else:
                    covariance = 1.0
                
                gps_msg.position_covariance = [
                    covariance, 0.0, 0.0,
                    0.0, covariance, 0.0,
                    0.0, 0.0, covariance * 10
                ]
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                
                self.gps_pub.publish(gps_msg)
            else:
                cv2.putText(frame, "GPS data is stale...", (10, 165),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        else:
            cv2.putText(frame, "Waiting for GPS fix...", (10, 165),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
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
        """Cleanup when shutting down"""
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=1.0)
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*60)
    print("GPS HARDWARE NODE - GT-U7 MODULE + APRILTAG")
    print("="*60)
    print("Connecting to GT-U7 GPS module...")
    print("Initializing AprilTag detector...")
    print("Note: GPS needs clear sky view for satellite lock")
    print("="*60 + "\n")
    
    try:
        node = GPSHardwareNode()
        rclpy.spin(node)
    except serial.SerialException:
        print("\nFailed to connect to GPS module!")
        print("Please check:")
        print("1. Is the GT-U7 connected to USB?")
        print("2. Run: ls /dev/tty* | grep -E 'USB|ACM'")
        print("3. You may need: sudo chmod 666 /dev/ttyUSB0")
        return
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
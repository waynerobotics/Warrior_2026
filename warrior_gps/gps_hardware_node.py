import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import serial
from sensor_msgs.msg import NavSatFix
import pynmea2
import threading
import time

class GPSHardwareNode(Node):
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',  # Replace with your actual GPS topic name
            self.gps_callback,
            10
        )

        # =================== CAMERA SETUP ===================
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('Camera not found! Check connection.')
            return


        super().__init__('gps_hardware_node')

        self.frame = np.zeros((300, 500, 3), dtype=np.uint8)
        cv2.namedWindow("AprilTag GPS Tracker")

        
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

        msg = NavSatFix()
        msg.latitude = self.current_lat
        msg.longitude = self.current_lon
        msg.altitude = self.current_alt
        self.frame[:] = (0, 0, 0)

# Write GPS data
        cv2.putText(self.frame, f"Latitude:  {msg.latitude:.6f}", (20, 80),  
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(self.frame, f"Longitude: {msg.longitude:.6f}", (20, 140), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(self.frame, f"Altitude:  {msg.altitude:.2f} m", (20, 200),  
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Display the frame
        cv2.imshow("AprilTag GPS Tracker", self.frame)
        cv2.waitKey(1)

        
        # Publishers
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            10
        )
        
        self.master_gps_pub = self.create_publisher(
            NavSatFix,
            '/master_gps_position',
            10
        )
        
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
            
            # Timer to publish GPS data
            self.timer = self.create_timer(0.1, self.publish_gps)  # 10 Hz
            
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
                        self.get_logger().info(f"[RAW NMEA] {line}")
                        if line.startswith('$'):
                            self.get_logger().info(f"[RAW NMEA] {line}")  # Log all NMEA lines like `cat`
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
                
                self.get_logger().info(f"[GPGGA] Fix Quality: {msg.gps_qual}, Satellites: {msg.num_sats}")
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
    
    def publish_gps(self):
        """Publish GPS data as ROS message"""
        if self.last_fix_time is None:
            # No GPS fix yet
            self.get_logger().warn("Waiting for GPS fix... Make sure GT-U7 has clear sky view")
            return
        
        # Check if data is recent (within 2 seconds)
        time_since_fix = (self.get_clock().now() - self.last_fix_time).nanoseconds / 1e9
        if time_since_fix > 2.0:
            self.get_logger().warn('GPS data is stale, waiting for new fix...')
            return
        
        # Create NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        
        # Set position
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
        
        # Set covariance based on HDOP and number of satellites
        # Better HDOP and more satellites = better accuracy
        if self.num_satellites >= 6 and self.hdop < 2.0:
            # Good fix
            covariance = 0.01  # 1cm accuracy
        elif self.num_satellites >= 4 and self.hdop < 5.0:
            # Moderate fix
            covariance = 0.1  # 10cm accuracy
        else:
            # Poor fix
            covariance = 1.0  # 1m accuracy
        
        gps_msg.position_covariance = [
            covariance, 0.0, 0.0,
            0.0, covariance, 0.0,
            0.0, 0.0, covariance * 10  # Altitude is less accurate
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        # Publish to both topics
        self.gps_pub.publish(gps_msg)
        self.master_gps_pub.publish(gps_msg)
        
        # Log status periodically
        self.get_logger().info(
            f'GPS Active: {self.current_lat:.7f}, {self.current_lon:.7f}, '
            f'Alt: {self.current_alt:.1f}m, Sats: {self.num_satellites}, HDOP: {self.hdop:.1f}'
        )
    
    def destroy_node(self):
        """Cleanup when shutting down"""
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=1.0)
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*60)
    print("GPS HARDWARE NODE - GT-U7 MODULE")
    print("="*60)
    print("Connecting to GT-U7 GPS module...")
    print("Note: GPS needs clear sky view for satellite lock")
    print("First fix may take 30 seconds to 2 minutes")
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
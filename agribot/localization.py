import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Bool, String, Float64
from nav_msgs.msg import Odometry
import math
import time
import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class AGVLocalization(Node):
    """
    Simplified AGV Localization node using ROS2's robot_localization package
    
    This node handles:
    1. Heading calculation from IMU
    2. Heading reset from dashboard
    3. GPS quality monitoring
    4. Publishing combined localization data
    
    The actual fusion of GPS + IMU is done by robot_localization's ekf_node
    """

    def __init__(self):
        super().__init__('agv_localization')
        
        # Internal state
        self.heading_offset = 0.0
        self.last_heading = None
        self.last_gps = None
        self.last_gps_msg = None
        
        # GPS quality parameters
        self.gps_quality = 1.0
        self.gps_quality_threshold = 0.3
        self.nmea_quality = 1.0
        self.last_gps_time = None
        self.last_nmea_update = None
        self.gps_timeout = 2.0
        self.nmea_timeout = 1.0
        self.num_satellites = 0
        self.hdop = 0.0
        self.fix_quality = 0
        
        # Debug variables
        self.imu_message_count = 0
        self.last_debug_time = time.time()
        self.raw_angular_velocity = 0.0
        
        # TF buffer/listener for getting transforms from robot_localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/data', 
            self.imu_callback, 
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            '/gps/fix', 
            self.gps_callback, 
            10
        )
        
        self.reset_sub = self.create_subscription(
            Bool, 
            '/reset_heading', 
            self.reset_callback, 
            10
        )
        
        self.nmea_sub = self.create_subscription(
            String,
            '/gps/nmea',
            self.nmea_callback,
            10
        )
        
        # Subscribe to the robot_localization output
        # This assumes robot_localization is running and publishing to /odometry/filtered
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_odom_callback,
            10
        )

        # Publishers
        self.localization_pub = self.create_publisher(
            Vector3, 
            '/agv_localization', 
            10
        )
        
        self.gps_quality_pub = self.create_publisher(
            Vector3, 
            '/gps_quality', 
            10
        )
        
        self.imu_debug_pub = self.create_publisher(
            Vector3, 
            '/imu_debug', 
            10
        )
        
        # Optional: Republish heading as a separate topic for easy visualization/debugging
        self.heading_pub = self.create_publisher(
            Float64,
            '/agv/heading',
            10
        )
        
        # Create a timer to publish the fused localization data regularly
        # This ensures that we provide updates even if EKF updates are slow
        self.timer = self.create_timer(0.1, self.publish_latest_localization)
        
        # Store the latest values from various inputs
        self.latest_filtered_position = (0.0, 0.0)
        self.latest_filtered_heading = 0.0
        self.latest_imu_heading = 0.0
        self.latest_imu_angular_velocity = 0.0
        
        # Log initialization
        self.get_logger().info('AGV Localization Node initialized. Using robot_localization for GPS+IMU fusion.')
        self.get_logger().info('Make sure robot_localization ekf_node is running with appropriate parameters')

    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180] range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(90, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.degrees(math.asin(sinp))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        
        return roll, pitch, yaw

    def update_gps_quality(self, gps_msg):
        """Update GPS quality based on message quality and time"""
        current_time = time.time()
        
        # Check if this is the first GPS message
        if self.last_gps_time is None:
            self.last_gps_time = current_time
            return
        
        # Calculate time difference
        dt = current_time - self.last_gps_time
        self.last_gps_time = current_time
        
        # Update quality based on time difference
        if dt > self.gps_timeout:
            self.gps_quality = max(0.0, self.gps_quality - 0.1)
        else:
            self.gps_quality = min(1.0, self.gps_quality + 0.05)
            
        # Update quality based on GPS message quality
        if hasattr(gps_msg, 'status') and hasattr(gps_msg, 'position_covariance'):
            # Check GPS status
            if gps_msg.status.status < 0:  # No fix
                self.gps_quality = max(0.0, self.gps_quality - 0.2)
            elif gps_msg.status.status == 0:  # No fix
                self.gps_quality = max(0.0, self.gps_quality - 0.1)
            elif gps_msg.status.status == 2:  # 2D fix
                self.gps_quality = min(1.0, self.gps_quality + 0.1)
            elif gps_msg.status.status == 3:  # 3D fix
                self.gps_quality = min(1.0, self.gps_quality + 0.2)
                
            # Check position covariance
            if hasattr(gps_msg, 'position_covariance'):
                cov = gps_msg.position_covariance[0]  # Use first diagonal element
                if cov > 100:  # High uncertainty
                    self.gps_quality = max(0.0, self.gps_quality - 0.1)
                elif cov < 1:  # Low uncertainty
                    self.gps_quality = min(1.0, self.gps_quality + 0.1)

    def update_nmea_quality(self, nmea_msg):
        """Update GPS quality based on NMEA message"""
        current_time = time.time()
        
        # Check timeout
        if self.last_nmea_update is not None:
            dt = current_time - self.last_nmea_update
            if dt > self.nmea_timeout:
                self.nmea_quality = max(0.0, self.nmea_quality - 0.1)
        
        self.last_nmea_update = current_time
        
        try:
            # Parse NMEA message
            msg_type = nmea_msg.split(',')[0]
            
            if msg_type == '$GPGGA':
                # Parse fix quality and HDOP
                parts = nmea_msg.split(',')
                if len(parts) >= 8:
                    self.fix_quality = int(parts[6])
                    self.hdop = float(parts[8])
                    
                    # Update quality based on fix quality
                    if self.fix_quality == 0:  # No fix
                        self.nmea_quality = max(0.0, self.nmea_quality - 0.2)
                    elif self.fix_quality == 1:  # GPS fix
                        self.nmea_quality = min(1.0, self.nmea_quality + 0.1)
                    elif self.fix_quality == 2:  # DGPS fix
                        self.nmea_quality = min(1.0, self.nmea_quality + 0.2)
                    
                    # Update quality based on HDOP
                    if self.hdop > 5.0:  # Poor accuracy
                        self.nmea_quality = max(0.0, self.nmea_quality - 0.1)
                    elif self.hdop < 1.0:  # Excellent accuracy
                        self.nmea_quality = min(1.0, self.nmea_quality + 0.1)
            
            elif msg_type == '$GPGSA':
                # Parse number of satellites
                parts = nmea_msg.split(',')
                if len(parts) >= 4:
                    self.num_satellites = sum(1 for x in parts[3:15] if x != '')
                    
                    # Update quality based on number of satellites
                    if self.num_satellites < 4:  # Minimum for 3D fix
                        self.nmea_quality = max(0.0, self.nmea_quality - 0.2)
                    elif self.num_satellites < 6:  # Good for 3D fix
                        self.nmea_quality = min(1.0, self.nmea_quality + 0.05)
                    elif self.num_satellites >= 6:  # Excellent for 3D fix
                        self.nmea_quality = min(1.0, self.nmea_quality + 0.1)
            
            # Combine NMEA quality with existing GPS quality
            self.gps_quality = (self.gps_quality + self.nmea_quality) / 2.0
            
        except Exception as e:
            self.get_logger().warn(f"Error parsing NMEA message: {e}")

    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract quaternion
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        # Convert to Euler angles
        _, _, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        
        # Store raw angular velocity for debugging
        self.raw_angular_velocity = msg.angular_velocity.z
        self.latest_imu_angular_velocity = msg.angular_velocity.z
        
        # Log raw IMU data with enhanced verbosity for debugging
        self.imu_message_count += 1
        if self.imu_message_count % 10 == 0:  # Log every 10th message to avoid flooding
            self.get_logger().debug(
                f"IMU Data: quat=[{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}], "
                f"yaw={yaw:.2f}°, angular_vel_z={msg.angular_velocity.z:.4f} rad/s"
            )
        
        # Apply heading offset to get true north heading
        true_north_heading = self.normalize_angle(yaw - self.heading_offset)
        self.last_heading = true_north_heading
        self.latest_imu_heading = true_north_heading
        
        # Publish debug info periodically
        current_time = time.time()
        if current_time - self.last_debug_time > 1.0:
            # Publish debug information
            debug_msg = Vector3()
            debug_msg.x = msg.angular_velocity.z  # Raw angular velocity
            debug_msg.y = 0.0  # We don't use bias in this simplified version
            debug_msg.z = true_north_heading  # Current heading
            self.imu_debug_pub.publish(debug_msg)
            
            # Log debug info
            self.get_logger().info(
                f"IMU Debug: yaw={yaw:.2f}°, true_north={true_north_heading:.2f}°, "
                f"angular_vel_z={msg.angular_velocity.z:.4f} rad/s, msg_count={self.imu_message_count}"
            )
            
            # Reset counters
            self.imu_message_count = 0
            self.last_debug_time = current_time
            
            # Publish current heading as standalone topic for easier debugging
            heading_msg = Float64()
            heading_msg.data = true_north_heading
            self.heading_pub.publish(heading_msg)

    def gps_callback(self, msg):
        """Process GPS data"""
        self.update_gps_quality(msg)
        self.last_gps = (msg.latitude, msg.longitude)
        self.last_gps_msg = msg
        
        # Publish GPS quality
        quality_msg = Vector3()
        quality_msg.x = self.gps_quality
        quality_msg.y = self.gps_quality_threshold
        quality_msg.z = 1.0 if self.gps_quality < self.gps_quality_threshold else 0.0
        self.gps_quality_pub.publish(quality_msg)
        
        # Log GPS fix information
        self.get_logger().debug(
            f"GPS Fix: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, "
            f"quality={self.gps_quality:.2f}"
        )

    def reset_callback(self, msg):
        """Process reset heading command from dashboard"""
        if msg.data and self.last_heading is not None:
            self.heading_offset = self.last_heading
            self.get_logger().info(f"Heading reset. Offset set to: {self.heading_offset:.2f}°")

    def nmea_callback(self, msg):
        """Process NMEA messages for GPS quality"""
        self.update_nmea_quality(msg.data)
        
        # Publish updated quality
        quality_msg = Vector3()
        quality_msg.x = self.gps_quality
        quality_msg.y = self.gps_quality_threshold
        quality_msg.z = 1.0 if self.gps_quality < self.gps_quality_threshold else 0.0
        self.gps_quality_pub.publish(quality_msg)
        
        # Log detailed quality information
        self.get_logger().info(
            f"GPS Quality: {self.gps_quality:.2f}, "
            f"NMEA Quality: {self.nmea_quality:.2f}, "
            f"Fix Quality: {self.fix_quality}, "
            f"HDOP: {self.hdop:.2f}, "
            f"Satellites: {self.num_satellites}"
        )

    def filtered_odom_callback(self, msg):
        """Process filtered odometry from robot_localization"""
        if self.last_gps is None:
            return  # No GPS data yet
            
        # Extract position and orientation from the filtered odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Store latest filtered odom values
        self.latest_filtered_position = (position.x, position.y)
        
        # Extract heading from the filtered orientation
        _, _, filtered_yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        # Apply heading offset and store
        self.latest_filtered_heading = self.normalize_angle(filtered_yaw - self.heading_offset)
        
        # Log filtered data (not too frequently)
        self.get_logger().debug(
            f"Filtered Odom: pos=({position.x:.2f}, {position.y:.2f}), "
            f"heading={self.latest_filtered_heading:.2f}°"
        )
        
        # We'll publish the combined localization data in the timer callback
        # to ensure we have the most up-to-date GPS coordinates

    def publish_latest_localization(self):
        """Publish combined localization data from GPS position and filtered heading"""
        if self.last_gps is None:
            return  # No GPS data yet
        
        # Use latest GPS coordinates for global position
        lat, lon = self.last_gps
        
        # Use the filtered heading from robot_localization, or IMU if not available
        heading = self.latest_filtered_heading
        if heading == 0.0:  # If not getting good data from filter, use IMU directly
            heading = self.latest_imu_heading
        
        # Create and publish localization message
        localization_msg = Vector3()
        localization_msg.x = lat
        localization_msg.y = lon
        localization_msg.z = heading
        self.localization_pub.publish(localization_msg)
        
        # Log published values (not too frequently)
        self.get_logger().info(
            f"Published: lat={lat:.6f}, lon={lon:.6f}, "
            f"heading={heading:.2f}°, GPS quality={self.gps_quality:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    agv_localization_node = AGVLocalization()
    rclpy.spin(agv_localization_node)
    agv_localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
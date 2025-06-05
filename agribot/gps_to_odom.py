import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL, ToLL
import math
import pyproj
from pyproj import Transformer
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Quaternion
import time

class GPStoOdom(Node):
    """
    Node to convert GPS (NavSatFix) messages to Odometry messages
    Uses UTM projection to convert between GPS and local coordinates
    """
    
    def __init__(self):
        super().__init__('gps_to_odom')
        
        # Parameters
        self.declare_parameter('utm_zone', 48)  # Default UTM zone for Vietnam
        self.declare_parameter('northern_hemisphere', True)
        self.declare_parameter('gps_noise_xy', 1.0)  # Default noise for GPS in meters
        
        # Get parameters
        self.utm_zone = self.get_parameter('utm_zone').value
        self.northern_hemisphere = self.get_parameter('northern_hemisphere').value
        self.gps_noise_xy = self.get_parameter('gps_noise_xy').value
        
        # Initialize reference point
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = 0.0
        
        # Create transformer for GPS to UTM conversion
        self.transformer = Transformer.from_crs(
            "EPSG:4326",  # WGS84
            f"+proj=utm +zone={self.utm_zone} +{'north' if self.northern_hemisphere else 'south'} +datum=WGS84 +units=m +no_defs",
            always_xy=True
        )
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            '/gps/odom',
            10
        )
        
        # Variables for quality monitoring
        self.last_fix_status = 0  # 0 = no fix, 1 = fix, 2 = differential fix
        self.covariance_scale = 1.0  # Scale factor for covariance based on GPS quality
        
        self.get_logger().info('GPS to Odometry converter initialized')
        self.get_logger().info(f'Using UTM zone {self.utm_zone} {"North" if self.northern_hemisphere else "South"}')
    
    def set_reference_point(self, lat, lon, alt):
        """Set the reference point for local coordinates"""
        self.ref_lat = lat
        self.ref_lon = lon
        self.ref_alt = alt
        
        # Convert reference to UTM
        utm_x, utm_y = self.transformer.transform(lon, lat)
        
        self.ref_utm_x = utm_x
        self.ref_utm_y = utm_y
        
        self.get_logger().info(f'Reference point set: lat={lat:.6f}, lon={lon:.6f}')
        self.get_logger().info(f'Reference UTM: x={utm_x:.2f}, y={utm_y:.2f}, zone={self.utm_zone}')
    
    def update_covariance_scale(self, msg):
        """Update covariance scaling based on GPS status and quality"""
        # Default high uncertainty
        scale = 10.0
        
        # Check if status field is available (standard NavSatFix)
        if hasattr(msg, 'status') and hasattr(msg.status, 'status'):
            status = msg.status.status
            # -1: No signal, 0: Invalid signal, 1: Unaugmented fix, 2: D/GNSS fix
            if status == -1 or status == 0:  # Poor quality
                scale = 100.0
            elif status == 1:  # Regular fix
                scale = 2.0
            elif status == 2:  # Differential fix
                scale = 1.0
            
            self.last_fix_status = status
        
        # Check position_covariance
        if hasattr(msg, 'position_covariance') and hasattr(msg, 'position_covariance_type'):
            if msg.position_covariance_type > 0:  # If we have covariance information
                # Use the trace (diagonal sum) as an indicator of overall uncertainty
                trace = msg.position_covariance[0] + msg.position_covariance[4] + msg.position_covariance[8]
                
                # Adjust scale based on covariance
                if trace < 1.0:  # Very good precision
                    scale *= 0.5
                elif trace > 100.0:  # Very poor precision
                    scale *= 2.0
                    
        self.covariance_scale = scale
        return scale
    
    def gps_callback(self, msg):
        """Convert GPS message to Odometry message"""
        # Update covariance scaling based on GPS quality
        cov_scale = self.update_covariance_scale(msg)
        
        # Set reference point if not set
        if self.ref_lat is None:
            self.set_reference_point(msg.latitude, msg.longitude, msg.altitude)
            return
        
        # Convert GPS to UTM
        utm_x, utm_y = self.transformer.transform(msg.longitude, msg.latitude)
        
        # Calculate local coordinates relative to reference
        local_x = utm_x - self.ref_utm_x
        local_y = utm_y - self.ref_utm_y
        local_z = msg.altitude - self.ref_alt if msg.altitude != 0.0 else 0.0
        
        # Create odometry message
        odom_msg = Odometry()
        
        # Set proper timestamp - use GPS message time if available, otherwise use current time
        current_time = self.get_clock().now().to_msg()
        if msg.header.stamp.sec != 0:  # If GPS message has a valid timestamp
            odom_msg.header.stamp = msg.header.stamp
        else:
            odom_msg.header.stamp = current_time
            
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = local_x
        odom_msg.pose.pose.position.y = local_y
        odom_msg.pose.pose.position.z = local_z
        
        # Set orientation to identity quaternion (we don't have orientation from GPS)
        # We'll get the actual orientation from IMU
        odom_msg.pose.pose.orientation.w = 1.0
        
        # Set covariance appropriately based on GPS quality
        # Start with a cleared covariance matrix
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        
        # Set position covariance - scale with GPS quality
        base_xy_cov = self.gps_noise_xy * cov_scale
        
        # Set diagonal position covariance (x, y, z)
        pose_cov[0] = base_xy_cov  # x variance
        pose_cov[7] = base_xy_cov  # y variance
        pose_cov[14] = base_xy_cov * 5.0  # z variance (less accurate)
        
        # Set extremely high covariance for orientation (we don't trust GPS for this)
        pose_cov[21] = 99999.0  # roll
        pose_cov[28] = 99999.0  # pitch
        pose_cov[35] = 99999.0  # yaw
        
        # If GPS provides valid covariance, use it for position only
        if (hasattr(msg, 'position_covariance') and 
            hasattr(msg, 'position_covariance_type') and 
            msg.position_covariance_type > 0):
            # Only copy position covariance from GPS
            pose_cov[0] = msg.position_covariance[0]  # x variance
            pose_cov[7] = msg.position_covariance[4]  # y variance
            pose_cov[14] = msg.position_covariance[8]  # z variance
        
        # Set high covariance for twist (no velocity from GPS)
        for i in range(36):
            twist_cov[i] = 99999.0
        
        # Copy covariance matrices to odometry message
        odom_msg.pose.covariance = pose_cov
        odom_msg.twist.covariance = twist_cov
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)
        
        # Log status
        if (self.count_subscribers('gps/odom') > 0) and (self.count_publishers('gps/odom') > 0):
            self.get_logger().debug(
                f"Published GPS as odom: UTM=({utm_x:.2f}, {utm_y:.2f}), "
                f"Local=({local_x:.2f}, {local_y:.2f}), "
                f"Status={self.last_fix_status}, Cov_scale={cov_scale:.1f}"
            )

def main(args=None):
    rclpy.init(args=args)
    gps_to_odom_node = GPStoOdom()
    rclpy.spin(gps_to_odom_node)
    gps_to_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
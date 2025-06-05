import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

class GPSDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver_node')
        self.get_logger().info('GPS Driver Node initialized')  # Chỉ giữ lại thông báo khởi tạo
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.publish_gps_data)

    def publish_gps_data(self):
        line = self.ser.readline().decode('ascii', errors='ignore')
        if line.startswith('$GNGGA'):
            try:
                msg = pynmea2.parse(line)
                gps_msg = NavSatFix()
                gps_msg.latitude = msg.latitude
                gps_msg.longitude = msg.longitude
                gps_msg.altitude = msg.altitude
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(gps_msg)
            except pynmea2.ParseError:
                pass  # Bỏ qua lỗi parse, không cần log

def main(args=None):
    rclpy.init(args=args)
    gps_node = GPSDriverNode()
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
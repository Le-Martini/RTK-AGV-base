import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool
import math
import numpy as np

class MainControl(Node):
    def __init__(self):
        super().__init__('main_control')

        # Subscribers
        self.localization_sub = self.create_subscription(Vector3, '/agv_localization', self.localization_callback, 10)
        self.waypoints_sub = self.create_subscription(Vector3, '/waypoints', self.waypoints_callback, 10)
        self.start_sub = self.create_subscription(Bool, '/start', self.start_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop', self.stop_callback, 10)

        # Publisher cho điều khiển động cơ
        self.motor_pub = self.create_publisher(Vector3, '/motor_control', 10)

        # Các biến trạng thái
        self.current_pos = {'lat': 0.0, 'lon': 0.0, 'heading': 0.0}
        self.waypoints = []  # List các điểm đích
        self.current_target = None  # Điểm đích hiện tại
        self.is_running = False  # Trạng thái chạy/dừng
        self.min_distance = 0.3  # Khoảng cách tối thiểu đến target (m)

        # Timer để điều khiển động cơ (10Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Main Control Node started')

    def localization_callback(self, msg):
        """Cập nhật vị trí hiện tại của AGV"""
        self.current_pos['lat'] = msg.x
        self.current_pos['lon'] = msg.y
        self.current_pos['heading'] = msg.z

    def waypoints_callback(self, msg):
        """Nhận waypoint mới"""
        self.waypoints.append({'lat': msg.x, 'lon': msg.y})
        if not self.current_target and self.waypoints:
            self.current_target = self.waypoints[0]
        self.get_logger().info(f'New waypoint received: lat={msg.x}, lon={msg.y}')

    def start_callback(self, msg):
        """Xử lý tín hiệu start"""
        if msg.data:
            self.is_running = True
            self.get_logger().info('AGV started')

    def stop_callback(self, msg):
        """Xử lý tín hiệu stop"""
        if msg.data:
            self.is_running = False
            self.emergency_stop()
            self.get_logger().warn('Emergency stop triggered')

    def emergency_stop(self):
        """Gửi lệnh dừng khẩn cấp"""
        stop_cmd = Vector3()
        stop_cmd.x = 0.0  # pulse
        stop_cmd.y = 0.0  # speed
        stop_cmd.z = 85.0  # servo angle (trung tâm)
        self.motor_pub.publish(stop_cmd)

    def calculate_distance(self, target):
        """Tính khoảng cách đến điểm đích (theo m)"""
        R = 6371000  # Bán kính Trái Đất (m)
        lat1, lon1 = math.radians(self.current_pos['lat']), math.radians(self.current_pos['lon'])
        lat2, lon2 = math.radians(target['lat']), math.radians(target['lon'])
        
        # Thêm kiểm tra tính hợp lệ của tọa độ
        if abs(lat1) > math.pi/2 or abs(lat2) > math.pi/2:
            self.get_logger().warn('Invalid latitude detected')
            return float('inf')
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Thêm xử lý trường hợp đặc biệt
        if abs(dlat) < 1e-10 and abs(dlon) < 1e-10:
            return 0.0
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        # Thêm logging chi tiết
        self.get_logger().debug(f'Distance calculation: current=({math.degrees(lat1)},{math.degrees(lon1)}), '
                               f'target=({math.degrees(lat2)},{math.degrees(lon2)}), distance={distance:.2f}m')
        
        return distance

    def calculate_target_heading(self, target):
        """Tính góc hướng đến điểm đích (độ)"""
        lat1, lon1 = math.radians(self.current_pos['lat']), math.radians(self.current_pos['lon'])
        lat2, lon2 = math.radians(target['lat']), math.radians(target['lon'])
        
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        heading = math.degrees(math.atan2(y, x))
        
        # Chuyển về khoảng -180 đến 180
        if heading > 180:
            heading -= 360
        
        return heading

    def control_loop(self):
        """Vòng lặp điều khiển chính"""
        if not self.is_running or not self.current_target:
            return

        # Tính khoảng cách đến điểm đích
        distance = self.calculate_distance(self.current_target)
        
        # Kiểm tra xem đã đến đích chưa
        if distance < self.min_distance:
            self.get_logger().info(f'Reached waypoint: lat={self.current_target["lat"]}, lon={self.current_target["lon"]}')
            self.waypoints.pop(0)  # Xóa waypoint đã đến
            if self.waypoints:  # Nếu còn waypoint tiếp theo
                self.current_target = self.waypoints[0]
            else:
                self.is_running = False
                self.emergency_stop()
                return

        # Tính góc cần xoay
        target_heading = self.calculate_target_heading(self.current_target)
        heading_error = target_heading - self.current_pos['heading']
        
        # Chuẩn hóa góc về khoảng -180 đến 180
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        # Tính toán góc lái (servo_angle)
        # Góc servo:
        # - 85 độ: chính giữa
        # - 35-84 độ: rẽ phải
        # - 86-135 độ: rẽ trái
        if heading_error > 0:  # Cần rẽ phải
            # Map heading_error từ [0, 180] sang [35, 84]
            servo_angle = 84 - (heading_error * (84 - 35) / 180)
            servo_angle = min(84, max(35, servo_angle))
        else:  # Cần rẽ trái
            # Map heading_error từ [-180, 0] sang [86, 135]
            servo_angle = 86 + (abs(heading_error) * (135 - 86) / 180)
            servo_angle = min(135, max(86, servo_angle))

        # Tính toán tốc độ dựa trên góc lái
        # Giảm tốc khi góc lái lớn
        base_speed = 80  # Tốc độ cơ bản
        speed_factor = 1.0 - (abs(heading_error) / 180) * 0.3  # Giảm tối đa 30% tốc độ khi góc lớn
        speed = base_speed * speed_factor

        # Số xung cố định
        pulse = 400

        # Log thông tin điều khiển
        self.get_logger().debug(
            f'Control: distance={distance:.2f}m, heading_error={heading_error:.2f}°, '
            f'servo={servo_angle:.1f}°, speed={speed:.1f}'
        )

        # Gửi lệnh điều khiển
        cmd = Vector3()
        cmd.x = float(pulse)
        cmd.y = float(speed)
        cmd.z = float(servo_angle)
        self.motor_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    main_control = MainControl()
    try:
        rclpy.spin(main_control)
    except KeyboardInterrupt:
        pass
    finally:
        main_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
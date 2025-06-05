import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from Adafruit_BNO055 import BNO055
import time

class BNO055IMU(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Tạo đối tượng BNO055
        self.bno = BNO055.BNO055(address=0x28)
        
        # Kiểm tra cảm biến
        if not self.bno.begin():
            self.get_logger().error('Can not initialize BNO055')
            return
        
        self.get_logger().info('IMU initialized')
        
        # Set mode
        self.bno.set_mode(BNO055.OPERATION_MODE_NDOF)
        
        # Khởi tạo publisher IMU
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
        # Tần suất gửi dữ liệu IMU (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        imu_data = Imu()
        
        # Lấy dữ liệu từ cảm biến BNO055
        (qx, qy, qz, qw) = self.bno.read_quaternion()
        (euler_heading, euler_roll, euler_pitch) = self.bno.read_euler()
        (linear_accel_x, linear_accel_y, linear_accel_z) = self.bno.read_linear_acceleration()
        (gyro_x, gyro_y, gyro_z) = self.bno.read_gyroscope()
        
        # Cập nhật dữ liệu vào IMU message
        imu_data.header.stamp = self.get_clock().now().to_msg()
        imu_data.header.frame_id = 'base_link'
        
        # Điền dữ liệu vào các trường của IMU message
        imu_data.orientation.x = qx
        imu_data.orientation.y = qy
        imu_data.orientation.z = qz
        imu_data.orientation.w = qw
        
        imu_data.linear_acceleration.x = linear_accel_x
        imu_data.linear_acceleration.y = linear_accel_y
        imu_data.linear_acceleration.z = linear_accel_z
        
        imu_data.angular_velocity.x = gyro_x
        imu_data.angular_velocity.y = gyro_y
        imu_data.angular_velocity.z = gyro_z
        
        # Set covariance matrices
        # Sử dụng orientation_covariance để lưu euler angles
        imu_data.orientation_covariance[0] = euler_roll
        imu_data.orientation_covariance[4] = euler_pitch
        imu_data.orientation_covariance[8] = euler_heading
        
        imu_data.angular_velocity_covariance = [0.0] * 9
        imu_data.linear_acceleration_covariance = [0.0] * 9

        # Gửi dữ liệu IMU
        self.publisher_.publish(imu_data)

def main(args=None):
    rclpy.init(args=args)
    imu_node = BNO055IMU()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

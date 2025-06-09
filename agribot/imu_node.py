# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from Adafruit_BNO055 import BNO055
# import time

# class BNO055IMU(Node):
#     def __init__(self):
#         super().__init__('imu_node')
        
#         # Tạo đối tượng BNO055
#         self.bno = BNO055.BNO055(address=0x28)
        
#         # Kiểm tra cảm biến
#         if not self.bno.begin():
#             self.get_logger().error('Can not initialize BNO055')
#             return
        
#         self.get_logger().info('IMU initialized')
        
#         # Set mode
#         self.bno.set_mode(BNO055.OPERATION_MODE_NDOF)
        
#         # Khởi tạo publisher IMU
#         self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
#         # Tần suất gửi dữ liệu IMU (100Hz)
#         self.timer = self.create_timer(0.01, self.timer_callback)

#     def timer_callback(self):
#         imu_data = Imu()
        
#         # Lấy dữ liệu từ cảm biến BNO055
#         (qx, qy, qz, qw) = self.bno.read_quaternion()
#         (euler_heading, euler_roll, euler_pitch) = self.bno.read_euler()
#         (linear_accel_x, linear_accel_y, linear_accel_z) = self.bno.read_linear_acceleration()
#         (gyro_x, gyro_y, gyro_z) = self.bno.read_gyroscope()
        
#         # Cập nhật dữ liệu vào IMU message
#         imu_data.header.stamp = self.get_clock().now().to_msg()
#         imu_data.header.frame_id = 'base_link'
        
#         # Điền dữ liệu vào các trường của IMU message
#         imu_data.orientation.x = qx
#         imu_data.orientation.y = qy
#         imu_data.orientation.z = qz
#         imu_data.orientation.w = qw
        
#         imu_data.linear_acceleration.x = linear_accel_x
#         imu_data.linear_acceleration.y = linear_accel_y
#         imu_data.linear_acceleration.z = linear_accel_z
        
#         imu_data.angular_velocity.x = gyro_x
#         imu_data.angular_velocity.y = gyro_y
#         imu_data.angular_velocity.z = gyro_z
        
#         # Set covariance matrices
#         # Sử dụng orientation_covariance để lưu euler angles
#         imu_data.orientation_covariance[0] = euler_roll
#         imu_data.orientation_covariance[4] = euler_pitch
#         imu_data.orientation_covariance[8] = euler_heading
        
#         imu_data.angular_velocity_covariance = [0.0] * 9
#         imu_data.linear_acceleration_covariance = [0.0] * 9

#         # Gửi dữ liệu IMU
#         self.publisher_.publish(imu_data)

# def main(args=None):
#     rclpy.init(args=args)
#     imu_node = BNO055IMU()
#     rclpy.spin(imu_node)
#     imu_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


### Witmotion HWT901B IMU
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
import math
import time

class HWT901BIMU(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Open Serial Port
        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.01)
        time.sleep(2.0)
        
        # Unlock and set output rate 100 Hz
        self._unlock()
        self._set_output_rate(0x09) # 100 Hz
        
        # Publisher IMU
        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Buffer to store data
        self.buffer = bytearray()

        # Save last values
        self.last_q = (1.0, 0.0, 0.0, 0.0)
        self.last_acc = (0.0, 0.0, 0.0)
        self.last_gyro = (0.0, 0.0, 0.0)
        self.last_rpy = (0.0, 0.0, 0.0)

        # Timer to publish data (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('IMU initialized')

    def _unlock(self):
        self.ser.write(bytes.fromhex('FF AA 69 88 B5'))
        time.sleep(0.05)

    def _set_output_rate(self, rate_code):
        # Unlock → RRATE → Save :contentReference[oaicite:2]{index=2}
        self._unlock()
        cmd = bytes.fromhex(f'FF AA 03 {rate_code:02X} 00')
        self.ser.write(cmd)
        time.sleep(0.05)
        self.ser.write(bytes.fromhex('FF AA 00 00 00'))
        time.sleep(0.05)

    def timer_callback(self):
         # Đọc tất cả dữ liệu có sẵn
        data = self.ser.read(256)
        if data:
            self.buffer += data

        # Xử lý từng gói 11 byte
        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                self.buffer.pop(0)
                continue
            packet = self.buffer[:11]
            # Checksum
            if (sum(packet[0:10]) & 0xFF) != packet[10]:
                # tìm header tiếp theo
                idx = self.buffer.find(b'\x55', 1)
                if idx == -1:
                    self.buffer.clear()
                else:
                    del self.buffer[:idx]
                continue

            typ = packet[1]
            # Quaternion packet
            if typ == 0x59:
                q0 = struct.unpack('<h', packet[2:4])[0] / 32768.0
                q1 = struct.unpack('<h', packet[4:6])[0] / 32768.0
                q2 = struct.unpack('<h', packet[6:8])[0] / 32768.0
                q3 = struct.unpack('<h', packet[8:10])[0] / 32768.0
                self.last_q = (q0, q1, q2, q3)
                # Tính roll/pitch/yaw từ quaternion
                self.last_rpy = self.quat_to_euler(q0, q1, q2, q3)
            # Acceleration packet
            elif typ == 0x51:
                ax = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 16 * 9.8
                ay = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 16 * 9.8
                az = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 16 * 9.8
                self.last_acc = (ax, ay, az)
            # Gyroscope packet
            elif typ == 0x52:
                wx = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 2000 * math.pi/180
                wy = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 2000 * math.pi/180
                wz = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 2000 * math.pi/180
                self.last_gyro = (wx, wy, wz)

            # Xóa gói đã xử lý
            del self.buffer[:11]

        # Publish Imu message
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'

        # Orientation quaternion
        imu.orientation.x = self.last_q[1]
        imu.orientation.y = self.last_q[2]
        imu.orientation.z = self.last_q[3]
        imu.orientation.w = self.last_q[0]

        # Acceleration (m/s²)
        imu.linear_acceleration.x = self.last_acc[0]
        imu.linear_acceleration.y = self.last_acc[1]
        imu.linear_acceleration.z = self.last_acc[2]

        # Angular velocity (rad/s)
        imu.angular_velocity.x = self.last_gyro[0]
        imu.angular_velocity.y = self.last_gyro[1]
        imu.angular_velocity.z = self.last_gyro[2]

        # Lưu roll/pitch/yaw vào orientation_covariance
        imu.orientation_covariance[0] = self.last_rpy[0]
        imu.orientation_covariance[4] = self.last_rpy[1]
        imu.orientation_covariance[8] = self.last_rpy[2]

        # Đặt covariance khác thành 0
        imu.angular_velocity_covariance = [0.0] * 9
        imu.linear_acceleration_covariance = [0.0] * 9

        self.pub.publish(imu)

    def quat_to_euler(self, q0, q1, q2, q3):
        """Trả về roll, pitch, yaw (°) từ quaternion"""
        # Roll
        sinr = 2 * (q0*q1 + q2*q3)
        cosr = 1 - 2 * (q1*q1 + q2*q2)
        roll = math.degrees(math.atan2(sinr, cosr))
        # Pitch
        sinp = 2 * (q0*q2 - q3*q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(90.0, sinp)
        else:
            pitch = math.degrees(math.asin(sinp))
        # Yaw
        siny = 2 * (q0*q3 + q1*q2)
        cosy = 1 - 2 * (q2*q2 + q3*q3)
        yaw = math.degrees(math.atan2(siny, cosy))
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = HWT901BIMU()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
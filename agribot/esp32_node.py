import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3
import serial
import time
from enum import Enum

class ESP32State(Enum):
    WAITING_READY = 1
    READY = 2
    EXECUTING = 3
    EMERGENCY_STOP = 4

class ESP32SerialNode(Node):
    def __init__(self):
        super().__init__('esp32_node')

        #ESP32 status
        self.state = ESP32State.WAITING_READY 
        #Buffer for queue command
        self.pending_command = None

        #Initialize Serial
        try:
            self.serial_port = serial.Serial(
                port='/dev/serial0',
                baudrate=115200,
                timeout=1.0
            )
            self.get_logger().info("Serial port connected!")
        except serial.SerialException as e:
            self.get_logger().error(f"Error connection failed: {str(e)}")
            return

        time.sleep(2.0)

        #Send "OK" to initialize ESP32
        self.serial_port.write("Ok\n".encode())

        #Create subcriber for recieving cmd from agv_control_node
        self.control_sub = self.create_subscription(
            Vector3,
            '/motor_control',
            self.control_callback, 
            10
        )

        #Create subscriber for emergency stop
        self.stop_sub = self.create_subscription(
            Bool,
            '/stop',
            self.stop_callback,
            1  # Higher priority (lower queue size)
        )

        #Create publisher for sending esp32 status
        self.status_pub = self.create_publisher(
            String,
            '/esp32_status',
            10
        )

        # Timer for reading feedback from ESP32
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("ESP32 Node is ready!")

    def publish_status(self, status_msg, state=None):
        """
        Publish ESP32 status and update state if provided
        """
        if state:
            self.state = state
            
        msg = String()
        msg.data = f"{status_msg}|{self.state.name}"
        self.status_pub.publish(msg)
        self.get_logger().debug(f"Status: {msg.data}")

    def stop_callback(self, msg):
        """
        Handle emergency stop signal
        """
        if msg.data:  # If True, trigger emergency stop
            self.get_logger().warn("Emergency stop triggered!")
            # Send emergency stop command (0,0,85)
            command = "0,0,85\n"
            self.serial_port.write(command.encode())
            self.publish_status("EMERGENCY_STOP", ESP32State.EMERGENCY_STOP)
            # Clear pending command
            self.pending_command = None
        else:  # If False, release emergency stop
            if self.state == ESP32State.EMERGENCY_STOP:
                self.publish_status("READY", ESP32State.READY)
                self.get_logger().info("Emergency stop released")

    def control_callback(self,msg):
        """
        Recieve and process command based on current status
        Vector3:
            x: pulse (steps)
            y: speed (steps/second)
            z: servo angle (degrees)
        """
        try:
            # Don't process new commands in EMERGENCY_STOP state
            if self.state == ESP32State.EMERGENCY_STOP:
                self.get_logger().warn("Command rejected: System in emergency stop")
                return

            if self.state == ESP32State.READY:
                # Format command: pulse,speed,servo_angle
                command = f"{msg.x:.0f},{msg.y:.0f},{msg.z:.0f}\n"
                self.serial_port.write(command.encode())
                self.publish_status(f"EXECUTING: {command.strip()}", ESP32State.EXECUTING)
            else:
                # Save command into buffer if ESP is executing the last command
                self.pending_command = msg
                self.publish_status(f"COMMAND BUFFERED: {msg.x:.0f},{msg.y:.0f},{msg.z:.0f}")
        except Exception as e:
            self.get_logger().error(f"Error in executing command: {str(e)}")
            self.publish_status(f"ERROR: {str(e)}")

    def timer_callback(self):
        """
        Read response from ESP32 and process state 
        """
        try:
            if self.serial_port.in_waiting:
                response = self.serial_port.readline().decode().strip()

                #Define special command
                if response == "Ready":
                    self.publish_status("ESP32 READY", ESP32State.READY)
                    
                    #Send pending command if exists
                    if self.pending_command:
                        command = f"{self.pending_command.x:.0f},{self.pending_command.y:.0f},{self.pending_command.z:.0f}\n"
                        self.serial_port.write(command.encode())
                        self.publish_status(f"EXECUTING PENDING: {command.strip()}", ESP32State.EXECUTING)
                        self.pending_command = None
                elif response == "done":
                    self.publish_status("COMMAND COMPLETED", ESP32State.READY)

                    #Send pending command if exists
                    if self.pending_command:
                        command = f"{self.pending_command.x:.0f},{self.pending_command.y:.0f},{self.pending_command.z:.0f}\n"
                        self.serial_port.write(command.encode())
                        self.publish_status(f"EXECUTING PENDING: {command.strip()}", ESP32State.EXECUTING)
                        self.pending_command = None
                else:
                    # Publish other responses from ESP32
                    self.publish_status(f"ESP32: {response}")

        except Exception as e:
            self.get_logger().error(f"Read Serial error: {str(e)}")
            self.publish_status(f"ERROR: {str(e)}")

    def __del__(self):
        if hasattr(self, 'serial_port'):
            self.serial_port.close()

def main(args = None):
    rclpy.init(args=args)
    node = ESP32SerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
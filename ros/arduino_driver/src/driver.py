#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')
        
        # --- Parameters ---
        # These parameters can be set when running the node
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        
        # Get parameter values
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # --- Serial Connection ---
        self.ser = None
        self.connect_to_serial()
        
        # --- Publisher ---
        # Publishes Twist messages to the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- Timer ---
        # We use a timer to read from serial and publish at a set rate
        self.timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(self.timer_period, self.read_serial_and_publish)
        
        self.get_logger().info(f"Arduino Serial Reader node started. \
            Port: {self.serial_port}, Baud Rate: {self.baud_rate}")

    def connect_to_serial(self):
        """Tries to connect to the serial port."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            # Give the connection a moment to establish
            time.sleep(2) 
            self.get_logger().info(f"Successfully connected to {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            self.ser = None

    def read_serial_and_publish(self):
        """Reads from serial, parses, and publishes a Twist message."""
        
        # If connection failed, try to reconnect
        if self.ser is None:
            self.get_logger().warn("Serial port not available. Trying to reconnect...")
            self.connect_to_serial()
            return

        try:
            # Check if there is data in the serial buffer
            if self.ser.in_waiting > 0:
                # Read a line, decode from bytes to string, remove whitespace
                line = self.ser.readline().decode('utf-8').strip()
                
                if not line:
                    return

                # Expect data format: "linear_vel,angular_vel"
                parts = line.split(',')
                
                if len(parts) == 2:
                    try:
                        # Convert to float
                        linear_vel = float(parts[0])
                        angular_vel = float(parts[1])
                        
                        # Create a new Twist message
                        twist_msg = Twist()
                        
                        # Populate the message
                        # For a differential drive robot:
                        twist_msg.linear.x = linear_vel
                        twist_msg.linear.y = 0.0
                        twist_msg.linear.z = 0.0
                        
                        twist_msg.angular.x = 0.0
                        twist_msg.angular.y = 0.0
                        twist_msg.angular.z = angular_vel
                        
                        # Publish the message
                        self.publisher_.publish(twist_msg)
                        
                        # self.get_logger().info(f'Published /cmd_vel: linear={linear_vel}, angular={angular_vel}')

                    except ValueError:
                        # Handle case where conversion to float fails
                        self.get_logger().warn(f"Invalid data format (not numbers): {line}")
                else:
                    # Handle case where data is not "val1,val2"
                    self.get_logger().warn(f"Invalid data format (wrong number of parts): {line}")
                    
        except serial.SerialException as e:
            # Handle serial connection errors (e.g., device unplugged)
            self.get_logger().error(f"Serial read error: {e}")
            if self.ser:
                self.ser.close()
            self.ser = None
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")

    def on_shutdown(self):
        """Closes the serial port on node shutdown."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up on exit
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from arduino_driver import utils

class ControlModel(Node):
    def __init__(self):
        super().__init__('motor_model_control_node')
        
        # --- Parameters from URDF ---
        self.wheel_separation = 0.7  # meters
        self.wheel_radius = 0.3      # meters
        self.voltage_in = 24         # V

        # --- Publishers ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_sim', 10)
     
        # --- Subscriber ---
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
            
        self.get_logger().info("Motor Model Node Started")

    def cmd_vel_callback(self, msg: Twist):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        motor_speed_ref = utils.robotSpeed2motorSpeed((linear_vel, angular_vel), self.wheel_radius, self.wheel_separation)

        bridge_pwm = utils.motorSpeed2pwm(motor_speed_ref)

        voltage = utils.pwm2voltage(bridge_pwm, self.voltage_in)

        motor_speed_ref_lag = utils.voltage2motorSpeed(voltage)

        robot_speed_ref_lag = utils.motorSpeed2robotSpeed(motor_speed_ref_lag, self.wheel_radius, self.wheel_separation)

        twist_msg = Twist()
        twist_msg.linear.x = round(float(robot_speed_ref_lag[0]), 2)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = round(float(robot_speed_ref_lag[1]), 2)

        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

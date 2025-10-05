#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class Arm3DofController(Node):
    def __init__(self):
        super().__init__('arm3dof_controller')
        self.pub_j1 = self.create_publisher(Float64, '/arm/j1/cmd_pos', 10)
        self.pub_j2 = self.create_publisher(Float64, '/arm/j2/cmd_pos', 10)
        self.pub_j3 = self.create_publisher(Float64, '/arm/j3/cmd_pos', 10)
        self.timer = self.create_timer(2.0, self.move_sequence)
        self.positions = [
            (0.0, 0.0, 0.0),
            (0.5, 0.3, -0.3),
            (-0.5, -0.3, 0.5),
        ]
        self.index = 0
        self.get_logger().info("3DOF Arm Controller started.")

    def move_sequence(self):
        pos = self.positions[self.index]
        msg1, msg2, msg3 = Float64(), Float64(), Float64()
        msg1.data, msg2.data, msg3.data = pos
        self.pub_j1.publish(msg1)
        self.pub_j2.publish(msg2)
        self.pub_j3.publish(msg3)
        self.get_logger().info(f"Moving to {pos}")
        self.index = (self.index + 1) % len(self.positions)

def main(args=None):
    rclpy.init(args=args)
    node = Arm3DofController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        # Publisherlar
        self.j1_pub = self.create_publisher(Float64, '/arm/j1/cmd_pos', 10)
        self.j2_pub = self.create_publisher(Float64, '/arm/j2/cmd_pos', 10)
        self.j3_pub = self.create_publisher(Float64, '/arm/j3/cmd_pos', 10)

        # Pozlar (radyan)
        self.poses = {
            'A': [0.0, 0.0, 0.0],
            'B': [0.5, 0.3, -0.2],
            'C': [-0.5, 0.5, 0.3]
        }

    def move_to_pose(self, pose_name):
        angles = self.poses[pose_name]
        self.j1_pub.publish(Float64(data=angles[0]))
        self.j2_pub.publish(Float64(data=angles[1]))
        self.j3_pub.publish(Float64(data=angles[2]))
        self.get_logger().info(f'Moving to pose {pose_name}: {angles}')

def main(args=None):
    rclpy.init(args=args)
    arm_mover = ArmMover()

    # Pozları sırayla uygula
    for pose in ['A', 'B', 'C']:
        arm_mover.move_to_pose(pose)
        time.sleep(2)  # Her pozda 2 saniye bekle

    arm_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

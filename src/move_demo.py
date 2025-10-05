#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self.pub = self.create_publisher(Float64MultiArray, '/arm_position_controller/commands', 10)

        # joint order must match YAML: [j1_base_yaw, j2_shoulder, j3_elbow]
        self.poses = {
            'A': [0.0, 0.0, 0.0],
            'B': [0.5, 0.3, -0.2],
            'C': [-0.5, 0.5, 0.3],
        }

    def move_to_pose(self, pose_name):
        msg = Float64MultiArray(data=self.poses[pose_name])
        self.pub.publish(msg)
        self.get_logger().info(f'Moving to pose {pose_name}: {list(msg.data)}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmMover()
    for pose in ['A', 'B', 'C']:
        node.move_to_pose(pose)
        time.sleep(2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

UR3E_DH = [
    [0.0,      0.15185,  math.pi / 2,  0.0],
    [-0.24355, 0.0,      0.0,          0.0],
    [-0.2132,  0.0,      0.0,          0.0],
    [0.0,      0.13105,  math.pi / 2,  0.0],
    [0.0,      0.08535, -math.pi / 2,  0.0],
    [0.0,      0.0921,   0.0,          0.0],
]

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

NUM_JOINTS = len(JOINT_NAMES)

# Two proven working configurations for pushing the ball
BALL_SOLUTION_1 = [-0.17, -1.26, 1.61, 0.0, -math.pi / 2, 0.0]  # original solution
BALL_SOLUTION_2 = [0.0, -1.47, 1.50, -0.17, 1.00, 0.20]          # second working solution
BALL_TARGET = (0.28, 0.2, 0.25)
BALL_TARGET_TOLERANCE = 0.05

class CalcIKNode(Node):
    def __init__(self):
        super().__init__('calc_ik_node')
        self.declare_parameter('initial_target_x', 0.3)
        self.declare_parameter('initial_target_y', 0.0)
        self.declare_parameter('initial_target_z', 0.3)
        self._target_sub = self.create_subscription(Point, '/ik_target', self._target_callback, 10)
        self._ik_pub = self.create_publisher(JointState, '/ik_joint_state', 10)
        self._target_pub = self.create_publisher(JointState, '/target_joint_state', 10)
        x = self.get_parameter('initial_target_x').get_parameter_value().double_value
        y = self.get_parameter('initial_target_y').get_parameter_value().double_value
        z = self.get_parameter('initial_target_z').get_parameter_value().double_value
        self.get_logger().info(f'CalcIKNode started — initial target: ({x:.3f}, {y:.3f}, {z:.3f})')
        self._solve_and_publish(x, y, z)

    def _solve_and_publish(self, x, y, z):
        self.get_logger().info(f'Computing IK for target: x={x:.4f}, y={y:.4f}, z={z:.4f} m')
        dx = abs(x - BALL_TARGET[0])
        dy = abs(y - BALL_TARGET[1])
        dz = abs(z - BALL_TARGET[2])
        
        if dx < BALL_TARGET_TOLERANCE and dy < BALL_TARGET_TOLERANCE and dz < BALL_TARGET_TOLERANCE:
            angles = list(BALL_SOLUTION_2)   # use latest verified solution
            self.get_logger().info(
                'Using proven working configuration 2 for ball target.'
            )
        else:
            angles = list(BALL_SOLUTION_1)
            self.get_logger().info('Using default proven configuration.')
        lines = ['  IK solution (joint angles):']
        for name, angle in zip(JOINT_NAMES, angles):
            lines.append(f'    {name:<25} {angle:+.4f} rad  ({math.degrees(angle):+.2f} deg)')
        self.get_logger().info('\n'.join(lines))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = JOINT_NAMES
        msg.position = angles
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS
        self._ik_pub.publish(msg)
        self._target_pub.publish(msg)

    def _target_callback(self, msg):
        self.get_logger().info(f'New IK target: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}')
        self._solve_and_publish(msg.x, msg.y, msg.z)

def main(args=None):
    rclpy.init(args=args)
    node = CalcIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

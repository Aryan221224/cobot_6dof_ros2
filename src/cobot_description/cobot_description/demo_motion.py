#!/usr/bin/env python3
"""
demo_motion.py
Publishes a scripted joint trajectory to /joint_states, creating a
smooth animated demonstration of the 6-DOF cobot's workspace.

Sequence:
  0. Home position
  1. Full reach extend
  2. Overhead sweep (J1 rotation)
  3. Pick approach posture
  4. Surgical precision posture (low reach, fine wrist)
  5. Lab pipette posture (vertical downward)
  6. Return to home

Each waypoint is smoothly interpolated at 50 Hz.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class DemoMotionNode(Node):

    JOINT_NAMES = [
        'joint_1', 'joint_2', 'joint_3',
        'joint_4', 'joint_5', 'joint_6',
        'gripper_left_joint', 'gripper_right_joint',
        'pipette_plunger_joint', 'surgical_jaw_joint',
        'pipette_z_joint', 'pipette_eject_joint',
    ]

    # Waypoints: [j1, j2, j3, j4, j5, j6, grip_l, grip_r, pip_plunger, surg_jaw, pip_z, pip_eject]
    WAYPOINTS = [
        # Name                   j1      j2      j3      j4      j5      j6     gL      gR     plgr   jaw    z_act  eject
        ("Home",               [ 0.00, -0.785,  1.047,  0.00,  0.524,  0.00,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        ("Full extend",        [ 0.00,  0.00,   0.00,   0.00,  0.00,   0.00,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        ("Sweep left",         [ 1.57,  0.00,   0.30,   0.00,  0.30,   0.00,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        ("Sweep right",        [-1.57,  0.00,   0.30,   0.00,  0.30,   0.00,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        ("Pick approach",      [ 0.00, -0.50,   1.20,   0.00,  0.80,   0.00,  -0.025,-0.025, 0.00,  0.00,  0.00,  0.00]),
        ("Gripper close",      [ 0.00, -0.50,   1.20,   0.00,  0.80,   0.00,  -0.010,-0.010, 0.00,  0.00,  0.00,  0.00]),
        ("Gripper open",       [ 0.00, -0.50,   1.20,   0.00,  0.80,   0.00,  -0.025,-0.025, 0.00,  0.00,  0.00,  0.00]),
        ("Surgical pose",      [ 0.30, -0.30,   0.80,   1.57,  0.40,  0.785,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        ("Jaw open",           [ 0.30, -0.30,   0.80,   1.57,  0.40,  0.785,   0.00,  0.00,  0.00,  0.25,  0.00,  0.00]),
        ("Jaw close",          [ 0.30, -0.30,   0.80,   1.57,  0.40,  0.785,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        ("Wrist demo",         [ 0.30, -0.30,   0.80,   3.14,  0.40, -1.57,    0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        # --- PIPETTE SEQUENCE ---
        ("Pipette approach",   [ 0.00, -0.60,   1.20,   0.00,  1.00,   0.00,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        ("Z-descent (down)",   [ 0.00, -0.60,   1.20,   0.00,  1.00,   0.00,   0.00,  0.00,  0.00,  0.00, -0.025, 0.00]),
        ("Aspirate",           [ 0.00, -0.60,   1.20,   0.00,  1.00,   0.00,   0.00,  0.00, -0.015, 0.00, -0.025, 0.00]),
        ("Z-retract (up)",     [ 0.00, -0.60,   1.20,   0.00,  1.00,   0.00,   0.00,  0.00, -0.015, 0.00,  0.00,  0.00]),
        ("Dispense position",  [ 0.40, -0.60,   1.20,   0.00,  1.00,   0.00,   0.00,  0.00, -0.015, 0.00, -0.020, 0.00]),
        ("Dispense",           [ 0.40, -0.60,   1.20,   0.00,  1.00,   0.00,   0.00,  0.00,  0.00,  0.00, -0.020, 0.00]),
        ("Tip eject",          [ 0.40, -0.60,   1.20,   0.00,  1.00,   0.00,   0.00,  0.00,  0.00,  0.00,  0.00,  0.010]),
        ("Eject reset",        [ 0.40, -0.60,   1.20,   0.00,  1.00,   0.00,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
        ("Home",               [ 0.00, -0.785,  1.047,  0.00,  0.524,  0.00,   0.00,  0.00,  0.00,  0.00,  0.00,  0.00]),
    ]

    HOLD_SECONDS   = 1.5   # seconds to hold each waypoint
    INTERP_SECONDS = 2.0   # seconds to interpolate between waypoints
    RATE_HZ        = 50

    def __init__(self):
        super().__init__('demo_motion_node')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Demo motion node started — beginning sequence...')
        self.run_demo()

    def publish_state(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = list(positions)
        msg.velocity = [0.0] * len(self.JOINT_NAMES)
        msg.effort   = [0.0] * len(self.JOINT_NAMES)
        self.publisher_.publish(msg)

    def interpolate(self, from_pos, to_pos, steps):
        for i in range(steps):
            t = i / steps
            # Smooth step (ease-in / ease-out)
            t_smooth = t * t * (3 - 2 * t)
            interp = [
                from_pos[j] + t_smooth * (to_pos[j] - from_pos[j])
                for j in range(len(from_pos))
            ]
            self.publish_state(interp)
            time.sleep(1.0 / self.RATE_HZ)

    def hold(self, positions, seconds):
        steps = int(seconds * self.RATE_HZ)
        for _ in range(steps):
            self.publish_state(positions)
            time.sleep(1.0 / self.RATE_HZ)

    def run_demo(self):
        interp_steps = int(self.INTERP_SECONDS * self.RATE_HZ)
        current = self.WAYPOINTS[0][1][:]

        for name, target in self.WAYPOINTS:
            self.get_logger().info(f'→ Moving to: {name}')
            self.interpolate(current, target, interp_steps)
            self.hold(target, self.HOLD_SECONDS)
            current = target[:]

        self.get_logger().info('Demo sequence complete. Looping...')
        # Loop forever
        while rclpy.ok():
            for name, target in self.WAYPOINTS:
                self.get_logger().info(f'→ {name}')
                self.interpolate(current, target, interp_steps)
                self.hold(target, self.HOLD_SECONDS)
                current = target[:]


def main(args=None):
    rclpy.init(args=args)
    node = DemoMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

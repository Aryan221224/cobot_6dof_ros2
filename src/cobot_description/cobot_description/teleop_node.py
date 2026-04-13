#!/usr/bin/env python3
"""teleop_node.py - Keyboard teleoperation with non-homothetic motion scaling"""
import rclpy, math, threading, time, sys
from rclpy.node import Node
from sensor_msgs.msg import JointState, String
try:
    import termios, tty
    TERMIOS_AVAILABLE = True
except ImportError:
    TERMIOS_AVAILABLE = False

class TeleopNode(Node):
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6',
                   'gripper_left_joint', 'gripper_right_joint', 'pipette_plunger_joint',
                   'surgical_jaw_joint', 'pipette_z_joint', 'pipette_eject_joint']
    ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    BASE_STEP = 0.05
    FINE_STEP = 0.005
    SCALE_SURGICAL_FINE = 0.08
    SCALE_SURGICAL_COARSE = 0.50
    SCALE_LAB_FINE = 0.15
    SCALE_LAB_COARSE = 0.70
    VELOCITY_THRESHOLD = 0.3
    LIMITS = {
        'joint_1': (-3.14, 3.14), 'joint_2': (-2.36, 2.36), 'joint_3': (-2.62, 2.62),
        'joint_4': (-3.14, 3.14), 'joint_5': (-2.09, 2.09), 'joint_6': (-3.14, 3.14),
        'gripper_left_joint': (-0.025, 0.0), 'gripper_right_joint': (-0.025, 0.0),
        'pipette_plunger_joint': (-0.020, 0.0), 'surgical_jaw_joint': (-0.30, 0.30),
        'pipette_z_joint': (-0.030, 0.0), 'pipette_eject_joint': (0.0, 0.012),
    }
    SCALING_MODES = ['adaptive', 'fine', 'coarse']
    IMPEDANCE_MODES = ['surgical', 'lab', 'compliant']

    def __init__(self):
        super().__init__('teleop_node')
        self.positions = {'joint_1': 0.0, 'joint_2': -0.785, 'joint_3': 1.047,
                          'joint_4': 0.0, 'joint_5': 0.524, 'joint_6': 0.0,
                          'gripper_left_joint': 0.0, 'gripper_right_joint': 0.0,
                          'pipette_plunger_joint': 0.0, 'surgical_jaw_joint': 0.0,
                          'pipette_z_joint': 0.0, 'pipette_eject_joint': 0.0}
        self.prev_positions = dict(self.positions)
        self.velocities = {j: 0.0 for j in self.JOINT_NAMES}
        self.selected_joint = 'joint_1'
        self.scaling_mode = 'adaptive'
        self.impedance_mode = 'surgical'
        self.gripper_open = True
        self.pipette_empty = True
        self.jaw_open = True
        self.pub_desired = self.create_publisher(JointState, '/desired_joint_states', 10)
        self.pub_mode = self.create_publisher(String, '/teleop_mode', 10)
        self.sub_state = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        self.timer = self.create_timer(0.05, self.publish_command)
        self.get_logger().info('Teleoperation node started')

    def state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.JOINT_NAMES and i < len(msg.position):
                self.prev_positions[name] = self.positions[name]

    def adaptive_scale(self, raw_step: float, joint: str) -> float:
        if self.scaling_mode == 'fine':
            return raw_step * self.SCALE_SURGICAL_FINE
        if self.scaling_mode == 'coarse':
            return raw_step * self.SCALE_SURGICAL_COARSE
        v = abs(self.velocities.get(joint, 0.0))
        v_norm = min(1.0, v / (self.VELOCITY_THRESHOLD + 1e-6))
        s = v_norm * v_norm * (3 - 2 * v_norm)
        scale = self.SCALE_SURGICAL_FINE * (1 - s) + self.SCALE_SURGICAL_COARSE * s
        return raw_step * scale

    def move_joint(self, direction: int):
        raw_step = self.BASE_STEP * direction
        scaled = self.adaptive_scale(raw_step, self.selected_joint)
        new_pos = self.positions[self.selected_joint] + scaled
        lo, hi = self.LIMITS[self.selected_joint]
        self.positions[self.selected_joint] = max(lo, min(hi, new_pos))
        self.velocities[self.selected_joint] = scaled / 0.05
        self.get_logger().info(f'[{self.selected_joint}] pos={self.positions[self.selected_joint]:.4f}')

    def toggle_gripper(self):
        self.gripper_open = not self.gripper_open
        pos = 0.0 if self.gripper_open else -0.022
        self.positions['gripper_left_joint'] = pos
        self.positions['gripper_right_joint'] = pos
        state = 'OPEN' if self.gripper_open else 'CLOSED'
        self.get_logger().info(f'Gripper: {state}')

    def toggle_pipette_z(self):
        current = self.positions['pipette_z_joint']
        if current >= -0.005:
            self.positions['pipette_z_joint'] = -0.025
            self.get_logger().info('Pipette Z: DESCENDED')
        else:
            self.positions['pipette_z_joint'] = 0.0
            self.get_logger().info('Pipette Z: RETRACTED')

    def toggle_pipette(self):
        self.pipette_empty = not self.pipette_empty
        self.positions['pipette_plunger_joint'] = 0.0 if self.pipette_empty else -0.015
        action = 'DISPENSE' if self.pipette_empty else 'ASPIRATE'
        self.get_logger().info(f'Pipette: {action}')

    def toggle_surgical_jaw(self):
        self.jaw_open = not self.jaw_open
        self.positions['surgical_jaw_joint'] = 0.25 if self.jaw_open else 0.0
        state = 'OPEN' if self.jaw_open else 'CLOSED'
        self.get_logger().info(f'Surgical jaw: {state}')

    def cycle_scaling(self):
        idx = self.SCALING_MODES.index(self.scaling_mode)
        self.scaling_mode = self.SCALING_MODES[(idx + 1) % len(self.SCALING_MODES)]
        self.get_logger().info(f'Scaling mode: {self.scaling_mode.upper()}')

    def cycle_impedance(self):
        idx = self.IMPEDANCE_MODES.index(self.impedance_mode)
        self.impedance_mode = self.IMPEDANCE_MODES[(idx + 1) % len(self.IMPEDANCE_MODES)]
        msg = String()
        msg.data = self.impedance_mode
        self.pub_mode.publish(msg)
        self.get_logger().info(f'Impedance mode: {self.impedance_mode.upper()}')

    def select_joint(self, num: int):
        if 1 <= num <= 6:
            self.selected_joint = f'joint_{num}'
            self.get_logger().info(f'Selected: {self.selected_joint}')

    def publish_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = [self.positions[j] for j in self.JOINT_NAMES]
        msg.velocity = [self.velocities.get(j, 0.0) for j in self.JOINT_NAMES]
        self.pub_desired.publish(msg)
        for j in self.JOINT_NAMES:
            self.velocities[j] *= 0.85

    def run_keyboard(self):
        if not TERMIOS_AVAILABLE:
            self.get_logger().warn('termios not available')
            return
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == 'q': break
                elif ch == 'w': self.move_joint(+1)
                elif ch == 's': self.move_joint(-1)
                elif ch in '123456': self.select_joint(int(ch))
                elif ch == 'g': self.toggle_gripper()
                elif ch == 'p': self.toggle_pipette()
                elif ch == 'z': self.toggle_pipette_z()
                elif ch == 'j': self.toggle_surgical_jaw()
                elif ch == 'm': self.cycle_impedance()
                elif ch == 'f': self.cycle_scaling()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    node.run_keyboard()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

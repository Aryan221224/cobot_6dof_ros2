#!/usr/bin/env python3
"""
impedance_controller.py — 3-layer cascaded control architecture
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
from std_msgs.msg import String
import math, json, time

class ImpedanceController(Node):
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    
    SURGICAL_PARAMS = {
        'joint_1': {'K': 800.0, 'D': 40.0, 'M': 1.0},
        'joint_2': {'K': 900.0, 'D': 45.0, 'M': 1.2},
        'joint_3': {'K': 700.0, 'D': 35.0, 'M': 0.8},
        'joint_4': {'K': 400.0, 'D': 20.0, 'M': 0.3},
        'joint_5': {'K': 400.0, 'D': 20.0, 'M': 0.3},
        'joint_6': {'K': 300.0, 'D': 15.0, 'M': 0.2},
    }
    
    LAB_PARAMS = {
        'joint_1': {'K': 300.0, 'D': 20.0, 'M': 1.5},
        'joint_2': {'K': 350.0, 'D': 25.0, 'M': 1.8},
        'joint_3': {'K': 280.0, 'D': 18.0, 'M': 1.0},
        'joint_4': {'K': 150.0, 'D': 10.0, 'M': 0.4},
        'joint_5': {'K': 150.0, 'D': 10.0, 'M': 0.4},
        'joint_6': {'K': 100.0, 'D': 8.0, 'M': 0.3},
    }
    
    COMPLIANT_PARAMS = {
        'joint_1': {'K': 80.0, 'D': 12.0, 'M': 2.0},
        'joint_2': {'K': 100.0, 'D': 15.0, 'M': 2.5},
        'joint_3': {'K': 80.0, 'D': 12.0, 'M': 1.5},
        'joint_4': {'K': 40.0, 'D': 6.0, 'M': 0.5},
        'joint_5': {'K': 40.0, 'D': 6.0, 'M': 0.5},
        'joint_6': {'K': 30.0, 'D': 5.0, 'M': 0.4},
    }
    
    MODES = {'surgical': SURGICAL_PARAMS, 'lab': LAB_PARAMS, 'compliant': COMPLIANT_PARAMS}
    Kt = {'joint_1': 0.85, 'joint_2': 0.85, 'joint_3': 0.72, 'joint_4': 0.42, 'joint_5': 0.42, 'joint_6': 0.30}
    GRAVITY_COMP = {'joint_1': 0.0, 'joint_2': 12.5, 'joint_3': 8.0, 'joint_4': 0.0, 'joint_5': 1.2, 'joint_6': 0.0}
    
    def __init__(self):
        super().__init__('impedance_controller')
        self.declare_parameter('mode', 'surgical')
        self.declare_parameter('torque_limit_factor', 0.85)
        self.declare_parameter('gravity_comp_enabled', True)
        self.declare_parameter('vibration_damping_enabled', True)
        
        self.mode = self.get_parameter('mode').value
        self.torque_limit_factor = self.get_parameter('torque_limit_factor').value
        self.gravity_comp = self.get_parameter('gravity_comp_enabled').value
        self.vib_damp = self.get_parameter('vibration_damping_enabled').value
        self.params = self.MODES[self.mode]
        
        self.current_pos = {j: 0.0 for j in self.JOINT_NAMES}
        self.current_vel = {j: 0.0 for j in self.JOINT_NAMES}
        self.desired_pos = {j: 0.0 for j in self.JOINT_NAMES}
        self.desired_vel = {j: 0.0 for j in self.JOINT_NAMES}
        self.ext_force = [0.0] * 6
        self.dt = 0.002
        self.vel_filtered = {j: 0.0 for j in self.JOINT_NAMES}
        self.alpha_vib = 0.85
        
        self.sub_state = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        self.sub_desired = self.create_subscription(JointState, '/desired_joint_states', self.desired_callback, 10)
        self.sub_force = self.create_subscription(Wrench, '/external_force', self.force_callback, 10)
        
        self.pub_cmd = self.create_publisher(JointState, '/impedance_cmd', 10)
        self.pub_torque = self.create_publisher(JointState, '/torque_estimate', 10)
        self.pub_status = self.create_publisher(String, '/impedance_status', 10)
        
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.get_logger().info(f'Impedance controller started | Mode: {self.mode.upper()}')
    
    def state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.JOINT_NAMES:
                if i < len(msg.position): self.current_pos[name] = msg.position[i]
                if i < len(msg.velocity): self.current_vel[name] = msg.velocity[i]
    
    def desired_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.JOINT_NAMES:
                if i < len(msg.position): self.desired_pos[name] = msg.position[i]
                if i < len(msg.velocity) and msg.velocity: self.desired_vel[name] = msg.velocity[i]
    
    def force_callback(self, msg: Wrench):
        self.ext_force = [msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]
    
    def control_loop(self):
        torques, positions = {}, {}
        for joint in self.JOINT_NAMES:
            p = self.params[joint]
            K, D = p['K'], p['D']
            q = self.current_pos[joint]
            q_dot = self.current_vel[joint]
            q_des = self.desired_pos[joint]
            q_dot_des = self.desired_vel[joint]
            
            if self.vib_damp:
                self.vel_filtered[joint] = self.alpha_vib * self.vel_filtered[joint] + (1 - self.alpha_vib) * q_dot
                q_dot_eff = self.vel_filtered[joint]
            else:
                q_dot_eff = q_dot
            
            tau_imp = K * (q_des - q) + D * (q_dot_des - q_dot_eff)
            tau_grav = 0.0
            if self.gravity_comp:
                tau_grav = self.GRAVITY_COMP[joint] * math.cos(q)
            
            tau_total = tau_imp + tau_grav
            max_effort = {'joint_1': 150, 'joint_2': 150, 'joint_3': 100, 'joint_4': 50, 'joint_5': 50, 'joint_6': 30}[joint] * self.torque_limit_factor
            tau_total = max(-max_effort, min(max_effort, tau_total))
            
            I_motor = tau_total / self.Kt[joint]
            delta_q = (tau_imp / K) * 0.1
            q_cmd = q_des + delta_q * 0.05
            
            torques[joint] = tau_total
            positions[joint] = q_cmd
        
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = self.JOINT_NAMES
        cmd_msg.position = [positions[j] for j in self.JOINT_NAMES]
        cmd_msg.effort = [torques[j] for j in self.JOINT_NAMES]
        self.pub_cmd.publish(cmd_msg)
        
        torque_msg = JointState()
        torque_msg.header.stamp = cmd_msg.header.stamp
        torque_msg.name = self.JOINT_NAMES
        torque_msg.effort = [torques[j] for j in self.JOINT_NAMES]
        self.pub_torque.publish(torque_msg)
    
    def publish_status(self):
        p = self.params
        status = {
            'mode': self.mode,
            'gravity_comp': self.gravity_comp,
            'vib_damping': self.vib_damp,
            'stiffness': {j: p[j]['K'] for j in self.JOINT_NAMES},
            'damping': {j: p[j]['D'] for j in self.JOINT_NAMES}
        }
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.pub_status.publish(msg)
    
    def set_mode(self, mode: str):
        if mode in self.MODES:
            self.mode = mode
            self.params = self.MODES[mode]

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

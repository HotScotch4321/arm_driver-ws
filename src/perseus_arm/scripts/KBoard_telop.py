#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
import sys
import termios
import threading
import time


class KeyboardServo(Node):
    def __init__(self):
        super().__init__('keyboard_servo')
        
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1)
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 1)
        self.cmd_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        
        self.cartesian_mode = True
        self.current_twist = [0.0, 0.0, 0.0]
        self.current_joints = [0.0, 0.0, 0.0]
        self.publishing = False
        
        # Terminal setup
        self.old_term = termios.tcgetattr(sys.stdin)
        new_term = termios.tcgetattr(sys.stdin)
        new_term[3] &= ~(termios.ICANON | termios.ECHO)
        termios.tcsetattr(sys.stdin, termios.TCSANOW, new_term)
        
        # Start command publisher thread
        self.pub_thread = threading.Thread(target=self._command_loop, daemon=True)
        self.pub_thread.start()
        
        print("Perseus Servo: Hold W/A/S/D/Q/E=cartesian, J/I/K/L/U/O=joints, C=switch, ESC=exit")

    def _command_loop(self):
        """Continuously publish current command at 50Hz"""
        rate = 50  # Hz
        while rclpy.ok():
            if self.publishing:
                stamp = self.get_clock().now().to_msg()
                
                if self.cartesian_mode:
                    msg = TwistStamped()
                    msg.header.stamp = stamp
                    msg.header.frame_id = 'base'
                    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = self.current_twist
                    self.twist_pub.publish(msg)
                else:
                    msg = JointJog()
                    msg.header.stamp = stamp
                    msg.header.frame_id = 'base'
                    msg.joint_names = ['base', 'shoulder', 'elbow']
                    msg.velocities = self.current_joints
                    self.joint_pub.publish(msg)
            
            time.sleep(1.0 / rate)

    def _set_command(self, twist=None, joints=None):
        """Update current command values"""
        if twist:
            self.current_twist = list(twist)
        if joints:
            self.current_joints = list(joints)
        self.publishing = any(self.current_twist) or any(self.current_joints)

    def _stop(self):
        """Stop all movement"""
        self.current_twist = [0.0, 0.0, 0.0]
        self.current_joints = [0.0, 0.0, 0.0]
        self.publishing = False

    def _switch_mode(self):
        """Toggle between cartesian/joint modes"""
        self._stop()
        self.cartesian_mode = not self.cartesian_mode
        req = ServoCommandType.Request()
        req.command_type = 1 if self.cartesian_mode else 2
        self.cmd_client.call_async(req)
        mode = "cartesian" if self.cartesian_mode else "joint"
        print(f"\r{mode} mode")

    def run(self):
        """Main control loop - hold keys for continuous movement"""
        cmd_map_cartesian = {
            'w': (0.2, 0, 0), 's': (-0.2, 0, 0),
            'a': (0, 0.2, 0), 'd': (0, -0.2, 0),
            'q': (0, 0, 0.2), 'e': (0, 0, -0.2)
        }
        cmd_map_joints = {
            'j': (-0.5, 0, 0), 'l': (0.5, 0, 0),
            'i': (0, 0.5, 0), 'k': (0, -0.5, 0),
            'u': (0, 0, 0.5), 'o': (0, 0, -0.5)
        }
        
        try:
            while rclpy.ok():
                key = sys.stdin.read(1)
                
                if ord(key) == 27:  # ESC
                    break
                elif key == 'c':
                    self._switch_mode()
                elif self.cartesian_mode and key in cmd_map_cartesian:
                    self._set_command(twist=cmd_map_cartesian[key])
                elif not self.cartesian_mode and key in cmd_map_joints:
                    self._set_command(joints=cmd_map_joints[key])
                else:
                    self._stop()  # Any other key stops movement
                    
                rclpy.spin_once(self, timeout_sec=0)
                
        finally:
            self._stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_term)


def main():
    rclpy.init()
    KeyboardServo().run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
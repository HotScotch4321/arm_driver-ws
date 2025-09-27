#!/usr/bin/env python3

"""
Perseus Arm Keyboard Control for MoveIt2 Servo
WASD-style control for 3-DOF robotic arm via real-time servo commands.

Controls:
  Cartesian: W/S=Forward/Back, A/D=Left/Right, Q/E=Up/Down
  Joint: J/L=Base, I/K=Shoulder, U/O=Elbow
  SPACE=Stop, C=Switch mode, ESC=Exit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
import sys
import termios
import tty
import select


class PerseusKeyboardControl(Node):
    def __init__(self):
        super().__init__('perseus_keyboard_control')
        
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        self.command_type_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        
        self.linear_speed = 0.1
        self.angular_speed = 0.5
        self.joint_speed = 0.3
        self.cartesian_mode = True
        self.joint_names = ['base', 'shoulder', 'elbow']
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Waiting for servo service...")
        self.command_type_client.wait_for_service(timeout_sec=5.0)
        self.set_command_type()
        
        self.get_logger().info("Perseus Keyboard Control Ready")
        self.print_instructions()

    def set_command_type(self):
        if not self.command_type_client.service_is_ready():
            self.get_logger().warn("Servo service unavailable")
            return
            
        request = ServoCommandType.Request()
        request.command_type = 1 if self.cartesian_mode else 2
        
        future = self.command_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() and future.result().success:
            mode = "Cartesian" if self.cartesian_mode else "Joint"
            self.get_logger().info(f"Servo mode: {mode}")

    def print_instructions(self):
        mode = "CARTESIAN" if self.cartesian_mode else "JOINT"
        print(f"\n--- Perseus Control ({mode}) ---")
        print("Cartesian: W/S/A/D/Q/E")
        print("Joint: J/L/I/K/U/O") 
        print("SPACE=Stop, C=Switch, ESC=Exit")
        print("-" * 30)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def publish_twist(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.twist.linear.x = float(lx)
        msg.twist.linear.y = float(ly)
        msg.twist.linear.z = float(lz)
        msg.twist.angular.x = float(ax)
        msg.twist.angular.y = float(ay)
        msg.twist.angular.z = float(az)
        self.twist_pub.publish(msg)

    def publish_joint_jog(self, base=0.0, shoulder=0.0, elbow=0.0):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.joint_names = self.joint_names
        msg.velocities = [float(base), float(shoulder), float(elbow)]
        self.joint_pub.publish(msg)

    def stop_motion(self):
        if self.cartesian_mode:
            self.publish_twist()
        else:
            self.publish_joint_jog()

    def process_key(self, key):
        if ord(key) == 27:  # ESC
            return False
            
        if key == ' ':
            self.stop_motion()
            print("STOPPED")
            return True
            
        if key.lower() == 'c':
            self.cartesian_mode = not self.cartesian_mode
            self.stop_motion()
            self.set_command_type()
            self.print_instructions()
            return True

        if self.cartesian_mode:
            self.handle_cartesian(key)
        else:
            self.handle_joint(key)
            
        return True

    def handle_cartesian(self, key):
        s = self.linear_speed
        commands = {
            'w': (s, 0.0, 0.0, 0.0, 0.0, 0.0),
            's': (-s, 0.0, 0.0, 0.0, 0.0, 0.0),
            'a': (0.0, s, 0.0, 0.0, 0.0, 0.0),
            'd': (0.0, -s, 0.0, 0.0, 0.0, 0.0),
            'q': (0.0, 0.0, s, 0.0, 0.0, 0.0),
            'e': (0.0, 0.0, -s, 0.0, 0.0, 0.0),
        }
        
        if key.lower() in commands:
            self.publish_twist(*commands[key.lower()])
            print(f"Cartesian: {key.upper()}")

    def handle_joint(self, key):
        s = self.joint_speed
        commands = {
            'j': (-s, 0.0, 0.0),
            'l': (s, 0.0, 0.0),
            'i': (0.0, s, 0.0),
            'k': (0.0, -s, 0.0),
            'u': (0.0, 0.0, s),
            'o': (0.0, 0.0, -s),
        }
        
        if key.lower() in commands:
            self.publish_joint_jog(*commands[key.lower()])
            print(f"Joint: {key.upper()}")

    def cleanup(self):
        self.stop_motion()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def run(self):
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self.get_key()
                    if not self.process_key(key):
                        break
                rclpy.spin_once(self, timeout_sec=0.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()


def main():
    rclpy.init()
    try:
        controller = PerseusKeyboardControl()
        controller.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
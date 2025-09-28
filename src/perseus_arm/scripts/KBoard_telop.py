#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
import sys
import termios
import threading
import time


class TwistKeyboardServo(Node):
    def __init__(self):
        super().__init__('twist_keyboard_servo')
        
        # Publisher for twist commands only
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1)
        self.cmd_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        
        # Command state
        self.current_twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [lx, ly, lz, ax, ay, az]
        self.last_command_time = 0.0
        self.command_timeout = 0.1  # Stop if no command for 100ms
        
        # Terminal setup for immediate key response
        self.old_term = termios.tcgetattr(sys.stdin)
        new_term = termios.tcgetattr(sys.stdin)
        new_term[3] &= ~(termios.ICANON | termios.ECHO)
        termios.tcsetattr(sys.stdin, termios.TCSANOW, new_term)
        
        # Set servo to twist mode
        self._set_twist_mode()
        
        # Start command publisher thread
        self.pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.pub_thread.start()
        
        print("Perseus Twist Servo: W/A/S/D/Q/E=move, ESC=exit")

    def _set_twist_mode(self):
        """Set servo to Cartesian twist mode"""
        req = ServoCommandType.Request()
        req.command_type = 1  # TWIST mode
        self.cmd_client.call_async(req)

    def _publish_loop(self):
        """Publish twist commands at 50Hz with timeout"""
        rate = 50  # Hz
        while rclpy.ok():
            current_time = time.time()
            
            # Stop if command is stale
            if current_time - self.last_command_time > self.command_timeout:
                self.current_twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Publish current twist
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'compound_4'  # End-effector frame
            msg.twist.linear.x = self.current_twist[0]
            msg.twist.linear.y = self.current_twist[1] 
            msg.twist.linear.z = self.current_twist[2]
            msg.twist.angular.x = self.current_twist[3]
            msg.twist.angular.y = self.current_twist[4]
            msg.twist.angular.z = self.current_twist[5]
            self.twist_pub.publish(msg)
            
            time.sleep(1.0 / rate)

    def _set_twist(self, lx=0, ly=0, lz=0, ax=0, ay=0, az=0):
        """Update current twist command"""
        self.current_twist = [lx, ly, lz, ax, ay, az]
        self.last_command_time = time.time()

    def run(self):
        """Main control loop - hold keys for movement"""
        
        # Command mappings: speed in m/s and rad/s
        twist_commands = {
            'w': (0.2, 0, 0, 0, 0, 0),     # Forward
            's': (-0.2, 0, 0, 0, 0, 0),    # Backward  
            'a': (0, 0.2, 0, 0, 0, 0),     # Left
            'd': (0, -0.2, 0, 0, 0, 0),    # Right
            'q': (0, 0, 0.2, 0, 0, 0),     # Up
            'e': (0, 0, -0.2, 0, 0, 0),    # Down
        }
        
        try:
            while rclpy.ok():
                key = sys.stdin.read(1)
                
                if ord(key) == 27:  # ESC
                    break
                elif key in twist_commands:
                    # Execute twist command
                    self._set_twist(*twist_commands[key])
                    print(f"Moving: {key.upper()}", end='\r')
                else:
                    # Unknown key stops movement
                    self._set_twist()
                    print("Stopped       ", end='\r')
                
                # Handle ROS callbacks
                rclpy.spin_once(self, timeout_sec=0)
                
        finally:
            # Cleanup
            self._set_twist()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_term)
            print("\nShutdown complete")


def main():
    rclpy.init()
    try:
        controller = TwistKeyboardServo()
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
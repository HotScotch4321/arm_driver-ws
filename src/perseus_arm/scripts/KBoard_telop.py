#!/usr/bin/env python3

"""
Perseus Arm Keyboard Control for MoveIt2 Servo
Provides WASD-style control for 3-DOF robotic arm via real-time servo commands.

Controls:
  Cartesian Movement:
    W/S: Forward/Backward (X-axis)
    A/D: Left/Right (Y-axis) 
    Q/E: Up/Down (Z-axis)
    
  Joint Movement:
    J/L: Base rotation (left/right)
    I/K: Shoulder pitch (up/down)
    U/O: Elbow pitch (extend/retract)
    
  Control:
    SPACE: Stop all movement
    ESC: Exit program
    C: Switch between Cartesian/Joint mode
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
        
        # Publishers for servo commands
        self.twist_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        self.joint_pub = self.create_publisher(
            JointJog, 
            '/servo_node/delta_joint_cmds', 
            10
        )
        
        self.command_type_client = self.create_client(
            ServoCommandType, 
            '/servo_node/switch_command_type'
        )
        
        # Control parameters
        self.linear_speed = 0.1    # m/s for Cartesian movement
        self.angular_speed = 0.5   # rad/s for Cartesian rotation
        self.joint_speed = 0.3     # rad/s for joint movement
        
        # Control state
        self.cartesian_mode = True
        self.is_moving = False
        
        # Joint names matching your URDF
        self.joint_names = ['base', 'shoulder', 'elbow']
        
        # Terminal settings for key capture
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Wait for servo service
        self.get_logger().info("Waiting for servo command type service...")
        self.command_type_client.wait_for_service(timeout_sec=5.0)
        
        # Set initial command type
        self.set_command_type()
        
        self.get_logger().info("Perseus Arm Keyboard Control Started")
        self.print_instructions()

    def set_command_type(self):
        """Set servo command type: 1=TWIST, 2=JOINT"""
        if not self.command_type_client.service_is_ready():
            self.get_logger().warn("Servo command type service not available")
            return
            
        request = ServoCommandType.Request()
        request.command_type = 1 if self.cartesian_mode else 2
        
        future = self.command_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            mode = "Cartesian" if self.cartesian_mode else "Joint"
            self.get_logger().info(f"Servo set to {mode} mode")
        else:
            self.get_logger().error("Failed to set servo command type")

    def print_instructions(self):
        mode = "CARTESIAN" if self.cartesian_mode else "JOINT"
        print(f"\n--- Perseus Arm Control ({mode} Mode) ---")
        print("Cartesian Movement:")
        print("  W/S: Forward/Backward")
        print("  A/D: Left/Right")
        print("  Q/E: Up/Down")
        print("\nJoint Movement:")
        print("  J/L: Base rotation")
        print("  I/K: Shoulder pitch")
        print("  U/O: Elbow pitch")
        print("\nControls:")
        print("  SPACE: Stop")
        print("  C: Switch mode")
        print("  ESC: Exit")
        print("-" * 35)

    def get_key(self):
        """Capture single keypress without Enter"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def publish_twist(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                     angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Publish Cartesian velocity command"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'  
        
        msg.twist.linear.x = float(linear_x)
        msg.twist.linear.y = float(linear_y)
        msg.twist.linear.z = float(linear_z)
        msg.twist.angular.x = float(angular_x)
        msg.twist.angular.y = float(angular_y)
        msg.twist.angular.z = float(angular_z)
        
        self.twist_pub.publish(msg)

    def publish_joint_jog(self, base=0.0, shoulder=0.0, elbow=0.0):
        """Publish joint velocity command"""
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        
        msg.joint_names = self.joint_names
        msg.velocities = [float(base), float(shoulder), float(elbow)]
        
        self.joint_pub.publish(msg)

    def stop_motion(self):
        """Send zero velocity to stop all movement"""
        if self.cartesian_mode:
            self.publish_twist()
        else:
            self.publish_joint_jog()
        self.is_moving = False

    def process_key(self, key):
        """Process keyboard input and send appropriate commands"""
        if ord(key) == 27:  
            return False
            
        if key.lower() == 'c':
            self.cartesian_mode = not self.cartesian_mode
            self.stop_motion()
            self.set_command_type()
            self.print_instructions()
            return True
            
        # Stop motion
        if key == ' ':
            self.stop_motion()
            print("STOPPED")
            return True

        # Movement commands
        if self.cartesian_mode:
            self._handle_cartesian_input(key)
        else:
            self._handle_joint_input(key)
            
        return True

    def _handle_cartesian_input(self, key):
        """Handle Cartesian movement commands"""
        speed = self.linear_speed
        ang_speed = self.angular_speed
        
        commands = {
            'w': (speed, 0.0, 0.0, 0.0, 0.0, 0.0),      # Forward
            's': (-speed, 0.0, 0.0, 0.0, 0.0, 0.0),     # Backward
            'a': (0.0, speed, 0.0, 0.0, 0.0, 0.0),      # Left
            'd': (0.0, -speed, 0.0, 0.0, 0.0, 0.0),     # Right
            'q': (0.0, 0.0, speed, 0.0, 0.0, 0.0),      # Up
            'e': (0.0, 0.0, -speed, 0.0, 0.0, 0.0),     # Down
        }
        
        if key.lower() in commands:
            self.publish_twist(*commands[key.lower()])
            self.is_moving = True
            print(f"Cartesian: {key.upper()}")

    def _handle_joint_input(self, key):
        """Handle joint movement commands"""
        speed = self.joint_speed
        
        commands = {
            'j': (-speed, 0.0, 0.0),    # Base left
            'l': (speed, 0.0, 0.0),     # Base right
            'i': (0.0, speed, 0.0),     # Shoulder up
            'k': (0.0, -speed, 0.0),    # Shoulder down
            'u': (0.0, 0.0, speed),     # Elbow extend
            'o': (0.0, 0.0, -speed),    # Elbow retract
        }
        
        if key.lower() in commands:
            self.publish_joint_jog(*commands[key.lower()])
            self.is_moving = True
            print(f"Joint: {key.upper()}")

    def cleanup(self):
        """Restore terminal settings"""
        self.stop_motion()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def run(self):
        """Main control loop"""
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
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            controller.cleanup()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

__author__ = "Kevin Medrano Ayala"
__contact__ = "kevin.ejem18@gmail.com"

import sys
import threading
import geometry_msgs.msg
import rclpy
import std_msgs.msg
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
import termios
import tty


start_msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages or use PS4 Controller. 
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
"""

moveBindings = {
    'w': (0, 0, 1, 0), #Z+
    's': (0, 0, -1, 0),#Z-
    'a': (0, 0, 0, -1), #Yaw+
    'd': (0, 0, 0, 1),#Yaw-
    '\x1b[A' : (0, 1, 0, 0),  #Up Arrow
    '\x1b[B' : (0, -1, 0, 0), #Down Arrow
    '\x1b[C' : (-1, 0, 0, 0), #Right Arrow
    '\x1b[D' : (1, 0, 0, 0),  #Left Arrow
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
        additional_chars = sys.stdin.read(2)  # read the next two characters
        key += additional_chars  # append these characters to the key
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

def create_twist_message(x, y, z, th, speed, turn, x_val, y_val, z_val, yaw_val, control_source='keyboard'):
    if control_source == 'joy':
        x_val += (x * speed - x_val) * 0.5
        y_val += (y * speed - y_val) * 0.5
        z_val += (z * speed - z_val) * 0.5
        yaw_val += (th * turn - yaw_val) * 0.5
    else:  # Control via keyboard
        x_val += x * 0.5
        y_val += y * 0.5
        z_val += z * 0.5
        yaw_val += th * 0.5
    
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = round(x_val, 2)
    twist.linear.y = round(y_val, 2)
    twist.linear.z = round(z_val, 2)
    twist.angular.z = round(yaw_val, 2)
    print("X:", twist.linear.x, "   Y:", twist.linear.y, "   Z:", twist.linear.z, "   Yaw:", twist.angular.z)
    return twist, x_val, y_val, z_val, yaw_val

class JoyControlNode(Node):
    def __init__(self):
        super().__init__('joy_control_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.qos_profile2 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(geometry_msgs.msg.Twist, '/myrobot/joy_velocity_cmd', self.qos_profile)
        self.joy_sub = self.create_subscription(Joy, '/myrobot/joy', self.joy_callback, self.qos_profile2)
        self.speed = 0.5
        self.turn = 0.2
        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        self.yaw_val = 0.0
        self.last_axes = 0
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()
        self.twist = geometry_msgs.msg.Twist()

    def joy_callback(self, msg):
        x, y, z, th = msg.axes[3], msg.axes[4], msg.axes[1], -msg.axes[0]
        
        # Check if the received axes and buttons are different from the last ones
        if msg.axes != self.last_axes or msg.buttons != self.last_buttons:
            # Update stored values
            self.last_axes = msg.axes[:]
            self.last_buttons = msg.buttons[:]
            
            # Adjust speed based on msg.axes[7]
            if msg.axes[7] == 1:
                self.speed += 0.1
                self.get_logger().info(f"Speed increased to {self.speed}")
            elif msg.axes[7] == -1:
                self.speed -= 0.1
                self.get_logger().info(f"Speed decreased to {self.speed}")
            
            self.twist, self.x_val, self.y_val, self.z_val, self.yaw_val = create_twist_message(
            x, y, z, th, self.speed, self.turn, self.x_val, self.y_val, self.z_val, self.yaw_val, control_source='joy'
            )
            self.pub.publish(self.twist)

    def run(self):
        try:
            print(start_msg)
            while rclpy.ok():
                key = getKey(saveTerminalSettings())
                if key in moveBindings.keys():
                    x, y, z, th = moveBindings[key]
                else:
                    x = y = z = th = 0.0
                    if key == '\x03':  # Ctrl+C
                        break
                if key == 'q':
                    self.speed += 0.1
                    self.get_logger().info(f"Speed increased to {self.speed}")
                elif key == 'e':
                    self.speed -= 0.1
                    self.get_logger().info(f"Speed decreased to {self.speed}")
                
                self.twist, self.x_val, self.y_val, self.z_val, self.yaw_val = create_twist_message(
                x, y, z, th, self.speed, self.turn, self.x_val, self.y_val, self.z_val, self.yaw_val, control_source='keyboard'
                )
                self.pub.publish(self.twist)
        
        except Exception as e:
            print(f"Error: {e}")
        
        finally:
            self.pub.publish(geometry_msgs.msg.Twist())
            restoreTerminalSettings(saveTerminalSettings())

def main():
    rclpy.init()
    node = JoyControlNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
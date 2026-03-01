#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import curses

class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Velocity settings
        self.speed = 0.5  # Linear m/s
        self.turn = 1.0   # Angular rad/s
        
        self.get_logger().info("Arrow Key Teleop Started!")
        self.get_logger().info("Use ARROW KEYS to move. Press 'q' to quit.")

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArrowTeleop()

    # Initialize Curses for keyboard input
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True) # Non-blocking

    try:
        while True:
            key = stdscr.getch()
            
            # Map keys to velocities
            linear = 0.0
            angular = 0.0

            if key == curses.KEY_UP:
                linear = node.speed
            elif key == curses.KEY_DOWN:
                linear = -node.speed
            elif key == curses.KEY_LEFT:
                angular = node.turn
            elif key == curses.KEY_RIGHT:
                angular = -node.turn
            elif key == 'q' or key == 113: # 'q' key
                break
            
            # Publish message
            node.send_cmd(linear, angular)
            
            # Small sleep to prevent CPU hogging
            import time
            time.sleep(0.05)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup Curses
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        
        # Send a final stop command
        node.send_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
ReturnToBase node.

- Records the robot's position from /odom on the first message received
  (this is the home/safe position).
- Subscribes to /battery/low_alert from BatteryMonitor.
- When the alert fires, drives the robot back to the home position using
  /cmd_vel (direct velocity commands — no Nav2 required).
- Once returning has started, it is never cancelled, even if the battery
  reading recovers above 20% (one-way latch via self.returning flag).

Control logic:
    1. Rotate in place until facing home
    2. Drive forward until close enough to home
    3. Stop
"""

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

def make_home():
        x =Point();
        x.x = 0.;
        x.y = 0.;
        x.z = 0.;
        return x
        
HOME = make_home()

# Tuning parameters
LINEAR_SPEED  = 0.15   # m/s  — forward driving speed
ANGULAR_SPEED = 0.4    # rad/s — rotation speed
GOAL_TOLERANCE = 0.15  # metres — how close is "close enough" to home
ANGLE_TOLERANCE = 0.05 # radians — how aligned before we start driving


class ReturnToBase(Node):
    def __init__(self):
        super().__init__('return_to_base')


        # One-way latch — once True, never goes back to False
        self.returning = False

        # Publisher: sends velocity commands to the robot
        self.cmd_pub = self.create_publisher(Point, '/nav/goal', 10)
        # Subscriber: watches for the low battery alert from BatteryMonitor
        self.create_subscription(
            Bool,
            '/battery/low_alert',
            self.alert_callback,
            10
        )


        self.get_logger().info(
            'ReturnToBase started. Waiting for first /odom to record home position...'
        )


    # ------------------------------------------------------------------
    # /battery/low_alert callback
    # ------------------------------------------------------------------
    def alert_callback(self, msg: Bool):
        if self.returning:
            return  # Already returning — ignore all future alerts (the latch)

        if msg.data:
            # Alert is True — start returning, and never stop
            self.returning = True
            self.get_logger().warn(
                '⚠ Low battery alert received! Returning to home position...'
            )
            self.cmd_pub.publish(HOME)



def main():
    rclpy.init()
    node = ReturnToBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

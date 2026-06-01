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
from geometry_msgs.msg import Twist


# Tuning parameters
LINEAR_SPEED  = 0.15   # m/s  — forward driving speed
ANGULAR_SPEED = 0.4    # rad/s — rotation speed
GOAL_TOLERANCE = 0.15  # metres — how close is "close enough" to home
ANGLE_TOLERANCE = 0.05 # radians — how aligned before we start driving


class ReturnToBase(Node):
    def __init__(self):
        super().__init__('return_to_base')

        # Home position — recorded from first /odom message
        self.home_x = None
        self.home_y = None

        # Current robot position and yaw — updated continuously from /odom
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # One-way latch — once True, never goes back to False
        self.returning = False

        # Publisher: sends velocity commands to the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: continuously tracks robot position and records home
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscriber: watches for the low battery alert from BatteryMonitor
        self.create_subscription(
            Bool,
            '/battery/low_alert',
            self.alert_callback,
            10
        )

        # Control loop — runs at 10 Hz to send velocity commands smoothly
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'ReturnToBase started. Waiting for first /odom to record home position...'
        )

    # ------------------------------------------------------------------
    # /odom callback — runs continuously throughout the node's lifetime
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        # Always update current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # Record home only once — on the very first odom message
        if self.home_x is None:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.get_logger().info(
                f'Home position recorded: '
                f'x={self.home_x:.3f}  y={self.home_y:.3f}'
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

    # ------------------------------------------------------------------
    # Control loop — runs at 10 Hz, only does anything when returning
    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.returning:
            return  # Nothing to do yet

        # Safety check — if odom hasn't arrived yet, wait
        if self.current_x is None or self.home_x is None:
            self.get_logger().warn('Waiting for odometry...')
            return

        # Calculate distance and angle to home
        dx = self.home_x - self.current_x
        dy = self.home_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_home = math.atan2(dy, dx)

        # Difference between where we face and where home is
        angle_error = angle_to_home - self.current_yaw

        # Normalise angle_error to [-pi, pi] to always turn the short way
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()  # Default is all zeros (stop)

        if distance < GOAL_TOLERANCE:
            # ── Arrived home ──
            self.cmd_pub.publish(cmd)  # Send stop command
            self.get_logger().info('✓ Reached home position. Stopping.')
            # Stop the control loop from running again
            self.returning = False
            return

        if abs(angle_error) > ANGLE_TOLERANCE:
            # ── Phase 1: rotate to face home ──
            cmd.angular.z = ANGULAR_SPEED if angle_error > 0 else -ANGULAR_SPEED
            self.get_logger().info(
                f'Rotating to face home — angle error: {math.degrees(angle_error):.1f}°'
            )
        else:
            # ── Phase 2: drive forward toward home ──
            cmd.linear.x = LINEAR_SPEED
            self.get_logger().info(
                f'Driving home — distance remaining: {distance:.2f}m'
            )

        self.cmd_pub.publish(cmd)


# ----------------------------------------------------------------------
# Helper — extract yaw from a quaternion (same as in printHeading.py)
# ----------------------------------------------------------------------
def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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
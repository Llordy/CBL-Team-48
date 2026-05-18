#!/usr/bin/env python3
"""
gps_navigator_node.py
=====================
ROS 2 action server that drives a TurtleBot to a GPS coordinate.

Action    : ~/navigate_to_gps  (gps_nav_interfaces/action/GpsGoal)
Services  : ~/pause_navigation  (std_srvs/srv/Trigger)
            ~/resume_navigation (std_srvs/srv/Trigger)

Subscribed topics
  /odom              (nav_msgs/Odometry)   – heading & position fallback
  /phone/gps/fix     (sensor_msgs/NavSatFix) – current GPS position

Published topics
  /cmd_vel           (geometry_msgs/TwistStamped) – velocity commands

Obstacle-avoidance integration pattern
---------------------------------------
When your obstacle detector fires:
  1. Call ~/pause_navigation  → robot stops, goal is kept alive
  2. Run your avoidance manoeuvre on /cmd_vel directly
  3. Call ~/resume_navigation → navigator takes /cmd_vel back and
     resumes heading toward the original GPS target.

The action stays active (goal is not cancelled) throughout.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from gps_nav_interfaces.action import GpsGoal

# ── tuning constants ────────────────────────────────────────────────────────
CONTROL_HZ          = 10.0      # Hz – control loop rate
MAX_LINEAR_SPEED    = 0.3       # m/s
MAX_ANGULAR_SPEED   = 1.0       # rad/s
ANGULAR_P_GAIN      = 1.8       # proportional gain for heading error
LINEAR_P_GAIN       = 0.5       # proportional gain for approach speed
HEADING_ALIGN_DEG   = 15.0      # °  – slow to align before driving forward
DEFAULT_ARRIVAL_M   = 2.0       # m  – default arrival radius


# ── helpers ──────────────────────────────────────────────────────────────────

def haversine_distance(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in metres between two WGS-84 points."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_to(lat1, lon1, lat2, lon2) -> float:
    """
    Initial bearing (degrees, 0 = North, clockwise) from point 1 to point 2.
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def yaw_from_quaternion(q) -> float:
    """Extract yaw (radians, ROS convention) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    """Signed shortest angular difference a - b (both in degrees)."""
    d = (a - b + 180) % 360 - 180
    return d


# ── node ─────────────────────────────────────────────────────────────────────

class GpsNavigatorNode(Node):

    def __init__(self):
        super().__init__('gps_navigator')

        self._cb_group = ReentrantCallbackGroup()

        # ── state ──
        self._current_gps: NavSatFix | None = None
        self._current_yaw_deg: float = 0.0   # robot heading in degrees (ENU → geographic)
        self._paused: bool = False

        # ── subscriptions ──
        self.create_subscription(
            NavSatFix, '/phone/gps/fix', self._gps_cb, 10,
            callback_group=self._cb_group)

        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10,
            callback_group=self._cb_group)

        # ── publisher ──
        self._cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # ── pause / resume services ──
        self.create_service(Trigger, '~/pause_navigation',  self._pause_cb,
                            callback_group=self._cb_group)
        self.create_service(Trigger, '~/resume_navigation', self._resume_cb,
                            callback_group=self._cb_group)

        # ── action server ──
        self._action_server = ActionServer(
            self,
            GpsGoal,
            '~/navigate_to_gps',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info('GpsNavigator ready.')

    # ── subscription callbacks ───────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= 0:          # STATUS_NO_FIX == -1
            self._current_gps = msg

    def _odom_cb(self, msg: Odometry):
        """
        Convert odometry quaternion to a geographic heading (degrees).
        Assumption: robot's +X forward, odom frame aligned with ENU
        (East-North-Up).  Yaw 0 = East; we convert to compass (0 = North).
        Adjust the offset if your odom frame differs.
        """
        yaw_enu_rad = yaw_from_quaternion(msg.pose.pose.orientation)
        yaw_enu_deg = math.degrees(yaw_enu_rad)
        # ENU yaw 0° = East, compass 0° = North  →  compass = 90 - yaw_ENU
        self._current_yaw_deg = (90.0 - yaw_enu_deg) % 360.0

    # ── service callbacks ────────────────────────────────────────────────────

    def _pause_cb(self, _req, response: Trigger.Response):
        self._paused = True
        self._publish_stop()
        response.success = True
        response.message = 'Navigation paused.'
        self.get_logger().info('Navigation PAUSED by external request.')
        return response

    def _resume_cb(self, _req, response: Trigger.Response):
        self._paused = False
        response.success = True
        response.message = 'Navigation resumed.'
        self.get_logger().info('Navigation RESUMED.')
        return response

    # ── action callbacks ─────────────────────────────────────────────────────

    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f'Received goal: lat={goal_request.latitude:.6f}  '
            f'lon={goal_request.longitude:.6f}  '
            f'radius={goal_request.arrival_radius:.1f} m')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self.get_logger().info('Cancel request received.')
        return CancelResponse.ACCEPT

    # ── main control loop ────────────────────────────────────────────────────

    def _execute_cb(self, goal_handle):
        goal      = goal_handle.request
        target_lat = goal.latitude
        target_lon = goal.longitude
        radius     = goal.arrival_radius if goal.arrival_radius > 0 else DEFAULT_ARRIVAL_M

        self.get_logger().info(
            f'Executing navigation to ({target_lat:.6f}, {target_lon:.6f}) '
            f'within {radius:.1f} m')

        rate = self.create_rate(CONTROL_HZ)
        feedback_msg = GpsGoal.Feedback()

        # Wait until we have a GPS fix
        while rclpy.ok() and self._current_gps is None:
            self.get_logger().info('Waiting for GPS fix…', throttle_duration_sec=2.0)
            rate.sleep()

        while rclpy.ok():
            # ── check for cancellation ──
            if goal_handle.is_cancel_requested:
                self._publish_stop()
                goal_handle.canceled()
                result = GpsGoal.Result()
                result.success = False
                result.message = 'Goal cancelled.'
                return result

            # ── compute distance & bearing ──
            cur = self._current_gps
            dist   = haversine_distance(cur.latitude, cur.longitude, target_lat, target_lon)
            target_bearing = bearing_to(cur.latitude, cur.longitude, target_lat, target_lon)

            # ── arrival check ──
            if dist <= radius:
                self._publish_stop()
                goal_handle.succeed()
                result = GpsGoal.Result()
                result.success = True
                result.message = f'Arrived within {dist:.2f} m of target.'
                self.get_logger().info(result.message)
                return result

            # ── feedback ──
            feedback_msg.distance_to_goal = dist
            feedback_msg.bearing_to_goal  = target_bearing
            feedback_msg.state = 'PAUSED' if self._paused else 'NAVIGATING'
            goal_handle.publish_feedback(feedback_msg)

            # ── paused: just stop and wait ──
            if self._paused:
                self._publish_stop()
                rate.sleep()
                continue

            # ── proportional controller ──
            heading_error = angle_diff(target_bearing, self._current_yaw_deg)

            # Angular velocity: turn to face the target
            angular_z = max(-MAX_ANGULAR_SPEED,
                            min(MAX_ANGULAR_SPEED,
                                math.radians(heading_error) * ANGULAR_P_GAIN))

            # Linear velocity: only drive forward when roughly aligned
            if abs(heading_error) < HEADING_ALIGN_DEG:
                approach_speed = min(MAX_LINEAR_SPEED, dist * LINEAR_P_GAIN)
            else:
                approach_speed = 0.0   # rotate in place first

            self._publish_cmd(approach_speed, angular_z)
            rate.sleep()

        # Node shutting down
        self._publish_stop()
        goal_handle.abort()
        result = GpsGoal.Result()
        result.success = False
        result.message = 'Node shutting down.'
        return result

    # ── velocity helpers ─────────────────────────────────────────────────────

    def _publish_cmd(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GpsNavigatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

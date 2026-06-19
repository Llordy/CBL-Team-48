#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger


class AvoidanceState(Enum):
    IDLE = 0
    TURNING = 1
    PASSING = 2


class TwinSafetyNode(Node):
    def __init__(self):
        super().__init__('twin_avoidance_node')

        self.declare_parameter('real_scan_topic', '/scan')
        self.declare_parameter('sim_scan_topic', '/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('real_cmd_topic', '/cmd_vel')
        self.declare_parameter('sim_cmd_topic', '/cmd_vel')

        self.declare_parameter('pause_service', '/gps_navigator/pause_navigation')
        self.declare_parameter('resume_service', '/gps_navigator/resume_navigation')
        self.declare_parameter('use_nav_services', True)

        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('front_clear_distance', 0.50)
        self.declare_parameter('side_clear_distance', 0.55)

        self.declare_parameter('front_angle_deg', 30.0)
        self.declare_parameter('side_sector_center_deg', 90.0)
        self.declare_parameter('side_sector_half_width_deg', 35.0)

        self.declare_parameter('turn_speed', 0.7)
        self.declare_parameter('pass_speed', 0.12)
        self.declare_parameter('takeover_only_on_forward_motion', True)
        self.declare_parameter('use_min_for_side_score', True)

        self.real_scan_topic = self.get_parameter('real_scan_topic').value
        self.sim_scan_topic = self.get_parameter('sim_scan_topic').value
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.real_cmd_topic = self.get_parameter('real_cmd_topic').value
        self.sim_cmd_topic = self.get_parameter('sim_cmd_topic').value

        self.pause_service_name = self.get_parameter('pause_service').value
        self.resume_service_name = self.get_parameter('resume_service').value
        self.use_nav_services = bool(self.get_parameter('use_nav_services').value)

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.front_clear_distance = float(self.get_parameter('front_clear_distance').value)
        self.side_clear_distance = float(self.get_parameter('side_clear_distance').value)

        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.side_sector_center_deg = float(self.get_parameter('side_sector_center_deg').value)
        self.side_sector_half_width_deg = float(self.get_parameter('side_sector_half_width_deg').value)

        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.pass_speed = float(self.get_parameter('pass_speed').value)
        self.takeover_only_on_forward_motion = bool(self.get_parameter('takeover_only_on_forward_motion').value)
        self.use_min_for_side_score = bool(self.get_parameter('use_min_for_side_score').value)

        self.real_front_min = float('inf')
        self.real_left_score = float('inf')
        self.real_right_score = float('inf')
        self.real_blocked = False

        self.sim_front_min = float('inf')
        self.sim_left_score = float('inf')
        self.sim_right_score = float('inf')
        self.sim_blocked = False

        self.state = AvoidanceState.IDLE
        self.turn_direction = 1.0
        self.tracked_side = 'right'
        self.navigator_paused = False

        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(
            LaserScan,
            self.real_scan_topic,
            self.real_scan_cb,
            scan_qos
        )

        self.create_subscription(
            LaserScan,
            self.sim_scan_topic,
            self.sim_scan_cb,
            scan_qos
        )

        self.create_subscription(
            TwistStamped,
            self.input_cmd_topic,
            self.cmd_cb,
            10
        )

        self.real_pub = self.create_publisher(TwistStamped, self.real_cmd_topic, 10)
        self.sim_pub = self.create_publisher(TwistStamped, self.sim_cmd_topic, 10)

        self.pause_client = self.create_client(Trigger, self.pause_service_name)
        self.resume_client = self.create_client(Trigger, self.resume_service_name)

        self.get_logger().info('Twin Safety Node with turn-and-pass avoidance started')

    def real_scan_cb(self, msg: LaserScan):
        front_min, left_score, right_score, blocked = self.evaluate_scan(msg)
        self.real_front_min = front_min
        self.real_left_score = left_score
        self.real_right_score = right_score
        self.real_blocked = blocked

    def sim_scan_cb(self, msg: LaserScan):
        front_min, left_score, right_score, blocked = self.evaluate_scan(msg)
        self.sim_front_min = front_min
        self.sim_left_score = left_score
        self.sim_right_score = right_score
        self.sim_blocked = blocked

    def evaluate_scan(self, msg: LaserScan) -> Tuple[float, float, float, bool]:
        front_ranges = self.get_sector_distances(msg, 0.0, self.front_angle_deg)
        left_ranges = self.get_sector_distances(msg, self.side_sector_center_deg, self.side_sector_half_width_deg)
        right_ranges = self.get_sector_distances(msg, -self.side_sector_center_deg, self.side_sector_half_width_deg)

        front_min = min(front_ranges) if front_ranges else float('inf')
        left_score = self.compute_side_score(left_ranges)
        right_score = self.compute_side_score(right_ranges)
        blocked = front_min < self.stop_distance

        return front_min, left_score, right_score, blocked

    def get_sector_distances(self, scan_msg: LaserScan, center_deg: float, half_width_deg: float) -> List[float]:
        center_rad = math.radians(center_deg)
        half_width_rad = math.radians(half_width_deg)
        selected = []

        for i, distance in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            diff = angle - center_rad
            diff = math.atan2(math.sin(diff), math.cos(diff))

            if abs(diff) <= half_width_rad:
                if math.isfinite(distance) and scan_msg.range_min < distance < scan_msg.range_max:
                    selected.append(distance)

        return selected

    def compute_side_score(self, distances: List[float]) -> float:
        if not distances:
            return 0.0
        if self.use_min_for_side_score:
            return min(distances)
        return sum(distances) / len(distances)

    def blocked_now(self) -> bool:
        return self.real_blocked or self.sim_blocked

    def conservative_front_min(self) -> float:
        return min(self.real_front_min, self.sim_front_min)

    def conservative_left_score(self) -> float:
        return min(self.real_left_score, self.sim_left_score)

    def conservative_right_score(self) -> float:
        return min(self.real_right_score, self.sim_right_score)

    def front_clear(self) -> bool:
        return self.conservative_front_min() > self.front_clear_distance

    def tracked_side_clear(self) -> bool:
        if self.tracked_side == 'right':
            return self.conservative_right_score() > self.side_clear_distance
        return self.conservative_left_score() > self.side_clear_distance

    def choose_turn_direction(self) -> float:
        left_clearance = self.conservative_left_score()
        right_clearance = self.conservative_right_score()

        if left_clearance > right_clearance:
            return 1.0
        if right_clearance > left_clearance:
            return -1.0
        return 1.0

    def set_turn_plan(self):
        self.turn_direction = self.choose_turn_direction()

        if self.turn_direction > 0.0:
            self.tracked_side = 'right'
        else:
            self.tracked_side = 'left'

    def build_turn_cmd(self, header) -> TwistStamped:
        cmd = TwistStamped()
        cmd.header = header
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = self.turn_direction * self.turn_speed
        return cmd

    def build_pass_cmd(self, header) -> TwistStamped:
        cmd = TwistStamped()
        cmd.header = header
        cmd.twist.linear.x = self.pass_speed
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        return cmd

    def call_trigger_sync(self, client, label: str) -> bool:
        if not self.use_nav_services:
            return True

        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'{label} service not available')
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.5)

        if not future.done() or future.result() is None:
            self.get_logger().warn(f'{label} service call failed or timed out')
            return False

        response = future.result()
        self.get_logger().info(f'{label}: success={response.success} message="{response.message}"')
        return response.success

    def pause_navigator(self):
        if not self.navigator_paused:
            ok = self.call_trigger_sync(self.pause_client, 'pause_navigation')
            if ok:
                self.navigator_paused = True

    def resume_navigator(self):
        if self.navigator_paused:
            ok = self.call_trigger_sync(self.resume_client, 'resume_navigation')
            if ok:
                self.navigator_paused = False

    def publish_both(self, msg: TwistStamped):
        self.real_pub.publish(msg)
        self.sim_pub.publish(msg)

    def cmd_cb(self, msg: TwistStamped):
        forward_requested = msg.twist.linear.x > 0.0

        self.get_logger().info(
            f'state={self.state.name} '
            f'real_blocked={self.real_blocked} sim_blocked={self.sim_blocked} '
            f'front={self.conservative_front_min():.2f} '
            f'left={self.conservative_left_score():.2f} '
            f'right={self.conservative_right_score():.2f} '
            f'tracked_side={self.tracked_side} '
            f'in_lin_x={msg.twist.linear.x:.2f} '
            f'in_ang_z={msg.twist.angular.z:.2f}'
        )

        if self.state == AvoidanceState.IDLE:
            if self.blocked_now() and (forward_requested or not self.takeover_only_on_forward_motion):
                self.pause_navigator()
                self.set_turn_plan()
                self.state = AvoidanceState.TURNING

                cmd = self.build_turn_cmd(msg.header)
                self.get_logger().warn(
                    f'Obstacle detected: entering TURNING, rotating '
                    f'{"LEFT" if self.turn_direction > 0.0 else "RIGHT"}'
                )
            else:
                cmd = msg

        elif self.state == AvoidanceState.TURNING:
            if self.front_clear():
                self.state = AvoidanceState.PASSING
                cmd = self.build_pass_cmd(msg.header)
                self.get_logger().warn(
                    f'Front is clear: entering PASSING, moving forward while tracking obstacle on {self.tracked_side}'
                )
            else:
                cmd = self.build_turn_cmd(msg.header)

        elif self.state == AvoidanceState.PASSING:
            if self.tracked_side_clear():
                self.state = AvoidanceState.IDLE
                self.resume_navigator()
                cmd = msg
                self.get_logger().info('Obstacle passed completely: returning control to upstream navigator/teleop')
            else:
                cmd = self.build_pass_cmd(msg.header)
                self.get_logger().warn(
                    f'Passing obstacle: continuing forward, waiting for {self.tracked_side} side to clear'
                )

        else:
            cmd = msg
            self.state = AvoidanceState.IDLE

        self.publish_both(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TwinSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

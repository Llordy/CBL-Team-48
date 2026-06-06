#!/usr/bin/env python3
"""
MPNavNode.py
Two-process GPS navigation node.

  SensorProcess  – ROS2 node, runs rclpy.spin(), owns all subscriptions.
  ControlProcess – plain Python loop (time.sleep), owns the cmd_vel publisher.

State is exchanged via a multiprocessing.shared_memory block that is created
by the parent before either child starts, so neither child needs to race for it.

Shared-memory layout  struct "13d?" (105 bytes):
  [0]  gps_lat
  [1]  gps_lon
  [2]  gps_odom_x        odom position snapshot at last GPS fix
  [3]  gps_odom_y
  [4]  odom_x
  [5]  odom_y
  [6]  odom_heading      degrees, ROS yaw convention
  [7]  compass_heading   degrees, from /heading
  [8]  odom_at_compass   odom_heading captured when /heading arrived
  [9]  goal_lat
  [10] goal_lon
  [11] pad
  [12] pad
  [13] has_goal          bool
"""

import math
import struct
import threading
import time
import multiprocessing
from multiprocessing.shared_memory import SharedMemory

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

# ─────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────
CONTROL_HZ        = 10
ARRIVAL_RADIUS    = 0.5
CONTROL_TOPIC     = "/cmd_vel"
GOAL_TOPIC        = "/nav/goal"

MAX_LINEAR        = 0.22
LINEAR_GAIN       = 1.0
MAX_ANGULAR       = 1.0
ANGULAR_GAIN      = 0.5
HEADING_THRESHOLD = 15

RADIUS        = 6371008.7714
CIRCUM        = 2 * math.pi * RADIUS
METERS_PER_DEG = CIRCUM / 360

SHM_NAME   = "navnode_state"
SHM_FORMAT = "13d?"
SHM_SIZE   = struct.calcsize(SHM_FORMAT)

# ─────────────────────────────────────────
#  Shared helpers
# ─────────────────────────────────────────
clamp = lambda lo, hi, x: min(hi, max(lo, x))

def heading_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return (720 - math.degrees(math.atan2(siny_cosp, cosy_cosp))) % 360

def offset_position(lat, lon, north, east):
    dlat = north / METERS_PER_DEG
    dlon = east  / (METERS_PER_DEG * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

def rotate(angle_rad, x, y):
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return c * x - s * y, s * x + c * y

def bearing_to(lat1, lon1, lat2, lon2):
    rl1, ro1, rl2, ro2 = map(math.radians, (lat1, lon1, lat2, lon2))
    x = math.cos(rl2) * math.sin(ro2 - ro1)
    y = math.cos(rl1) * math.sin(rl2) - math.sin(rl1) * math.cos(rl2) * math.cos(ro2 - ro1)
    return (720 + math.degrees(math.atan2(x, y))) % 360

def gps_distance(lat1, lon1, lat2, lon2):
    p1, o1, p2, o2 = map(math.radians, (lat1, lon1, lat2, lon2))
    hav = 0.5 * (1 - math.cos(p2 - p1) + math.cos(p1) * math.cos(p2) * (1 - math.cos(o2 - o1)))
    return 2 * math.asin(math.sqrt(hav)) * RADIUS


# ─────────────────────────────────────────
#  Sensor process
# ─────────────────────────────────────────
class SensorNode(Node):
    """ROS2 node: subscribes to everything, writes shared memory."""

    def __init__(self, shm: SharedMemory):
        super().__init__('sensor_node')
        self.log   = self.get_logger()
        self._shm  = shm
        self._lock = threading.Lock()

        self._gps_lat         = 0.0
        self._gps_lon         = 0.0
        self._gps_odom_x      = 0.0
        self._gps_odom_y      = 0.0
        self._odom_x          = 0.0
        self._odom_y          = 0.0
        self._odom_heading    = 0.0
        self._compass_heading = 0.0
        self._odom_at_compass = 0.0
        self._goal_lat        = 0.0
        self._goal_lon        = 0.0
        self._has_goal        = False

        self.create_subscription(NavSatFix, '/phone/gps/fix', self._gps_cb,     10)
        self.create_subscription(Odometry,  '/odom',          self._odom_cb,    10)
        self.create_subscription(Float32,   '/heading',       self._heading_cb, 10)
        self.create_subscription(Point,     GOAL_TOPIC,       self._goal_cb,    10)

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        with self._lock:
            self._gps_lat    = msg.latitude
            self._gps_lon    = msg.longitude
            self._gps_odom_x = self._odom_x
            self._gps_odom_y = self._odom_y
        self._write()

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        with self._lock:
            self._odom_x       = pos.x
            self._odom_y       = pos.y
            self._odom_heading = heading_from_quaternion(msg.pose.pose.orientation)
        self._write()

    def _heading_cb(self, msg: Float32):
        with self._lock:
            self._compass_heading = float(msg.data)
            self._odom_at_compass = self._odom_heading
        self._write()

    def _goal_cb(self, msg: Point):
        self.log.warn(f"New goal: {msg.x}, {msg.y}")
        with self._lock:
            self._goal_lat = msg.x
            self._goal_lon = msg.y
            self._has_goal = True
        self._write()

    def _write(self):
        with self._lock:
            data = struct.pack(
                SHM_FORMAT,
                self._gps_lat, self._gps_lon,
                self._gps_odom_x, self._gps_odom_y,
                self._odom_x, self._odom_y,
                self._odom_heading, self._compass_heading, self._odom_at_compass,
                self._goal_lat, self._goal_lon,
                0.0, 0.0,       # padding
                self._has_goal,
            )
        self._shm.buf[:SHM_SIZE] = data


class SensorProcess(multiprocessing.Process):
    def __init__(self, shm_name: str):
        super().__init__(name="SensorProcess", daemon=True)
        self._shm_name = shm_name

    def run(self):
        shm = SharedMemory(name=self._shm_name, create=False, size=SHM_SIZE)
        rclpy.init()
        node = SensorNode(shm)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
            shm.close()


# ─────────────────────────────────────────
#  Control process
# ─────────────────────────────────────────
class ControlNode(Node):
    """Minimal ROS2 node: only owns the publisher and clock. Never spun."""

    def __init__(self):
        super().__init__('control_node')
        self._pub = self.create_publisher(TwistStamped, CONTROL_TOPIC, 10)

    def publish(self, forward: float, turn: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x  = float(forward)
        msg.twist.angular.z = float(turn)
        self._pub.publish(msg)


class ControlProcess(multiprocessing.Process):
    def __init__(self, shm_name: str):
        super().__init__(name="ControlProcess", daemon=True)
        self._shm_name = shm_name

    def run(self):
        shm = SharedMemory(name=self._shm_name, create=False, size=SHM_SIZE)
        rclpy.init()
        node = ControlNode()
        log  = node.get_logger()

        interval = 1.0 / CONTROL_HZ
        try:
            while rclpy.ok():
                t0 = time.monotonic()
                self._tick(shm, node, log)
                elapsed = time.monotonic() - t0
                time.sleep(max(0.0, interval - elapsed))
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
            shm.close()

    @staticmethod
    def _tick(shm: SharedMemory, node: ControlNode, log):
        f = struct.unpack_from(SHM_FORMAT, shm.buf)
        gps_lat, gps_lon       = f[0],  f[1]
        gps_odom_x, gps_odom_y = f[2],  f[3]
        odom_x, odom_y         = f[4],  f[5]
        odom_heading           = f[6]
        compass_heading        = f[7]
        odom_at_compass        = f[8]
        goal_lat, goal_lon     = f[9],  f[10]
        has_goal               = f[13]

        if not has_goal:
            log.info("No goal")
            return

        # fuse GPS + odometry dead-reckoning
        pos_offset = (odom_x - gps_odom_x, -(odom_y - gps_odom_y))
        heading_err = compass_heading - odom_at_compass
        ne_offset   = rotate(math.radians(heading_err), *pos_offset)

        est_pos     = offset_position(gps_lat, gps_lon, *ne_offset)
        est_heading = (720 + odom_heading - heading_err) % 360

        distance = gps_distance(*est_pos, goal_lat, goal_lon)
        if distance < ARRIVAL_RADIUS:
            # clear goal in-place
            patched = list(f)
            patched[13] = False
            shm.buf[:SHM_SIZE] = struct.pack(SHM_FORMAT, *patched)
            node.publish(0.0, 0.0)
            log.info("Goal reached.")
            return

        log.info(f"pos={est_pos}  heading={est_heading:.1f}°")

        target  = bearing_to(*est_pos, goal_lat, goal_lon)
        err     = target - est_heading
        if   err >  180: err -= 360
        elif err < -180: err += 360
        log.info(f"target={target:.1f}°  error={err:.1f}°")

        angular = clamp(-MAX_ANGULAR, MAX_ANGULAR, -ANGULAR_GAIN * err)
        linear  = clamp(-MAX_LINEAR,  MAX_LINEAR,   LINEAR_GAIN  * distance)
        if abs(err) > HEADING_THRESHOLD:
            log.info("rotating to target, no forward")
            linear = 0.0

        log.info(f"linear={linear:.3f}  angular={angular:.3f}")
        node.publish(linear, angular)


# ─────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────
def main():
    # Parent creates the shared block; children only open it.
    shm = SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    # Zero-initialise so control process sees has_goal=False before first write.
    shm.buf[:SHM_SIZE] = bytes(SHM_SIZE)

    sensor  = SensorProcess(SHM_NAME)
    control = ControlProcess(SHM_NAME)

    sensor.start()
    control.start()

    try:
        sensor.join()
        control.join()
    except KeyboardInterrupt:
        pass
    finally:
        sensor.terminate()
        control.terminate()
        sensor.join()
        control.join()
        shm.close()
        shm.unlink()


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')   # safe with ROS2 / rclpy
    main()

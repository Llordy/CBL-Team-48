#!/usr/bin/env python3
"""
sensor_node.py
GPS navigation sensor node managing ROS2 interactions and shared memory updates.
"""

import math
import struct
import threading
import multiprocessing
from multiprocessing.shared_memory import SharedMemory

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

# ─────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────
GOAL_TOPIC  = "/nav/goal"

RADIUS         = 6371008.7714
CIRCUM         = 2 * math.pi * RADIUS
METERS_PER_DEG = CIRCUM / 360

SHM_NAME   = "navnode_state"
SHM_FORMAT = "13d?2d?"
SHM_SIZE   = struct.calcsize(SHM_FORMAT)

def heading_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return (720 - math.degrees(math.atan2(siny_cosp, cosy_cosp))) % 360


class SensorNode(Node):
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

        # Create ROS2 subscriptions
        self.create_subscription(NavSatFix, '/phone/gps/fix', self._gps_cb,     10)
        self.create_subscription(Odometry,  '/odom',          self._odom_cb,    10)
        self.create_subscription(Float32,   '/heading',       self._heading_cb, 10)
        self.create_subscription(Point,     GOAL_TOPIC,       self._goal_cb,    10)

        # Publisher moved strictly here
        self._goal_pub = self.create_publisher(Point, GOAL_TOPIC, 10)

        # Polling timer to check for incoming dashboard goal commands in SHM
        self.create_timer(0.1, self._check_dashboard_commands)

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
        self.log.warn(f"New goal received on topic: {msg.x}, {msg.y}")
        with self._lock:
            self._goal_lat = msg.x
            self._goal_lon = msg.y
            self._has_goal = True
        self._write()

    def _check_dashboard_commands(self):
        """Checks if the dashboard wrote a command trigger into SHM."""
        try:
            f = list(struct.unpack_from(SHM_FORMAT, self._shm.buf))
            cmd_trigger = f[16] # Index 16 matches the last boolean flag
            
            if cmd_trigger:
                cmd_lat = f[14]
                cmd_lon = f[15]
                
                self.log.info(f"Dashboard command caught in SHM! Publishing to {GOAL_TOPIC}: {cmd_lat}, {cmd_lon}")
                
                # Publish to ROS network
                msg = Point()
                msg.x = cmd_lat
                msg.y = cmd_lon
                self._goal_pub.publish(msg)
                
                # Clear trigger flag down in SHM
                f[16] = False
                with self._lock:
                    self._shm.buf[:SHM_SIZE] = struct.pack(SHM_FORMAT, *f)
        except Exception as e:
            self.log.error(f"Error checking dashboard commands: {e}")

    def _write(self):
        with self._lock:
            # We preserve existing cmd entries during internal routine updates
            current_shm = struct.unpack_from(SHM_FORMAT, self._shm.buf)
            cmd_lat, cmd_lon, cmd_trig = current_shm[14], current_shm[15], current_shm[16]

            data = struct.pack(
                SHM_FORMAT,
                self._gps_lat, self._gps_lon,
                self._gps_odom_x, self._gps_odom_y,
                self._odom_x, self._odom_y,
                self._odom_heading, self._compass_heading, self._odom_at_compass,
                self._goal_lat, self._goal_lon,
                0.0, 0.0,
                self._has_goal,
                cmd_lat, cmd_lon, cmd_trig
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


def main():
    # Primary setup process holding lifecycle management of SHM
    shm = SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    shm.buf[:SHM_SIZE] = bytes(SHM_SIZE)

    sensor = SensorProcess(SHM_NAME)
    sensor.start()

    print("[Sensor Node] Running. Waiting for lifecycle termination signals...")
    try:
        sensor.join()
    except KeyboardInterrupt:
        pass
    finally:
        sensor.terminate()
        sensor.join()
        shm.close()
        shm.unlink()


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')
    main()

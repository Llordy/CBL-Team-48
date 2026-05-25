
#!/usr/bin/env python3
import json
import time
from typing import ForwardRef
import urllib.request
import threading
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry


class GpsNode(Node):
    def __init__(self):
        super().__init__('phone_gps')
        self.pub = self.create_publisher(NavSatFix, '/phone/gps/fix', 10)
        self.create_subscription(Odometry, '/odom',self.callback, 10)

    def publish(self, data: dict):

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude  = data['lat']
        msg.longitude = data['lon']
        msg.altitude  = data['alt']
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub.publish(msg)
        self.get_logger().info(
            f"lat: {data['lat']:.6f}  lon: {data['lon']:.6f}  "
            f"alt: {data['alt']:.1f}m  heading: {data['heading']:.1f}°"
        )

    def callback(self,msg: Odometry):
        SCALE = 0.01
        VSCALE = 1
        
        # x - forward
        # y - left
        # z - up
        pos = msg.pose.pose.position;
        msg = {
                'lat': pos.x * SCALE,
                'lon': -pos.y * SCALE,
                'alt': pos.z * VSCALE, 
                'heading': 0.
                }
        self.publish(msg)




rclpy.init()
node = GpsNode()
rclpy.spin(node)


#!/usr/bin/env python3
import json
from math import isnan
import time
import urllib.request
import threading
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

parser = argparse.ArgumentParser()
parser.add_argument('--relay', required=True, help='Relay server URL, e.g. http://192.168.1.50:8080')
parser.add_argument('--rate', type=float, default=1.0, help='Poll rate in Hz')
parser.add_argument('--token', required=True, help="Secret token for api access")
args = parser.parse_args()

class GpsNode(Node):
    def __init__(self):
        super().__init__('phone_gps')
        self.pub = self.create_publisher(NavSatFix, '/phone/gps/fix', 10)
        self.head_pub = self.create_publisher(Float32, '/heading', 10)

    def publish(self, data: dict):
        status = data.get('status', 'Disconnected')
        if status != 'Fix':
            self.get_logger().info(f'Status: {status}, not publishing')
            return

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
        if not isnan(data['heading']): self.head_pub.publish(data['heading'])
        self.get_logger().info(
            f"lat: {data['lat']:.6f}  lon: {data['lon']:.6f}  "
            f"alt: {data['alt']:.1f}m  heading: {data['heading']:.1f}°"
        )

rclpy.init()
node = GpsNode()
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

url = f"{args.relay.rstrip('/')}/gps"
interval = 1.0 / args.rate

print(f"Polling {url} at {args.rate} Hz...")
while True:
    try:
        req = urllib.request.Request(
            url,
            headers={
                'X-GPS-Token': args.token,
                'Accept': 'application/json',
            },
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode())
            node.publish(data)
    except Exception as e:
        node.get_logger().warn(f"Poll failed: {e}")
    time.sleep(interval)

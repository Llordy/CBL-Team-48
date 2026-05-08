#!/usr/bin/env python3
import json
import socket
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

class GpsNode(Node):
    def __init__(self):
        super().__init__('phone_gps')
        self.pub = self.create_publisher(NavSatFix, '/phone/gps/fix', 10)

    def publish(self, lat, lon, alt, accuracy):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.position_covariance[0] = accuracy ** 2
        msg.position_covariance[4] = accuracy ** 2
        msg.position_covariance[8] = accuracy ** 2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)

rclpy.init()
node = GpsNode()

threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5005))

print("GPS server listening on UDP port 5005...")
while True:
    data, _ = sock.recvfrom(1024)
    try:
        d = json.loads(data.decode())
        node.publish(
            d['latitude'],
            d['longitude'],
            d.get('altitude', 0.0),
            d.get('accuracy', 10.0),
        )
    except Exception as e:
        print(f"Bad packet: {e}")

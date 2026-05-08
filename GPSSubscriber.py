#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.create_subscription(NavSatFix, '/phone/gps/fix', self.callback, 10)
        self.get_logger().info('Subscribed to /phone/gps/fix, waiting for messages...')

    def callback(self, msg):
        self.get_logger().info(
            f'lat: {msg.latitude:.6f}  '
            f'lon: {msg.longitude:.6f}  '
            f'alt: {msg.altitude:.1f}m  '
            f'accuracy: {msg.position_covariance[0]**0.5:.1f}m'
        )

def main():
    rclpy.init()
    node = GpsSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

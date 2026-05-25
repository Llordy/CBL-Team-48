
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

def yaw_from_quaternion(q) -> float:
    """Extract yaw (radians, ROS convention) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def heading_from_quaternion(q) -> float:
    yaw = yaw_from_quaternion(q)
    return (720 - math.degrees(yaw)) % 360
    

class HeadingSubscriber(Node):
    def __init__(self):
        super().__init__("Heading_printer")
        self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.get_logger().info('Subscribed to /odom, waiting for messages...')

    def callback(self, msg):
        self.get_logger().info(
            f'heading: {heading_from_quaternion(msg.pose.pose.orientation)}'
        )

def main():
    rclpy.init()
    node = HeadingSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
FakeBatteryPublisher — waits 10s, then starts publishing 
battery level 19% (to trigger emergency return to base).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

# Voltage constants matching BatteryMonitor.py
BATTERY_MAX_VOLTAGE = 12.6
BATTERY_MIN_VOLTAGE =  9.5

VOLTAGE_BELOW_THRESHOLD = 9.5 + 0.19 * (12.6 - 9.5)  # 19% →  ~9.99V
DELAY_SECONDS = 10.0  # How long to wait before switching to low battery


class FakeBatteryPublisher(Node):
    def __init__(self):
        super().__init__('fake_battery_publisher')

        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.low = False  

        # Timer that fires every second to publish battery state
        self.create_timer(1.0, self.publish_callback)

        # One-shot timer that flips the state after DELAY_SECONDS
        self.create_timer(DELAY_SECONDS, self.trigger_low_battery)

        self.get_logger().info(
            f'FakeBatteryPublisher started. '
            f'Silent for {DELAY_SECONDS:.0f}s, then publishing 19% to trigger alert.'
        )

    def publish_callback(self):
        if not self.low:
            return  # Don't publish anything yet — let real OpenCR data through

        voltage = VOLTAGE_BELOW_THRESHOLD
        percentage = (voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = voltage
        msg.percentage = percentage
        self.pub.publish(msg)

        self.get_logger().info(
            f'Publishing fake low battery: {voltage:.2f}V  |  {percentage*100:.1f}%'
    )
        
    def trigger_low_battery(self):
        self.low = True
        self.get_logger().warn('Switching to low battery state (19%)...')



def main():
    rclpy.init()
    node = FakeBatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
BatteryMonitor node — digital twin battery state synchronization.

Subscribes to the physical robot's /battery_state topic, re-publishes
a clean voltage, estimated percentage, and a low-battery alert flag
on separate topics owned by the digital twin.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState  # Standard message from OpenCR
from std_msgs.msg import Float32, Bool    # Simple types for our output topics


# --- Turtlebot 3 Burger LiPo constants (3-cell, 11.1V nominal) ---
BATTERY_MAX_VOLTAGE = 12.6   # Volts — fully charged
BATTERY_MIN_VOLTAGE =  9.5   # Volts — considered empty / unsafe
BATTERY_LOW_THRESHOLD = 0.20 # Alert when below 20%


def voltage_to_percentage(voltage: float) -> float:
    #Linearly maps voltage to a 0.0–1.0 percentage.
    pct = (voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)
    return max(0.0, min(1.0, pct))


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # --- Subscriber: listen to the physical robot ---
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        # --- Publishers: the digital twin's own battery state topics ---
        self.pub_voltage = self.create_publisher(Float32, '/battery/voltage', 10)
        self.pub_percentage = self.create_publisher(Float32, '/battery/percentage', 10)
        self.pub_low_alert = self.create_publisher(Bool, '/battery/low_alert', 10)

        self.get_logger().info(
            'BatteryMonitor started. Waiting for /battery_state from robot...'
        )

    def battery_callback(self, msg: BatteryState):
        voltage = msg.voltage

        # Compute percentage from voltage (OpenCR may not populate msg.percentage)
        percentage = voltage_to_percentage(voltage)
        is_low = percentage < BATTERY_LOW_THRESHOLD

        # --- Publish to digital twin topics ---
        v_msg = Float32()
        v_msg.data = voltage
        self.pub_voltage.publish(v_msg)

        p_msg = Float32()
        p_msg.data = percentage
        self.pub_percentage.publish(p_msg)

        a_msg = Bool()
        a_msg.data = is_low
        self.pub_low_alert.publish(a_msg)

        # Log clearly so you can see state sync happening in the terminal
        status = 'LOW BATTERY' if is_low else 'BATTERY OK'
        self.get_logger().info(
            f'Battery — {voltage:.2f}V  |  {percentage*100:.1f}%  |  {status}'
        )


def main():
    rclpy.init()
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# FILE: scripts/test_battery_failure.py
# Updated to work with direct PX4 connection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import BatteryStatus
import time

class BatteryFailureSimulator(Node):
    def __init__(self):
        super().__init__('battery_failure_simulator')
        
        # QoS profile for PX4
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publisher for battery status
        self.battery_pub = self.create_publisher(
            BatteryStatus,
            '/fmu/out/battery_status',
            qos_profile=px4_qos
        )
        
        # Timer to simulate battery drain
        self.timer = self.create_timer(1.0, self.simulate_battery_drain)
        
        # Simulation parameters
        self.initial_battery = 100.0
        self.current_battery = 100.0
        self.drain_rate = 2.0  # Percent per second (fast drain for testing)
        
        self.get_logger().info('Battery failure simulator started')
        
    def simulate_battery_drain(self):
        """Simulate rapid battery drain"""
        if self.current_battery > 0:
            self.current_battery -= self.drain_rate
            if self.current_battery < 0:
                self.current_battery = 0
                
        # Create battery status message
        msg = BatteryStatus()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        msg.voltage_v = 14.8 * (self.current_battery / 100.0)  # Simulate voltage drop
        msg.current_a = 5.0  # Constant current draw
        msg.remaining = self.current_battery / 100.0
        msg.capacity = 5000  # mAh
        
        self.battery_pub.publish(msg)
        
        self.get_logger().info(f'Battery: {self.current_battery:.1f}%, Voltage: {msg.voltage_v:.1f}V')
        
        if self.current_battery <= 0:
            self.get_logger().warn('Battery depleted - stopping simulation')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = BatteryFailureSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

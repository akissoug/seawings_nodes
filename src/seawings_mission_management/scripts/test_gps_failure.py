#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import SensorGps
import time

class GpsFailureSimulator(Node):
    def __init__(self):
        super().__init__('gps_failure_simulator')
        
        # QoS profile for PX4
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publisher for GPS position
        self.gps_pub = self.create_publisher(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            qos_profile=px4_qos
        )
        
        # Timer to simulate GPS failure
        self.timer = self.create_timer(1.0, self.simulate_gps_failure)
        
        # Simulation parameters
        self.simulation_time = 0
        self.failure_start_time = 30  # Start failure after 30 seconds
        self.failure_duration = 20    # Failure lasts 20 seconds
        
        self.get_logger().info('GPS failure simulator started')
        
    def simulate_gps_failure(self):
        """Simulate GPS failure after certain time"""
        self.simulation_time += 1
        
        # Create GPS message
        msg = SensorGps()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.device_id = 0
        
        # Simulate GPS failure window
        if (self.failure_start_time <= self.simulation_time <= 
            self.failure_start_time + self.failure_duration):
            # GPS failure - no fix
            msg.fix_type = 0  # No fix
            msg.satellites_used = 0
            msg.lat = 0
            msg.lon = 0
            msg.alt = 0
            msg.hdop = 99.99
            msg.vdop = 99.99
            status = "GPS FAILURE"
        else:
            # Normal GPS operation
            msg.fix_type = 3  # 3D fix
            msg.satellites_used = 8
            msg.lat = int(47.397742 * 1e7)  # Example coordinates
            msg.lon = int(8.545594 * 1e7)
            msg.alt = int(488.0 * 1e3)
            msg.hdop = 1.2
            msg.vdop = 1.5
            status = "GPS OK"
            
        self.gps_pub.publish(msg)
        
        self.get_logger().info(f'Time: {self.simulation_time}s, Status: {status}, '
                              f'Satellites: {msg.satellites_used}, Fix: {msg.fix_type}')

def main(args=None):
    rclpy.init(args=args)
    node = GpsFailureSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
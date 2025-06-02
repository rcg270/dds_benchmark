#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import BatteryInfo
import argparse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class MinimalMonitor(Node):
    def __init__(self, topic_name, msg_type_str, duration):
        super().__init__('minimal_monitor')
        self.topic_name = topic_name
        self.duration = float(duration)
        self.msg_type_str = msg_type_str
        self.start_time = self.get_clock().now().to_sec()
        self.end_time = self.start_time + self.duration
        self.count = 0

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        try:
            pkg, msg = self.msg_type_str.split('/msg/')
            module = __import__(f'{pkg}.msg', fromlist=[msg])
            self.msg_type = getattr(module, msg)
            self.subscription = self.create_subscription(
                self.msg_type,
                self.topic_name,
                self.listener_callback,
                qos_profile)
            self.get_logger().info(f"Monitoring '{topic_name}' (type: '{self.msg_type_str}') for {duration} seconds...")
        except ImportError as e:
            self.get_logger().error(f"Could not import message type '{self.msg_type_str}': {e}")
            exit(1)
        except AttributeError as e:
            self.get_logger().error(f"Message type not found: {e}")
            exit(1)
        except ValueError as e:
            self.get_logger().error(f"Invalid message type format: {e}")
            exit(1)

    def listener_callback(self, msg):
        self.count += 1
        self.get_logger().info(f'I heard: {msg}')
        if self.get_clock().now().to_sec() > self.end_time:
            self.get_logger().info(f'Received {self.count} messages in {self.duration} seconds.')
            raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Minimal ROS 2 Topic Monitor')
    parser.add_argument('topic', help='Topic name to monitor')
    parser.add_argument('--duration', type=float, default=10.0, help='Monitoring duration in seconds')
    parser.add_argument('--msg-type', required=True, help='Message type')
    args = parser.parse_args()

    minimal_monitor = MinimalMonitor(args.topic, args.msg_type, args.duration)
    try:
        rclpy.spin(minimal_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
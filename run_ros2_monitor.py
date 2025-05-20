#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import numpy as np
import psutil
import os
from std_msgs.msg import Header  # Adjust based on your message type


class ROS2Monitor(Node):
    def __init__(self, topic_name, msg_type, duration=10):
        super().__init__('ros2_monitor')
        self.topic_name = topic_name
        self.duration = duration
        self.start_time = time.time()
        self.end_time = self.start_time + duration
        self.latencies = []
        self.msg_sizes = []
        self.count = 0
        self.process = psutil.Process(os.getpid())

        # QoS profile
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.callback,
            qos_profile)

        # Periodic reporting
        self.timer = self.create_timer(1.0, self.periodic_report)
        self.get_logger().info(f"Monitoring {topic_name} for {duration} seconds...")

    def callback(self, msg):
        now = time.time()
        self.count += 1

        # Calculate latency if message has a header with timestamp
        if hasattr(msg, 'header') and isinstance(msg.header, Header):
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            latency = now - msg_time
            self.latencies.append(latency)

        # Record message size (approximate)
        try:
            self.msg_sizes.append(len(msg_to_bytes(msg)))  # Need custom serialization
        except Exception:
            pass

        # Check if duration has elapsed
        if time.time() > self.end_time:
            self.final_report()
            raise KeyboardInterrupt  # Graceful exit

    def periodic_report(self):
        cpu = self.process.cpu_percent()
        mem = self.process.memory_info().rss / (1024 * 1024)  # MB
        elapsed = time.time() - self.start_time
        hz = self.count / elapsed if elapsed > 0 else 0

        self.get_logger().info(
            f"[Interim] Msgs: {self.count} | Rate: {hz:.2f} Hz | "
            f"CPU: {cpu:.1f}% | MEM: {mem:.2f} MB"
        )

    def final_report(self):
        elapsed = time.time() - self.start_time
        hz = self.count / elapsed if elapsed > 0 else 0

        # Calculate bandwidth if we captured message sizes
        bw = (sum(self.msg_sizes) / (1024 * 1024)) / elapsed if self.msg_sizes and elapsed > 0 else 0

        # Latency statistics
        latencies_ms = np.array(self.latencies) * 1000  # convert to milliseconds
        latency_stats = {
            'avg': np.mean(latencies_ms) if self.latencies else 0,
            'min': np.min(latencies_ms) if self.latencies else 0,
            'max': np.max(latencies_ms) if self.latencies else 0,
            'std': np.std(latencies_ms) if self.latencies else 0
        }

        self.get_logger().info(
            f"[Final] Samples: {self.count} | "
            f"Avg: {latency_stats['avg']:.3f} ms | "
            f"Min: {latency_stats['min']:.3f} ms | "
            f"Max: {latency_stats['max']:.3f} ms | "
            f"Std: {latency_stats['std']:.3f} ms | "
            f"Throughput: {hz:.3f} msg/s | "
            f"Bandwidth: {bw:.3f} MB/s | "
            f"Duration: {elapsed:.3f} s"
        )

        # Log to CSV
        self.log_to_csv(elapsed, hz, bw, latency_stats)

    def log_to_csv(self, duration, hz, bw, latency_stats):
        import csv
        csv_file = 'ros2_monitor_log.csv'
        file_exists = os.path.isfile(csv_file)

        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    'timestamp', 'topic', 'duration', 'count',
                    'avg_latency_ms', 'min_latency_ms', 'max_latency_ms', 'std_latency_ms',
                    'rate_hz', 'bandwidth_mb_s', 'cpu_avg', 'mem_avg_mb'
                ])

            cpu = self.process.cpu_percent()
            mem = self.process.memory_info().rss / (1024 * 1024)

            writer.writerow([
                time.strftime("%Y-%m-%d %H:%M:%S"),
                self.topic_name,
                round(duration, 3),
                self.count,
                round(latency_stats['avg'], 3),
                round(latency_stats['min'], 3),
                round(latency_stats['max'], 3),
                round(latency_stats['std'], 3),
                round(hz, 3),
                round(bw, 3),
                round(cpu, 1),
                round(mem, 2)
            ])


def msg_to_bytes(msg):
    """Helper function to estimate message size"""
    from rclpy.serialization import serialize_message
    return len(serialize_message(msg))


def main():
    import argparse
    parser = argparse.ArgumentParser(description='ROS 2 Topic Monitor')
    parser.add_argument('topic', help='Topic name to monitor')
    parser.add_argument('--duration', type=float, default=10.0, help='Monitoring duration in seconds')
    parser.add_argument('--msg-type', default='sensor_msgs/msg/Image', help='Message type')
    args = parser.parse_args()

    rclpy.init()

    # Dynamically import the message type
    from importlib import import_module
    pkg, msg = args.msg_type.split('/msg/')
    module = import_module(f'{pkg}.msg')
    msg_type = getattr(module, msg)

    try:
        monitor = ROS2Monitor(args.topic, msg_type, args.duration)
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
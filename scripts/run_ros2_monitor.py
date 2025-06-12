#!/usr/bin/env python3
"""
This script monitors a ROS 2 topic for message rates, latencies, and sizes.
"""
import rclpy
from rclpy.node import Node
import time
import numpy as np
import psutil
import os
from std_msgs.msg import Header
from importlib import import_module
import argparse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def msg_to_bytes(msg):
    """Helper function to estimate message size"""
    from rclpy.serialization import serialize_message

    return len(serialize_message(msg))


class ROS2Monitor(Node):
    def __init__(self, topic_name, msg_type_str, duration):
        super().__init__("ros2_monitor")
        self.topic_name = topic_name
        self.duration = float(duration)
        self.msg_type_str = msg_type_str
        self.start_time = time.time()
        self.end_time = self.start_time + self.duration
        self.latencies = []
        self.msg_sizes = []
        self.count = 0
        self.process = psutil.Process(os.getpid())
        self.msg_type = None
        self.first_msg_time = None
        self.first_recv_time = None

        # QoS profile
        qos_profile = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.RELIABLE
        )

        try:
            pkg, msg = self.msg_type_str.split("/msg/")
            module = import_module(f"{pkg}.msg")
            self.msg_type = getattr(module, msg)
            self.subscription = self.create_subscription(
                self.msg_type, self.topic_name, self.callback, qos_profile
            )
            self.get_logger().info(
                f"Monitoring '{topic_name}' (type: '{self.msg_type_str}') for {duration} seconds..."
            )
            self.get_logger().info(
                f"Subscription created for topic '{topic_name}' with message type '{self.msg_type_str}'."
            )
        except ImportError as e:
            self.get_logger().error(
                f"Could not import message type '{self.msg_type_str}': {e}"
            )
            exit(1)
        except AttributeError as e:
            self.get_logger().error(
                f"Message type '{msg}' not found in package '{pkg}': {e}"
            )
            exit(1)
        except ValueError as e:
            self.get_logger().error(
                f"Invalid message type format '{self.msg_type_str}': {e}. Expected package/msg/MessageType"
            )
            exit(1)

        # Periodic reporting
        self.timer = self.create_timer(1.0, self.periodic_report)

    def callback(self, msg):
        now = time.time()
        self.count += 1

        if hasattr(msg, "header") and isinstance(msg.header, Header):
            msg_time_sec = msg.header.stamp.sec
            msg_time_nanosec = msg.header.stamp.nanosec
            msg_time_ros = msg_time_sec + msg_time_nanosec / 1e9

            if self.first_msg_time is None:
                self.first_msg_time = msg_time_ros
                self.first_recv_time = now

            if self.first_msg_time is not None:
                relative_msg_time = msg_time_ros - self.first_msg_time
                relative_recv_time = now - self.first_recv_time
                latency = relative_recv_time - relative_msg_time
                self.latencies.append(latency)
        elif hasattr(msg, "header"):
            self.get_logger().warn(
                f"Topic: {self.topic_name} has a header but not of type std_msgs.msg.Header."
            )

        try:
            self.msg_sizes.append(msg_to_bytes(msg))
        except Exception as e:
            self.get_logger().warn(
                f"Error getting message size for {self.topic_name}: {e}"
            )

        if time.time() > self.end_time:
            self.final_report()
            raise KeyboardInterrupt

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

        bw = (
            (sum(self.msg_sizes) / (1024 * 1024)) / elapsed  # MB/s
            if self.msg_sizes and elapsed > 0
            else 0
        )

        latencies_ms = (
            np.array(self.latencies) * 1000
        )  # Convert to milliseconds
        latency_stats = {
            "avg": np.mean(latencies_ms) if self.latencies else 0,
            "min": np.min(latencies_ms) if self.latencies else 0,
            "max": np.max(latencies_ms) if self.latencies else 0,
            "std": np.std(latencies_ms) if self.latencies else 0,
        }

        self.get_logger().info(
            f"[Final] Samples: {self.count} | "
            f"Avg Latency: {latency_stats['avg']:.3f} ms | "
            f"Min Latency: {latency_stats['min']:.3f} ms | "
            f"Max Latency: {latency_stats['max']:.3f} ms | "
            f"Std Latency: {latency_stats['std']:.3f} ms | "
            f"Throughput: {hz:.3f} msg/s | "
            f"Bandwidth: {bw:.3f} MB/s | "
            f"Duration: {elapsed:.3f} s"
        )

        self.log_to_csv(elapsed, hz, bw, latency_stats)

    def log_to_csv(self, duration, hz, bw, latency_stats):
        import csv

        csv_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "ros2_monitor_log.csv"
        )
        file_exists = os.path.isfile(csv_file)

        with open(csv_file, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(
                    [
                        "timestamp",
                        "topic",
                        "duration",
                        "count",
                        "avg_latency_ms",
                        "min_latency_ms",
                        "max_latency_ms",
                        "std_latency_ms",
                        "rate_hz",
                        "bandwidth_mb_s",
                        "cpu_avg",
                        "mem_avg_mb",
                    ]
                )

            cpu = self.process.cpu_percent()
            mem = self.process.memory_info().rss / (1024 * 1024)

            writer.writerow(
                [
                    time.strftime("%Y-%m-%d %H:%M:%S"),
                    self.topic_name,
                    round(duration, 3),
                    self.count,
                    round(latency_stats["avg"], 3),
                    round(latency_stats["min"], 3),
                    round(latency_stats["max"], 3),
                    round(latency_stats["std"], 3),
                    round(hz, 3),
                    round(bw, 3),
                    round(cpu, 1),
                    round(mem, 2),
                ]
            )


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="ROS 2 Topic Monitor")
    parser.add_argument("topic", help="Topic name to monitor")
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Monitoring duration in seconds",
    )
    parser.add_argument(
        "--msg-type",
        required=True,
        help="Message type (e.g., sensor_msgs/msg/Image)",
    )
    args = parser.parse_args()

    try:
        monitor = ROS2Monitor(args.topic, args.msg_type, args.duration)
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

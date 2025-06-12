#!/usr/bin/env python3
"""
Runs a benchmark for different DDS implementations in ROS 2 while connected to
Avular's Origin One via Zenoh bridge.
"""
import os
import subprocess
import time
from datetime import datetime
import argparse

RMW_IMPLEMENTATIONS = ["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"]
# Potential implementations to test: "rmw_zenoh_cpp", "rmw_connext_cpp", "rmw_gurumdds_cpp"
LOG_DIR = "logs"
TOPICS_TO_TEST = [
    # High-frequency sensor data
    ("/robot/camera/color/image_raw", "sensor_msgs/msg/Image"),
    ("/robot/lidar/points", "sensor_msgs/msg/PointCloud2"),
    ("/robot/odom", "nav_msgs/msg/Odometry"),
    ("/autopilot/lidar/points_filtered", "sensor_msgs/msg/PointCloud2"),
    ("/autopilot/lidar/scan_filtered", "sensor_msgs/msg/LaserScan"),
    ("/robot/joint_states", "sensor_msgs/msg/JointState"),
    ("/tf", "tf2_msgs/msg/TFMessage"),
    # Medium-frequency control data
    ("/robot/cmd_vel", "geometry_msgs/msg/Twist"),
    # Low-frequency but critical data
    ("/robot/battery/info", "origin_msgs/msg/BatteryInfo")
]


def setup_logging(rmw_impl, topic_name):
    """Create log directory structure"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_topic_name = topic_name.replace('/', '_')[1:]
    log_dir = os.path.join(LOG_DIR, rmw_impl, safe_topic_name, timestamp)
    os.makedirs(log_dir, exist_ok=True)
    return log_dir


def run_monitor(rmw_impl, topic_name, msg_type, duration):
    log_dir = setup_logging(rmw_impl, topic_name)
    log_file = os.path.join(log_dir, "monitor_output.log")

    cmd = [
        "python3", "scripts/run_ros2_monitor.py",
        topic_name,
        "--duration", str(duration),
        "--msg-type", msg_type
    ]
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = rmw_impl

    print(f"Running: {' '.join(cmd)} with RMW: {rmw_impl}")
    with open(log_file, 'w') as f:
        process = subprocess.Popen(
            cmd,
            env=env,
            stdout=f,
            stderr=subprocess.STDOUT
        )
        return process, log_file


def main():
    parser = argparse.ArgumentParser(description='ROS 2 DDS Benchmark')
    parser.add_argument('-t', '--test-duration', type=int, default=10,
                        help='Duration of each test in seconds')
    args = parser.parse_args()
    test_duration = args.test_duration

    os.makedirs(LOG_DIR, exist_ok=True)
    processes = []

    # Run tests for each RMW implementation
    for rmw_impl in RMW_IMPLEMENTATIONS:
        print(f"\n=== Starting tests with {rmw_impl} ===")
        try:
            subprocess.run(
                ["ros2", "doctor", "--report"],
                env={**os.environ, "RMW_IMPLEMENTATION": rmw_impl},
                check=True,
                capture_output=True,
                timeout=10
            )
        except subprocess.CalledProcessError as e:
            print(f"Skipping {rmw_impl} - not properly configured yet.\n{e.stderr.decode()}")
            continue
        except subprocess.TimeoutExpired:
            print(f"Timeout while running ros2 doctor on {rmw_impl}.")
            continue

        for topic, msg_type in TOPICS_TO_TEST:
            print(f"\nTesting {topic} ({msg_type}) with {rmw_impl} for {test_duration} seconds...")
            process, log_file = run_monitor(rmw_impl, topic, msg_type, test_duration)
            processes.append((process, topic, rmw_impl, log_file))
            time.sleep(1)

    # Wait for all monitor processes to complete
    for process, topic, rmw_impl, log_file in processes:
        try:
            process.wait(timeout=test_duration + 15)
            if process.returncode != 0:
                print(f"Monitor for {topic} with {rmw_impl} exited with code: {process.returncode}."
                      f"Check {log_file} for details.")
        except subprocess.TimeoutExpired:
            process.terminate()
            print(f"Timeout expired waiting for monitor of {topic} with {rmw_impl}")

    print("All tests completed. Results saved to 'logs/' directory")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Run a comprehensive test suite for different DDS implementations in ROS 2.
"""
import os
import subprocess
import time
from datetime import datetime
import psutil

# Configuration
TEST_DURATION = 30  # seconds per test
RMW_IMPLEMENTATIONS = ["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp", "rmw_zenoh_cpp"]
LOG_DIR = "logs"

# Selected topics with their types (automatically detected if not specified)
TOPICS_TO_TEST = [
    # High-frequency sensor data
    ("/robot/camera/color/image_raw", "sensor_msgs/msg/Image"),
    ("/robot/lidar/points", "sensor_msgs/msg/PointCloud2"),
    ("/robot/odom", "nav_msgs/msg/Odometry"),

    # Medium-frequency control data
    ("/robot/cmd_vel", "geometry_msgs/msg/Twist"),
    ("/autopilot/plan_twist", "geometry_msgs/msg/Twist"),

    # Low-frequency but critical data
    ("/robot/battery/info", "sensor_msgs/msg/BatteryState"),
    ("/autopilot/estimated_pose", "geometry_msgs/msg/PoseWithCovarianceStamped")
]


def setup_logging(rmw_impl, topic_name):
    """Create log directory structure"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_topic_name = topic_name.replace('/', '_')[1:]  # Remove leading slash
    log_dir = os.path.join(LOG_DIR, rmw_impl, safe_topic_name, timestamp)
    os.makedirs(log_dir, exist_ok=True)
    return log_dir


def run_monitor(rmw_impl, topic_name, msg_type, duration):
    """Run the monitoring process for a specific topic"""
    log_dir = setup_logging(rmw_impl, topic_name)
    log_file = os.path.join(log_dir, "monitor.log")

    cmd = [
        "python3", "ros2_monitor.py",
        topic_name,
        "--msg-type", msg_type,
        "--duration", str(duration)
    ]

    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = rmw_impl

    with open(log_file, 'w') as f:
        process = subprocess.Popen(
            cmd,
            env=env,
            stdout=f,
            stderr=subprocess.STDOUT
        )

        # Monitor system resources during the test
        monitor_resources(process.pid, os.path.join(log_dir, "resources.log"), duration)

        try:
            process.wait(timeout=duration + 5)  # Add buffer time
        except subprocess.TimeoutExpired:
            process.terminate()
            print(f"Timeout expired for {topic_name} with {rmw_impl}")


def monitor_resources(pid, log_file, duration):
    """Log system resource usage during the test"""
    process = psutil.Process(pid)
    start_time = time.time()

    with open(log_file, 'w') as f:
        f.write("timestamp,cpu_percent,memory_mb\n")
        while time.time() - start_time < duration:
            try:
                cpu = process.cpu_percent(interval=1)
                mem = process.memory_info().rss / (1024 * 1024)  # MB
                f.write(f"{time.time()},{cpu},{mem}\n")
                f.flush()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                break


def main():
    # Create main log directory
    os.makedirs(LOG_DIR, exist_ok=True)

    # Run tests for each RMW implementation
    for rmw_impl in RMW_IMPLEMENTATIONS:
        print(f"\n=== Starting tests with {rmw_impl} ===")

        # Verify implementation is available
        try:
            subprocess.run(
                ["ros2", "doctor", "--report"],
                env={**os.environ, "RMW_IMPLEMENTATION": rmw_impl},
                check=True,
                capture_output=True
            )
        except subprocess.CalledProcessError:
            print(f"Skipping {rmw_impl} - not properly installed")
            continue

        # Test each selected topic
        for topic, msg_type in TOPICS_TO_TEST:
            print(f"\nTesting {topic} ({msg_type})...")
            run_monitor(rmw_impl, topic, msg_type, TEST_DURATION)
            time.sleep(2)  # Brief pause between tests

    print("\nAll tests completed! Results saved to 'logs/' directory")


if __name__ == "__main__":
    main()
"""
run_dds_benchmark.py
Main file to test different rmw_implementations, creating ros2
talker and listener nodes and tracking performance.
"""

import os
import subprocess
import time
import psutil

rmw_implementations = [
    "rmw_fastrtps_cpp",
    "rmw_cyclonedds_cpp",
    "rmw_connextdds",
    # opendds & Gurumdds tbd
]


# Parameters for the talker and listener
msg_count = 1000  # Number of messages to send
frequency_hz = 100  # Publish frequency in Hz
payload_size = 1000000  # Size of the payload in bytes
timeout = 10  # Timeout in seconds


def log_resource_usage(pid, output_file, duration=60):
    process = psutil.Process(pid)
    process.cpu_percent(interval=None)
    with open(output_file, 'w') as f:
        for _ in range(duration):
            if not process.is_running():
                break
            try:
                cpu = process.cpu_percent(interval=1)
                mem = process.memory_info().rss / (1024 * 1024)
                f.write(f"CPU: {cpu:.2f}% | MEM: {mem:.2f} MB\n")
            except psutil.NoSuchProcess:
                break
            except Exception as e:
                f.write(f"Error logging resource usage: {e}\n")
                break


def run_with_rmw(rmw_impl):
    os.environ["RMW_IMPLEMENTATION"] = rmw_impl
    print(f"\n=== Testing {rmw_impl} ===")

    subprocess.run("bash -c 'source /opt/ros/humble/setup.bash && colcon build'", shell=True,
                   check=True)
    subprocess.run("bash -c 'source install/setup.bash'", shell=True, check=True)

    log_dir = f"logs/{rmw_impl}"
    os.makedirs(log_dir, exist_ok=True)

    talker_proc = subprocess.Popen(
        "ros2 run dds_benchmark dds_talker {msg_count} {frequency_hz} {payload_size}".format(
            msg_count=msg_count,
            frequency_hz=frequency_hz,
            payload_size=payload_size
        ),
        shell=True,
        stdout=open(f"{log_dir}/talker.txt", "w"),
        stderr=subprocess.STDOUT
    )
    listener_proc = subprocess.Popen(
        "ros2 run dds_benchmark dds_listener {msg_count} {timeout}".format(
            msg_count=msg_count,
            timeout=timeout
        ),
        shell=True,
        stdout=open(f"{log_dir}/listener.txt", "w"),
        stderr=subprocess.STDOUT
    )

    talker_proc.wait()
    listener_proc.wait()

    print(f">>> {rmw_impl} test completed. Logs saved to {log_dir}")


def main():
    for rmw in rmw_implementations:
        run_with_rmw(rmw)
        time.sleep(5)


if __name__ == "__main__":
    main()

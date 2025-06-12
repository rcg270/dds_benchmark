"""
Main file to locally test different rmw_implementations, creating ros2
talker and listener nodes and tracking performance.
"""

import os
import subprocess
import time
import psutil
import datetime

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

num_local_runs = 5


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


def run_with_rmw(rmw_impl, run_num):
    os.environ["RMW_IMPLEMENTATION"] = rmw_impl

    current_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"\n=== Testing {rmw_impl} (Run {run_num}/{num_local_runs}) ===")

    log_dir = f"local_logs/{rmw_impl}/{current_timestamp}"
    os.makedirs(log_dir, exist_ok=True)

    talker_log_path = os.path.join(log_dir, "talker.log")
    listener_log_path = os.path.join(log_dir, "listener.log")

    print(f"  Logs will be saved to: {log_dir}")

    talker_command = (
        f"ros2 run dds_benchmark dds_talker {msg_count} {frequency_hz} {payload_size}"
    )
    listener_command = (
        f"ros2 run dds_benchmark dds_listener {msg_count} {timeout}"
    )

    with open(talker_log_path, "w") as talker_log_file, \
         open(listener_log_path, "w") as listener_log_file:

        talker_proc = subprocess.Popen(
            talker_command,
            shell=True,
            stdout=talker_log_file,
            stderr=subprocess.STDOUT
        )
        listener_proc = subprocess.Popen(
            listener_command,
            shell=True,
            stdout=listener_log_file,
            stderr=subprocess.STDOUT
        )

        talker_proc.wait()
        listener_proc.wait()

    print(f">>> {rmw_impl} (Run {run_num}) test completed. Logs saved to {log_dir}")


def main():
    subprocess.run("bash -c 'source /opt/ros/humble/setup.bash && colcon build'", shell=True,
                   check=True)
    subprocess.run("bash -c 'source install/setup.bash'", shell=True, check=True)
    print("Build and setup complete.\n")

    for rmw in rmw_implementations:
        for i in range(1, num_local_runs + 1):
            run_with_rmw(rmw, i)
            time.sleep(3)


if __name__ == "__main__":
    main()

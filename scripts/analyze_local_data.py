import os
import re
import pandas as pd
import numpy as np


def analyze_local_dds_logs(log_base_dir="local_logs"):
    """
    Analyzes listener.log files from local DDS benchmarks to extract final statistics
    and average CPU/Memory usage.

    Args:
        log_base_dir (str): The base directory where the 'local_logs' folder is located.

    Returns:
        pandas.DataFrame: A DataFrame containing aggregated results for local tests.
    """
    all_local_results = []

    # Regex for the final summary line
    final_line_pattern = re.compile(
        r"Samples: (\d+) \| Avg: ([-+]?\d+\.\d+) ms \| Min: ([-+]?\d+\.\d+) ms \| "
        r"Max: ([-+]?\d+\.\d+) ms \| Std: ([-+]?\d+\.\d+) ms \| Throughput: (\d+\.\d+) msg/s \| "
        r"Dropped: (\d+) \| Drop rate: (\d+\.\d+)% \| Duration: (\d+\.\d+) s"
    )

    # Regex for periodic CPU/MEM lines
    # Example: [100] CPU: 0.00% | MEM: 61.15 MB
    cpu_mem_pattern = re.compile(
        r"\[\d+\] CPU: ([-+]?\d+\.\d+)% \| MEM: ([-+]?\d+\.\d+) MB"
    )

    print(f"Starting local log analysis from: {os.path.abspath(log_base_dir)}")

    for rmw_impl in os.listdir(log_base_dir):
        rmw_impl_path = os.path.join(log_base_dir, rmw_impl)
        if not os.path.isdir(rmw_impl_path):
            continue

        print(f"Processing RMW: {rmw_impl}")
        for timestamp_dir in os.listdir(rmw_impl_path):
            timestamp_path = os.path.join(rmw_impl_path, timestamp_dir)
            if not os.path.isdir(timestamp_path):
                continue

            listener_log_file_path = os.path.join(
                timestamp_path, "listener.log"
            )
            if not os.path.exists(listener_log_file_path):
                print(f"  Warning: listener.log not found in {timestamp_path}")
                continue

            current_cpu_usages = []
            current_mem_usages = []
            run_data = {}

            with open(listener_log_file_path, "r") as f:
                for line in f:
                    # Try to match the final summary line
                    final_match = final_line_pattern.search(line)
                    if final_match:
                        data = final_match.groups()
                        run_data = {
                            "RMW_Implementation": rmw_impl,
                            "Timestamp_Run": timestamp_dir,
                            "Samples": int(data[0]),
                            "Avg_Latency_ms": float(data[1]),
                            "Min_Latency_ms": float(data[2]),
                            "Max_Latency_ms": float(data[3]),
                            "Std_Latency_ms": float(data[4]),
                            "Throughput_msg_s": float(data[5]),
                            "Dropped_Messages": int(data[6]),
                            "Drop_Rate_percent": float(data[7]),
                            "Duration_s": float(data[8]),
                        }

                    # Try to match periodic CPU/MEM lines
                    cpu_mem_match = cpu_mem_pattern.search(line)
                    if cpu_mem_match:
                        current_cpu_usages.append(float(cpu_mem_match.group(1)))
                        current_mem_usages.append(float(cpu_mem_match.group(2)))

            # Calculate average CPU/MEM for this run, ignoring 0.00% if they represent idle times
            # A common approach is to only average non-zero CPU if some systems report 0% for idle.
            # However, for consistency, let's average all captured values.
            run_data["Avg_CPU_percent"] = (
                np.mean(current_cpu_usages) if current_cpu_usages else 0.0
            )
            run_data["Avg_MEM_MB"] = (
                np.mean(current_mem_usages) if current_mem_usages else 0.0
            )

            if run_data:  # Only add if a final summary was found
                all_local_results.append(run_data)
            else:
                print(
                    f"  Warning: No final summary line found in {listener_log_file_path}"
                )

    local_summary_df = pd.DataFrame(all_local_results)
    return local_summary_df


if __name__ == "__main__":
    local_summary_results = analyze_local_dds_logs(log_base_dir="./local_logs")

    if not local_summary_results.empty:
        print("\n--- Local Benchmark Results (Summary) ---")
        print(local_summary_results.to_string())
        local_summary_results.to_csv("local_benchmark_summary.csv", index=False)
        print("\nLocal aggregated results saved to local_benchmark_summary.csv")
    else:
        print("No local benchmark data found to display or save.")

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os


def plot_local_benchmark_data(
    summary_csv_path="local_benchmark_summary.csv", output_dir="plots_local"
):
    """
    Reads local benchmark summary data and generates plots.

    Args:
        summary_csv_path (str): Path to the CSV file containing local summary data.
        output_dir (str): Directory to save the generated plots.
    """
    try:
        df_local = pd.read_csv(summary_csv_path)
    except FileNotFoundError:
        print(
            f"Error: Local summary CSV file not found at '{summary_csv_path}'"
        )
        return
    except Exception as e:
        print(f"Error reading local CSV file: {e}")
        return

    if df_local.empty:
        print("Local DataFrame is empty. No plots to generate.")
        return

    os.makedirs(output_dir, exist_ok=True)

    # Reformat RMW names for plotting
    df_local["RMW_Implementation"] = df_local["RMW_Implementation"].replace(
        {
            "rmw_cyclonedds_cpp": "CycloneDDS",
            "rmw_fastrtps_cpp": "Fast RTPS",
            "rmw_connextdds": "Connext DDS",
        }
    )

    print("Generating local benchmark plots...")

    # --- Plot 1: Average Latency Comparison ---
    plt.figure(figsize=(8, 6))
    sns.barplot(
        x="RMW_Implementation",
        y="Avg_Latency_ms",
        data=df_local,
        errorbar="sd",
        capsize=0.1,
        palette="viridis",
    )
    plt.title("Local Host: Average Latency")
    plt.xlabel("RMW Implementation")
    plt.ylabel("Average Latency (ms)")
    plt.grid(True, linestyle="--", alpha=0.7, axis="y")
    plt.tight_layout()
    plot_filename = f"{output_dir}/local_avg_latency.png"
    plt.savefig(plot_filename, dpi=300)
    plt.close()
    print(f"  Saved {plot_filename}")

    # --- Plot 2: Throughput Comparison ---
    plt.figure(figsize=(8, 6))
    sns.barplot(
        x="RMW_Implementation",
        y="Throughput_msg_s",
        data=df_local,
        errorbar="sd",
        capsize=0.1,
        palette="viridis",
    )
    plt.title("Local Host: Average Throughput")
    plt.xlabel("RMW Implementation")
    plt.ylabel("Average Throughput (msg/s)")
    plt.grid(True, linestyle="--", alpha=0.7, axis="y")
    plt.tight_layout()
    plot_filename = f"{output_dir}/local_throughput.png"
    plt.savefig(plot_filename, dpi=300)
    plt.close()
    print(f"  Saved {plot_filename}")

    # --- Plot 3: Average CPU Usage ---
    plt.figure(figsize=(8, 6))
    sns.barplot(
        x="RMW_Implementation",
        y="Avg_CPU_percent",
        data=df_local,
        errorbar="sd",
        capsize=0.1,
        palette="viridis",
    )
    plt.title("Local Host: Average CPU Usage")
    plt.xlabel("RMW Implementation")
    plt.ylabel("Average CPU Usage (%)")
    plt.grid(True, linestyle="--", alpha=0.7, axis="y")
    plt.tight_layout()
    plot_filename = f"{output_dir}/local_avg_cpu.png"
    plt.savefig(plot_filename, dpi=300)
    plt.close()
    print(f"  Saved {plot_filename}")

    # --- Plot 4: Average Memory Usage ---
    plt.figure(figsize=(8, 6))
    sns.barplot(
        x="RMW_Implementation",
        y="Avg_MEM_MB",
        data=df_local,
        errorbar="sd",
        capsize=0.1,
        palette="viridis",
    )
    plt.title("Local Host: Average Memory Usage")
    plt.xlabel("RMW Implementation")
    plt.ylabel("Average Memory Usage (MB)")
    plt.grid(True, linestyle="--", alpha=0.7, axis="y")
    plt.tight_layout()
    plot_filename = f"{output_dir}/local_avg_memory.png"
    plt.savefig(plot_filename, dpi=300)
    plt.close()
    print(f"  Saved {plot_filename}")

    # --- Plot 5: Dropped Messages (if any) ---
    if (
        "Dropped_Messages" in df_local.columns
        and df_local["Dropped_Messages"].sum() > 0
    ):
        plt.figure(figsize=(8, 6))
        sns.barplot(
            x="RMW_Implementation",
            y="Dropped_Messages",
            data=df_local,
            errorbar="sd",
            capsize=0.1,
            palette="viridis",
        )
        plt.title("Local Host: Total Dropped Messages")
        plt.xlabel("RMW Implementation")
        plt.ylabel("Dropped Messages")
        plt.grid(True, linestyle="--", alpha=0.7, axis="y")
        plt.tight_layout()
        plot_filename = f"{output_dir}/local_dropped_messages.png"
        plt.savefig(plot_filename, dpi=300)
        plt.close()
        print(f"  Saved {plot_filename}")
    else:
        print("  No dropped messages detected in local tests, skipping plot.")


if __name__ == "__main__":
    plot_local_benchmark_data(
        summary_csv_path="local_benchmark_summary.csv", output_dir="plots_local"
    )

#!/usr/bin/env python3
import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import warnings

# Configuration
LOG_ROOT = "data_logs/logs"
OUTPUT_DIR = "analysis_results"
PLOT_FORMAT = "png"  # or "svg", "pdf"
IGNORE_ZERO_RATE = True  # Skip files with 0Hz message rates

# Create output directory
os.makedirs(OUTPUT_DIR, exist_ok=True)

def parse_monitor_log(filepath):
    """Extract metrics from monitor.log files"""
    data = {
        'timestamps': [],
        'msg_counts': [],
        'rates': [],
        'cpu_percents': [],
        'mem_mbs': [],
        'final_stats': None
    }

    try:
        with open(filepath, 'r') as f:
            for line in f:
                # Parse interim reports
                if '[Interim]' in line:
                    parts = re.findall(r"Msgs: (\d+) \| Rate: ([\d.]+) Hz \| CPU: ([\d.]+)% \| MEM: ([\d.]+) MB", line)
                    if parts:
                        data['msg_counts'].append(int(parts[0][0]))
                        data['rates'].append(float(parts[0][1]))
                        data['cpu_percents'].append(float(parts[0][2]))
                        data['mem_mbs'].append(float(parts[0][3]))

                # Parse final report
                elif '[Final]' in line:
                    stats = re.search(
                        r"Samples: (\d+) \| Avg: ([\d.-]+) ms \| "
                        r"Min: ([\d.-]+) ms \| Max: ([\d.-]+) ms \| "
                        r"Std: ([\d.]+) ms \| Throughput: ([\d.]+) msg/s \| "
                        r"Bandwidth: ([\d.]+) MB/s \| Duration: ([\d.]+) s",
                        line
                    )
                    if stats:
                        data['final_stats'] = {
                            'samples': int(stats.group(1)),
                            'avg_latency': float(stats.group(2)),
                            'min_latency': float(stats.group(3)),
                            'max_latency': float(stats.group(4)),
                            'std_latency': float(stats.group(5)),
                            'throughput': float(stats.group(6)),
                            'bandwidth': float(stats.group(7)),
                            'duration': float(stats.group(8))
                        }

    except Exception as e:
        warnings.warn(f"Failed to parse {filepath}: {str(e)}")
        return None

    # Skip empty or invalid logs
    if not data['rates'] or (IGNORE_ZERO_RATE and max(data['rates']) <= 0):
        return None

    return data


def parse_resources_log(filepath):
    """Parse the resources.log CSV file"""
    try:
        df = pd.read_csv(filepath)
        return {
            'cpu_avg': df['cpu_percent'].mean(),
            'cpu_max': df['cpu_percent'].max(),
            'mem_avg': df['memory_mb'].mean(),
            'mem_max': df['memory_mb'].max()
        }
    except Exception as e:
        warnings.warn(f"Failed to parse {filepath}: {str(e)}")
        return None


def analyze_experiment(exp_path):
    """Analyze a single experiment directory"""
    monitor_log = exp_path / "monitor.log"
    resources_log = exp_path / "resources.log"
    print(f"Analyzing: {monitor_log}")
    print(f"Resources: {resources_log}")
    if not monitor_log.exists():
        return None

    results = {
        'rmw': exp_path.parent.parent.name.replace('rmw_', '').replace('_cpp', ''),
        'topic': exp_path.parent.name,
        'timestamp': exp_path.name,
        'network': 'ethernet' if 'eth' in exp_path.name.lower() else 'wifi'
    }

    # Parse logs
    monitor_data = parse_monitor_log(monitor_log)
    if monitor_data is None:
        return None

    resources_data = parse_resources_log(resources_log) if resources_log.exists() else {}

    # Combine all metrics
    results.update({
        'msg_rate_avg': np.mean(monitor_data['rates']),
        'msg_rate_std': np.std(monitor_data['rates']),
        'cpu_avg': np.mean(monitor_data['cpu_percents']),
        'mem_avg': np.mean(monitor_data['mem_mbs']),
        **monitor_data['final_stats'],
        **(resources_data or {})
    })

    return results


def generate_plots(all_data):
    """Generate comparative visualizations"""
    df = pd.DataFrame(all_data)

    # 1. Message Rate Comparison
    plt.figure(figsize=(12, 6))
    for rmw in df['rmw'].unique():
        subset = df[df['rmw'] == rmw]
        plt.errorbar(
            subset['topic'], subset['msg_rate_avg'],
            yerr=subset['msg_rate_std'],
            fmt='o', label=rmw, capsize=5
        )
    plt.title('Message Rate by Topic and RMW Implementation')
    plt.ylabel('Message Rate (Hz)')
    plt.xticks(rotation=45, ha='right')
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{OUTPUT_DIR}/message_rates.{PLOT_FORMAT}")
    plt.close()

    # 2. Latency Distribution
    plt.figure(figsize=(12, 6))
    for rmw in df['rmw'].unique():
        subset = df[df['rmw'] == rmw]
        plt.scatter(
            subset['topic'], subset['avg_latency'],
            s=subset['throughput']*5,  # Bubble size by throughput
            label=rmw, alpha=0.7
        )
    plt.title('Latency vs Topic (Bubble Size = Throughput)')
    plt.ylabel('Average Latency (ms)')
    plt.xticks(rotation=45, ha='right')
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{OUTPUT_DIR}/latency.{PLOT_FORMAT}")
    plt.close()

    # 3. Resource Usage
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    for rmw in df['rmw'].unique():
        subset = df[df['rmw'] == rmw]
        ax1.scatter(subset['topic'], subset['cpu_avg'], label=rmw)
        ax2.scatter(subset['topic'], subset['mem_avg'], label=rmw)
    ax1.set_title('CPU Usage by Topic')
    ax1.set_ylabel('CPU (%)')
    ax2.set_title('Memory Usage by Topic')
    ax2.set_ylabel('Memory (MB)')
    for ax in [ax1, ax2]:
        ax.set_xticklabels(df['topic'].unique(), rotation=45, ha='right')
        ax.legend()
    plt.tight_layout()
    plt.savefig(f"{OUTPUT_DIR}/resources.{PLOT_FORMAT}")
    plt.close()


def main():
    all_data = []
    skipped_files = []
    for rmw_dir in Path(LOG_ROOT).glob("rmw_*"):
        for topic_dir in rmw_dir.iterdir():
            if not topic_dir.is_dir():
                continue
            for exp_dir in topic_dir.iterdir():
                if exp_dir.is_dir() and len(exp_dir.name) == 15:
                    # print(f"Analyzing: {exp_dir}")
                    result = analyze_experiment(exp_dir)
                    if result:
                        all_data.append(result)
                    else:
                        skipped_files.append(str(exp_dir))

    # Save raw data
    pd.DataFrame(all_data).to_csv(f"{OUTPUT_DIR}/all_metrics.csv", index=False)

    # Generate plots
    if all_data:
        generate_plots(all_data)
        print(f"Analysis complete! Results saved to {OUTPUT_DIR}/")
    else:
        print("No valid data found to analyze.")

    # Report skipped files
    if skipped_files:
        print("\nSkipped the following files/directories:")
        for f in skipped_files:
            print(f" - {f}")


if __name__ == "__main__":
    main()
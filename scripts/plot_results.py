"""
Plotting script for DDS benchmark results that are saved in benchmark_summary.csv.
(first run analyze_data.py to generate this file)
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os


def plot_benchmark_data(summary_csv_path="benchmark_summary.csv", output_dir="plots"):
    try:
        df_summary = pd.read_csv(summary_csv_path)
    except FileNotFoundError:
        print(f"Error: Summary CSV file not found at '{summary_csv_path}'")
        return
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    if df_summary.empty:
        print("Summary DataFrame is empty, no plots to generate.")
        return

    os.makedirs(output_dir, exist_ok=True)

    # Reformat RMW names for plotting
    df_summary['RMW_Implementation'] = df_summary['RMW_Implementation'].replace({
        'rmw_cyclonedds_cpp': 'CycloneDDS',
        'rmw_fastrtps_cpp': 'Fast RTPS'
    })

    topics = df_summary['Topic'].unique()
    rmw_implementations = df_summary['RMW_Implementation'].unique()

    print(f"Generating per topic plots for {len(rmw_implementations)} RMWs and {len(topics)}")
    print("Generating Average Latency Plots")
    for topic in topics:
        topic_data = df_summary[df_summary['Topic'] == topic]

        plt.figure(figsize=(8, 6))
        sns.barplot(x='RMW_Implementation', y='Avg_Latency_ms', data=topic_data,
                    errorbar='sd', capsize=0.1)
        plt.axhline(0, color='red', linestyle='--', label='Zero Latency')
        plt.title(f'Average Latency for {topic}')
        plt.xlabel('RMW Implementation')
        plt.ylabel('Average Latency (ms)')
        plt.grid(True, linestyle='--', alpha=0.7, axis='y')
        plt.legend()
        plt.tight_layout()
        plot_filename = f"{output_dir}/avg_latency_bar_{topic.replace('/', '_')}.png"
        plt.savefig(plot_filename, dpi=300)
        plt.close()

    print("Generating Throughput Plots")
    for topic in topics:
        topic_data = df_summary[df_summary['Topic'] == topic]

        plt.figure(figsize=(8, 6))
        sns.barplot(x='RMW_Implementation', y='Throughput_msg_s', data=topic_data,
                    errorbar='sd', capsize=0.1)
        plt.title(f'Average Throughput for {topic}')
        plt.xlabel('RMW Implementation')
        plt.ylabel('Average Throughput (msg/s)')
        plt.grid(True, linestyle='--', alpha=0.7, axis='y')
        plt.tight_layout()
        plot_filename = f"{output_dir}/throughput_bar_{topic.replace('/', '_')}.png"
        plt.savefig(plot_filename, dpi=300)
        plt.close()

    print("Generating Bandwidth Plots")
    for topic in topics:
        topic_data = df_summary[df_summary['Topic'] == topic]

        plt.figure(figsize=(8, 6))
        sns.barplot(x='RMW_Implementation', y='Bandwidth_MB_s', data=topic_data,
                    errorbar='sd', capsize=0.1)
        plt.title(f'Average Bandwidth for {topic}')
        plt.xlabel('RMW Implementation')
        plt.ylabel('Average Bandwidth (MB/s)')
        plt.grid(True, linestyle='--', alpha=0.7, axis='y')
        plt.tight_layout()
        plot_filename = f"{output_dir}/bandwidth_bar_{topic.replace('/', '_')}.png"
        plt.savefig(plot_filename, dpi=300)
        plt.close()

    print("Generating Min/Max/Avg Latency Plots")
    for topic in topics:
        topic_data = df_summary[df_summary['Topic'] == topic]

        plt.figure(figsize=(10, 7))
        melted_latency_data = topic_data[['RMW_Implementation', 'Min_Latency_ms',
                                          'Max_Latency_ms', 'Avg_Latency_ms']].melt(
                                          id_vars='RMW_Implementation',
                                          var_name='Latency_Type', value_name='Latency_ms'
        )

        melted_latency_data['Latency_Type'] = melted_latency_data['Latency_Type'].replace({
            'Min_Latency_ms': 'Min Latency',
            'Max_Latency_ms': 'Max Latency',
            'Avg_Latency_ms': 'Avg Latency'
        })

        sns.barplot(x='RMW_Implementation', y='Latency_ms',
                    hue='Latency_Type', data=melted_latency_data,
                    palette='viridis', errorbar=None)

        plt.axhline(0, color='red', linestyle='--', label='Zero Latency')
        plt.title(f'Min/Max/Avg Latency Comparison for {topic}')
        plt.xlabel('RMW Implementation')
        plt.ylabel('Latency (ms)')
        plt.grid(True, linestyle='--', alpha=0.7, axis='y')
        plt.legend(title='Latency Type')
        plt.tight_layout()
        plot_filename = f"{output_dir}/min_max_avg_latency_grouped_bar_{topic.replace('/', '_')}.png"
        plt.savefig(plot_filename, dpi=300)
        plt.close()

    print("\nGenerating  Comparison Plots (all topics on one graph)...")
    # --- Plot 1: avg latency ---
    print("  Generating Global Average Latency Plot...")
    plt.figure(figsize=(16, 8))
    sns.barplot(x='Topic', y='Avg_Latency_ms', hue='RMW_Implementation', data=df_summary,
                errorbar='sd', capsize=0.1, palette='deep')
    plt.axhline(0, color='red', linestyle='--', label='Zero Latency')
    plt.title('Average Latency Comparison Across All Topics')
    plt.xlabel('Topic')
    plt.ylabel('Average Latency (ms)')
    plt.xticks(rotation=45, ha='right')
    plt.grid(True, linestyle='--', alpha=0.7, axis='y')
    plt.legend(title='RMW Implementation')
    plt.tight_layout()
    plot_filename = f"{output_dir}/global_avg_latency_all_topics.png"
    plt.savefig(plot_filename, dpi=300)
    plt.close()
    print(f"  Saved {plot_filename}")

    # --- Plot 2: avg throughput ---
    print("  Generating Global Throughput Plot...")
    plt.figure(figsize=(16, 8))
    sns.barplot(x='Topic', y='Throughput_msg_s', hue='RMW_Implementation', data=df_summary,
                errorbar='sd', capsize=0.1, palette='deep')
    plt.title('Average Throughput Comparison Across All Topics')
    plt.xlabel('Topic')
    plt.ylabel('Average Throughput (msg/s)')
    plt.xticks(rotation=45, ha='right')
    plt.grid(True, linestyle='--', alpha=0.7, axis='y')
    plt.legend(title='RMW Implementation')
    plt.tight_layout()
    plot_filename = f"{output_dir}/global_throughput_all_topics.png"
    plt.savefig(plot_filename, dpi=300)
    plt.close()
    print(f"  Saved {plot_filename}")

    # --- Plot 3: avg bandwidth ---
    print("  Generating Global Bandwidth Plot...")
    plt.figure(figsize=(16, 8))
    sns.barplot(x='Topic', y='Bandwidth_MB_s', hue='RMW_Implementation', data=df_summary,
                errorbar='sd', capsize=0.1, palette='deep')
    plt.title('Average Bandwidth Comparison Across All Topics')
    plt.xlabel('Topic')
    plt.ylabel('Average Bandwidth (MB/s)')
    plt.xticks(rotation=45, ha='right')
    plt.grid(True, linestyle='--', alpha=0.7, axis='y')
    plt.legend(title='RMW Implementation')
    plt.tight_layout()
    plot_filename = f"{output_dir}/global_bandwidth_all_topics.png"
    plt.savefig(plot_filename, dpi=300)
    plt.close()
    print(f"  Saved {plot_filename}")

    # --- Plot 4: Min/Max/Avg Latency for all topics ---
    print("  Generating Global Min/Max/Avg Latency Plot...")
    melted_latency_all_topics = df_summary[['RMW_Implementation', 'Topic', 'Min_Latency_ms',
                                            'Max_Latency_ms', 'Avg_Latency_ms']].melt(
                                            id_vars=['RMW_Implementation', 'Topic'],
                                            var_name='Latency_Type', value_name='Latency_ms'
    )

    melted_latency_all_topics['Latency_Type'] = melted_latency_all_topics['Latency_Type'].replace({
        'Min_Latency_ms': 'Min Latency',
        'Max_Latency_ms': 'Max Latency',
        'Avg_Latency_ms': 'Avg Latency'
    })

    g = sns.catplot(
        x='Topic', y='Latency_ms', hue='RMW_Implementation', col='Latency_Type',
        data=melted_latency_all_topics, kind='bar', height=6, aspect=1.5,
        col_wrap=1,
        sharey=True, palette='viridis', errorbar=None,
        legend_out=True
    )

    g.set_axis_labels("Topic", "Latency (ms)")
    g.set_titles("Latency Type: {col_name}")
    g.set_xticklabels(rotation=45, ha='right')
    g.fig.suptitle('Min/Max/Avg Latency Comparison Across All Topics', y=1)

    for ax in g.axes.flat:
        ax.axhline(0, color='red', linestyle='--', alpha=0.7)
        ax.grid(True, linestyle='--', alpha=0.7, axis='y')
        ax.set_title(ax.get_title(), fontsize=12, y=1.05)

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    plot_filename = f"{output_dir}/global_min_max_avg_latency_all_topics.png"
    plt.savefig(plot_filename, dpi=300)
    plt.close()
    print(f"  Saved {plot_filename}")


if __name__ == "__main__":
    # example use:
    plot_benchmark_data(summary_csv_path="benchmark_ethernet_summary.csv",
                        output_dir="plots_ethernet")

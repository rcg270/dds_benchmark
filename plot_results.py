"""
generated plot stuff
"""

import pandas as pd
import matplotlib.pyplot as plt
import os

log_dir = "logs"
csv_files = [f for f in os.listdir(log_dir) if f.endswith(".csv")]

# Combine all RMW results into one DataFrame
dfs = []
for file in csv_files:
    df = pd.read_csv(os.path.join(log_dir, file))
    dfs.append(df)
all_data = pd.concat(dfs, ignore_index=True)

# Plot avg latency with min/max error bars
plt.figure(figsize=(8, 5))
plt.bar(all_data['rmw_impl'], all_data['avg_latency_ms'], color='skyblue', label='Avg Latency')
plt.errorbar(all_data['rmw_impl'], all_data['avg_latency_ms'],
             yerr=[all_data['avg_latency_ms'] - all_data['min_latency_ms'],
                   all_data['max_latency_ms'] - all_data['avg_latency_ms']],
             fmt='o', ecolor='black', capsize=5, label='Min/Max')
plt.ylabel('Latency (ms)')
plt.title('Latency by RMW Implementation')
plt.legend()
plt.tight_layout()
plt.show()

# Plot throughput
plt.figure(figsize=(8, 5))
plt.bar(all_data['rmw_impl'], all_data['throughput_msgs_per_sec'], color='mediumseagreen')
plt.ylabel('Throughput (msg/s)')
plt.title('Throughput by RMW Implementation')
plt.tight_layout()
plt.show()

# Plot drop rate
plt.figure(figsize=(8, 5))
plt.bar(all_data['rmw_impl'], all_data['drop_rate'], color='salmon')
plt.ylabel('Drop Rate (%)')
plt.title('Message Drop Rate by RMW')
plt.tight_layout()
plt.show()

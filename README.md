# DDS Benchmark

This repository contains benchmarking tools and scripts for evaluating the performance of Data Distribution Service (DDS) implementations.

## Features

- Benchmarking scripts for various DDS vendors
- Automated test scenarios for throughput and latency
- Result aggregation and visualization tools

## Getting Started

1. **Clone the repository:**
    ```bash
    git clone https://github.com/yourusername/dds_benchmark.git
    cd dds_benchmark
    ```

2. **Install dependencies:**
    Install ros2 humble,
    zenoh-bridge-ros2dds=0.11.0
    ros-humble-rmw-cyclonedds-cp

    Optionally uncomment GurumDDS and connextDDS installation

3. **Run a benchmark:**
    To run a local benchmark:
    ```bash
    python3 run_local_benchmark.py
    ```
    To run the in-scenario benchmark:
    First connect to the Avular Origin One using zenoh:
    https://avular-robotics.github.io/origin_one/1.0-2/software_development/ros2/ros2_connection/?h=zenoh
    Using the appropriate robot-ip based on Ethernet or Wi-Fi,
    You can test the connection by running:
    ```bash
    ros2 topic list
    ```
    If a correct output is shown, you can run the benchmark:
    ```bash
    python3 run_benchmark.py
    ```

## Configuration

Benchmark parameters can be adjusted in the `configs/` directory. See `configs/default.yaml` for an example.

## Results

Benchmark results are saved in the `results/` directory and can be visualized using the provided scripts.

## Contributing

Contributions are welcome! Please open issues or submit pull requests.

## License

Using MIT License.
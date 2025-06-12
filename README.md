# DDS Benchmark

This repository contains benchmarking tools and scripts for evaluating the performance of Data Distribution Service (DDS) implementations.

## Features

- Benchmarking scripts for various DDS vendors
- Automated test scenarios for throughput and latency
- Analyze results and visualization tools

## Getting Started

### Requirements
-Linux operating system, Ubuntu 22.04 LTS tested.
-For full functionality, access to Avular's origin one https://avular-robotics.github.io/origin_one/

1. **Clone the repository:**
    ```bash
    git clone https://github.com/rcg270/dds_benchmark.git
    cd dds_benchmark
    ```

2. **Install dependencies:**
    Install ros2 humble: https://docs.ros.org/en/humble/Installation.html
    ```bash
    sudo apt install zenoh-bridge-ros2dds=0.11.0
    sudo apt install ros-humble-rmw-cyclonedds-cp
    ```


    Optionally, GurumDDS or connextDDS installation.

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

Benchmark parameters for local testing can be adjusted in the run_local_benchmark.py file.
For in-scenario, the duration in seconds can be adjusted using the -t option:
    ```bash
    python3 run_benchmark.py -t=30
    ```


## Results

Benchmark results are saved in the `logs/` directory and can be visualized using the provided scripts.
for local:
    ```bash
    python3 analyze_local_data.py
    python3 plot_local_results.py

    ```
For in-scenario:
    ```bash
    python3 analyze_data.py
    python3 plot_results.py

    ```


## License

Using MIT License.
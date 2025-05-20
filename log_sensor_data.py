#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import json
from datetime import datetime
from threading import Timer
import numpy as np

# Configure these
TOPICS_TO_LOG = [
    ('/robot/lidar/points', 'sensor_msgs/msg/PointCloud2'),
    ('/robot/camera/depth/image_rect_raw/compressedDepth', 'sensor_msgs/msg/CompressedImage'),
    ('/robot/camera/depth/image_rect_raw', 'sensor_msgs/msg/Image'),
    ('/autopilot/lidar/points_filtered', 'sensor_msgs/msg/PointCloud2'),
    ('/robot/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo'),
    ('/robot/camera/depth/camera_info', 'sensor_msgs/msg/CameraInfo'),
    ('/robot/camera/color/image_raw', 'sensor_msgs/msg/Image')
]
LOG_DURATION = 60  # seconds
OUTPUT_DIR = "sensor_logs"


class SensorLogger(Node):
    def __init__(self):
        super().__init__('sensor_logger')
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        # Create log files for each topic
        self.log_files = {}
        for topic, msg_type in TOPICS_TO_LOG:
            safe_name = topic.replace('/', '_')[1:]
            self.log_files[topic] = open(
                f"{OUTPUT_DIR}/{safe_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl",
                'w'
            )

            # Dynamically create subscribers
            self.create_subscription(
                self.get_message_class(msg_type),
                topic,
                lambda msg, t=topic: self.callback(msg, t),
                10
            )

        # Set timer to stop logging
        Timer(LOG_DURATION, self.stop_logging).start()
        self.get_logger().info(f"Logging started for {LOG_DURATION} seconds...")

    def get_message_class(self, msg_type):
        """Dynamically import message type"""
        package, msg = msg_type.split('/msg/')
        module = __import__(f'{package}.msg', fromlist=[msg])
        return getattr(module, msg)

    def callback(self, msg, topic):
        """Convert message to JSON-serializable dict"""
        try:
            data = {
                'timestamp': datetime.now().isoformat(),
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                } if hasattr(msg, 'header') else None,
                'data_stats': {
                    'size': len(msg.data),
                    'mean': float(np.mean(msg.data[:1000])) if hasattr(msg.data, '__len__') else None,
                    'min': float(np.min(msg.data[:1000])) if hasattr(msg.data, '__len__') else None,
                    'max': float(np.max(msg.data[:1000])) if hasattr(msg.data, '__len__') else None
                } if hasattr(msg, 'data') else None,
                'shape': {
                    'height': msg.height,
                    'width': msg.width
                } if hasattr(msg, 'height') else None
            }

            self.log_files[topic].write(json.dumps(data) + '\n')
            self.log_files[topic].flush()

        except Exception as e:
            self.get_logger().error(f"Error processing {topic}: {str(e)}")

    def stop_logging(self):
        self.get_logger().info("Logging duration complete. Shutting down...")
        for f in self.log_files.values():
            f.close()
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    logger = SensorLogger()
    rclpy.spin(logger)


if __name__ == '__main__':
    main()

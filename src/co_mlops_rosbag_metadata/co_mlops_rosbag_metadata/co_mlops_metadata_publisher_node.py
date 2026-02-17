# Copyright 2026 TIER IV, Inc.

"""
ROS 2 node that publishes the contents of a YAML file as std_msgs/String on a topic.

The file at the "path" parameter is read, validated as parseable YAML, then published once
after an optional delay and optionally republished at a configurable frequency (e.g. for
late-started rosbag recording).
"""

from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def read_file(path: str) -> str | None:
    """
    Read the entire contents of a file into a string.

    Args:
        path: Path to the file.

    Returns:
        File contents as string, or None if the file could not be read.
    """
    try:
        return Path(path).read_text(encoding="utf-8")
    except OSError:
        return None


def is_valid_yaml(content: str) -> bool:
    """
    Check whether a string is parseable as YAML.

    Args:
        content: String content to validate.

    Returns:
        True if the content is valid YAML, False otherwise.
    """
    try:
        yaml.safe_load(content)
        return True
    except yaml.YAMLError:
        return False


class CoMLOpsMetadataPublisherNode(Node):
    """
    ROS 2 node that publishes the contents of a YAML file as std_msgs/String on a topic.

    If the path is missing, the file cannot be read, or the file is not valid YAML,
    the node does not start publishing and main() exits with code 1.
    """

    def __init__(self) -> None:
        super().__init__("co_mlops_metadata_publisher_node")
        self._initialized = False
        self._msg: String | None = None
        self._publisher = None
        self._first_timer = None
        self._period_timer = None

        self.declare_parameter("path", "")
        self.declare_parameter("topic", "/metadata")
        self.declare_parameter("delay_before_first_publish", 0.0)
        self.declare_parameter("frequency", 1.0)

        path = self.get_parameter("path").get_parameter_value().string_value
        if not path:
            self.get_logger().error(
                "Parameter 'path' is required. Example: --ros-args -p path:=/path/to/config.yaml"
            )
            return

        content = read_file(path)
        if content is None:
            self.get_logger().error(f"Failed to read file: {path}")
            return

        if not is_valid_yaml(content):
            self.get_logger().error(f"File is not valid YAML: {path}")
            return

        self._initialized = True
        topic = self.get_parameter("topic").get_parameter_value().string_value
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(String, topic, qos)
        self._msg = String()
        self._msg.data = content

        delay_param = (
            self.get_parameter("delay_before_first_publish").get_parameter_value().double_value
        )
        delay = max(0.0, delay_param)
        frequency = self.get_parameter("frequency").get_parameter_value().double_value

        if delay <= 0.0:
            self._publish_once()
        else:
            self._first_timer = self.create_timer(delay, self._on_first_publish)
        if frequency > 0.0:
            period = 1.0 / frequency
            self._period_timer = self.create_timer(period, self._publish_once)

    def _on_first_publish(self) -> None:
        """Publish once and cancel the first timer."""
        if self._first_timer is not None:
            self._first_timer.cancel()
            self._first_timer = None
        self._publish_once()

    def _publish_once(self) -> None:
        """Publish the loaded YAML content once to the configured topic."""
        if self._publisher and self._msg:
            self._publisher.publish(self._msg)
            self.get_logger().info(
                f"Published vehicle config to {self.get_parameter('topic').get_parameter_value().string_value} "
                f"({len(self._msg.data)} bytes)"
            )

    @property
    def initialized(self) -> bool:
        """Return whether the node successfully loaded the YAML and is ready to publish."""
        return self._initialized


def main(args: list | None = None) -> None:
    """Run the node; exit with code 1 if initialization fails."""
    rclpy.init(args=args)
    node = CoMLOpsMetadataPublisherNode()
    if not node.initialized:
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

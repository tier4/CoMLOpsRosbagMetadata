# Copyright 2026 TIER IV, Inc.

"""Integration test for the Co-MLOps Metadata Publisher Node (CoMLOpsMetadataPublisherNode): launch the node and check /metadata content."""

import os
import sys
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch_ros.actions import Node
from std_msgs.msg import String


def _node_additional_env():
    """PYTHONPATH so the node process can import the package (e.g. when run via poetry run pytest)."""
    prefix = get_package_prefix("co_mlops_rosbag_metadata")
    py_ver = f"{sys.version_info.major}.{sys.version_info.minor}"
    site_packages = os.path.join(prefix, "lib", f"python{py_ver}", "site-packages")
    existing = os.environ.get("PYTHONPATH", "")
    return {"PYTHONPATH": os.pathsep.join([site_packages, existing]) if existing else site_packages}


@pytest.mark.rostest
def generate_test_description():
    """Return launch description and context for the test."""
    share_dir = get_package_share_directory("co_mlops_rosbag_metadata")
    fixture_path = os.path.join(share_dir, "test", "fixtures", "sample_metadata.yaml")

    node = Node(
        package="co_mlops_rosbag_metadata",
        executable="co_mlops_metadata_publisher_node",
        parameters=[{"path": fixture_path}],
        additional_env=_node_additional_env(),
    )

    return (
        launch.LaunchDescription(
            [
                node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"fixture_path": fixture_path},
    )


class TestMetadataPublisherNode(unittest.TestCase):
    """Active test: subscribe to /metadata and assert content matches fixture."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_metadata_subscriber")

    def tearDown(self):
        self.node.destroy_node()

    def test_published_content_matches_fixture(
        self,
        launch_service,
        fixture_path,
        proc_output,
    ):
        """Expect one message on /metadata with content equal to the fixture file."""
        with open(fixture_path, encoding="utf-8") as f:
            expected = f.read()

        msgs = []
        sub = self.node.create_subscription(
            String,
            "/metadata",
            lambda msg: msgs.append(msg),
            10,
        )
        try:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if msgs:
                    break
            self.assertGreater(len(msgs), 0, "No message received on /metadata within timeout")
            self.assertEqual(msgs[0].data, expected, "Published content does not match fixture")
        finally:
            self.node.destroy_subscription(sub)

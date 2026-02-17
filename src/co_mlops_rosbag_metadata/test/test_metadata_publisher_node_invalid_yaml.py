# Copyright 2026 TIER IV, Inc.

"""Integration test: node must exit with error when given invalid YAML."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Launch the node with path to invalid YAML; it should exit with code 1."""
    share_dir = get_package_share_directory("co_mlops_rosbag_metadata")
    invalid_path = os.path.join(share_dir, "test", "fixtures", "invalid_metadata.yaml")

    node = Node(
        package="co_mlops_rosbag_metadata",
        executable="co_mlops_metadata_publisher_node",
        parameters=[{"path": invalid_path}],
    )

    return (
        launch.LaunchDescription([
            node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"node": node},
    )


class TestInvalidYamlWaitsForShutdown(unittest.TestCase):
    """Active test: wait for the node to exit (it exits quickly with invalid YAML)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_exits_and_we_wait_for_shutdown(self, launch_service, proc_info, node):
        """Wait for node shutdown so post-shutdown test can assert exit code."""
        proc_info.assertWaitForShutdown(process=node, timeout=10)


@launch_testing.post_shutdown_test()
class TestInvalidYamlExitCode(unittest.TestCase):
    """Post-shutdown: assert the node exited with code 1 (invalid YAML error)."""

    def test_node_exits_with_error_code(self, proc_info, node):
        """Node must exit with code 1 when the YAML file is invalid."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=node,
            allowable_exit_codes=[1],
        )

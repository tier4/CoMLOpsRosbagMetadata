# Copyright 2026 TIER IV, Inc.

"""Test that the node exits with code 1 when given invalid YAML (no launch_testing)."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import pytest


def test_node_exits_with_error_code_on_invalid_yaml():
    """Run the node with invalid YAML path and assert it exits with code 1."""
    prefix = get_package_prefix("co_mlops_rosbag_metadata")
    exe = os.path.join(
        prefix,
        "lib",
        "co_mlops_rosbag_metadata",
        "co_mlops_metadata_publisher_node",
    )
    share_dir = get_package_share_directory("co_mlops_rosbag_metadata")
    invalid_path = os.path.join(share_dir, "test", "fixtures", "invalid_metadata.yaml")

    env = os.environ.copy()
    env["ROS_LOCALHOST_ONLY"] = "1"

    result = subprocess.run(
        [exe, "--ros-args", "-p", f"path:={invalid_path}"],
        env=env,
        capture_output=True,
        timeout=10,
    )

    assert result.returncode == 1, (
        f"Expected exit code 1, got {result.returncode}. "
        f"stderr: {result.stderr.decode()!r}"
    )

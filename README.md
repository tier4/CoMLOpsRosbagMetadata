# Co-MLOps Rosbag Metadata

Tools for recording **Co-MLOps Rosbag Metadata** as a ROS 2 topic into rosbag during data collection, so that bags can be handled uniformly on the Co-MLOps Platform.

## Co-MLOps Platform and this repository

**Co-MLOps Platform** is a vehicle data sharing platform. By uploading rosbag data to the cloud, users can visualize, manage, and share that data and use it for MLOps. The metadata managed in this repository (**Co-MLOps Rosbag Metadata**) acts as middleware for working with rosbags on the Co-MLOps Platform: by recording the metadata as a ROS topic together with the rosbag and including it in the bag, data collected from any vehicle platform can be described in a unified way and handled consistently on the Co-MLOps Platform. The metadata format (sensing system, modules, sensors, etc.) is defined in [SCHEMA.md](SCHEMA.md).

## co_mlops_rosbag_metadata

This package provides the **Co-MLOps** metadata publisher node (`CoMLOpsMetadataPublisherNode`), which publishes the contents of a YAML file as `std_msgs/String` on a configurable topic. The file at `path` is read and validated as parseable YAML; if it is not valid YAML, the node exits with an error. Start this node when recording; by including the topic (e.g. `/metadata`) in the bag, the config can be restored on replay.

### Installation

1. **Clone the repository** (or add it to your ROS 2 workspace):

   ```bash
   git clone https://github.com/<your-org>/CoMLOpsRosbagMetadata.git
   cd CoMLOpsRosbagMetadata
   ```

2. **Install ROS 2** (if not already installed): [Install ROS 2](https://docs.ros.org/en/humble/Installation.html) (Humble or later).

3. **Install dependencies**:

   - **ROS and system dependencies** (package.xml + rosdep):

     ```bash
     rosdep update
     rosdep install --from-paths src --ignore-src -r -y
     ```

   - **Python-only dependencies** (optional; use Poetry for scripts or tooling):

     ```bash
     poetry install
     ```

   Convention: ROS-related packages are declared in `package.xml` and installed via rosdep; Python-only packages are managed in `pyproject.toml` (Poetry).

### Build

```bash
cd /path/to/CoMLOpsRosbagMetadata
source /opt/ros/<ROS_DISTRIBUTION>/setup.bash
colcon build --packages-select co_mlops_rosbag_metadata
# To build with tests
# colcon build --packages-select co_mlops_rosbag_metadata --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

### Testing

```bash
colcon test --packages-select co_mlops_rosbag_metadata
colcon test-result --verbose
```

The last command prints the test output; omit it if you only need the pass/fail result.

### Usage

#### Required parameter

- `path`: Path to the YAML file to load.

#### Optional parameters

- `topic`: Topic name (default: `/metadata`).
- `delay_before_first_publish`: Delay in seconds before the first publish; 0 or less = best effort (default: 0.0).
- `frequency`: Republish rate in Hz; use 0 for one-shot (default: 1.0).

#### Example

```bash
ros2 launch co_mlops_rosbag_metadata co_mlops_rosbag_metadata_publisher.launch.xml \
  path:=/path/to/config.yaml
```

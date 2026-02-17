# CoMLOpsRosbagMetadata

Tools for recording metadata (e.g. vehicle configuration YAML) as a ROS 2 topic into rosbag during data collection. **CoMLOps** is a platform for sharing rosbag data collected from vehicles.

## co_mlops_rosbag_metadata

This package provides the **CoMLOpsMetadataPublisherNode** Python node, which publishes the contents of a YAML file as `std_msgs/String` on a configurable topic. The file at `path` is read and validated as parseable YAML; if it is not valid YAML, the node exits with an error. Start this node when recording; by including the topic (e.g. `/metadata`) in the bag, the config can be restored on replay.

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
source install/setup.bash
```

To build with tests enabled (required before running tests), add `--cmake-args -DBUILD_TESTING=ON`:

```bash
colcon build --packages-select co_mlops_rosbag_metadata --cmake-args -DBUILD_TESTING=ON
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

Optional arguments: `topic`, `delay_before_first_publish`, `frequency`.

#### Recording example

Start the bag in one terminal, then run the node in another:

```bash
# Terminal 1
ros2 bag record -a

# Terminal 2
ros2 launch co_mlops_rosbag_metadata co_mlops_rosbag_metadata_publisher.launch.xml \
  path:=/path/to/config.yaml
```

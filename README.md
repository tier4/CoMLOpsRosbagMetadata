# Co-MLOps Rosbag Metadata

Tools for recording **Co-MLOps Rosbag Metadata** as a ROS 2 topic into rosbag during data collection, so that bags can be handled uniformly on the Co-MLOps Platform.

## Co-MLOps Platform

**Co-MLOps Platform** is a vehicle data sharing platform mainly for sharing data used in developing ADAS and end-to-end (E2E) autonomous driving models. By uploading rosbag data to the cloud, users can visualize, manage, and share that data and use it for MLOps.

## Co-MLOps Rosbag Metadata

Co-MLOps Rosbag Metadata is middleware for working with rosbags on the Co-MLOps Platform. By recording the metadata as a ROS topic together with the rosbag and including it in the bag, data collected from any vehicle can be described in a unified way and handled consistently on the platform—without out-of-band configs. The metadata is embedded in the rosbag saved by each module; on the Co-MLOps Platform, rosbags recorded with the same sensing system ID are automatically merged.

### Concepts

- **Module**: An ECU that subscribes to ROS topics and saves them as rosbag. To support cases where rosbags are saved in a distributed manner across multiple ECUs, Co-MLOps Rosbag Metadata is embedded as a ROS topic in the rosbag saved by each module.
- **Sensing system**: A collection of modules. For example, if a single central ECU subscribes to all sensor topics and saves them as rosbag, that sensing system has one module. If recording is distributed across N ECUs (e.g. when the number of sensors is large), the sensing system has N modules.

The schema (field names, types, and semantics) is defined in [SCHEMA.md](SCHEMA.md).

### Versioning

This repository uses two independent version numbers: **package version** matches the repository release version (or `0.0.0` when unreleased), and **schema version** is defined in [SCHEMA.md](SCHEMA.md). Release tags follow `vX.X.X-Y.Y.Y` (e.g. `v0.2.0-0.1.0`), where X.X.X is the package version and Y.Y.Y is the schema version; the package version written into the build is `X.X.X-Y.Y.Y`. They can diverge.

## co_mlops_rosbag_metadata

This package provides the **Co-MLOps** metadata publisher node (`CoMLOpsMetadataPublisherNode`), which publishes the contents of a YAML file as `std_msgs/String` on a configurable topic. The file at `path` is read and validated as parseable YAML; if it is not valid YAML, the node exits with an error. Start this node when recording; by including the topic (e.g. `/metadata`) in the bag, the config can be restored on replay.

### Prerequisites

This repository uses [pixi](https://pixi.sh). No system ROS 2 install is required — pixi provisions ROS 2 from RoboStack and the toolchain from conda-forge:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

### Environments

Two ROS 2 distributions are available as pixi environments; `jazzy` is the default.

- `jazzy` (default)
- `humble`

Add `-e <distro>` to any command to select a distribution; omit it to use the default (`jazzy`).

### Build

```bash
pixi run build            # default (jazzy)
pixi run -e humble build  # Humble
```

### Testing

```bash
pixi run test             # default (jazzy)
pixi run -e humble test   # Humble
```

### Usage

The `launch` task builds (if needed) and starts the publisher node. Pass the YAML path as the task argument:

```bash
pixi run launch /path/to/config.yaml
```

Or open an interactive shell with ROS 2 on `PATH` and launch manually:

```bash
pixi shell -e jazzy
ros2 launch co_mlops_rosbag_metadata co_mlops_rosbag_metadata_publisher.launch.xml \
  path:=/path/to/config.yaml
```

#### Required parameter

- `path`: Path to the YAML file to load.

#### Optional parameters

- `topic`: Topic name (default: `/metadata`).
- `delay_before_first_publish`: Delay in seconds before the first publish; 0 or less = best effort (default: 0.0).
- `frequency`: Republish rate in Hz; use 0 for one-shot (default: 1.0).

The `pixi run launch` task forwards only `path`. To set the optional parameters, use the `pixi shell` form above and pass them to `ros2 launch` (e.g. `topic:=/metadata frequency:=1.0`).

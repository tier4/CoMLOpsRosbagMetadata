# Co-MLOps Rosbag Metadata Schema

Schema version: 0.1.0

This metadata describes _what_ was recorded: which sensing system (e.g. vehicle), which module (ECU), and which sensors produced the data. It is published to a ROS 2 topic (e.g. `/metadata`) and recorded with the bag so the Co-MLOps Platform—and anyone who receives the bag—can handle it in a unified way without out-of-band configs. One metadata file describes one module (one ECU).

## Concepts

- **Sensing system**: A collection of modules (e.g. one or more ECUs); the top-level unit (e.g. a vehicle).
- **Module**: An ECU that records ROS topics into rosbag(s). One metadata file describes one module. One ECU saves at most one type of rosbag.

## Schema version and backward compatibility

Format: **MAJOR.MINOR.PATCH** (e.g. `"0.1.0"`).

| Component | Role                                                                                                      |
| --------- | --------------------------------------------------------------------------------------------------------- |
| **MAJOR** | Compatibility boundary. Same MAJOR ⇒ backward compatible. Different MAJOR ⇒ may include breaking changes. |
| **MINOR** | Additive only (new optional fields, new sensor types). Readers may ignore unknown fields.                 |
| **PATCH** | Clarifications only. No structural or type changes.                                                       |

## Example

Below is a full example; each property is described in the following sections.

```yaml
schema_version: "0.1.0"
sensing_system_name: "id1_rav4"
sensing_system_id: "6yb9g3aj"
module_id: "qu159UZU"
module_name: "ecu0"
storage_type: "mcap"
sensors:
  lidar:
    - original_topic: "/sensing/lidar/front/nebula_packets"
      mapped_topic: "/sensing/lidar/front/lidar_packets"
      frame_id: "lidar_front"
      type: "nebula_msgs/msg/NebulaPackets"
      hz: 10.0
      tos_delay_msec: 0.0
      name: "LiDAR Front"
      model: "falcon_k2c"
      maker: "seyond"
    - original_topic: "/sensing/lidar/right/nebula_packets"
      mapped_topic: "/sensing/lidar/right/lidar_packets"
      frame_id: "lidar_right"
      type: "nebula_msgs/msg/NebulaPackets"
      hz: 10.0
      tos_delay_msec: 0.0
      name: "LiDAR Right"
      model: "robin_w"
      maker: "seyond"
  camera:
    - original_topic: "/sensing/camera/camera0/image_raw/compressed"
      mapped_topic: "/sensing/camera/front_narrow/image_raw/compressed"
      frame_id: "camera0/camera_link"
      type: "sensor_msgs/msg/CompressedImage"
      hz: 20.0
      tos_delay_msec: 50.0
      name: "Camera Front Narrow"
      model: "c3_030"
      maker: "tier_iv"
      image_w: 3840
      image_h: 2160
    - original_topic: "/sensing/camera/camera1/image_raw/compressed"
      mapped_topic: "/sensing/camera/front_wide/image_raw/compressed"
      frame_id: "camera1/camera_link"
      type: "sensor_msgs/msg/CompressedImage"
      hz: 20.0
      tos_delay_msec: 50.0
      name: "Camera Front Wide"
      model: "c3_123"
      maker: "tier_iv"
      image_w: 3840
      image_h: 2160
    - original_topic: "/sensing/camera/camera2/image_raw/compressed"
      mapped_topic: "/sensing/camera/front_right/image_raw/compressed"
      frame_id: "camera2/camera_link"
      type: "sensor_msgs/msg/CompressedImage"
      hz: 20.0
      tos_delay_msec: 50.0
      name: "Camera Front Right"
      model: "c2_120"
      maker: "tier_iv"
      image_w: 2880
      image_h: 1860
    - original_topic: "/sensing/camera/camera3/image_raw/compressed"
      mapped_topic: "/sensing/camera/back_right/image_raw/compressed"
      frame_id: "camera3/camera_link"
      type: "sensor_msgs/msg/CompressedImage"
      hz: 20.0
      tos_delay_msec: 50.0
      name: "Camera Back Right"
      model: "c2_120"
      maker: "tier_iv"
      image_w: 2880
      image_h: 1860
```

## Top-level fields

| Field                 | Type   | Description                                                                                                           |
| --------------------- | ------ | --------------------------------------------------------------------------------------------------------------------- |
| `schema_version`      | string | Schema version (see above). Format: `MAJOR.MINOR.PATCH`.                                                              |
| `sensing_system_name` | string | Human-readable label for the sensing system (e.g. vehicle name). Not required to be unique.                           |
| `sensing_system_id`   | string | Unique identifier of the sensing system. Must be unique within the Co-MLOps Platform (may be issued by the Platform). |
| `module_id`           | string | Unique identifier of this module (ECU). Must be unique within the same sensing system.                                |
| `module_name`         | string | Human-readable module name (e.g. `"ecu0"`).                                                                           |
| `storage_type`        | string | Bag storage format. Supported: `"mcap"`, `"sqlite3"`.                                                                 |
| `sensors`             | object | Sensor lists keyed by type (`lidar`, `camera`, etc.). See below.                                                      |

## sensors structure

`sensors` is an object whose keys are sensor categories (e.g. `lidar`, `camera`). Each value is a list of sensor/topic entries. Each entry has the common fields described under **sensors.\*** below; some sensor types add type-specific properties (see per-type sections).

## sensors.\* (common fields)

Every sensor entry (in `sensors.lidar`, `sensors.camera`, or any other `sensors.<type>`) has these fields:

| Field            | Type   | Description                                                |
| ---------------- | ------ | ---------------------------------------------------------- |
| `original_topic` | string | ROS topic name as recorded on the ECU.                     |
| `mapped_topic`   | string | Canonical topic name (e.g. for replay or mapping).         |
| `frame_id`       | string | TF frame ID for this sensor.                               |
| `type`           | string | ROS message type (e.g. `"nebula_msgs/msg/NebulaPackets"`). |
| `hz`             | float  | Nominal publish rate (Hz).                                 |
| `tos_delay_msec` | float  | Time-of-flight or trigger delay (ms).                      |
| `name`           | string | Human-readable sensor name.                                |
| `model`          | string | Sensor model identifier.                                   |
| `maker`          | string | Sensor manufacturer or provider.                           |

## sensors.lidar

No type-specific properties. Each entry uses only the common fields in **sensors.\***.

## sensors.camera

Entries under `sensors.camera` add the following type-specific properties:

| Field     | Type    | Description            |
| --------- | ------- | ---------------------- |
| `image_w` | integer | Image width (pixels).  |
| `image_h` | integer | Image height (pixels). |

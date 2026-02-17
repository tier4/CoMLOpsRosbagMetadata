// Copyright 2026 TIER IV, Inc.

#ifndef COMLOPS_ROSBAG_METADATA__CO_MLOPS_ROSBAG_METADATA_PUBLISHER_NODE_HPP_
#define COMLOPS_ROSBAG_METADATA__CO_MLOPS_ROSBAG_METADATA_PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace co_mlops_rosbag_metadata
{

/**
 * @brief ROS 2 node that publishes the contents of a YAML file as std_msgs/String on a topic.
 *
 * The file at the "path" parameter is read, validated as parseable YAML, then published once
 * after an optional delay and optionally republished at a configurable frequency (e.g. for
 * late-started rosbag recording).
 */
class CoMLOpsMetadataPublisherNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the node, load and validate the YAML file, and set up publishing.
   *
   * If the path is missing, the file cannot be read, or the file is not valid YAML,
   * the node does not start publishing and initialized() returns false.
   */
  CoMLOpsMetadataPublisherNode();

  /**
   * @brief Return whether the node successfully loaded the YAML and is ready to publish.
   * @return true if the file was loaded and validated, false otherwise.
   */
  bool initialized() const;

private:
  /**
   * @brief Read the entire contents of a file into a string.
   * @param path Path to the file.
   * @param out Output string; set only on success.
   * @return true if the file was read successfully, false otherwise.
   */
  static bool read_file(const std::string & path, std::string & out);

  /**
   * @brief Check whether a string is parseable as YAML.
   * @param content String content to validate.
   * @return true if the content is valid YAML, false otherwise.
   */
  static bool is_valid_yaml(const std::string & content);

  /**
   * @brief Publish the loaded YAML content once to the configured topic.
   */
  void publish_once();

  /** @brief Publisher for the YAML content as std_msgs/String. */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /** @brief One-shot timer for the first publish after delay. */
  rclcpp::TimerBase::SharedPtr first_timer_;
  /** @brief Optional periodic timer for republishing. */
  rclcpp::TimerBase::SharedPtr period_timer_;
  /** @brief Cached message containing the YAML file content. */
  std_msgs::msg::String msg_;
  /** @brief Topic name to publish to. */
  std::string topic_;
  /** @brief Whether the node successfully loaded and validated the YAML file. */
  bool initialized_;
};

}  // namespace co_mlops_rosbag_metadata

#endif  // COMLOPS_ROSBAG_METADATA__CO_MLOPS_ROSBAG_METADATA_PUBLISHER_NODE_HPP_

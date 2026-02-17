// Copyright 2026 TIER IV, Inc.

#include "co_mlops_rosbag_metadata/co_mlops_rosbag_metadata_publisher_node.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

namespace co_mlops_rosbag_metadata
{

CoMLOpsMetadataPublisherNode::CoMLOpsMetadataPublisherNode()
: Node("co_mlops_metadata_publisher_node"), initialized_(false)
{
  declare_parameter<std::string>("path", "");
  declare_parameter<std::string>("topic", "/metadata");
  declare_parameter<double>("delay_before_first_publish", 0.0);
  declare_parameter<double>("frequency", 1.0);

  const std::string path = get_parameter("path").as_string();
  if (path.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Parameter 'path' is required. Example: --ros-args -p "
      "path:=/path/to/config.yaml");
    return;
  }

  std::string content;
  if (!read_file(path, content)) {
    RCLCPP_ERROR(get_logger(), "Failed to read file: %s", path.c_str());
    return;
  }

  if (!is_valid_yaml(content)) {
    RCLCPP_ERROR(get_logger(), "File is not valid YAML: %s", path.c_str());
    return;
  }

  initialized_ = true;

  topic_ = get_parameter("topic").as_string();
  publisher_ = create_publisher<std_msgs::msg::String>(topic_, 10);
  msg_.data = content;

  const double delay_param = get_parameter("delay_before_first_publish").as_double();
  const double delay = (delay_param > 0.0) ? delay_param : 0.0;  // 0 or less = best effort
  const double frequency = get_parameter("frequency").as_double();

  // Publish first time: after delay if positive, else best effort (delay 0)
  first_timer_ = create_wall_timer(std::chrono::duration<double>(delay), [this]() {
    publish_once();
    first_timer_->cancel();
  });

  // Optionally republish at given rate so late-started rosbag still gets config
  if (frequency > 0.0) {
    const double period = 1.0 / frequency;
    period_timer_ =
      create_wall_timer(std::chrono::duration<double>(period), [this]() { publish_once(); });
  }
}

bool CoMLOpsMetadataPublisherNode::initialized() const { return initialized_; }

bool CoMLOpsMetadataPublisherNode::read_file(const std::string & path, std::string & out)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    return false;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

bool CoMLOpsMetadataPublisherNode::is_valid_yaml(const std::string & content)
{
  try {
    YAML::Load(content);
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

void CoMLOpsMetadataPublisherNode::publish_once()
{
  publisher_->publish(msg_);
  RCLCPP_INFO(
    get_logger(), "Published vehicle config to %s (%zu bytes)", topic_.c_str(), msg_.data.size());
}

}  // namespace co_mlops_rosbag_metadata

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<co_mlops_rosbag_metadata::CoMLOpsMetadataPublisherNode>();
  if (!node->initialized()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

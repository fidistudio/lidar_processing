#include "lidar_processing/obstacle_extractor_node.hpp"

#include <cmath>
#include <numeric>
#include <string>
#include <vector>

namespace lidar_processing {

// ── Constructor
// ───────────────────────────────────────────────────────────────

ObstacleExtractorNode::ObstacleExtractorNode(
    std::shared_ptr<ITransformStrategy> strategy,
    const rclcpp::NodeOptions &options)
    : Node("obstacle_extractor", options), strategy_{std::move(strategy)} {
  declare_parameter("scan_topic", "/scan");
  declare_parameter("obstacles_topic", "/obstacles");

  const auto scan_topic = get_parameter("scan_topic").as_string();
  const auto obstacles_topic = get_parameter("obstacles_topic").as_string();

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 10,
      [this](const sensor_msgs::msg::LaserScan &msg) { scanCallback(msg); });

  obstacle_pub_ = create_publisher<robot_interfaces::msg::ObstacleArray>(
      obstacles_topic, 10);

  RCLCPP_INFO(get_logger(),
              "ObstacleExtractorNode ready | scan: %s → obstacles: %s",
              scan_topic.c_str(), obstacles_topic.c_str());
}

// ── Strategy setter
// ───────────────────────────────────────────────────────────

void ObstacleExtractorNode::setStrategy(
    std::shared_ptr<ITransformStrategy> strategy) {
  strategy_ = std::move(strategy);
}

// ── Scan callback
// ─────────────────────────────────────────────────────────────

void ObstacleExtractorNode::scanCallback(
    const sensor_msgs::msg::LaserScan &msg) {
  robot_interfaces::msg::ObstacleArray obstacles_msg;
  obstacles_msg.header.stamp = msg.header.stamp;
  obstacles_msg.header.frame_id = "base_footprint";

  if (!strategy_) {
    RCLCPP_WARN_ONCE(get_logger(), "No transform strategy set — scan ignored.");
    return;
  }

  // 1. Collect valid indices (finite range, above minimum threshold)
  std::vector<int> valid_indices;
  valid_indices.reserve(msg.ranges.size());

  for (int i = 0; i < static_cast<int>(msg.ranges.size()); ++i) {
    const float r = msg.ranges[i];
    if (std::isfinite(r) && r > 0.1f) {
      valid_indices.push_back(i);
    }
  }

  if (valid_indices.empty()) {
    obstacle_pub_->publish(obstacles_msg);
    return;
  }

  // 2. Cluster contiguous indices
  const auto clusters = clusterIndices(valid_indices);

  for (const auto &cluster : clusters) {
    if (cluster.empty()) {
      continue;
    }

    // 3. Compute centroid in LIDAR frame
    double sum_x = 0.0;
    double sum_y = 0.0;

    for (const int idx : cluster) {
      const double angle = msg.angle_min + idx * msg.angle_increment;
      sum_x += msg.ranges[idx] * std::cos(angle);
      sum_y += msg.ranges[idx] * std::sin(angle);
    }

    const double cx = sum_x / static_cast<double>(cluster.size());
    const double cy = sum_y / static_cast<double>(cluster.size());

    // 4. Delegate transformation to the injected strategy
    const auto point =
        strategy_->transform(cx, cy, msg.header.stamp, msg.header.frame_id);

    if (!point.has_value()) {
      continue; // Transform unavailable this cycle; skip cluster silently
    }

    // 5. Build and append the obstacle
    robot_interfaces::msg::Obstacle obs;
    obs.x = point->x;
    obs.y = point->y;
    obs.distance = std::hypot(point->x, point->y);
    obstacles_msg.obstacles.push_back(obs);
  }

  obstacle_pub_->publish(obstacles_msg);
}

// ── Helpers
// ───────────────────────────────────────────────────────────────────

std::vector<std::vector<int>>
ObstacleExtractorNode::clusterIndices(const std::vector<int> &valid_indices) {
  std::vector<std::vector<int>> clusters;

  if (valid_indices.empty()) {
    return clusters;
  }

  clusters.push_back({valid_indices.front()});

  for (std::size_t i = 1; i < valid_indices.size(); ++i) {
    if (valid_indices[i] - valid_indices[i - 1] > 1) {
      clusters.push_back({}); // Gap detected → start new cluster
    }
    clusters.back().push_back(valid_indices[i]);
  }

  return clusters;
}

} // namespace lidar_processing

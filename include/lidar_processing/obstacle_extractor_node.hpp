#pragma once

#include "lidar_processing/transform_strategy.hpp"

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/obstacle.hpp"
#include "robot_interfaces/msg/obstacle_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace lidar_processing {

/**
 * @brief ROS 2 node that extracts obstacles from a LaserScan.
 *
 * Pipeline per scan:
 *   1. Filter invalid / out-of-range readings.
 *   2. Cluster contiguous valid indices.
 *   3. Compute the centroid of each cluster in the LIDAR frame.
 *   4. Delegate frame transformation to an ITransformStrategy.
 *   5. Publish the resulting ObstacleArray.
 *
 * The node is completely decoupled from the transformation method:
 * inject ManualTransform for simulation or Tf2Transform for the real robot.
 */
class ObstacleExtractorNode : public rclcpp::Node {
public:
  /**
   * @param strategy  Concrete transform strategy (injected at construction).
   * @param options   Standard NodeOptions forwarded to rclcpp::Node.
   */
  explicit ObstacleExtractorNode(
      std::shared_ptr<ITransformStrategy> strategy,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});

  /**
   * @brief Replace the active transform strategy at runtime.
   *
   * Intended for the real-robot main(), which needs to construct the node
   * first (to get a clock for tf2) and inject Tf2Transform right after,
   * before the first scan arrives.
   */
  void setStrategy(std::shared_ptr<ITransformStrategy> strategy);

private:
  // ── ROS callbacks ──────────────────────────────────────────────────────────
  void scanCallback(const sensor_msgs::msg::LaserScan &msg);

  // ── Processing helpers ────────────────────────────────────────────────────
  static std::vector<std::vector<int>>
  clusterIndices(const std::vector<int> &valid_indices);

  // ── Members ───────────────────────────────────────────────────────────────
  std::shared_ptr<ITransformStrategy> strategy_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<robot_interfaces::msg::ObstacleArray>::SharedPtr
      obstacle_pub_;
};

} // namespace lidar_processing

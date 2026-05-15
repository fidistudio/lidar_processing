#pragma once

#include "lidar_processing/transform_strategy.hpp"

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace lidar_processing {

/**
 * @brief Real-robot strategy using tf2 runtime transforms.
 *
 * Queries tf2 using the LaserScan timestamp for temporal consistency.
 */
class Tf2Transform final : public ITransformStrategy {
public:
  /**
   * @param node          ROS node providing clock + logger.
   * @param target_frame  Destination frame.
   */
  explicit Tf2Transform(rclcpp::Node &node,
                        std::string target_frame = "base_footprint");

  [[nodiscard]] std::optional<Point2D>
  transform(double cx, double cy, const rclcpp::Time &stamp,
            const std::string &lidar_frame) const override;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::string target_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace lidar_processing

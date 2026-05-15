#pragma once

#include "lidar_processing/transform_strategy.hpp"

#include <array>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace lidar_processing {

/**
 * @brief Simulation strategy: applies a fixed XY offset to the centroid.
 *
 * Reads the LIDAR→base_footprint translation directly from the URDF
 * (or from ROS parameters), with no runtime TF lookup.
 * Intended for Gazebo / mock environments where tf2 may not be running.
 */
class ManualTransform final : public ITransformStrategy {
public:
  /**
   * @param translation_x  X offset from LIDAR origin to base_footprint [m].
   * @param translation_y  Y offset from LIDAR origin to base_footprint [m].
   */
  explicit ManualTransform(double translation_x, double translation_y);

  [[nodiscard]] std::optional<Point2D>
  transform(double cx, double cy, const rclcpp::Time &stamp,
            const std::string &lidar_frame) const override;

private:
  double translation_x_;
  double translation_y_;
};

} // namespace lidar_processing

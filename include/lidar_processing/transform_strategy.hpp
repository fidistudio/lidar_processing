#pragma once

#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace lidar_processing {

/**
 * @brief Represents a 2D point in the target frame.
 */
struct Point2D {
  double x{0.0};
  double y{0.0};
};

/**
 * @brief Pure abstract interface for centroid transform strategies.
 *
 * Implementations decide how to convert a centroid expressed in the
 * LIDAR frame into the desired target frame (e.g. base_footprint).
 *
 * Single Responsibility  – only knows about coordinate transformation.
 * Open/Closed            – new strategies extend this interface without
 *                          modifying ObstacleExtractorNode.
 * Liskov Substitution    – every concrete strategy is a drop-in replacement.
 * Interface Segregation  – one focused method; no unrelated surface.
 * Dependency Inversion   – ObstacleExtractorNode depends on this abstraction,
 *                          not on ManualTransform or Tf2Transform directly.
 */
class ITransformStrategy {
public:
  virtual ~ITransformStrategy() = default;

  /**
   * @brief Transform a centroid from the LIDAR frame to the target frame.
   *
   * @param cx          Centroid X in the LIDAR frame [m].
   * @param cy          Centroid Y in the LIDAR frame [m].
   * @param stamp       Timestamp of the originating LaserScan message.
   * @param lidar_frame tf frame_id reported by the LaserScan header.
   * @return            Centroid expressed in the target frame, or
   *                    std::nullopt when the transform is unavailable.
   */
  [[nodiscard]] virtual std::optional<Point2D>
  transform(double cx, double cy, const rclcpp::Time &stamp,
            const std::string &lidar_frame) const = 0;
};

} // namespace lidar_processing

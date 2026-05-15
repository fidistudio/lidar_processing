#include "lidar_processing/manual_transform.hpp"

namespace lidar_processing {

ManualTransform::ManualTransform(double translation_x, double translation_y)
    : translation_x_{translation_x}, translation_y_{translation_y} {}

std::optional<Point2D>
ManualTransform::transform(double cx, double cy, const rclcpp::Time & /*stamp*/,
                           const std::string & /*lidar_frame*/) const {
  return Point2D{cx + translation_x_, cy + translation_y_};
}

} // namespace lidar_processing

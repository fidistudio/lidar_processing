#include "lidar_processing/tf2_transform.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2/exceptions.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace lidar_processing {

Tf2Transform::Tf2Transform(rclcpp::Node &node, std::string target_frame)
    : logger_{node.get_logger()}, clock_{node.get_clock()},
      target_frame_{std::move(target_frame)},
      tf_buffer_{std::make_shared<tf2_ros::Buffer>(clock_)} {

  tf_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, &node, false);
}

std::optional<Point2D>
Tf2Transform::transform(double cx, double cy, const rclcpp::Time &stamp,
                        const std::string &lidar_frame) const {

  geometry_msgs::msg::TransformStamped tf_stamped;

  try {

    tf_stamped = tf_buffer_->lookupTransform(
        target_frame_, lidar_frame, stamp, rclcpp::Duration::from_seconds(0.1));

  } catch (const tf2::TransformException &ex) {

    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Cannot get TF %s -> %s: %s",
                         lidar_frame.c_str(), target_frame_.c_str(), ex.what());

    return std::nullopt;
  }

  geometry_msgs::msg::PointStamped point_in;

  point_in.header.stamp = stamp;
  point_in.header.frame_id = lidar_frame;

  point_in.point.x = cx;
  point_in.point.y = cy;
  point_in.point.z = 0.0;

  geometry_msgs::msg::PointStamped point_out;

  tf2::doTransform(point_in, point_out, tf_stamped);

  return Point2D{point_out.point.x, point_out.point.y};
}

} // namespace lidar_processing

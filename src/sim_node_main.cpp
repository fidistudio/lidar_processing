#include "lidar_processing/manual_transform.hpp"
#include "lidar_processing/obstacle_extractor_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Read translation from a temporary node so we can use ROS params
  // before constructing the real node.
  auto param_node = std::make_shared<rclcpp::Node>("_lidar_param_reader");
  param_node->declare_parameter("lidar_tf_x", 0.3);
  param_node->declare_parameter("lidar_tf_y", 0.0);

  const double tf_x = param_node->get_parameter("lidar_tf_x").as_double();
  const double tf_y = param_node->get_parameter("lidar_tf_y").as_double();

  // Inject ManualTransform → simulation / Gazebo
  auto strategy =
      std::make_shared<lidar_processing::ManualTransform>(tf_x, tf_y);
  auto node =
      std::make_shared<lidar_processing::ObstacleExtractorNode>(strategy);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

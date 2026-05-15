#include "lidar_processing/obstacle_extractor_node.hpp"
#include "lidar_processing/tf2_transform.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Tf2Transform needs a node reference to grab the clock and spin
  // the TransformListener — we pass the extractor node itself.
  //
  // To avoid a chicken-and-egg problem we first let the node construct
  // without a strategy, then build the strategy with the node reference,
  // and finally assign it. We do this cleanly by using a two-phase init
  // helper struct instead of raw pointers.

  // Phase 1 – create the ROS node (no strategy yet).
  auto node = std::make_shared<lidar_processing::ObstacleExtractorNode>(
      nullptr // temporary; replaced immediately below
  );

  // Phase 2 – build Tf2Transform referencing the live node.
  auto strategy = std::make_shared<lidar_processing::Tf2Transform>(*node);

  // Phase 3 – inject strategy before the first scan arrives.
  node->setStrategy(strategy);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

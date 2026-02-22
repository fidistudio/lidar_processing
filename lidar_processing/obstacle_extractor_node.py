import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from robot_interfaces.msg import Obstacle, ObstacleArray

import numpy as np
import math


class ObstacleExtractor(Node):

    def __init__(self):
        super().__init__("obstacle_extractor")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("obstacles_topic", "/obstacles")
        self.declare_parameter("target_frame", "base_footprint")

        scan_topic = self.get_parameter("scan_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value

        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.obstacle_pub = self.create_publisher(ObstacleArray, obstacles_topic, 10)

        # Matriz de transformación manual LIDAR -> base_footprint
        # Traslación según URDF: (0.3, 0, 0.5)
        self.tf_translation = np.array([0.3, 0.0, 0.5])

    def scan_callback(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges)
        valid_idx = np.where(np.isfinite(ranges))[0]

        obstacles_msg = ObstacleArray()
        obstacles_msg.header.stamp = msg.header.stamp
        obstacles_msg.header.frame_id = "base_footprint"

        if len(valid_idx) == 0:
            self.obstacle_pub.publish(obstacles_msg)
            return

        clusters = self._cluster_indices(valid_idx)

        for i, cluster in enumerate(clusters):
            cluster = np.array(cluster, dtype=int)
            if len(cluster) == 0:
                continue

            cluster_ranges = ranges[cluster]
            angles = msg.angle_min + cluster * msg.angle_increment

            # Coordenadas en el frame del LIDAR
            xs = cluster_ranges * np.cos(angles)
            ys = cluster_ranges * np.sin(angles)

            # Centroide
            cx = float(np.mean(xs))
            cy = float(np.mean(ys))

            # Log centroide en LIDAR
            # self.get_logger().info(
            #    f"[Cluster {i}] Centroide en LIDAR: x={cx:.3f}, y={cy:.3f}"
            # )

            # Transformación manual a base_footprint
            bx = cx + self.tf_translation[0]
            by = cy + self.tf_translation[1]

            # self.get_logger().info(
            #     f"[Cluster {i}] Transformado a base_footprint: x={bx:.3f}, y={by:.3f}"
            # )

            # Crear Obstacle
            obs = Obstacle()
            obs.x = bx
            obs.y = by
            obs.distance = math.hypot(bx, by)
            obstacles_msg.obstacles.append(obs)

        self.obstacle_pub.publish(obstacles_msg)

    @staticmethod
    def _cluster_indices(indices):
        splits = np.where(np.diff(indices) > 1)[0] + 1
        return np.split(indices, splits)


def main():
    rclpy.init()
    node = ObstacleExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from robot_interfaces.msg import Obstacle, ObstacleArray

import numpy as np
import math

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs


class ObstacleExtractor(Node):

    def __init__(self):
        super().__init__("obstacle_extractor")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("obstacles_topic", "/obstacles")
        self.declare_parameter("target_frame", "base_footprint")

        scan_topic = self.get_parameter("scan_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value
        self.target_frame = self.get_parameter("target_frame").value

        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.obstacle_pub = self.create_publisher(ObstacleArray, obstacles_topic, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def scan_callback(self, msg: LaserScan) -> None:

        ranges = np.array(msg.ranges)

        # Índices válidos (sin inf / nan)
        valid_idx = np.where(np.isfinite(ranges))[0]

        obstacles_msg = ObstacleArray()
        obstacles_msg.header = msg.header
        obstacles_msg.header.frame_id = self.target_frame

        if len(valid_idx) == 0:
            self.obstacle_pub.publish(obstacles_msg)
            return

        clusters = self._cluster_indices(valid_idx)

        for cluster in clusters:

            # Aseguramos que sea array 1D
            cluster = np.array(cluster, dtype=int)

            if len(cluster) == 0:
                continue

            cluster_ranges = ranges[cluster]

            # Ángulos en lidar_link
            angles = msg.angle_min + cluster * msg.angle_increment

            # Coordenadas en lidar_link
            xs = cluster_ranges * np.cos(angles)
            ys = cluster_ranges * np.sin(angles)

            # Centroide en lidar_link
            cx = float(np.mean(xs))
            cy = float(np.mean(ys))

            point_lidar = PointStamped()
            point_lidar.header.frame_id = msg.header.frame_id
            point_lidar.header.stamp = msg.header.stamp
            point_lidar.point.x = cx
            point_lidar.point.y = cy
            point_lidar.point.z = 0.0

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id, msg.header.stamp
                )

                point_transformed = tf2_geometry_msgs.do_transform_point(
                    point_lidar, transform
                )

            except TransformException as ex:
                self.get_logger().warn(f"TF error: {ex}")
                return

            obs = Obstacle()
            obs.x = point_transformed.point.x
            obs.y = point_transformed.point.y
            obs.distance = math.hypot(obs.x, obs.y)

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

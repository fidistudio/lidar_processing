import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from robot_interfaces.msg import Obstacle, ObstacleArray

import numpy as np
import math

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # registra el soporte de transformación para PointStamped


class ObstacleExtractor(Node):
    def __init__(self):
        super().__init__("obstacle_extractor")

        # ── Parámetros ──────────────────────────────────────────────────────
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("obstacles_topic", "/obstacles")
        self.declare_parameter("target_frame", "base_footprint")

        scan_topic = self.get_parameter("scan_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value
        self.target_frame_ = self.get_parameter("target_frame").value

        # ── tf2 ──────────────────────────────────────────────────────────────
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # ── Pub / Sub ────────────────────────────────────────────────────────
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.obstacle_pub_ = self.create_publisher(ObstacleArray, obstacles_topic, 10)

        self.get_logger().info(
            f"ObstacleExtractor iniciado | scan: {scan_topic} "
            f"→ frame: {self.target_frame_}"
        )

    # ── Callback principal ───────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges)
        valid_idx = np.where(np.isfinite(ranges) & (ranges > 0.1))[0]
        obstacles_msg = ObstacleArray()
        obstacles_msg.header.stamp = msg.header.stamp
        obstacles_msg.header.frame_id = self.target_frame_

        if len(valid_idx) == 0:
            self.obstacle_pub_.publish(obstacles_msg)
            return

        # Obtener la transformación lidar_frame → target_frame UNA vez por scan.
        # Usar el timestamp del scan para máxima precisión temporal.
        try:
            tf = self.tf_buffer_.lookup_transform(
                self.target_frame_,  # frame destino
                msg.header.frame_id,  # frame origen (lidar_link)
                msg.header.stamp,  # instante del scan
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"No se pudo obtener TF {msg.header.frame_id} → "
                f"{self.target_frame_}: {e}",
                throttle_duration_sec=2.0,
            )
            return

        clusters = self._cluster_indices(valid_idx)

        for cluster in clusters:
            cluster = np.array(cluster, dtype=int)
            if len(cluster) == 0:
                continue

            cluster_ranges = ranges[cluster]
            angles = msg.angle_min + cluster * msg.angle_increment

            # Coordenadas en el frame del LIDAR
            xs = cluster_ranges * np.cos(angles)
            ys = cluster_ranges * np.sin(angles)

            # Centroide en frame LIDAR
            cx = float(np.mean(xs))
            cy = float(np.mean(ys))

            # Transformar centroide a target_frame usando tf2
            point_lidar = PointStamped()
            point_lidar.header.stamp = msg.header.stamp
            point_lidar.header.frame_id = msg.header.frame_id
            point_lidar.point.x = cx
            point_lidar.point.y = cy
            point_lidar.point.z = 0.0

            point_base = tf2_geometry_msgs.do_transform_point(point_lidar, tf)

            bx = point_base.point.x
            by = point_base.point.y

            obs = Obstacle()
            obs.x = bx
            obs.y = by
            obs.distance = math.hypot(bx, by)
            obstacles_msg.obstacles.append(obs)

        self.obstacle_pub_.publish(obstacles_msg)

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _cluster_indices(indices):
        splits = np.where(np.diff(indices) > 1)[0] + 1
        return np.split(indices, splits)


# ── Main ─────────────────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node = ObstacleExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

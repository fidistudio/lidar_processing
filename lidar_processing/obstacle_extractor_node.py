import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from robot_interfaces.msg import Obstacle, ObstacleArray
import numpy as np
import math


class ObstacleExtractor(Node):

    def __init__(self):
        super().__init__("obstacle_extractor")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("obstacles_topic", "/obstacles")

        scan_topic = self.get_parameter("scan_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value

        self.pose = None

        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.create_subscription(Pose2D, pose_topic, self.pose_callback, 10)

        self.obstacle_pub = self.create_publisher(ObstacleArray, obstacles_topic, 10)

    def pose_callback(self, msg: Pose2D) -> None:
        self.pose = msg

    def scan_callback(self, msg: LaserScan) -> None:
        if self.pose is None:
            return

        ranges = np.array(msg.ranges)
        valid_idx = np.where(np.isfinite(ranges))[0]

        obstacles_msg = ObstacleArray()

        if len(valid_idx) == 0:
            self.obstacle_pub.publish(obstacles_msg)
            return

        clusters = self._cluster_indices(valid_idx)

        for cluster in clusters:
            cluster_ranges = ranges[cluster]

            mask = np.isfinite(cluster_ranges)
            cluster = cluster[mask]
            cluster_ranges = cluster_ranges[mask]

            if len(cluster) == 0:
                continue

            angles = msg.angle_min + cluster * msg.angle_increment + self.pose.theta

            # Puntos XY globales
            xs = self.pose.x + cluster_ranges * np.cos(angles)
            ys = self.pose.y + cluster_ranges * np.sin(angles)

            # Centroide
            cx = float(np.mean(xs))
            cy = float(np.mean(ys))

            # Distancia centroide -> robot
            dx = cx - self.pose.x
            dy = cy - self.pose.y
            d_centroid = float(math.sqrt(dx * dx + dy * dy))

            obs = Obstacle()
            obs.x = cx
            obs.y = cy
            obs.distance = d_centroid

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

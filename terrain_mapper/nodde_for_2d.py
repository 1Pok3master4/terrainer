import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Empty
import math
import csv
import numpy as np

class TerrainMapping(Node):
    def __init__(self):
        super().__init__("twod_mapper")
        self.lidar_dt = self.create_subscription(PointCloud2, "/simple_drone/lidar/out", self.lidar_dt_callback, 10)
        self.get_pose = self.create_subscription(Pose, "/simple_drone/gt_pose", self.pose_callback, 10)
        self.land = self.create_publisher(Empty, "/simple_drone/land", 10)
        self.map_dt = set()
        self.saved = False

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def pose_callback(self, msg: Pose):
        pose = msg
        self.x = round(pose.position.x)
        self.y = round(pose.position.y)
        self.z = round(pose.position.z)

    
    def lidar_dt_callback(self, msg: PointCloud2):
        # This will extract points as a generator of (x, y, z)
        points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)

        # Convert to list or numpy for easy processing
        #point_list = list(points)
        for point in points:
            x, y, z = point
            coord = (round(x+self.x, 2), round(y+self.y, 2), round(z+self.z, 1))
            key = (x, y)
            if key not in self.map_dt:
                self.map_dt.add(coord)

        if len(self.map_dt) >= 500000:
            self.save_to_csv()
            self.get_logger().info("Saved points to terrain_map.csv")
            self.land.publish(Empty)
        print("Number of points saved", len(self.map_dt))

    def save_to_csv(self):
        with open('/home/mocha1410/ros2_ws/src/terrain_mapper/terrain_mapper/maps/terrain_map.csv', mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "z"])
            for coord in self.map_dt:
                writer.writerow(coord)

def main(args=None):
    rclpy.init(args=args)
    node = TerrainMapping()
    rclpy.spin(node)
    rclpy.shutdown()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose
import csv

class TerrainMapper(Node):
    def __init__(self):
        super().__init__("Terrain_Mapper")
        
        #Subscribers

        self.sonar_out = self.create_subscription(Range, "/simple_drone/sonar/out", self.sonar_dt_callback, 10)
        self.gt_pose = self.create_subscription(Pose, "/simple_drone/gt_pose", self.pose_callback, 10)
        
        #Publishers

        #self.takeoff = self.create_publisher(Empty, "/simple_drone/takeoff", 10)
        #self.vel = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)
        #self.land = self.create_publisher(Empty, "/simple_drone/land", 10)
        
        #self.create_timer(0.1, self.vel_callback)
        
        #self.takeoff.publish(Empty())
        self.flag = False
        self.positions_set = set()
        self.map_dt = []
        self.object_depth = None
        self.map_length = 10000

        

    def sonar_dt_callback(self, msg: Range):
        if msg is not None:
            sonar_dt = msg
            self.object_depth = sonar_dt.range

    def save_map_to_file(self):
        with open('/home/mocha1410/ros2_ws/src/terrain_mapper/terrain_mapper/maps/terrain_map.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'height'])  # Write header
            for point in self.map_dt:
                writer.writerow(point)

    def pose_callback(self, msg: Pose):
        if msg is not None:
            pose = msg
            current_position = (round(pose.position.x, 2), round(pose.position.y, 2))
            if self.object_depth is not None:
                height = pose.position.z - self.object_depth
                dt = (round(pose.position.x, 2), round(pose.position.y, 2), round(height, 2))
                if current_position not in self.positions_set:
                    self.positions_set.add(current_position)
                    self.map_dt.append(dt)
                    if len(self.map_dt) == self.map_length:
                        self.get_logger().info(f"Saving map to file (mapped {len(self.map_dt)} points)")
                        self.save_map_to_file()
            else:
                pass
        if self.flag is False:
            print(self.map_length)
            self.flag = True


def main(args=None):
    rclpy.init(args=args)
    node = TerrainMapper()
    rclpy.spin(node)
    rclpy.shutdown()
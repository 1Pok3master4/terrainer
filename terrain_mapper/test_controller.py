#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty, String
import json
from sensor_msgs.msg import PointCloud2
import numpy as np
import csv
import sensor_msgs_py.point_cloud2 as pc2
import math


class AutonomousSurvey(Node):
    def __init__(self) -> None:
        super().__init__('survey_node')
        self.takeoff = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.takeoff.publish(Empty())
        self.lidar_dt = self.create_subscription(PointCloud2, "/simple_drone/lidar/out", self.lidar_dt_callback, 10)
        self.get_pose = self.create_subscription(Pose, "/simple_drone/gt_pose", self.pose_callback, 10)
        self.land = self.create_publisher(Empty, "/simple_drone/land", 10)
        self.waypoints_dt = self.create_publisher(String, "/waypoints", 10)
        self.map_dt = set()
        self.grid_size = 8
        self.vel_cmd = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.vel_callback)
        self.state = "initial"
        self.x = None
        self.y = None
        self.z = None
        self.height = 5.0
        self.target_spacing = np.round(1.65*self.height*math.tan(math.pi/6), 1)
        self.targets = {}
        self.targets_number = int(self.grid_size/self.target_spacing)+1
        for i in range(0, 2*self.targets_number+2, 2):
            l = float((i/2)*self.target_spacing)
            if i%4 ==0:
                key1 = "Waypoint " f" {i}"
                key2 = "Waypoint " f" {i+1}"
            if i%4 !=0:
                key2 = "Waypoint " f" {i}"
                key1 = "Waypoint " f" {i+1}"
            self.targets[key1] = (l, 0)
            self.targets[key2] = (l, float(self.target_spacing*self.targets_number))
        self.current_waypoint = 0
        
        self.timer1 = self.create_timer(0.1, self.waypoint_callback)

    def waypoint_callback(self):
        msg = String()
        msg.data = json.dumps(self.targets)
        self.waypoints_dt.publish(msg)
    
    def pose_callback(self, msg: Pose):
        if msg is not None:
            pose = msg
            self.x = round(pose.position.x, 2)
            self.y = round(pose.position.y, 2)
            self.z = round(pose.position.z, 2)
        else:
            self.takeoff.publish(Empty())

    
    def vel_callback(self):
        if self.x is None or self.y is None or self.z is None:
            return
        vel = Twist()
        target_height = self.height
        error = target_height - self.z
        current_waypoint = self.current_waypoint
        if self.current_waypoint < len(self.targets):
            target = self.targets["Waypoint " f" {current_waypoint}"]
            target_x = target[0]
            target_y = target[1]
            kp = 0.1
            error_x = target_x - self.x
            error_y = target_y - self.y
            vel_x = kp*error_x
            vel_y = kp*error_y
        
        
        if abs(error)>0.1:
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.6*error
        else:
            vel.linear.z = 0.0
            self.state = "data_collection"
            if "Waypoint " f" {current_waypoint}" in self.targets:
                if abs(error_x)>0.3 or abs(error_y)>0.3:
                    vel.linear.x = vel_x
                    vel.linear.y = vel_y
                else:
                    vel.linear.x = vel.linear.y = 0.0
                    self.current_waypoint += 1

            if "Waypoint " f" {current_waypoint}" not in self.targets:
                vel.linear.x = vel.linear.y = vel.linear.z = 0.0
                self.state = "returning"
        if self.state == "returning":
            target_x = 0.0
            target_y = 0.0
            kp = 0.1
            error_x = target_x - self.x
            error_y = target_y - self.y
            vel.linear.x = kp*error_x
            vel.linear.y = kp*error_y
        if self.state == "returning" and abs(self.x) <0.1 and abs(self.y)<0.1:
            vel.linear.x = vel.linear.y = 0.0
            target = 0.0
            error_z = target-self.z
            vel.linear.z = 0.5*error_z
            if abs(error_z)<0.1:
                self.land.publish(Empty)

        self.vel_cmd.publish(vel)

    def lidar_dt_callback(self, msg: PointCloud2):
        points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)

        if self.state == "data_collection":
            for point in points:
                x, y, z = point
                coord = (round(x+self.x, 1), round(y+self.y, 1), round(z+self.z, 1))
                key = (x, y)
                if key not in self.map_dt:
                    self.map_dt.add(coord)
            self.save_to_csv()
            self.get_logger().info("Saved " f"{len(self.map_dt)}"  "points to terrain_map.csv")

        elif self.state == "initial":
            self.get_logger().info("Drone is setting up")
        elif self.state == "returning":
            self.get_logger().info("All waypoints reached. Drone back on the way")


    def save_to_csv(self):
        with open('/home/mocha1410/ros2_ws/src/terrain_mapper/terrain_mapper/maps/terrain_map.csv', mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "z"])
            for coord in self.map_dt:
                writer.writerow(coord)
                
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSurvey()
    rclpy.spin(node)
    rclpy.shutdown()
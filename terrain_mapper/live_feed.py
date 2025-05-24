#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import json
import csv
import numpy as np

CSV_PATH = '/home/mocha1410/ros2_ws/src/terrain_mapper/terrain_mapper/maps/terrain_map.csv'

class LiveMapVisualizer(Node):
    def __init__(self):
        super().__init__('live_map_visualizer')
        self.lock = threading.Lock()
        self.waypoints = {}
        self.map_points = []
        self.drone_pos = [None, None]
        self.received = False

        self.waypoints_dt = self.create_subscription(String, '/waypoints', self.waypoints_callback, 10)
        self.pose_dt = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_callback, 10)

        # Matplotlib figure will be set up in main thread
        self.fig = None
        self.ax = None
        self.map_sc = None
        self.wp_sc = None
        self.drone_sc = None

    def setup_plot(self, fig, ax, map_sc, wp_sc, drone_sc):
        self.fig = fig
        self.ax = ax
        self.map_sc = map_sc
        self.wp_sc = wp_sc
        self.drone_sc = drone_sc

    def waypoints_callback(self, msg: String):
        if not self.received:
            self.waypoints = json.loads(msg.data)
            self.get_logger().info(f'Waypoints received: {self.waypoints}')
            self.received = True
            self.destroy_subscription(self.waypoints_dt)
        else:
            pass

    def pose_callback(self, msg: Pose):
        with self.lock:
            self.drone_pos[0] = msg.position.x
            self.drone_pos[1] = msg.position.y

    def read_map_csv(self):
        points = []
        try:
            with open(CSV_PATH, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = float(row['x'])
                    y = float(row['y'])
                    z = float(row['z'])
                    points.append((x, y, z))
        except Exception as e:
            self.get_logger().warn(f"Could not read CSV: {e}")
        return points

    def update_plot(self, frame):
        with self.lock:
            self.map_points = self.read_map_csv()

            if self.map_points:
                map_xy = np.array([[x, y] for x, y, z in self.map_points])
                map_z = np.array([z for x, y, z in self.map_points])
                # Set color using z-values and a colormap
                self.map_sc.set_offsets(map_xy)
                self.map_sc.set_array(map_z)
                self.map_sc.set_cmap('viridis')
                self.map_sc.set_clim(np.min(map_z), np.max(map_z))
            else:
                self.map_sc.set_offsets(np.empty((0, 2)))
                self.map_sc.set_array(np.array([]))

            if self.waypoints:
                wp_xy = np.array(list(self.waypoints.values()))
                self.wp_sc.set_offsets(wp_xy)
                for t in self.ax.texts[:]:
                    t.remove()
                for label, (x, y) in self.waypoints.items():
                    self.ax.annotate(label, (x, y), fontsize=8, color='blue')
            else:
                self.wp_sc.set_offsets(np.empty((0, 2)))

            if None not in self.drone_pos:
                self.drone_sc.set_offsets([self.drone_pos])
            else:
                self.drone_sc.set_offsets(np.empty((0, 2)))

            # Manual autoscale for all points
            all_x = []
            all_y = []
            if self.map_points:
                all_x += [x for x, y, z in self.map_points]
                all_y += [y for x, y, z in self.map_points]
            if self.waypoints:
                all_x += [x for x, y in self.waypoints.values()]
                all_y += [y for x, y in self.waypoints.values()]
            if None not in self.drone_pos:
                all_x.append(self.drone_pos[0])
                all_y.append(self.drone_pos[1])

            if all_x and all_y:
                pad = 1.0
                self.ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
                self.ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

        return self.map_sc, self.wp_sc, self.drone_sc

def main(args=None):
    rclpy.init(args=args)
    node = LiveMapVisualizer()

    # Set up matplotlib plot in MAIN thread
    fig, ax = plt.subplots()
    map_sc = ax.scatter([], [], c='green', s=1, label='Map Points')
    wp_sc = ax.scatter([], [], c='blue', label='Waypoints')
    drone_sc = ax.scatter([], [], c='red', label='Drone')
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Live Map and Waypoints Visualization')

    node.setup_plot(fig, ax, map_sc, wp_sc, drone_sc)

    # Spin ROS in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Animation and plt.show() IN MAIN THREAD
    ani = animation.FuncAnimation(
        fig,
        node.update_plot,
        interval=1000,
        cache_frame_data=False   # suppresses the matplotlib warning
    )
    plt.show()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
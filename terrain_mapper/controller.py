#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import sys
import termios
import tty


MSG = """
Control Your Drone!
---------------------------
Moving around:
        w
    a   s    d
        x

t/l: takeoff/land (upper/lower case)
q/e : increase/decrease linear and angular velocity (upper/lower case)
A/D: rotate left/right
r/f : rise/fall (upper/lower case)

---------------------------
CTRL-C to quit
---------------------------

"""


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, '/simple_drone/land', 10)

        # Velocity parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_increment = 0.05
        self.angular_increment = 0.05
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0

    def get_velocity_msg(self) -> str:
        return f"Linear Velocity: {self.linear_velocity:.2f}\nAngular Velocity: {self.angular_velocity:.2f}\n"

    def read_keyboard_input(self) -> None:
        print(MSG + self.get_velocity_msg())

        key = self.get_key()

        if key.lower() == 'q':
            self.linear_velocity = min(self.linear_velocity + self.linear_increment, self.max_linear_velocity)
            self.angular_velocity = min(self.angular_velocity + self.angular_increment, self.max_angular_velocity)
        elif key.lower() == 'e':
            self.linear_velocity = max(self.linear_velocity - self.linear_increment, -self.max_linear_velocity)
            self.angular_velocity = max(self.angular_velocity - self.angular_increment, -self.max_angular_velocity)
        elif key.lower() == 'w':
            linear_vec = Vector3(x=self.linear_velocity)
            self.publish_cmd_vel(linear_vec)
        elif key.lower() == 'x':
            linear_vec = Vector3(x=-self.linear_velocity)
            self.publish_cmd_vel(linear_vec)
        elif key.lower() == 's':
            self.publish_cmd_vel()
        elif key == 'a':
            linear_vec = Vector3(y=self.linear_velocity)
            self.publish_cmd_vel(linear_vec)
        elif key == 'd':
            linear_vec = Vector3(y=-self.linear_velocity)
            self.publish_cmd_vel(linear_vec)
        elif key == 'A':
            angular_vec = Vector3(z=self.angular_velocity)
            self.publish_cmd_vel(angular_vec=angular_vec)
        elif key == 'D':
            angular_vec = Vector3(z=-self.angular_velocity)
            self.publish_cmd_vel(angular_vec=angular_vec)
        elif key.lower() == 'r':
            linear_vec = Vector3(z=self.linear_velocity)
            self.publish_cmd_vel(linear_vec)
        elif key.lower() == 'f':
            linear_vec = Vector3(z=-self.linear_velocity)
            self.publish_cmd_vel(linear_vec)
        elif key.lower() == 't':
            self.takeoff_publisher.publish(Empty())
        elif key.lower() == 'l':
            self.publish_cmd_vel()
            self.land_publisher.publish(Empty())

    def get_key(self) -> str:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_cmd_vel(self, linear_vec: Vector3 = Vector3(), angular_vec: Vector3 = Vector3()) -> None:
        twist = Twist(linear=linear_vec, angular=angular_vec)
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    try:
        while rclpy.ok():
            teleop_node.read_keyboard_input()
    except KeyboardInterrupt:
        print("\nTeleop node interrupted by user.")
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

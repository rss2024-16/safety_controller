#!/usr/bin/env python3
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from wall_follower.visualization_tools import VisualizationTools

Kp = 5000 #
# Kd = 2
max_steering_angle = 0.34
wheelbase: 0.325
loop_period = 0.05 # (seconds) of 20 hz

class SafetyController(Node):

    WALL_TOPIC = "/wall"

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("safety_topic", "default")
        self.declare_parameter("navigation_topic", "default")

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter('safett_topic').get_parameter_value().string_value
        self.NAVIGATION_TOPIC = self.get_parameter('navigation_topic').get_parameter_value().integer_value

	# TODO: Initialize your publishers and subscribers here

        self.subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC, # /scan
            self.listener_callback,
            10)

         # prevent unused variable warning

        self.nav_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.NAVIGATION_TOPIC,
            10)

        self.safety_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.SAFETY_TOPIC,
            10)

        self.subscription

    # TODO: Write your callback functions here
        self.wall_saved = []

    def listener_callback(self, msg):

        self.RANGES = msg.ranges
        self.ANGLE_MIN = msg.angle_min
        self.ANGLE_MAX = msg.angle_max
        self.ANGLE_INCREMENT = msg.angle_increment
        # self.TIME_INCREMENT = msg.time_increment

        drive_msg = AckermannDriveStamped()

        drive_msg.drive.speed = 1.0
        self.publisher_.publish(drive_msg)

    def safety_controller(self):

        dist_arr = self.RANGES
        x_val = []
        y_val = []
        filtered_dist = []

        # Removes outlier data points
        for dist in dist_arr:
            if abs(dist) < 4:
                filtered_dist.append(dist)
            else:
                pass

        # if car comes across a whole in the wall, continue straight based on the previous filtered points

        if (len(filtered_dist) > 10):
            self.wall_saved = filtered_dist
        elif (len(filtered_dist) < 10):
            filtered_dist = self.wall_saved

        for dist in filtered_dist:
            angle = ((self.ANGLE_MIN) + ((self.ANGLE_INCREMENT)*(dist_arr.index(dist))))
            x_coord = np.cos(angle) * dist
            y_coord = np.sin(angle) * dist
            x_val.append(x_coord)
            y_val.append(y_coord)

        # Slice the ranges data to only consider distances for a wall on one side

        # left angles --> -2/3pi to -2/9pi
        # forward angles --> -2/9pi to 2/9pi
        # right angles --> 2/9pi to 2/3pi

        arr_size = len(x_val) // 3
        # self.get_logger().info(f"arr_size: {arr_size}")

        # Slice the array into three chunks for x and y
        right_x = x_val[:arr_size ] # right of the robot
        # self.get_logger().info(f"right_x: {right_x}")
        front_x = x_val[arr_size :2 * arr_size ] # front of robot
        left_x = x_val[2 * arr_size :] # left of robot
        # self.get_logger().info(f"left_x: {left_x}")

        right_y= y_val[:arr_size] # right of the robot
        front_y = y_val[arr_size:2 * arr_size] # front of robot
        left_y = y_val[2 * arr_size:] # left of robot

        side = self.SIDE
        # + 1 represents the left wall and -1 represents the right wall

        if side == 1:
            x = left_x
            y = left_y

        if side == -1:
            x = right_x
            y = right_y

        m,c = np.polyfit(x, y, 1)

        y_regress = []
        for num in x:
            # equation for the line:
            y_regress_val = (num*m) + c
            y_regress.append(y_regress_val)

        self.get_logger().info(f"x_regress: {x}")
        self.get_logger().info(f"y_regress: {y_regress}")

        VisualizationTools.plot_line(x , y_regress, self.line_pub, color = (1., 0., 0.),frame = "/laser")
        return m, c

def main():

    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

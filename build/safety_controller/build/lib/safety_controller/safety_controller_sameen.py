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
        self.declare_parameter("stop_range", "default")


        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter('safety_topic').get_parameter_value().string_value
        self.NAVIGATION_TOPIC = self.get_parameter('navigation_topic').get_parameter_value().integer_value
        self.STOP_RANGE = self.get_parameter("stop_range").get_parameter_value().double_value


	# TODO: Initialize your publishers and subscribers here

        self.subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC, # /scan
            self.listener_callback,
            10)

         # prevent unused variable warning

        self.nav_publisher = self.create_subscription(
            AckermannDriveStamped,
            self.NAVIGATION_TOPIC,
            self.navigation_callback,
            10)

        self.safety_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.SAFETY_TOPIC,
            10)

        self.subscription

    # TODO: Write your callback functions here
        self.wall_saved = []

    def slice_ranges(self, laser_scan: LaserScan):
        '''
        Given an array of ranges (from the laser scan), returns the appropriate slice of data of (distances, thetas)
        Just the front
        '''
        ranges = laser_scan.ranges
        length = len(ranges)

        #laser scans counterclockwise
        #part 1: filter the ranges data to just the front
        ranges = ranges[length//3 : 2*length//3]
        distances : np.ndarray = np.array(ranges)

        angles : np.ndarray = np.linspace(laser_scan.angle_min, laser_scan.angle_max, num=length)
        thetas = angles[length//3 : 2*length//3]

        return distances, thetas


    def listener_callback(self, msg):

        '''
        Process laser scan data to detect obstacles
        If an obstacle is detected, issue a stop command
        '''
        distances, thetas = self.slice_ranges(laser_scan)
        stop_cmd = AckermannDriveStamped()
        if min(distances) < self.STOP_RANGE:  # Example threshold, adjust as needed
            stop_cmd.drive.speed = 0.0
            stop_cmd.drive.steering_angle = 0.0
        else:
            stop_cmd.drive.speed = 1.0
            stop_cmd.drive.steering_angle = 0.0
        self.pub_safety.publish(stop_cmd)

    def navigation_callback(self, msg: AckermannDriveStamped):

        self.safety_publisher.publish(msg)


def main():

    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

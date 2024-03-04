#!/usr/bin/env python
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

#TODO: make a separate repository safety_controller 

from wall_follower.visualization_tools import VisualizationTools
import math

#how to test
#run the wall follower sim
#run the safety controller sim

class SafetyController:
    def __init__(self):
        '''
        The below section on muxes will help you decide which topic your safety controller 
        should publish to once deployed on the racecar. 
        
        Make this topic a ROS parameter so that you can easily change it between the simulation and the racecar.
        '''
        super().__init__("safety_controller")

        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("safety_topic", "default")
        self.declare_parameter("navigation_topic", "default")
        self.declare_parameter("stop_range", "default")

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter('safety_topic').get_parameter_value().string_value
        self.NAVIGATION_TOPIC = self.get_parameter('navigation_topic').get_parameter_value().string_value
        self.STOP_RANGE = self.get_parameter("stop_range").get_parameter_value().double_value
        
        self.sub_navigation = self.create_subscription(AckermannDriveStamped, self.NAVIGATION_TOPIC, self.navigation_callback, 10)  
        self.sub_scan = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.pub_safety = self.create_publisher(AckermannDriveStamped, self.SAFETY_TOPIC, 10)


    def navigation_callback(self, msg: AckermannDriveStamped):
        '''
        Process navigation commands here
        For now, let's just pass them through
        '''
        self.pub_safety.publish(msg)

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
    
    def scan_callback(self, laser_scan: LaserScan):
        '''
        Process laser scan data to detect obstacles
        If an obstacle is detected, issue a stop command
        '''
        distances, thetas = self.slice_ranges(laser_scan)
        if min(distances) < self.STOP_RANGE:  # Example threshold, adjust as needed
            stop_cmd = AckermannDriveStamped()
            stop_cmd.drive.speed = 0.0
            stop_cmd.drive.steering_angle = 0.0
            self.pub_safety.publish(stop_cmd)


def main():
    
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
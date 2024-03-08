#!/usr/bin/env python
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header

# from sim import SIM

class SafetyController(Node):
    def __init__(self):
        '''
        The below section on muxes will help you decide which topic your safety controller
        should publish to once deployed on the racecar.

        Make this topic a ROS parameter so that you can easily change it between the simulation and the racecar.
        '''
        super().__init__("sc_max")

        SIM = False
        if SIM:
            navigation_topic = 'sim_navigation_topic'
            safety_topic = 'sim_safety_topic'
        else:
            navigation_topic = 'robot_navigation_topic'
            safety_topic = 'robot_safety_topic'

        params = [
            ('scan_topic', 'default'),
            ('stop_range', 'default'),
            (navigation_topic, 'default'),
            (safety_topic, 'default')
        ]

        self.declare_parameters(namespace='', parameters=params)

        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.STOP_RANGE = self.get_parameter("stop_range").get_parameter_value().double_value
        self.NAVIGATION_TOPIC = self.get_parameter(navigation_topic).get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter(safety_topic).get_parameter_value().string_value

        self.sub_navigation = self.create_subscription(AckermannDriveStamped, self.NAVIGATION_TOPIC, self.navigation_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.pub_safety = self.create_publisher(AckermannDriveStamped, self.SAFETY_TOPIC, 10)
        
        self.stop_distance = 0.4 # m
        self.stopping = False
        self.VELOCITY = 1.0
        self.a = None
        self.get_logger().info('HERE "%s"' % self.SAFETY_TOPIC)

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

    def make_drive_msg(self, steering_angle=0.0, steering_angle_velocity=0.0, speed=0.0, acceleration=0.0, jerk=0.0):
        drive_msg = AckermannDriveStamped()
       
        header = Header()
        time_now = self.get_clock().now() # rclpy.time.Time
        header.stamp = time_now.to_msg() # builtin_interfaces/Time
        header.frame_id = "base_link"

        drive_msg.header = header

        drive_cmd = AckermannDrive()
        drive_cmd.steering_angle = float(steering_angle)
        drive_cmd.steering_angle_velocity = float(steering_angle_velocity)
        drive_cmd.speed = float(speed)
        drive_cmd.acceleration = float(acceleration)
        drive_cmd.jerk = float(jerk)

        drive_msg.drive = drive_cmd

        return drive_msg

    def stop(self):
        # self.VELOCITY += self.a
        # if self.VELOCITY <= 0:
        #     self.VELOCITY = 0
        
        self.get_logger().info('stopping "%s"' % self.a)
        drive_cmd = self.make_drive_msg(acceleration=self.a)
        self.pub_safety.publish(drive_cmd)

    def scan_callback(self, laser_scan: LaserScan):
        '''
        Process laser scan data to detect obstacles
        If an obstacle is detected, issue a stop command
        '''
        if not self.stopping:
            distances, thetas = self.slice_ranges(laser_scan)

            dist_to_obstacle = min(distances)

            if dist_to_obstacle < self.STOP_RANGE:
                if dist_to_obstacle > self.stop_distance:
                    self.stopping = True
                    delta_x = dist_to_obstacle - self.stop_distance
                    self.a = -(self.VELOCITY**2) / (2 * delta_x)
                    self.timer = self.create_timer(1.0, self.stop) #every second
                else: # emergency stop
                    drive_cmd = self.make_drive_msg(speed = 0.0)
            else:
                drive_cmd = self.make_drive_msg(speed = self.VELOCITY)

            if not self.stopping:
                self.get_logger().info('publishing "%s"' % drive_cmd.drive.speed)
                self.pub_safety.publish(drive_cmd)

def main():

    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

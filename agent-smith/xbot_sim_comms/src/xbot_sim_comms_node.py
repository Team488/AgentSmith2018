#!/usr/bin/env python

"""
Author: Rohit Bansal 
Intent: Interface main robot code with Gazebo 
"""

import rospy
import copy
from std_msgs.msg import Uint8, Float64
from geometry_msgs.msg import Quaternion
from xbot_robot_comms import *

MAX_SPEED = 20

class SimComms:

    def send_distance_forward_packet(self, distanceForward):
        distanceForward.distance_forward 

    def send_drive_power_command_packet(self, drivePowerCommand):
        leftWheelVelRadians = 6 * driveVelocityCommand.left_drive * MAX_SPEED
        rightWheelVelRadians = 6 * driveVelocityCommand.right_drive * MAX_SPEED
        
        # Publish left wheel velocity in radians
        front_left_velocity_publisher.publish(leftWheelVelRadians)
        mid_left_velocity_publisher.publish(leftWheelVelRadians)
        back_left_velocity_publisher.publish(leftWheelVelRadians)

        # Publish right wheel velocity in radians
        front_right_velocity_publisher.publish(rightWheelVelRadians)
        mid_right_velocity_publisher.publish(rightWheelVelRadians)
        back_right_velocity_publisher.publish(rightWheelVelRadians)

    def send_drive_vel_command_packet(self, driveVelocityCommand):
        leftWheelVelRadians = 6 * driveVelocityCommand.left_drive
        rightWheelVelRadians = 6 * driveVelocityCommand.right_drive
        
        # Publish left wheel velocity in radians
        front_left_velocity_publisher.publish(leftWheelVelRadians)
        mid_left_velocity_publisher.publish(leftWheelVelRadians)
        back_left_velocity_publisher.publish(leftWheelVelRadians)

        # Publish right wheel velocity in radians
        front_right_velocity_publisher.publish(rightWheelVelRadians)
        mid_right_velocity_publisher.publish(rightWheelVelRadians)
        back_right_velocity_publisher.publish(rightWheelVelRadians)

    def send_command_result(self, robotCommandResult):
        distanceForward.distance_forward
    
    def __init__(self):
        # Initialize Node
        rospy.init_node('xbot_nav_sim_comms', log_level=rospy.DEBUG)
        
        # Setup publisher and Subscriber
        
        # self.wheel_odometry_publisher =  rospy.Publisher("/comms/wheel_odometry", RobotWheelOdomPacket, 5)
        # self.orientation_publisher = rospy.Publisher("/comms/orientation", Quaternion, 5)
        # self.heading_publisher =  rospy.Publisher("/comms/heading", Float64, 5, latch=true) #Currently in degrees
        # self.mode_publisher = rospy.Publisher("/comms/set_mode", Uint8, 5, latch=true)

        # self.command_from_gazebo = rospy.Publisher"/comms/current_command_from_rio", Uint8, 1, true)
        # self.command_to_gazebo = rospy.Subscriber("/comms/command_result_to_rio", CommandResultPacket, self.send_command_result, queue_size=1)
        
        # self.distance_forward_subscriber =  rospy.Subscriber("/comms/distance_forward", DistanceForwardPacket, self.send_distance_forward_packet, queue_size=2)
        self.drive_power_command_subscriber = rospy.Subscriber("/comms/send_drive_power", DriveCommandPacket, self.send_drive_power_command_packet, queue_size=2)
        self.drive_vel_command_subscriber = rospy.Subscriber("/comms/send_drive_vel", DriveCommandPacket, self.send_drive_vel_command_packet, queue_size=2)

        self.front_left_velocity_publisher  =  rospy.Publisher("/sim/front_left_velocity_controller/command" , Float64, 5, latch=true) #Currently in degrees
        self.mid_left_velocity_publisher    =  rospy.Publisher("/sim/mid_left_velocity_controller/command"   , Float64, 5, latch=true) #Currently in degrees
        self.back_left_velocity_publisher   =  rospy.Publisher("/sim/back_left_velocity_controller/command"  , Float64, 5, latch=true) #Currently in degrees
        self.front_right_velocity_publisher =  rospy.Publisher("/sim/front_right_velocity_controller/command", Float64, 5, latch=true) #Currently in degrees
        self.mid_right_velocity_publisher   =  rospy.Publisher("/sim/mid_right_velocity_controller/command"  , Float64, 5, latch=true) #Currently in degrees
        self.back_right_velocity_publisher  =  rospy.Publisher("/sim/back_right_velocity_controller/command" , Float64, 5, latch=true) #Currently in degrees

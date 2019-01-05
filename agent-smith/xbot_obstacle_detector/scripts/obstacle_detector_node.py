#!/usr/bin/env python
#TO-DO: Implement lookup table for calculations to reduce processing time
"""
Author: Andy Khuu 
Intent: Find how far our robot can travel forward without hitting something 
"""

import rospy
import copy
import math
from xbot_obstacle_detector.msg import ObstacleInfo
from sensor_msgs.msg import LaserScan

class ObstacleDetectorNode:
    
    def detect_obstacles(self, inputScan):
        #https://github.com/robopeak/rplidar_ros/wiki
        #Assume back of lidar is 0 
        ranges = inputScan.ranges

        #lidar's position relative to edge of robot (in meters) >> Currently BS #s
        left_side = .5
        right_side = .5

        front_ranges = int(3.14/inputScan.angle_increment)
        spacer_from_blocked_angles = int(1.57/inputScan.angle_increment) #90 degrees
        a_increment = inputScan.angle_increment
        
        in_front = []
        for x in range(1,front_ranges)
            if range[x] != float(inf):
                side = left_side
                if x > (spacer_from_blocked_angles + front_ranges)/2:
                    side = right_side
                side_angle = a_increment*(x) 
                robot_side_limit = math.abs(side/math.cos(side_angle))
                #check if within robot's front
                if(ranges[x] < robot_side_limit):
                    distance_forward = math.abs((math.sin(side_angle) * ranges[x]))
                    in_front.append(distance_forward * 1000)

        possible_forward_dist = min(in_front)
            
        obstacle = ObstacleInfo(possible_forward_dist)
        self.obstacle_publisher.publish(obstacle)

    def __init__(self):
        # Initialize Node
        rospy.init_node('ObstacleDetectorNode', log_level=rospy.DEBUG)  
    
        #setup Publisher and Subscriber
        self.obstacle_publisher = rospy.Publisher('/obstacle', ObstacleInfo, queue_size = None)
        self.inputScan = rospy.Subscriber('/scan', LaserScan, self.detect_obstacles, queue_size=1)
        
# This is the program's main function
if __name__ == '__main__':
    
    # Create ObstacleDetectorNode object
    obstacleDetector = ObstacleDetectorNode()
    rospy.spin()
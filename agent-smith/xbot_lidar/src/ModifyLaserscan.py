#!/usr/bin/env python

"""
Author: Rohit Bansal 
Intent: Read a raw laserscan and publish a modified laserscan.  
"""

import rospy
import copy
from sensor_msgs.msg import LaserScan
from math import pi
from math import degrees
from math import radians

class ModifyLaserscan:
    
    def fixScanAngles(self, inputScan):
        inputScan.ranges = list(inputScan.ranges)
        inputScan.intensities = list(inputScan.intensities)

        blocked_angles = rospy.get_param('blocked_angles')
        #blocked_angles should be placed in config file as a list of lists of angles which are unnecessary/blocked,
        #Last given angle in interval is excluded [[0.91]] - blocks 0-90
        for angles in blocked_angles:
            for i in range(angles[0],angles[1]):
                inputScan.ranges[i] = float('inf')
                inputScan.intensities[i] = 0

        self.scanPublisher.publish(inputScan); 

    #There is roughly 360 readings per cycle of lidar
    def __init__(self):
        # Initialize Node
        rospy.init_node('ModifyLaserscanNode', log_level=rospy.DEBUG)  
        # Setup publisher and Subscriber
        self.scanPublisher = rospy.Publisher('/scan', LaserScan, queue_size=None)
        self.inputScan = rospy.Subscriber('/scan_raw', LaserScan, self.fixScanAngles, queue_size=1)
    
# This is the program's main function
if __name__ == '__main__':
    
    # Create ModifyLaserScan object
    modifyLaserScan = ModifyLaserscan()
    rospy.spin()
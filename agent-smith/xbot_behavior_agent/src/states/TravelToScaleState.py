import smach
import smach_ros
import rospy
import time

#from BaseXbotState import BaseXbotState
from constants import Outcomes, ALL_OUTCOMES

class TravelToScaleState(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self,outcomes=ALL_OUTCOMES,output_keys=['target_scale'])
        self.comms = comms

    def execute(self, userdata):
        rate = rospy.Rate(50)
        rospy.loginfo("Pathplanning To Scale")
        userdata.target_scale = 0

        
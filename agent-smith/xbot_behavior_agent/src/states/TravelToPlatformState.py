import smach
import smach_ros
import rospy
import time

from BaseXbotState import BaseXbotState
from constants import Outcomes, ALL_OUTCOMES

class TravelToPlatformState(BaseXbotState):
    def __init__(self, comms):
        BaseXbotState.__init__(self)
        self.comms = comms

    def execute(self, userdata):
        rate = rospy.Rate(50)
        rospy.loginfo("Pathplanning to Platform")
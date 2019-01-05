import math

import smach
import smach_ros
import rospy

from BaseNavState import BaseNavState
from constants import Outcomes

class NavToTestGoal(BaseNavState):
    def __init__(self, comms):
        BaseNavState.__init__(self, comms)

    def execute(self, userdata):
        rate = rospy.Rate(50)
        rospy.loginfo('Test nav command beginning')

        self._begin_execute()

        while not rospy.is_shutdown():
            if self._aborted():
                self._end_execute()
                return Outcomes.Aborted
        
        self._end_execute()
        return Outcomes.Aborted
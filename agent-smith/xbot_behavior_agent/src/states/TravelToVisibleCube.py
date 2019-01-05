import smach
import smach_ros
import rospy
import time

from constants import Outcomes
from BaseXbotState import BaseXbotState


class TravelToVisibleCubeState(BaseXbotState):
    def __init__(self, comms):
        BaseXbotState.__init__(self)
        self.comms = comms

    def execute(self, userdata):
        rate = rospy.Rate(50)
        
        start_time = time.time()
        # Currently BS numbers, needs fine tuning
        while not rospy.is_shutdown() and (time.time() - start_time) < 5:
            if self._aborted():
                return Outcomes.Aborted
            # Currently BS numbers, needs fine tuning
            self.comms.publish_drive_command(-0.3, -0.3)
            rate.sleep()
        
        return Outcomes.Success
import smach
import smach_ros
import rospy
import time

from BaseXbotState import BaseXbotState
from constants import Outcomes, ALL_OUTCOMES

class InitState(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=["NavToVisibleCubeState", "NavToTestGoal"])
        self.comms = comms
        rospy.loginfo("Initializing")

    def execute(self, userdata):
        rate = rospy.Rate(50)
        rospy.loginfo("Init with ID " + str(self.comms.current_command))
        if self.comms.current_command == 1:
            return 'NavToVisibleCubeState'
        elif self.comms.current_command == 2:
            return 'NavToTestGoal'
        else:
            rospy.logwarn("NOT A VALID COMMAND")
            return Outcomes.Aborted

        

        



        
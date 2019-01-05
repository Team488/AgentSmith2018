import smach
import smach_ros
import rospy
import time

#from BaseXbotState import BaseXbotState
from constants import Outcomes, ALL_OUTCOMES

class PlaceCubeState(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self,outcomes=ALL_OUTCOMES,input_keys=['target_input'])
        self.comms = comms

    def execute(self, userdata):
        target = userdata.target_input
        rate = rospy.Rate(50)
        #Insert specific code to place cube on switch
        if target == 1:
            rospy.loginfo("Adjusting Robot and Cube for Placement")
        
            rospy.loginfo("Placing Cube")

        else:
            return
            #insert specific code to place cube on scale 


        
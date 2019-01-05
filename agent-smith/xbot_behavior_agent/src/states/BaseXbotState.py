import smach
import rospy

from constants import ALL_OUTCOMES

class BaseXbotState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=ALL_OUTCOMES)
    
    def _aborted(self):
        if self.preempt_requested():
            self.service_preempt()
            rospy.loginfo("Aborting")
            return True
        
        return False
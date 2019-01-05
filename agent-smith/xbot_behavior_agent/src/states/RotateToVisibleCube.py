import smach
import smach_ros
import rospy

from BaseXbotState import BaseXbotState
from constants import Outcomes, ALL_OUTCOMES

class RotateToVisibleCubeState(BaseXbotState):
    def __init__(self, comms):
        BaseXbotState.__init__(self)
        self.comms = comms

    def execute(self, userdata):
        rate = rospy.Rate(50)
        rospy.loginfo('Rotate beginning')
        while self.comms.target_cube is None:
            rate.sleep()
        rospy.loginfo('Rotate has at least one cube to track')
        
        target = self.comms.target_cube
        
        while not rospy.is_shutdown() and abs(target.heading_deflection_radians) > 0.05:
            if self._aborted():
                return Outcomes.Aborted
            
            # Currently All BS numbers, needs fine tuning
            power = 1 * target.heading_deflection_radians
            if power > 0.3:
                power = 0.3
            if power < -0.3:
                power = -0.3
            
            self.comms.publish_drive_command(power, -power)
            rate.sleep()

            lost_target_time = rospy.get_time()
            while self.comms.target_cube is None:
                # TODO: differentiate stop reasons and send result codes
                if rospy.get_time() - lost_target_time > 4:
                    return Outcomes.Aborted
                if self._aborted():
                    return Outcomes.Aborted
                rate.sleep()
            
            target = self.comms.target_cube
        
        rospy.loginfo("Finishing rotate")
        return Outcomes.Success # Success
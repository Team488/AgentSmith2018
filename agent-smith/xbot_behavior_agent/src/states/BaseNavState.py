import math

import smach
import smach_ros
import rospy

from geometry_msgs.msg import Twist

from BaseXbotState import BaseXbotState

class BaseNavState(BaseXbotState):
    def __init__(self, comms):
        BaseXbotState.__init__(self)
        self.comms = comms

    def handle_commanded_velocity(self, cmd_vel):
        base_velocity = cmd_vel.linear.x
        velocity_diff = cmd_vel.angular.z * 0.6425

        vel_left = base_velocity + velocity_diff / 2
        vel_right = base_velocity - velocity_diff / 2

        self.comms.publish_drive_vel_command(vel_left, vel_right)

    def _begin_execute(self):
        self.nav_command_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.handle_commanded_velocity)

    def _end_execute(self):
        self.nav_command_subscriber.unregister()
        self.nav_command_subscriber = None
        # TODO: Figure out why this doesn't work
        self.comms.publish_drive_power_command(0, 0)
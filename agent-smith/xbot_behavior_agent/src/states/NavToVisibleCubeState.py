import math

import smach
import smach_ros
import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from BaseNavState import BaseNavState
from constants import Outcomes, ALL_OUTCOMES

class NavToVisibleCubeState(BaseNavState):
    def __init__(self, comms):
        BaseNavState.__init__(self, comms)
        self.transform_listener = tf.TransformListener()

    def execute(self, userdata):
        rate = rospy.Rate(50)
        rospy.loginfo('Nav to cube beginning')

        self._begin_execute()

        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        while self.comms.target_cube_point is None:
           rate.sleep()
        rospy.loginfo('Nav to visible cube has at least one cube to track')
        
        try:
           target_cube_point_base_frame = self.transform_listener.transformPoint("/base_link", self.comms.target_cube_point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
           rospy.logerror("Failed to transform cube target point")
           self._end_execute()
           return Outcomes.Failed

        # ROS coordinates dictate that X is forward, so args to atan2 are reversed
        angle_to_cube = math.atan2(target_cube_point_base_frame.point.x, target_cube_point_base_frame.point.y)

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time()

        goal.target_pose.pose.position = target_cube_point_base_frame.point
        goal.target_pose.pose.orientation = tf.transformations.quaternion_about_axis(angle_to_cube, (0, 0, 1))

        rospy.loginfo("Sending goal for nav to cube")

        class Object(object):
            pass

        goal_status_container = Object()
        goal_status_container.status = None
        def move_base_done_cb(status):
           goal_status_container.status = status
        
        move_base_client.send_goal(goal, move_base_done_cb)

        while not rospy.is_shutdown():
            if self._aborted():
                self._end_execute()
                return Outcomes.Aborted

            goal_status = goal_status_container.status
            if goal_status is not None:
                if goal_status is actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Finishing nav to cube")
                    self._end_execute()
                    return Outcomes.Success

                rospy.logerror("move_base failed with goal status " + str(goal_status))
                self._end_execute()
                return Outcomes.Failed
        
        self._end_execute()
        return Outcomes.Aborted
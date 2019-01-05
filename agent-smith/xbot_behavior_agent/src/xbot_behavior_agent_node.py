#!/usr/bin/env python

import time
import threading

import rospy
import smach
import smach_ros

from xbot_vision.msg import CubeTargetInfo
from xbot_robot_comms.msg import DriveCommandPacket, CommandResultPacket
from std_msgs.msg import UInt8

from constants import Outcomes, ALL_OUTCOMES, STATUS_CODES

from states import *

class StateMachineThread(threading.Thread):
    def __init__(self, state_machine, comms):
        self.state_machine = state_machine
        self.comms = comms
        super(StateMachineThread, self).__init__()

    def run(self):
        outcome = self.state_machine.execute()
        status_code = STATUS_CODES[outcome]
        self.comms.publish_command_result_to_rio(status_code)

class CommsInterface(object):
    def __init__(self):
        self.robot_vision_subscriber = rospy.Subscriber('/vision/tracked_cube_target', CubeTargetInfo, self.detected_cubes_callback, queue_size=1)

        self.command_result_publisher = rospy.Publisher('/comms/command_result_to_rio', CommandResultPacket, queue_size=5)
        self.robot_drive_power_publisher = rospy.Publisher('/comms/send_drive_power', DriveCommandPacket, queue_size=1)
        self.robot_drive_vel_publisher = rospy.Publisher('/comms/send_drive_vel', DriveCommandPacket, queue_size=1)

        self.current_command = 0

        self.target_cube_point = None

    def publish_command_result_to_rio(self, status_code):
        msg = CommandResultPacket()
        msg.command_id = self.current_command
        msg.status_code = status_code
        self.command_result_publisher.publish(msg)

    def publish_drive_power_command(self, left_drive, right_drive):
        msg = DriveCommandPacket()
        msg.owner_command = self.current_command
        msg.left_drive = left_drive
        msg.right_drive = right_drive
        self.robot_drive_power_publisher.publish(msg)

    def publish_drive_vel_command(self, left_drive, right_drive):
        msg = DriveCommandPacket()
        msg.owner_command = self.current_command
        msg.left_drive = right_drive
        msg.right_drive = left_drive
        self.robot_drive_vel_publisher.publish(msg)

    def detected_cubes_callback(self, robot_vision_packet):
        if robot_vision_packet.has_tracked_cube:
            self.target_cube_point = robot_vision_packet.target_cube_point
        else:
            self.target_cube_point = None

class BehaviorAgentNode(object):
    def __init__(self):
        rospy.init_node('behavior_agent_node')
        rospy.loginfo('Initializing Behvior Agent Node')

        # Create a SMACH state machine
        self.state_machine = smach.StateMachine(outcomes=ALL_OUTCOMES)
        self.state_machine_thread = None
        self.state_machine.userdata.current_target = 0

        self.comms = CommsInterface()

        with self.state_machine:
        #    smach.StateMachine.add('SEARCH', Search(),
        #                            transitions={'success': 'ROTATE',})
            smach.StateMachine.add('INIT',InitState(self.comms),
                                    transitions={'NavToVisibleCubeState' : 'NAVCUBE', 'NavToTestGoal' : 'NAVTEST'})
            smach.StateMachine.add('NAVCUBE', NavToVisibleCubeState(self.comms),
                                    transitions={})
            smach.StateMachine.add('NAVTEST', NavToTestGoal(self.comms),
                                    transitions={})
            smach.StateMachine.add('ROTATE', RotateToVisibleCubeState(self.comms),
                                    transitions={'failed': 'ROTATE',
                                                'success': 'TRAVELCUBE',
                                                })
            smach.StateMachine.add('TRAVELCUBE', TravelToVisibleCubeState(self.comms),
                                    transitions={'failed': 'ROTATE',
                                                })
            smach.StateMachine.add('TRAVELSCALE', TravelToScaleState(self.comms),
                                    transitions ={'success' : 'PLACECUBE'},
                                    remapping={'target_scale' : 'target'})
            smach.StateMachine.add('TRAVELSWITCH', TravelToSwitchState(self.comms),
                                    transitions ={'success' : 'PLACECUBE'},
                                    remapping={'target_switch' : 'target'})
            smach.StateMachine.add('PLACECUBE', PlaceCubeState(self.comms),
                                    transitions ={},
                                    remapping = {'target_input' : 'target'})
            smach.StateMachine.add('TRAVELPLATFORM', TravelToPlatformState(self.comms),
                                    transitions ={})
                                    
                    
    
            # smach.StateMachine.add('ACQUIRE', Acquire(),
            #                         transitions={'failed': 'SEARCH'})
        
        sis = smach_ros.IntrospectionServer('introspection_server', self.state_machine, '/SM_ROOT')
        sis.start()

        self.command_subscriber = rospy.Subscriber('/comms/current_command_from_rio', UInt8, self.command_callback, queue_size=5)
        self.comms_interface = CommsInterface()

        rospy.spin()
        sis.stop()

    def command_callback(self, new_command_packet):
        rospy.loginfo("Command called: " + str(new_command_packet.data))

        if not self.state_machine_thread is None:
            self.state_machine.request_preempt()
            self.state_machine_thread.join()
            self.comms.current_command = 0
            rospy.loginfo("State machine thread stopped")

        if new_command_packet.data != 0:
            rospy.loginfo("Starting sm thread for command " + str(new_command_packet.data))
            self.comms.current_command = new_command_packet.data
            self.state_machine.recall_preempt()
            self.state_machine_thread = StateMachineThread(self.state_machine, self.comms)
            self.state_machine_thread.start()

#def stop():
#    rospy.loginfo('Stop Command Executed')
#    msg = DriveCommandPacket()
#    msg.left_drive = 0
#    msg.right_drive = 0
#    msg.owner_command = 0x01
#    publish_drive_command(msg)
#    rospy.loginfo('Stop Command Done')


if __name__ == '__main__':
    try:
        BehaviorAgentNode()
    except rospy.exceptions.ROSInterruptException:
        pass

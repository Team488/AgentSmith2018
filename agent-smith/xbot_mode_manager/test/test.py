#!/usr/bin/env python

import sys
import time
import unittest

import rospy
import rostest
import roslib.scriptutil as scriptutil
from std_msgs.msg import Bool, UInt8

NAME = "xbot_mode_manager_test"

class TestModeManager(unittest.TestCase):
    def __init__(self, *args):
        super(TestModeManager, self).__init__(*args)
        self.node_enable_states = {}

    def callback(self, message, node_name):
        self.node_enable_states[node_name] = message.data

    def test_mode_1(self):
        self._test_mode(0x01, {
            "node_1": True,
            "node_2": False,
            "node_3": False
        })
    
    def test_mode_2(self):
        self._test_mode(0x02, {
            "node_1": False,
            "node_2": True,
            "node_3": True
        })

    def test_mode_3(self):
        self._test_mode(0x03, {
            "node_1": True,
            "node_2": True,
            "node_3": False
        })
    
    def _test_mode(self, mode_id, expected_state):
        rospy.Subscriber("/node_enable/node_1", Bool, self.callback, callback_args="node_1")
        rospy.Subscriber("/node_enable/node_2", Bool, self.callback, callback_args="node_2")
        rospy.Subscriber("/node_enable/node_3", Bool, self.callback, callback_args="node_3")

        rospy.init_node(NAME, anonymous=True)

        self.node_enable_states = {}
        mode_pub = rospy.Publisher("/comms/set_mode", UInt8, latch=True)
        message = UInt8(data=mode_id)
        mode_pub.publish(message)

        timeout_t = time.time() + 10.0*1000 #10 seconds
        while not rospy.is_shutdown() and len(self.node_enable_states) != 3 and time.time() < timeout_t:
            time.sleep(0.1)
        
        self.assertDictEqual(expected_state, self.node_enable_states)

if __name__ == '__main__':
    rostest.rosrun("xbot_mode_manager", NAME, TestModeManager, sys.argv)

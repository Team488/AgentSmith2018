#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, Bool

class ModeManagerNode():
    """
    Publishes to /node_enable/<node_name> topics with enable/disable flags
    based on the current system mode 
    """

    def __init__(self):
        rospy.init_node('ModeManagerNode', log_level=rospy.DEBUG)

        self.load_modes_by_node()

        self.node_publishers = {}
        for node in self.modes_by_node.iterkeys():
            self.node_publishers[node] = rospy.Publisher("/node_enable/" + node, Bool, queue_size=5, latch=True)

        self.set_mode_subscriber = rospy.Subscriber("/comms/set_mode", UInt8, self.handle_new_mode)

    def load_modes_by_node(self):
        # TODO: check keys
        mode_id_mapping = rospy.get_param("/ModeManagerNode/mode_id_mapping")
        name_indexed = rospy.get_param("/ModeManagerNode/modes_by_node")

        id_indexed = { node_name: [mode_id_mapping[mode] for mode in modes] for (node_name, modes) in name_indexed.iteritems() }
        self.modes_by_node = id_indexed

    def handle_new_mode(self, mode_message):
        for node, publisher in self.node_publishers.iteritems():
            is_enabled = mode_message.data in self.modes_by_node[node]

            message = Bool()
            message.data = is_enabled
            publisher.publish(message)


if __name__ == '__main__':
    try:
        mode_manager = ModeManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
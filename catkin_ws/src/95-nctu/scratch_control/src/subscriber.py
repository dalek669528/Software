#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class SubscribeNobe(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))

        self.sub = rospy.Subscriber("chatter", String, self.callback)
        self.sub_scratch_msg = rospy.Subscriber("Scratch_msg", String, self.scratch_callback)

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)

    def scratch_callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "%s", msg)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=False)
    subscribe_node=SubscribeNobe()
    rospy.spin()

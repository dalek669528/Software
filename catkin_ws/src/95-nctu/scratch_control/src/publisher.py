#!/usr/bin/env python
# license removed for brevity
import rospy
import thread
from std_msgs.msg import String

class PublishNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))

        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.rate = rospy.Rate(0.5)
        # Setup timer
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def talker(self):
        while not rospy.is_shutdown():
            self.hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(self.hello_str)
            self.pub.publish(self.hello_str)
            self.rate.sleep()

if __name__ == '__main__':

    rospy.init_node('talker', anonymous=False)
    publish_node = PublishNode()
    thread.start_new_thread(publish_node.talker, ())
    rospy.spin()
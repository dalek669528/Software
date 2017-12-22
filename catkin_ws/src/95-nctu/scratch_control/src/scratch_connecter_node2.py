#!/usr/bin/env python
import socket
import rospy
from array import array
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped

class ScratchConnecter(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None
        self.state_scratch = False
        self.port = 42001
        self.host = rospy.get_param("/scratch_IP")
        rospy.loginfo("Connecting...")
        self.scratchSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.scratchSock.connect((self.host, self.port))
        rospy.loginfo("Connected!")

        # Publications
        self.pub_msg_debug = rospy.Publisher("scratch_msg_debug", String, queue_size=10)
        self.pub_msg = rospy.Publisher("joy_with_scratch", Joy, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

        self.listener()

    def cbJoy(self, joy_msg):
        if not self.state_scratch:
            self.joy = joy_msg
            self.pub_msg.publish(self.joy)

    def listener(self):
        while True:
            if not self.joy: break
            data = self.scratchSock.recv(1024)
            if not data: break
            l = list(data)
            msg_len = (ord(l[0]) << 24) + (ord(l[1]) << 16) + (ord(l[2]) << 8) + ord(l[3])
            l2 = l[4:]
            msg_str = ''.join(l2)
            rospy.loginfo("received %d bytes:%s" % (msg_len, msg_str))
            if(len(msg_str) != msg_len):
                rospy.logerr("-E- ERROR - message length differs from sent length.  (%d vs %d)" % (msg_len, len(msg_str)))
                
            self.pub_msg_debug.publish(msg_str)
            if(msg_str == "broadcast \"go\""):
                self.joy.axes[1] = 1.0
                self.joy.axes[3] = 0.0
                self.state_scratch = True
            elif(msg_str == "broadcast \"back\""):
                self.joy.axes[1] = -1.0
                self.joy.axes[3] = 0.0
                self.state_scratch = True
            elif(msg_str == "broadcast \"left\""):
                self.joy.axes[1] = 0.0
                self.joy.axes[3] = 1.0
                self.state_scratch = True
            elif(msg_str == "broadcast \"right\""):
                self.joy.axes[1] = 0.0
                self.joy.axes[3] = -1.0
                self.state_scratch = True
            elif(msg_str == "broadcast \"stop\""):
                self.joy.axes[1] = 0.0
                self.joy.axes[3] = 0.0
                self.state_scratch = False
            self.pub_msg.publish(self.joy)

    def sendScratchCommand(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratchSock.send(a.tostring() + cmd)

if __name__ == '__main__':
    rospy.init_node("scratch_connecter2",anonymous=False)
    scratch_connecter = ScratchConnecter()
    rospy.spin()


            

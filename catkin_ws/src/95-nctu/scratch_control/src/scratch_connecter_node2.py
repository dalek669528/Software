#!/usr/bin/env python
#   2012 Jon Stephan
#   jfstepha@gmail.com

from array import array
import socket
import rospy
import roslib
from std_msgs.msg import String

############################################################

    
############################################################
class ScratchConnecter(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
 

        self.port = 42001
        self.host = rospy.get_param("/scratch_IP")
        self.rospy.loginfo("Connecting...")
        self.scratchSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.scratchSock.connect((host, port))
        rospy.loginfo("Connected!")

        # Publications
        self.pub_msg_debug = rospy.Publisher("scratch_msg_debug", String, queue_size=10)
        self.pub_msg = rospy.Publisher("scratch_msg", String, queue_size=1)

        # Subscriptions
        #self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        #self.sub_scratch = rospy.Subscriber("scratch_msg", String, self.cbScratch, queue_size=1)
        

        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        #self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        #self.has_complained = False

        self.listener()

    def listener(self):
        while True:
            data = scratchSock.recv(1024)
            if not data: break
            l = list(data)
            msg_len = (ord(l[0]) << 24) + (ord(l[1]) << 16) + (ord(l[2]) << 8) + ord(l[3])
            l2 = l[4:]
            msg_str = ''.join(l2)
            #rospy.loginfo("received %d bytes:%s" % (msg_len, msg_str))
            if(len(msg_str) != msg_len):
                rospy.logerr("-E- ERROR - message length differs from sent length.  (%d vs %d)" % (msg_len, len(msg_str)))
                
            pub_msg_debug.publish(msg_str)

            if(msg_str == "broadcast \"go\""):
                pub_msg.publish("go")
            if(msg_str == "broadcast \"back\""):
                pub_msg.publish("back")
            if(msg_str == "broadcast \"left\""):
                pub_msg.publish("left")
            if(msg_str == "broadcast \"right\""):
                pub_msg.publish("right")
            if(msg_str == "broadcast \"stop\""):
                pub_msg.publish("stop")

    def sendScratchCommand(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratchSock.send(a.tostring() + cmd)

if __name__ == '__main__':
    rospy.init_node('scratch_connector',anonymous=False)
    scratch_connector = ScratchConnector()
    rospy.spin()


            

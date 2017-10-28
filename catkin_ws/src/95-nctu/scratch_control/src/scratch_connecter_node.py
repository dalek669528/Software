#!/usr/bin/env python
#   2012 Jon Stephan
#   jfstepha@gmail.com

from array import array
import socket
import time
import sys
import rospy
import roslib
import re
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import String

############################################################
def sendScratchCommand(cmd):
    n = len(cmd)
    a = array('c')
    a.append(chr((n >> 24) & 0xFF))
    a.append(chr((n >> 16) & 0xFF))
    a.append(chr((n >>  8) & 0xFF))
    a.append(chr(n & 0xFF))
    scratchSock.send(a.tostring() + cmd)
    
############################################################
if __name__ == '__main__':
    rospy.loginfo("Scratch_connector started")
    rospy.init_node('scratch_connector')
    PORT = 42001
    HOST = 'localhost'
    HOST = '192.168.0.25'
    rospy.loginfo("Connecting...")
    scratchSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    scratchSock.connect((HOST, PORT))
    rospy.loginfo("Connected!")

    pub_msg_debug = rospy.Publisher("scratch_msg_debug", String, queue_size=10)
   
    pub_axes = [] 
    pub_axes.append(rospy.Publisher("scratch_msg_x", Float32, queue_size=1))
    pub_axes.append(rospy.Publisher("scratch_msg_y", Float32, queue_size=1))
    pub_msg.append(rospy.Publisher("scratch_msg", String, queue_size=1))

    while True:
        data = scratchSock.recv(1024)
        if not data: break
        l = list(data)
        msg_len = (ord(l[0]) << 24) + (ord(l[1]) << 16) + (ord(l[2]) << 8) + ord(l[3])
        l2 = l[4:]
        msg_str = ''.join(l2)
        rospy.loginfo("received %d bytes:%s" % (msg_len, msg_str))
        if(len(msg_str) != msg_len):
            rospy.logerr("-E- ERROR - message length differs from sent length.  (%d vs %d)" % (msg_len, len(msg_str)))
            
        pub_msg.publish(msg_str)

        if(msg_str == "broadcast \"go\""):
            pub_axes[0].publish(1.0)
            pub_msg.publish("go")
        if(msg_str == "broadcast \"back\""):
            pub_axes[0].publish(-1.0)
            pub_msg.publish("back")
        if(msg_str == "broadcast \"left\""):
            pub_axes[1].publish(1.0)
            pub_msg.publish("left")
        if(msg_str == "broadcast \"right\""):
            pub_axes[1].publish(-1.0)
            pub_msg.publish("right")
        if(msg_str == "broadcast \"stop\""):
            pub_axes[0].publish(0.0)
            pub_axes[1].publish(0.0)
            pub_msg.publish("stop")

            

#!/usr/bin/env python
import socket
import rospy
from array import array
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from geometry_msgs.msg import PoseArray, Pose
class ScratchConnecter(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = Joy()
        self.state_scratch = False
        self.port = 42001
        self.host = rospy.get_param("/scratch_IP")
        rospy.loginfo("Connecting...")
        self.scratchSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.scratchSock.connect((self.host, self.port))
        rospy.loginfo("Connected!")

        # Publications
        self.pub_msg = rospy.Publisher("~joy_with_scratch", Joy, queue_size=1)
        self.pub_vehicle_pose_pair = rospy.Publisher("~vehicle_pose_pair", PoseArray, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        #self.sub_vehicle_pose_pair = rospy.Subscriber("~vehicle_pose_pair", PoseArray, self.cbPoseArray, queue_size=1)
        self.listener()

    def cbJoy(self, joy_msg):
        if not self.state_scratch:
            self.joy = joy_msg
            self.pub_msg.publish(self.joy)

    def listener(self):
        while True:
            #rospy.loginfo("start receive")
            data = self.scratchSock.recv(1024)
            if not data: break
            #rospy.loginfo("received data")
            l = list(data)
            msg_len = (ord(l[0]) << 24) + (ord(l[1]) << 16) + (ord(l[2]) << 8) + ord(l[3])
            l2 = l[4:]
            msg_str = ''.join(l2)
            #rospy.loginfo("received %d bytes:%s" % (msg_len, msg_str))
            if(len(msg_str) != msg_len):
                rospy.logerr("-E- ERROR - message length differs from sent length.  (%d vs %d)" % (msg_len, len(msg_str)))

            if(msg_str.find('joy')!=-1):
                axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                msg_str = msg_str[msg_str.find('joy'):]
                if(msg_str == "joy go\""):
                    axes[1] = 1.0
                    self.state_scratch = True
                elif(msg_str == "joy back\""):
                    axes[1] = -1.0
                    self.state_scratch = True
                elif(msg_str == "joy left\""):
                    axes[3] = 1.0
                    self.state_scratch = True
                elif(msg_str == "joy right\""):
                    axes[3] = -1.0
                    self.state_scratch = True
                elif(msg_str == "joy isMoving\" 0"):
                    self.state_scratch = False

                if(msg_str == "joy override msg True\""):
                    buttons[6] = 1
                elif(msg_str == "joy override msg False\""):
                    buttons[7] = 1
                elif(msg_str == "joy state verbose\""):
                    buttons[5] = 1
                elif(msg_str == "joy state parallel autonomy\""):
                    buttons[4] = 1
                elif(msg_str == "joy anti instagram message\""):
                    buttons[3] = 1
                elif(msg_str == "joy E-stop message\""):
                    buttons[8] = 1
                elif(msg_str == "joy start lane following with avoidance mode\""):
                    buttons[9] = 1
                self.joy.axes = tuple(axes)
                self.joy.buttons = tuple(buttons)
                self.pub_msg.publish(self.joy)
            elif((msg_str.find('mouse ')!=-1)):
                vehicle_pose_pair_msg = PoseArray()
                vehicle_pose = Pose()
                self.pub_vehicle_pose_pair.publish(vehicle_pose_pair_msg)
                if((msg_str.find('x')!=-1)):
                    vehicle_pose.position.x = float(msg_str[msg_str.find('x')+3:])
                    print msg_str[msg_str.find('x')+3:]
                if((msg_str.find('y')!=-1)):
                    vehicle_pose.position.y = float(msg_str[msg_str.find('y')+3:])
                    print msg_str[msg_str.find('y')+3:]
                vehicle_pose.position.z = 0.0
                print vehicle_pose.position.x
                print vehicle_pose.position.y
                print vehicle_pose.position.z
                vehicle_pose_pair_msg.poses.append(vehicle_pose)
                self.pub_vehicle_pose_pair.publish(vehicle_pose_pair_msg)

    def sendScratchCommand(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratchSock.send(a.tostring() + cmd)

if __name__ == '__main__':
    rospy.init_node("scratch_connecter",anonymous=False)
    scratch_connecter = ScratchConnecter()
    rospy.spin()


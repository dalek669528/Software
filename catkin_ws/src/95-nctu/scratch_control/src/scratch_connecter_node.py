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
        self.target_pose = Pose()
        self.state_scratch = False
        self.state_target_pose_x = False
        self.state_target_pose_y = False
        self.port = 42001
        self.host = rospy.get_param("/scratch_IP")
        rospy.loginfo("Connecting...")
        self.scratchSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.scratchSock.connect((self.host, self.port))
        rospy.loginfo("Connected!")

        # Publications
        self.pub_msg = rospy.Publisher("~joy_with_scratch", Joy, queue_size=1)
        self.pub_target_pose_pair = rospy.Publisher("~target_pose_pair", PoseArray, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_vehicle_pose_pair = rospy.Subscriber("~vehicle_pose_pair", PoseArray, self.cbPoseArray, queue_size=1)
        self.listener()

    def cbJoy(self, joy_msg):
        if not self.state_scratch:
            #rospy.loginfo("Using Joystick")
            self.joy = joy_msg
            self.pub_msg.publish(self.joy)

    def cbPoseArray(self, pose_msg):
        vehicle_pose_x = (pose_msg.poses.position.x-0.75)*(-320)
        vehicle_pose_y = (pose_msg.poses.position.y-0.75)*(-240)
        self.sendScratchCommand("sensor-update \"vehicle x\" " + str(vehicle_pose_x) + " \"")
        self.sendScratchCommand("sensor-update \"vehicle y\" " + str(vehicle_pose_y) + " \"")
                self.sendScratchCommand("broadcast \"set vehicle pose\"")

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
                continue

            if(msg_str.find('joy')!=-1):
                self.joy.header.stamp = rospy.Time.now()
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
                elif(msg_str == "joy isMoving\" 0 "):
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
                #rospy.loginfo("Using Scratch")
                self.pub_msg.publish(self.joy)
            elif((msg_str.find('mouse ')!=-1)):
                target_pose_pair_msg = PoseArray()
                target_pose_pair_msg.header.stamp = rospy.Time.now()
                if((msg_str.find('x')!=-1)):
                    self.target_pose.position.x = (float(msg_str[msg_str.find('x')+3:])/(-320))+0.75
                    if(self.target_pose.position.x > 1.5):
                        self.target_pose.position.x = 1.5
                    if(self.target_pose.position.x < 0.0):
                        self.target_pose.position.x = 0.0
                    self.state_target_pose_x = True
                if((msg_str.find('y')!=-1)):
                    self.target_pose.position.y = (float(msg_str[msg_str.find('y')+3:])/(-240))+0.75
                    if(self.target_pose.position.y > 1.5):
                        self.target_pose.position.y = 1.5
                    if(self.target_pose.position.y < 0.0):
                        self.target_pose.position.y = 0.0
                    self.state_target_pose_y = True
                if(self.state_target_pose_x and self.state_target_pose_y):
                    self.target_pose.position.z = 0.0
                    target_pose_pair_msg.poses.append(self.target_pose)
                    self.state_target_pose_x = False
                    self.state_target_pose_y = False
                    self.pub_target_pose_pair.publish(target_pose_pair_msg)
                    vehicle_pose_x = (pose_msg.poses.position.x-0.75)*(-320)
                    vehicle_pose_y = (pose_msg.poses.position.y-0.75)*(-240)
                    self.sendScratchCommand("sensor-update \"vehicle x\" " + str((self.target_pose.position.x-0.75)*(-320)) + " \"")
                    self.sendScratchCommand("sensor-update \"vehicle y\" " + str((self.target_pose.position.y-0.75)*(-240)) + " \"")
                    self.sendScratchCommand("broadcast \"set vehicle pose\"")

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


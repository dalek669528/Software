#!/usr/bin/env python
import math
import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoyMapperNode(object):
	
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))

		self.v = 0.3

		#Subscriber
		self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

		#Publisher
		self.pub_A = rospy.Publisher("~pub_A", BoolStamped, queue_size=1)
		self.pub_B = rospy.Publisher("~pub_B", BoolStamped, queue_size=1)
		self.pub_Y = rospy.Publisher("~pub_Y", BoolStamped, queue_size=1)
		self.pub_X = rospy.Publisher("~pub_X", BoolStamped, queue_size=1)
		self.pub_carcmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)


	def cbJoy(self, joy_msg):
		self.JoyAxes(joy_msg)
		self.JoyButton(joy_msg)

	def JoyAxes(self, joy_msg):

		#carcmd = XXXX
		#carcmd.v = XXXX
		#publish??

	def JoyButton(self, joy_msg):

		if(joy_msg.buttons[0] == 1):
			rospy.loginfo("[%s] You Press Button A " %(self.node_name))
			#state = XXX
			#state.data = XXX
			#publush

		elif(XXXXXXXXXXXX):
			rospy.loginfo("[%s] You Press Button B " %(self.node_name))
			#state = XXX
			#state.data = XXX
			#publush

		elif(XXXXXXXXXXXX):
			rospy.loginfo("[%s] You Press Button X " %(self.node_name))
			#state = XXX
			#state.data = XXX
			#publush

		elif(XXXXXXXXXXXX):
			rospy.loginfo("[%s] You Press Button Y " %(self.node_name))
			#state = XXX
			#state.data = XXX
			#publush

		else:
			some_active = sum(joy_msg.buttons) 
			if some_active:
				rospy.loginfo("[%s] No binding buttons " %(self.node_name))


if __name__ == '__main__':
	rospy.init_node("joy_mapper_node", anonymous=True)
	joy_mapper = JoyMapperNode()
	rospy.spin()

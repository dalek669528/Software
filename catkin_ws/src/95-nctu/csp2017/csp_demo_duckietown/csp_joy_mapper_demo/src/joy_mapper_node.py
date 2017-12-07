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
		car_cmd_msg = Twist2DStamped()
		car_cmd_msg.header.stamp = joy_msg.header.stamp
		car_cmd_msg.v = joy_msg.axes[4] * self.v #Left stick V-axis. Up is positive
		# Holonomic Kinematics for Normal Driving
		car_cmd_msg.omega = joy_msg.axes[3]
		self.pub_carcmd.publish(car_cmd_msg)

	def JoyButton(self, joy_msg):

		if(joy_msg.buttons[0] == 1):
			rospy.loginfo("[%s] You Press Button A " %(self.node_name))
			state = BoolStamped()
			state.data = False
			self.pub_A.publish(state)

		elif(joy_msg.buttons[1] == 1):
			rospy.loginfo("[%s] You Press Button B " %(self.node_name))
			state = BoolStamped()
			state.data = True
			self.pub_B.publish(state)

		elif(joy_msg.buttons[2] == 1):
			rospy.loginfo("[%s] You Press Button X " %(self.node_name))
			state = BoolStamped()
			state.data = True
			self.pub_X.publish(state)

		elif(joy_msg.buttons[3] == 1):
			rospy.loginfo("[%s] You Press Button Y " %(self.node_name))
			state = BoolStamped()
			state.data = True
			self.pub_Y.publish(state)

		else:
			some_active = sum(joy_msg.buttons) 
			if some_active:
				rospy.loginfo("[%s] No binding buttons " %(self.node_name))


if __name__ == '__main__':
	rospy.init_node("joy_mapper_node", anonymous=True)
	joy_mapper = JoyMapperNode()
	rospy.spin()

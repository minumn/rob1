#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from dynamixel_msgs.msg import *

class torque_listener:
	
	def callback(self, data):
		print data.load

	def __init__(self):
		print "subscribed :-)"
		self.sub = rospy.Subscriber("joint2/state", JointState, self.callback)


if __name__ == "__main__":
	rospy.init_node("Torque_Data")
	node = torque_listener()
	rospy.spin()

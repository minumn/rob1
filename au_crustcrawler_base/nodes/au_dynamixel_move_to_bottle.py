#!/usr/bin/env python
# coding: utf8

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
import math
from math import cos, sin, atan2, sqrt, pi
from std_msgs.msg    import Float64
from geometry_msgs.msg import Vector3



def invkin(xyz):
	"""
	Python implementation of the the inverse kinematics for the crustcrawler
	Input: xyz position
	Output: Angels for each joint: q1,q2,q3,q4
	
	You might adjust parameters (d1,a1,a2,d4).
	The robot model shown in rviz can be adjusted accordingly by editing au_crustcrawler_ax12.urdf
	"""

	print("Invkin called with " + str(xyz))

	# Settings
	elbovUP = True
	limit_q3 = 1.8
  	limit_q1 = 1.5
	limit_q2 = 1.8

	d1 = 16.6; # cm (height of 2nd joint)
	a1 = 0.0; # (distance along "y-axis" to 2nd joint)
	a2 = 17.2; # (distance between 2nd and 3rd joints)
	d4 = 24.8; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	# Functionality
	xc = xyz[0]
	yc = xyz[1]
	zc = xyz[2]

    #Calculate q1
	q1 = atan2(yc, xc)
    

    #Calculate q2 and q3
	
	x = sqrt(yc**2 + xc**2)
	y = zc-d1
	a1=a2
	a2=d4	
	
	D = (x**2+y**2-a1**2-a2**2)/(2*a1*a2)
    
	elbov = sqrt(1-D**2)

	if elbovUP:
		elbov = -elbov

	theta2 = atan2(elbov,D)
	theta1 = atan2(y, x)-atan2((a2*sin(theta2)),(a1+a2*cos(theta2)))
	
	q2 = theta1 - pi/2
	q3 = theta2

    #Calculate q4
	q4 = 0 # We don't consider rotation yet
	
	if q1>limit_q1:
		q1=limit_q1
	elif q1 < -limit_q1
		q1 = -limit_q1

	if q2>limit_q2:
		q2=limit_q2
	elif q2 < -limit_q2
		q2 = -limit_q2

	if q3>limit_q3:
		q3=limit_q3
	elif q3 < -limit_q3
		q3 = -limit_q3
		
	print('Leaving:' + str((q1,q2,q3,q4)))
	return [q1,q2,q3,q4]

class ActionExampleNode:

	N_JOINTS = 4
	def __init__(self,server_name):
		self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)

		self.joint_positions = []
		self.names =["joint1",
				"joint2",
				"joint3",
				"joint4"
				]
		# the list of xyz points we want to plan
		# self.setCoordinates(24.5)

		

	def send_command(self):
		self.client.wait_for_server()
		print "------ wait_for_server ------- \n" 
		print self.goal
		print "------------------------------ \n"
		self.client.send_goal(self.goal)
		

		print "------ wait_for_result ------- \n"
		self.client.wait_for_result()
		print self.client.get_result()
		print "------------------------------ \n"

	def setCoordinates(self, data):
		xyz_positions = [
		[data.x, data.y, data.z]
		]
		
		dur = rospy.Duration(1)

		# construct a list of joint positions by calling invkin for each xyz point
		self.joint_positions = []
		for p in xyz_positions:
			jtp = JointTrajectoryPoint(positions=invkin(p),velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
			dur += rospy.Duration(2)
			self.joint_positions.append(jtp)

		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )
		

def callback_xyz(data):
	rospy.loginfo("XYZ received here : {<}")
	
	global node
	node.setCoordinates(data)
	node.send_command()
	rospy.spin()
    
        

if __name__ == "__main__":
	global node
	rospy.init_node('listener_robot_mover', anonymous=True)
	rospy.Subscriber("setRobotXYZ", Vector3, callback_xyz)
	node= ActionExampleNode("/arm_controller/follow_joint_trajectory")
	rospy.spin()
	
	

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



def invkin(xyz):
	"""
	Python implementation of the the inverse kinematics for the crustcrawler
	Input: xyz position
	Output: Angels for each joint: q1,q2,q3,q4
	
	You might adjust parameters (d1,a1,a2,d4).
	The robot model shown in rviz can be adjusted accordingly by editing au_crustcrawler_ax12.urdf
	"""

	print("Invkin called with " + str(xyz))

	elbovUP = True

	d1 = 16.6; # cm (height of 2nd joint)
	a1 = 0.0; # (distance along "y-axis" to 2nd joint)
	a2 = 17.2; # (distance between 2nd and 3rd joints)
	d4 = 24.8; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	# Insert code here!!!
	oc = xyz #[2, 2, 1]
	xc = oc[0]
	yc = oc[1]
	zc = oc[2]

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
  
	limit_ = 1.5
	if q1>limit_:
		q1=limit_
	elif q1 < -limit_
		q1 = -limit_

	limit_ = 1.8
	if q2>limit_:
		q2=limit_
	elif q2 < -limit_
		q2 = -limit_

	limit_ = 1.8
	if q3>limit_:
		q3=limit_
	elif q3 < -limit_
		q3 = -limit_
		
	print('Leaving:' + str((q1,q2,q3,q4)))
	return [q1,q2,q3,q4]
	#return [0,0,0,0]

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
		xyz_positions = [
		[24.5, -16.7, 30],
		[24.5, -16.7, 20],
		]
		
		#rospy.Publisher("/gripper/command", Float64).publish(0.5)
		#x = rospy.Subscriber("center_x", Float64, callback_x)
		# initial duration
		dur = rospy.Duration(1)

		# construct a list of joint positions by calling invkin for each xyz point
		for p in xyz_positions:
			jtp = JointTrajectoryPoint(positions=invkin(p),velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
			#jtp = JointTrajectoryPoint(positions=[0,0,0,0],velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
			dur += rospy.Duration(2)
			self.joint_positions.append(jtp)

		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )
		

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

def callback_x(data):
	rospy.loginfo("X received here : %f",data.data)
    

def callback_y(data):
	rospy.loginfo("%f",data.data)
        
    
#def listener_x():
	#global x

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
	#rospy.init_node('listener_x', anonymous=True)

	#x = rospy.Subscriber("center_x", Float64, callback_x)
	#rospy.spin()
	#return x
	
#def listener_y():
#	global y
#	
#	rospy.init_node('listener_y',anonymous=True)
 #       
	#y = rospy.Subscriber("center_y", Float64, callback_y)
#	rospy.spin()


#listener_y()
if __name__ == "__main__":
	global x
	rospy.init_node('listener_x', anonymous=True)
	x = rospy.Subscriber("center_x", Float64, callback_x)
	#listener_x()
	#rospy.init_node("au_dynamixel_test_node")
	node= ActionExampleNode("/arm_controller/follow_joint_trajectory")
	node.send_command()
	rospy.spin()

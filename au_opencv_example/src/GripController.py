#!/usr/bin/env python

import rospy 
from std_msgs.msg    import Float64

class GripController:
    def __init__(self):
        self.joint_cmd_pub = rospy.Publisher("/gripper/command", Float64)
    
    def openGrip(self):
        self.joint_cmd_pub.publish(-0.25)

    def closeGrip(self):
        self.joint_cmd_pub.publish(0.89)



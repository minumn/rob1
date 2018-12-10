#!/usr/bin/env python

import rospy 
from std_msgs.msg    import Float64

class single_motor_node:
    def __init__(self):
        self.joint_cmd_pub = rospy.Publisher("/gripper/command", Float64)
        #self.sub = rospy.Subscriber()
        self.toggle = False
        self.timer = rospy.Timer(rospy.Duration(.5), self.on_timer)
        
    def on_timer(self,eventArg):
        rospy.loginfo("Timer called")
        if self.toggle:
            self.joint_cmd_pub.publish(0.5)
        else:
            self.joint_cmd_pub.publish(0.8)
        
        #self.toggle =  not self.toggle

if __name__ == "__main__":
    rospy.init_node("simple_motor_test")
    node = single_motor_node()
    rospy.spin()

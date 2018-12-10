#!/usr/bin/env python

# Run:
# roscore
# rosrun joy joy_node
# rosrun joystick joystick.py 

import rospy
from sensor_msgs.msg import Joy

def callback(data):
  print ("Callback...")
  #print data
  print data.axes[0], data.axes[1]
  print data.buttons 

# Intializes everything
def start():
  print("Starting...")
  rospy.Subscriber("/joy", Joy, callback)
  # starts the node
  rospy.init_node("JoyTest")
  rospy.spin()

if __name__ == '__main__':
  start()
    

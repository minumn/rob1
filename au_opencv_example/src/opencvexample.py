#!/usr/bin/env python
import cv2
import urllib
import numpy as np
import math
import rostopic
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import rospy



def get_from_webcam():
    """
    Fetches an image from the webcam
    """
    print "try fetch from webcam..."
    stream=urllib.urlopen('http://192.168.0.20/image/jpeg.cgi')
    bytes=''
    bytes+=stream.read(64500)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')

    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]
        i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
        return i
    else:
        print "did not receive image, try increasing the buffer size in line 13:"

def get_from_file(filename):
    """
    Loads image from file
    """
    print "loading from file..."
    return cv2.imread(filename)

def get_bricks(contours):
    global x
    global y
    global Center
    """
    For each contour in contours
        approximate the contours such that small variations are removed
        calulate the area of the contour
        if the area is within the desired range we append the box points to the
        bricks.
    """
    bricks = []
    for cnt in contours:

        (x,y), radius=cv2.minEnclosingCircle(cnt)
        center=(int(x),int(y))
        radius=int(radius)
        y = (300-center[0])/11.6
        x = (300-center[1])/8.5
        if radius > 10 and radius < 20:
			Center = (x,y)
			circle = cv2.circle(image,center,radius,(30,255,255),2)
			print("coordinate",x,y)
			talker()
			return bricks
    return center

def extract_single_color_range(image,hsv,lower,upper):
    """
    Calculates a mask for which all pixels within the specified range is set to 1
    the ands this mask with the provided image such that color information is
    still present, but only for the specified range
    """
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(image,image, mask= mask)
    #cv2.imshow('res',res)
    return res

def threshold_image(image,debug=False):
    """
    Thresholds the image within the desired range and then dilates with a 3x3 matrix
    such that small holes are filled. Afterwards the 'blobs' are closed using a
    combination of dilate and erode
    """
    ret,th1 = cv2.threshold(image,50,255,cv2.THRESH_BINARY)
    if debug: cv2.imshow('th1',th1)
    resdi = cv2.dilate(th1,np.ones((3,3),np.uint8))
    if debug: cv2.imshow('dilated',resdi)
    closing = cv2.morphologyEx(resdi, cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    if debug: cv2.imshow('closing',closing)
    

    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE,np.ones((1,1),np.uint8))    
    
    #cv2.imshow('closing', closing)
    #cv2.imshow('opened',opening)
    return closing

def contours(image,debug=False):
    """
    Extract the contours of the image by first converting it to grayscale and then
    call findContours
    """
    imgray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    if debug: cv2.imshow('gray_scale_contour',imgray)
    im2, contours, hierarchy = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    return contours,hierarchy


def do_full(image,hsv,upper,lower,debug=False):
    """
    Main methods for processing an image and detect rectangles in the given
    hsv color range

    set debug to True in order to show the intermediate images
    """

    single_color_img = extract_single_color_range(image,hsv,lower,upper)
    if debug:
        cv2.imshow('single_color_img',single_color_img)
    single_channel = threshold_image(single_color_img,debug)
    if debug:
        cv2.imshow('single_channel',single_channel)
    cont,hierarchy = contours(single_channel,debug)

    if debug:
        for i,cnt in enumerate(cont):
            cv2.drawContours(single_channel,cont,i,(0,0,255),2)
    if debug: cv2.imshow('contours',single_channel)

    return get_bricks(cont)

def show_bricks(image,bricks,color):
    for b in bricks:
        cv2.drawContours(image,[b],0,color,2)

def talker():
    global x
    global y
    data = Vector3(x, y, 30)
    pub = rospy.Publisher('setRobotXYZ', Vector3, queue_size=0)
    rospy.init_node('TalkerCameraCoordinates', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():

    rospy.loginfo(data)
    pub.publish(data)



#image = get_from_file('test.jpg')
image = get_from_webcam()
#cv2.imshow('raw',image)

image = image[77:414, 32:630]
#cv2.imshow('cropped', image)


hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_blue = np.array([100,50,50])
upper_blue = np.array([130,255,255])

lower_green = np.array([35,50,50])
upper_green = np.array([90,255,255])

lower_yellow = np.array([20,50,50])
upper_yellow = np.array([30,255,255])

lower_red = np.array([0,50,50])
upper_red = np.array([20,255,255])


#blue_bricks = do_full(crop_image,hsv,upper_blue,lower_blue,True)
#green_bricks = do_full(image,hsv,upper_green,lower_green)
yellow_bricks = do_full(image,hsv,upper_yellow,lower_yellow)
#red_bricks = do_full(image,hsv,upper_red,lower_red)

#show_bricks(image,blue_bricks,(255,0,0))
#show_bricks(image,green_bricks,(0,255,0))
show_bricks(image,yellow_bricks,(0,255,255))
#show_bricks(image,red_bricks,(0,0,255))



#cv2.imshow('result',image)
cv2.imwrite('result.jpg',image)
while True:
    c = cv2.waitKey(5)
    #if c != -1:
    cv2.destroyAllWindows()
    exit(0)


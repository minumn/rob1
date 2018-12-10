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

DEBUG = True
DATAFROMWEBCAM = False

# REF: Inspiration found on https://github.com/au-crustcrawler/au_opencv_example/blob/master/src/opencvexample.py?fbclid=IwAR0FTOImwGA_EJhDEodd2QmzwBPFp0cXghX7iKaL780cvCc5QA8qySCZg30

class RecognizeFlask:

	def __init__(self):
		self.brickCoordinates = list()
		self.brickCoordinatesBad = list()
		
		imageRaw = self.getImageRaw(fromWebcam=DATAFROMWEBCAM)

		self.image = image = self.cutImageMargins(imageRaw)

		self.hsv = hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		if DEBUG:
			cv2.imshow('raw', imageRaw)
			cv2.imshow('cropped', image)
			cv2.imshow('hsv', hsv)
			
		self.initColorMargins()
		
		color = "yellow"
		
		contours, hierarchy = self.findContours(image, hsv, color)

		bricks = self.getBricksFromContours(contours)
			
		cv2.imwrite('result.jpg',image)
	
	def  getImageRaw(self, fromWebcam = True):
		if fromWebcam:
			return self.get_from_webcam()
		else:
			return self.get_from_file('test.jpg')
	
	def cutImageMargins(self, image_raw):
		return image_raw[77:414, 32:630]
	
	def initColorMargins(self):
		self.lower_blue = np.array([100,50,50])
		self.upper_blue = np.array([130,255,255])

		self.lower_green = np.array([35,50,50])
		self.upper_green = np.array([90,255,255])

		self.lower_yellow = np.array([5,175,175])
		self.upper_yellow = np.array([75,230,230])

		self.lower_red = np.array([0,50,50])
		self.upper_red = np.array([20,255,255])
			
	def showBricks(self, bricks, color = "yellow"):
		color = color.lower()
		
		if color == "yellow":
			self.show_bricks(bricks, (0,255,255))
		elif color == "blue":
			self.show_bricks(bricks, (255,0,0))
		elif color == "green":
			self.show_bricks(bricks, (0,255,0))
		elif color == "red":
			self.show_bricks(bricks, (0,0,255))
		else:
			raise Exception("Color '{}' is not supported".format(color))
	
	def show_bricks(self, bricks, color):
		if bricks != None:
			for b in bricks:
				cv2.drawContours(self.image, [b], 0, color, 2)
	
	def get_from_webcam(self):
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
			print "did not receive image, try increasing the buffer size."

	def get_from_file(self, filename):
		"""
		Loads image from file
		"""
		print "loading from file..."
		file = cv2.imread(filename)
		if file == None:
			raise Exception("File not loaded correctly.")
		return file

	def getBricksFromContours(self, contours):
		"""
		For each contour in contours
			approximate the contours such that small variations are removed
			calulate the area of the contour
			if the area is within the desired range we append the box points to the
			bricks.
		"""
		for contour in contours:
			(x, y), radius=cv2.minEnclosingCircle(contour)
			center = (int(x), int(y))
			radius = int(radius)
			
			y = (300-center[0])/11.6
			x = (300-center[1])/8.5
			
			if radius > 10 and radius < 20:
				center = (x,y)
				circle = cv2.circle(image, center, radius, (30,255,255), 2)
				print("coordinate found: ", x, y)
				self.brickCoordinates.append((x, y))
			else:
				self.brickCoordinatesBad.append((x, y))
			
			if DEBUG:
				self.showBricks(bricks, color)

				cv2.imshow('result',image)

				# c = cv2.waitKey(5)
				# cv2.destroyAllWindows()
				# exit(0)
			
			return self.brickCoordinates			

	def extract_single_color_range_helper(self, image, hsv, color):

		if color == "yellow":
			return self.extract_single_color_range(image, hsv, self.upper_yellow, self.lower_yellow)
		elif color == "blue":
			return self.extract_single_color_range(image, hsv, self.upper_blue, self.lower_blue)
		elif color == "green":
			return self.extract_single_color_range(image, hsv, self.upper_green, self.lower_green)
		elif color == "red":
			return self.extract_single_color_range(image, hsv, self.upper_red, self.lower_red)
		else:
			raise Exception("Color '{}' is not supported".format(color))
	
	def extract_single_color_range(self, image, hsv, upper, lower):
		"""
		Calculates a mask for which all pixels within the specified range is set to 1
		the ands this mask with the provided image such that color information is
		still present, but only for the specified range
		"""
		
		mask = cv2.inRange(image, lower, upper)
		res = cv2.bitwise_and(image,image, mask= mask)
		cv2.imshow('res1',res)
		return res

	def threshold_image(self, image):
		"""
		Thresholds the image within the desired range and then dilates with a 3x3 matrix
		such that small holes are filled. Afterwards the 'blobs' are closed using a
		combination of dilate and erode
		"""
		ret,th1 = cv2.threshold(image,50,255,cv2.THRESH_BINARY)
		if DEBUG: cv2.imshow('th1',th1)
		resdi = cv2.dilate(th1,np.ones((3,3),np.uint8))
		if DEBUG: cv2.imshow('dilated',resdi)
		closing = cv2.morphologyEx(resdi, cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
		if DEBUG: cv2.imshow('closing',closing)
		

		opening = cv2.morphologyEx(image, cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
		closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE,np.ones((1,1),np.uint8))    
		
		cv2.imshow('closing', closing)
		cv2.imshow('opened',opening)
		return closing

	def findContours(self, image, hsv, color):
		"""
		Extract the contours of the image by first converting it to grayscale and then
		call findContours
		"""
		
		single_color_img = self.extract_single_color_range_helper(image, hsv, color)
		single_channel = self.threshold_image(single_color_img)
		
		imgray = cv2.cvtColor(single_channel,cv2.COLOR_BGR2GRAY)


		im2, contours, hierarchy = cv2.findContours(imgray, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		if DEBUG: 
			cv2.imshow('gray_scale_contour', imgray)
			
			cv2.imshow('single_color_img', single_color_img)
			
			cv2.imshow('single_channel', single_channel)
			
			for i, cnt in enumerate(contours):
				cv2.drawContours(single_channel,cnt,i,(0,0,255),2)
				
			cv2.imshow('contours',single_channel)
			
		return contours, hierarchy
		
	def getFlaskCoordinates(self):
		return self.brickCoordinates

	def getFlaskCoordinatesBad(self):
		return self.brickCoordinatesBad

def publishResult(resultList):
	(x, y) = resultList[1]
	
	data = Vector3(x, y, 30)
	pub = rospy.Publisher('setRobotXYZ', Vector3, queue_size=0)
	rospy.init_node('TalkerCameraCoordinates', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	rospy.loginfo(data)
	pub.publish(data)


if __name__ == "__main__":
	recognizeFlask = RecognizeFlask()
	flaskCoordinates = recognizeFlask.getFlaskCoordinates()
	flaskCoordinatesBad = recognizeFlask.getFlaskCoordinatesBad()
	print ""
	print "Good spots:"
	print(flaskCoordinates)
	print("Bad spots: ")
	print(flaskCoordinatesBad)
	#publishResult(flaskCoordinates)
		
		

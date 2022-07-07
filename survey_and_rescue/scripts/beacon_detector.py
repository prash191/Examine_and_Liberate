#!/usr/bin/env python



from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from cv_bridge import CvBridge, CvBridgeError
import random
import pickle
# import imutils
import copy
import json

clm = ['A','B','C','D','E','F']
row = ['1','2','3','4','5','6']
A = [] #global variable to store the whycon coordinate of RoIs as a list.



class sr_determine_colors():

	def __init__(self):
		self.detect_info_msg = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10) 
 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
 		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
 		self.img = None
 		self.data = {} #dictionary to store RoIs and there respective whycon coordinates.
 		self.last_pub = [] # list to store lit leds from previous frame to avoid redundant detection.


# 

# * Function Name: load_rois..
# * Input: NONE.
# * Output: NONE.
# * Logic: store the whycon coordinate from RoIs.json file.
# *
# * Example Call: called once in main function.

# 

	def load_rois(self, file_path = 'rect_info.pkl'):
		with open("/home/prashant/catkin_ws/src/Examine_and_Liberate/survey_and_rescue/scripts/RoIs.json", "r") as read_file:
			self.data = json.load(read_file)
			for i in clm:
			    for j in row:
			    	# stores the whycon coordinate of RoIs in row major format in list.(eg:A1,A2,A3....B1,B2,B3...etc)
			        A.extend([self.data[i+j]])

	



# 

# * Function Name: image_callback
# * Input: data
# * Output: NONE
# * Logic: stores the current image of video in self.img.
# *
# * Example Call: called when "/usb_cam/image_rect_color" topic is subscribed.

# 

 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 		except CvBridgeError as e:
 			print(e)


 	def serviced_callback(self, msg):
 		pass
 	
# 

# * Function Name: find_color_contour_centers.
# * Input: NONE.
# * Output: NONE.
# * Logic: scan all the RoIs for lit leds and store the lit leds RoIs in the list.
# *
# * Example Call: called repeatedly in main function.

# 
	def find_color_contour_centers(self):
		
		inf=[] #list to store lit leds RoIs and its type in current frame led.
		j=0
		for i in A: # check all 36 RoIs for lit led.

			loc=str(unichr(j//6+65))+str(j%6+1)
			j=j+1
			st=""
			x,y,w,h= i
			image = self.img
			cp_image=image[y+5:y+h-20,x+5:x+h-20] #------------------
			hsv = cv2.cvtColor(cp_image,cv2.COLOR_BGR2HSV)
			# cv2.imshow("hsv",hsv)
			# cv2.waitKey(0)
			lower_red = np.array([140,105,210])
			upper_red = np.array([255,255,255])
			mask = cv2.inRange(hsv, lower_red, upper_red)

			M = cv2.moments(mask) 
			if M["m00"] <1000: # no red led.
				pass
			else: # red led detected.
				st="RESCUE"
				inf.append((loc,st))# append RoI and its type(rescue) to "inf" list.
				continue



			# blue LED
			# lower_blue = np.array([100,150,150])
			lower_blue = np.array([100,150,170])
			upper_blue = np.array([130,255,255])# upper limit of hsv for blue led detection.
			mask = cv2.inRange(hsv, lower_blue, upper_blue)
			M = cv2.moments(mask)
			if M["m00"] < 1000: # no blue led.
				pass
			else: # blue led detected.
				st="MEDICINE"
				inf.append((loc,st))# append RoI and its type(food) to "inf" list.
				continue



			#greeen LED
			lower_green = np.array([52,25,212])
			# lower_green = np.array([62,35,222])
			upper_green = np.array([80,255,255])# upper limit of hsv for green led detection.
			mask = cv2.inRange(hsv, lower_green, upper_green)

			M = cv2.moments(mask)
			if M["m00"] < 1000: # no green led.
				pass
			else: # green led detected.
				st="FOOD"
				inf.append((loc,st)) # append RoI and its type(medicine) to "inf" list.
				continue

			
#  inf list stores the lit leds in current frame.
#  self.last_pub stores the lit leds in previous frame.
#  in next lines of code, we compare both the list and publish the newly lit leds 
#* and ignore the already publish leds.

		for i in inf: # all the lit leds in current frame.
			flag = False #
			for j in self.last_pub: # compare the current lit led in inf list with all the lit leds in previous frame(self.last_pub) for redundant detection.
				if i==j: # if the current led in inf was already published.
					flag=True
					break
			if flag==False: # if the current led is not previously published.
				self.detect_info_msg.location = i[0]
				self.detect_info_msg.info = i[1]
				self.detect_pub.publish(self.detect_info_msg)
				
		self.last_pub = inf # make the current frame lit leds as previous frame lit leds for computation in next frame.



	def check_whether_lit(self):
		pass



# 

# * Function Name: main
# * Input: NONE
# * Output: NONE
# * Logic: keep running find_color_contour_centers if rospy is not shutdown.
# *
# * Example Call: NONE

# 

def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		'''You may choose a suitable rate to run the node at.
		Essentially, you will be proceesing that many number of frames per second.
		Since in our case, the max fps is 30, increasing the Rate beyond that
		will just lead to instances where the same frame is processed multiple times.'''
		rate = rospy.Rate(30)
		# rate = rospy.Rate(5)
		s.load_rois()
		while s.img is None:
			pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.find_color_contour_centers()
			s.check_whether_lit()
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

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
import copy
import json

class sr_determine_colors():
	def __init__(self):
		self.detect_info_msg = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10) 
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		self.clm = ['A', 'B', 'C', 'D', 'E', 'F']
		self.row = ['1', '2', '3', '4', '5', '6']
 		self.img=None
 		self.rect_list=[]
 		self.color_bound=np.array([[255,0,0],[0,255,0],[0,0,255]])
 		self.info=("MEDICINE","FOOD","RESCUE")
 		self.beacons={}
 		rospy.sleep(2)
 		

	# def load_rois(self, file_path = 'rect_info.pkl'):
	# 	try:
	# 		# s.rois = np.load("rois.npy")
	# 		with open(file_path, 'rb') as input:
   	# 			self.rect_list = pickle.load(input)
	# 	except IOError as ValueError:
			# print("File doesn't exist or is corrupted")

	def load_rois(self, file_path = 'RoIs.json'):
		try:
			with open(file_path, "r") as read_file:
				self.data = json.load(read_file)
			for i in self.clm:
			    for j in self.row:
			        self.rect_list.extend([self.data[i+j]])
		except IOError as ValueError:
			print("File doesn't exist or is corrupte")

	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
 			print(e)

	def serviced_callback(self, msg):
 		rospy.sleep(0.2)
 		self.beacons[msg.location]="OFF"
 	
	def detect_color_contour_centers(self, color_str):
		img_copy=cv2.threshold(self.img, 127, 255, cv2.THRESH_BINARY)[1]
		cv2.imshow("abc", img_copy)
		cv2.waitKey(0)
		k=0
		for i in self.rect_list:
				location=str(chr(k/6+65)+str(k%6+1))
				x,y,w,h = i
				cell=img_copy[y:y+h ,x:x+w]
				for l in range(0,3):
					mask=cv2.inRange(cell,self.color_bound[l],self.color_bound[l])
					if np.sum(mask==255)>30:
						try:
						    if self.beacons[location]!=self.info[l]:
								self.detect_info_msg.location=location
								self.detect_info_msg.info=self.info[l]
								self.detect_pub.publish(self.detect_info_msg)
								self.beacons[location]=self.info[l]
						except KeyError:
							self.detect_info_msg.location=location
							self.detect_info_msg.info=self.info[l]
							self.detect_pub.publish(self.detect_info_msg)
							self.beacons[location]=self.info[l]
						break
				k=k+1

	def check_whether_lit(self):
		pass


def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		'''You may choose a suitable rate to run the node at.
		Essentially, you will be proceesing that many number of frames per second.
		Since in our case, the max fps is 30, increasing the Rate beyond that
		will just lead to instances where the same frame is processed multiple times.'''
		rate = rospy.Rate(20)
		# rate = rospy.Rate(5)
		s.load_rois()
		# while s.img is None:
		# 	print("herer-------")
		# 	pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.detect_color_contour_centers(None)
			s.check_whether_lit()
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
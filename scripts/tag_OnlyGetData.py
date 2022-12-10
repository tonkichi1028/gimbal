#!/usr/bin/env python
# -*- coding: utf-8 -*-
# PWM : 35.743 ~ 59.179 -- 61.133 ~ 86.523

import rospy
import sys
import cv2
import message_filters
import numpy as np
import math
import rosparam
import yaml
import time
import Jetson.GPIO as GPIO
import csv
# msg
from sensor_msgs.msg import Image, CameraInfo
from apriltag_ros.msg import AprilTagDetectionArray

from cv_bridge import CvBridge, CvBridgeError



class tracking_apriltag(object):
	def __init__(self):
		# ROS
		rospy.init_node("tracking_apriltag")
		rospy.on_shutdown(self.cleanup)

		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/masking_image", Image, queue_size=1)
		self.info_pub = rospy.Publisher("/masking_info", CameraInfo, queue_size=1)
		self.tag_det = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_camera_callback)
		sub1 = message_filters.Subscriber('/usb_cam/image_raw', Image)
		sub2 = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		# Tag_camera
		self.Position_old_camera = [0, 0, 0]
		self.Position_predicted_camera = [0, 0, 0]
		self.delta_Position_camera = [0, 0, 0]

		# flag
		self.flag_camera = 0
		self.flag_image = 0
		self.flag_detection = 0
		self.flag_data = 0

		# Time
		self.time_start = 0
		self.time = 0

		# data
		self.data = []
		self.TagPosCamera_data = [["time"],["x"],["y"],["z"]]

		self.save_time = 1000



	# Save Data
	def get_data(self):
		f = open('/home/wanglab/catkin_ws/src/gimbal/data/2022.12.06/test.csv', 'w')

		self.data.extend(self.TagPosCamera_data)
		data_all = self.data
		writer = csv.writer(f)

		for data in data_all:
			writer.writerow(data)
		f.close()
		print("finish!!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
		self.flag_data = 1



	def image_callback(self, ros_image,camera_info):
		output_image = ros_image
		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)



	def tag_camera_callback(self, data_camera):
		if len(data_camera.detections) >= 1:			
			self.Position_now_camera = data_camera.detections[0].pose.pose.pose.position
			self.TagPosCamera_data[0].append(self.time)
			self.TagPosCamera_data[1].append(self.Position_now_camera.x)
			self.TagPosCamera_data[2].append(self.Position_now_camera.y)
			self.TagPosCamera_data[3].append(self.Position_now_camera.z)



	def timer(self,event=None):	
		# TIME
		if self.time_start == 0:
			self.time_start = rospy.get_time()
		else:
			self.time = rospy.get_time()-self.time_start

		# get_data
		if int(self.time) == self.save_time:
			if self.flag_data == 0:
				self.get_data()



	def cleanup(self):
		cv2.destroyAllWindows()



if __name__ == "__main__":
	ts = tracking_apriltag()
	rospy.Timer(rospy.Duration(1.0/100), ts.timer)
	rospy.spin()


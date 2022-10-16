#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import cv2
import message_filters
import numpy as np
import math
import tf
import rosparam
import tf2_ros
import yaml
import time
#msg
from sensor_msgs.msg import Image, CameraInfo
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetectionPositionArray

from cv_bridge import CvBridge, CvBridgeError
from collections import deque






class tracking_apriltag(object):
	def __init__(self):
		#ROS
		rospy.init_node("tracking_apriltag")
		rospy.on_shutdown(self.cleanup)

		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/masking_image", Image, queue_size=1)
		self.info_pub = rospy.Publisher("/masking_info", CameraInfo, queue_size=1)
		self.tag_det = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_camera_callback)
		self.tag_pos = rospy.Subscriber('/tag_position',AprilTagDetectionPositionArray,self.tag_image_callback)
		sub1 = message_filters.Subscriber('/usb_cam/image_raw', Image)
		sub2 = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		#Tag_camera
		self.Position_old_camera = [0, 0, 0]

		#Tag_image
		self.Position_old_image = [640, 360]
		
		#data
		self.data = [["time"],["tagdata[pix]"]]
		self.time = 0
		self.time_start = 0
		self.area_mask = 0
		self.time_old = 0
		self.margin_u = 640
		self.margin_v = 360
		self.fps = 0



	

	
	def get_data(self):
			f = open('/home/wanglab/catkin_ws/src/gimbal/data/2022.10.06_data/Mask_data2_many.txt', 'w')
			f.write(str(self.data))
			f.close()
			print("finish!!!!")


	def image_callback(self, ros_image,camera_info):
		#TIME
		if self.time_start == 0:
			self.time_start = time.time()
		else:
			self.time = time.time()-self.time_start
	
		#print(self.time)
		if int(self.time) == 20:
			#pass
			self.get_data()
		else:
			pass
		
		input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		output_image = self.image_process(input_image)
		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)








	def image_process(self, input_image):
		
		mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.Wide_Mask()
		
		mask0_u0 = int(mask0_u0)
		mask0_v0 = int(mask0_v0)
		mask0_u1 = int(mask0_u1)
		mask0_v1 = int(mask0_v1)
		
		#self.area_mask = (mask0_u1-mask0_u0)*(mask0_v1-mask0_v0)

		mask_image = cv2.rectangle(input_image,(0,0),(1280,mask0_v0),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(0,mask0_v1),(1280,720),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(0,0),(mask0_u0,720),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(mask0_u1,0),(1280,720),color=0, thickness=-1)
		
		#mask_image = input_image

		output_image = self.bridge.cv2_to_imgmsg(np.array(mask_image), "bgr8")
		
		return output_image






	
	def Wide_Mask(self):

		center_u = self.Position_old_image[0]
		center_v = self.Position_old_image[1]
		margin_u = 640
		margin_v = 360
		less_pix = 5
		
		mask0_u0 = center_u - self.margin_u 
		mask0_u1 = center_u + self.margin_u
		mask0_v0 = center_v - self.margin_v
		mask0_v1 = center_v + self.margin_v
		"""
		mask0_u0 = center_u - margin_u 
		mask0_u1 = center_u + margin_u
		mask0_v0 = center_v - margin_v
		mask0_v1 = center_v + margin_v
		
		if less_pix*self.time < 280:
			mask0_u0 = mask0_u0 + less_pix*self.time 
			mask0_u1 = mask0_u1 - less_pix*self.time
#			mask0_v0 = mask0_v0 + less_pix*self.time
#			mask0_v1 = mask0_v1 - less_pix*self.time
		else:
			mask0_u0 = mask0_u0 + less_pix*self.time 
			mask0_u1 = mask0_u1 - less_pix*self.time
			mask0_v0 = mask0_v0 + less_pix*self.time -280
			mask0_v1 = mask0_v1 - less_pix*self.time +280
		"""		
		
		return mask0_u0,mask0_u1,mask0_v0,mask0_v1





	def tag_camera_callback(self,data_camera):

		if len(data_camera.detections) >= 1:
			pass
				

		else:
			self.margin_u = 640
			self.margin_v = 360



	def tag_image_callback(self, data_image):

		if len(data_image.detect_positions) >= 1:
			
			#tag
			self.Position_old_image[0] = data_image.detect_positions[0].x
			self.Position_old_image[1] = data_image.detect_positions[0].y

			self.data[1].append(self.Position_old_image[0])
			self.data[0].append(self.time)
			self.margin_u = 50
			self.margin_v = 50
							

		else:
			self.Position_old_image = [640, 360]















		



	def cleanup(self):
		cv2.destroyAllWindows()



if __name__ == "__main__":
	tracking_apriltag()
	rospy.spin()

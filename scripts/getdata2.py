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
from apriltag_ros.msg import AprilTagDetectionPositionArray

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
		self.tag_pos = rospy.Subscriber('/tag_position',AprilTagDetectionPositionArray,self.tag_image_callback)
		sub1 = message_filters.Subscriber('/usb_cam/image_raw', Image)
		sub2 = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		# Tag_camera
		self.Position_old_camera = [0, 0, 0]
		self.Position_predicted_camera = [0, 0, 0]
		self.delta_Position_camera = [0, 0, 0]

		# Tag_image
		self.Position_old_image = [0, 0]
		self.Position_now_image = [0, 0]
		self.Position_predicted_image = [0, 0]
		self.delta_Position_image = [0, 0]
		self.delta_delta_Position_image = [0, 0]

		# gimbal_init
		pitch_pin = 32
		yaw_pin = 33
		GPIO.setmode(GPIO.BOARD)

		# pitch init
		GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.pitch = GPIO.PWM(pitch_pin, 400)
		self.pitch.start(60.156)
		# yaw init
		GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.yaw = GPIO.PWM(yaw_pin, 400)
		self.yaw.start(60.156)

		# pwm_input_value, [0]=t, [1]=t-1, center_value=60.156
		self.pitch_input_pwm = 60.156
		self.yaw_input_pwm = 60.156

		# error_value_deg, [0]=t, [1]=t-1, [2]=t-2
		self.pitch_error = [0.00, 0.00, 0.00]
		self.yaw_error = [0.00, 0.00, 0.00]
		
		# flag
		self.flag_camera = 0
		self.flag_image = 0
		self.flag_detection = 0
		self.flag_data = 0

		# Pitch PID
		self.pitch_P = 0.055
		self.pitch_I = 0.0006
		self.pitch_D = 0.0025

		# yaw PID
		self.yaw_P = 0.055
		self.yaw_I = 0.0001
		self.yaw_D = 0.003

		# Time
		self.time_start = 0
		self.time = 0

		# data
		self.data = []
		self.TagPosImg_data = [["time"],["Image_u"],["Image_v"],["area"]]

		self.save_time = 60



	# Save Data
	def get_data(self):
		f = open('/home/wanglab/catkin_ws/src/gimbal/data/2022.12.08/AreaFixMetroFix.csv', 'w')

		self.data.extend(self.TagPosImg_data)
		data_all = self.data
		writer = csv.writer(f)

		for data in data_all:
			writer.writerow(data)
		f.close()
		print("finish!!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
		self.flag_data = 1



	def image_callback(self, ros_image,camera_info):

		if self.flag_detection == 1:
			input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
			output_image = self.image_process(input_image)
		else:
			output_image = ros_image

		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)



	def image_process(self, input_image):
		mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.Wide_Mask()
		#mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.Wide_Tag(mask0_u0,mask0_u1,mask0_v0,mask0_v1)

		mask_image = cv2.rectangle(input_image,(0,0),(1280,int(mask0_v0)),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(0,int(mask0_v1)),(1280,720),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(0,0),(int(mask0_u0),720),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(int(mask0_u1),0),(1280,720),color=0, thickness=-1)
		"""
		# Point Predicted
		ppu = self.Position_predicted_image[0]
		ppv = self.Position_predicted_image[1]
		mask_image = cv2.circle(input_image,(int(ppu),int(ppv)),5,color=(0,0,255), thickness=-1)
		"""
		output_image = self.bridge.cv2_to_imgmsg(np.array(mask_image), "bgr8")

		return output_image




	
	def Wide_Mask(self):
		center_u = self.Position_predicted_image[0]
		center_v = self.Position_predicted_image[1]

		f = 1581
		z = self.Position_predicted_camera[2]
		Length_Tag_world = 0.043

		Length_Tag_image = f * Length_Tag_world / z
		alpha = 1.3
		"""
		mask0_u0 = center_u - Length_Tag_image * alpha 
		mask0_u1 = center_u + Length_Tag_image * alpha
		mask0_v0 = center_v - Length_Tag_image * alpha
		mask0_v1 = center_v + Length_Tag_image * alpha
		"""
		mask0_u0 = center_u - 80 
		mask0_u1 = center_u + 80
		mask0_v0 = center_v - 80
		mask0_v1 = center_v + 80

		# GetData
		self.TagPosImg_data[0].append(self.time)
		self.TagPosImg_data[1].append(self.Position_now_image.x)
		self.TagPosImg_data[2].append(self.Position_now_image.y)
		self.TagPosImg_data[3].append((mask0_u1-mask0_u0)*(mask0_v1-mask0_v0))
	
		return mask0_u0,mask0_u1,mask0_v0,mask0_v1



	def Wide_Tag(self,mask0_u0,mask0_u1,mask0_v0,mask0_v1):
		alpha = 4
		delta_Position_Tag = self.delta_Position_image
		# u
		if self.delta_Position_image[0] >= 0:
			mask0_u1 = mask0_u1 + self.delta_Position_image[0]*alpha
		else:
			mask0_u0 = mask0_u0 + self.delta_Position_image[0]*alpha
		# v
		if self.delta_Position_image[1] >= 0:
			mask0_v1 = mask0_v1 + self.delta_Position_image[1]*alpha
		else:
			mask0_v0 = mask0_v0 + self.delta_Position_image[1]*alpha
		
		return mask0_u0,mask0_u1,mask0_v0,mask0_v1



	def tag_camera_callback(self,data_camera):
		if len(data_camera.detections) >= 1:

			if self.flag_camera == 0:
				self.flag_camera = 1
				self.Position_old_camera = data_camera.detections[0].pose.pose.pose.position
			else:
				Position_now_camera = data_camera.detections[0].pose.pose.pose.position
				self.Position_predicter_camera(Position_now_camera)
				self.Position_old_camera = Position_now_camera
				self.flag_detection = 1
		else:
			self.Position_old_camera = [0, 0, 0]
			self.flag_camera = 0
			self.flag_detection = 0



	def tag_image_callback(self, data_image):
		if len(data_image.detect_positions) >= 1:
			
			if self.flag_image == 0:
				self.Position_old_image = data_image.detect_positions[0]
				self.flag_image = 1
			else:
				self.Position_now_image = data_image.detect_positions[0]

				self.Position_predicter_image()
				self.pixel_error()
				# controller
				#self.pitch_pid_controller()
				#self.yaw_pid_controller()

				self.Position_old_image = self.Position_now_image
		else:
			# init
			self.pitch_input_pwm = 60.156
			self.pitch.ChangeDutyCycle(self.pitch_input_pwm)
			self.yaw_input_pwm = 60.156
			self.yaw.ChangeDutyCycle(self.yaw_input_pwm)

			self.pitch_error = [0.00, 0.00, 0.00]
			self.yaw_error = [0.00, 0.00, 0.00]

			self.Position_old_image = [0, 0, 0]
			self.Position_predicted_image = [640, 360]
			self.flag_image = 0



	def pitch_pid_controller(self,event=None):
		P = self.pitch_P
		I = self.pitch_I
		D = self.pitch_D

		P = P*(self.pitch_error[0]-self.pitch_error[1])
		I = I*self.pitch_error[0]
		D = D*((self.pitch_error[0]-self.pitch_error[1])-(self.pitch_error[1]-self.pitch_error[2]))

		self.pitch_input_pwm =  self.pitch_input_pwm + P + I + D

		# commandable area of PWM
		if self.pitch_input_pwm >= 86.523:
			self.pitch_input_pwm = 86.523
			self.pitch.ChangeDutyCycle(self.pitch_input_pwm)

		elif self.pitch_input_pwm <= 35.743:
			self.pitch_input_pwm = 35.743
			self.pitch.ChangeDutyCycle(self.pitch_input_pwm)

		else:
			self.pitch.ChangeDutyCycle(self.pitch_input_pwm)

		# storage of error values
		self.pitch_error[2] = self.pitch_error[1]
		self.pitch_error[1] = self.pitch_error[0]



	def yaw_pid_controller(self,event=None):
		P = self.yaw_P
		I = self.yaw_I
		D = self.yaw_D
		
		P = P*(self.yaw_error[0]-self.yaw_error[1])
		I = I*self.yaw_error[0]
		D = D*((self.yaw_error[0]-self.yaw_error[1])-(self.yaw_error[1]-self.yaw_error[2]))

		self.yaw_input_pwm = self.yaw_input_pwm + P + I + D

		# commandable area of PWM
		if self.yaw_input_pwm >= 86.523:
			self.yaw_input_pwm = 86.523
			self.yaw.ChangeDutyCycle(self.yaw_input_pwm)
			
		elif self.yaw_input_pwm <= 35.743:
			self.yaw_input_pwm = 35.743
			self.yaw.ChangeDutyCycle(self.yaw_input_pwm)

		else:
			self.yaw.ChangeDutyCycle(self.yaw_input_pwm)
		# storage of error values
		self.yaw_error[2] = self.yaw_error[1]
		self.yaw_error[1] = self.yaw_error[0]



	def Position_predicter_camera(self,Position_now_camera):
		self.delta_Position_camera[0] = Position_now_camera.x - self.Position_old_camera.x
		self.delta_Position_camera[1] = Position_now_camera.y - self.Position_old_camera.y
		self.delta_Position_camera[2] = Position_now_camera.z - self.Position_old_camera.z

		self.Position_predicted_camera[0] = Position_now_camera.x + self.delta_Position_camera[0]
		self.Position_predicted_camera[1] = Position_now_camera.y + self.delta_Position_camera[1]
		self.Position_predicted_camera[2] = Position_now_camera.z + self.delta_Position_camera[2]



	def Position_predicter_image(self):
		self.delta_Position_image[0] = self.Position_now_image.x - self.Position_old_image.x
		self.delta_Position_image[1] = self.Position_now_image.y - self.Position_old_image.y
		"""
		self.Position_predicted_image[0] = self.Position_now_image.x
		self.Position_predicted_image[1] = self.Position_now_image.y
		"""
		self.Position_predicted_image[0] = self.Position_now_image.x + self.delta_Position_image[0]
		self.Position_predicted_image[1] = self.Position_now_image.y + self.delta_Position_image[1]
		"""
		self.Position_predicted_image[0] = self.Position_now_image.x + self.delta_Position_image[0] + 1/2*(self.delta_Position_image[0] - self.delta_delta_Position_image[0])
		self.Position_predicted_image[1] = self.Position_now_image.y + self.delta_Position_image[1] + 1/2*(self.delta_Position_image[1] - self.delta_delta_Position_image[1])
		"""
		self.delta_delta_Position_image = self.delta_Position_image

	def pixel_error(self):
		error_pitch = -(360 - self.Position_predicted_image[1])
		error_yaw = (640 - self.Position_predicted_image[0])
		safe_pix = 0
		# tolerance of pixel
		if -safe_pix <= error_pitch <= safe_pix:
			self.pitch_error[0] = 0
		else:
			self.pitch_error[0] = error_pitch

		if -safe_pix <= error_yaw <= safe_pix:
			self.yaw_error[0] = 0
		else:
			self.yaw_error[0] = error_yaw



	def cleanup(self):
		cv2.destroyAllWindows()
		self.pitch.stop()
		self.yaw.stop()
		GPIO.cleanup()		



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



if __name__ == "__main__":
	ts = tracking_apriltag()
	rospy.Timer(rospy.Duration(1.0/100), ts.timer)
	rospy.spin()


#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pwm: 7.25 +- 2.5

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
import matplotlib.pyplot as plt
# msg
from sensor_msgs.msg import Image, CameraInfo
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetectionPositionArray
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError






class tracking_apriltag(object):
	def __init__(self):
		# ROS
		rospy.init_node("tracking_apriltag")
		rospy.on_shutdown(self.cleanup)

		self.bridge = CvBridge()
		self.timer_pub = rospy.Publisher("/timer", Float64, queue_size=1)
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
		self.Position_predicted_image = [0, 0]
		self.delta_Position_image = [0, 0]

		# gimbal_init
		pitch_pin = 32
		yaw_pin = 33
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.pitch = GPIO.PWM(pitch_pin, 50)
		GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.yaw = GPIO.PWM(yaw_pin, 400)
		self.yaw.start(60.156)


		# pwm_input_value, [0]=t, [1]=t-1, center_value=7.422
		self.pitch_input_pwm = 7.422
		self.yaw_input_pwm = 60.156#7.422#14.844#30.079#60.156
		self.stop_pwm = 7.422

		# error_value_deg, [0]=t, [1]=t-1, [2]=t-2
		self.pitch_error = [0.00, 0.00, 0.00]
		self.yaw_error = [0.00, 0.00, 0.00]
		
		# flag
		self.flag_camera = 0
		self.flag_image = 0
		self.flag_detection = 0
		self.flag_yaw_graph = 1

		# Time
		self.time_start = 0
		self.time = 0
		self.Hz = 0

		# data
		self.data = []
		self.TagPosImg_data = [["time"],["image_u"],["image_v"]]
		self.pwm_data = [["time"],["pwm_pitch"],["time"],["pwm_yaw"]]
		self.error_data = [["time"],["error0"]]


		# Pitch PID
		self.pitch_P = 0.0008
		self.pitch_I = 0.00
		self.pitch_D = 0.00

		# yaw PID
		self.yaw_P = 0.05#78
		self.yaw_I = 0.0000#6
		self.yaw_D = 0.0#1#17
		#self.yaw_P = 0.00582
		#self.yaw_I = 0.0005
		#self.yaw_D = 0.00065

		self.save_time = 5

		self.sleep_time = 1/50



	# Save Yaw Graph	
	def get_yaw_graph(self):
		p = self.yaw_P
		i = self.yaw_I
		d = self.yaw_D

		t1 = self.TagPosImg_data[0][1:]
		u_axis = self.TagPosImg_data[1][1:]
		"""
		t2 = self.pwm_data[2][1:]
		pwm = self.pwm_data[3][1:]
		"""
		fig = plt.figure(linewidth=1)

		# 上
		# pid line
		#ax1 = fig.add_subplot(2, 1, 1)
		ax1 = fig.add_subplot(1, 1, 1)
		ax1.set_title("P : %1.5f   "%p + "I : %1.5f   "%i + "D : %1.5f"%d, fontsize=16)
		#ax1.set_title("sleep_time : %1.7f"%self.sleep_time, fontsize=16)		
		ax1.set_xlabel('t[s]', fontsize=18)
		ax1.set_ylabel('u-axis[pix]', fontsize=18)
		ax1.plot(t1, u_axis, marker='.', label = "response")
		# center line
		center = 0
		ax1.axhline(center, ls = "--",color = "black",  label = "center")
		ax1.legend(loc="upper right")
		"""
		# 下
		# pwm line
		ax2 = fig.add_subplot(2, 1, 2)
		ax2.set_title('pwm', fontsize=22)
		ax2.set_xlabel("t[s]", fontsize=18)
		ax2.set_ylabel("pwm", fontsize=18)
		ax2.plot(t2, pwm, marker='.', label = "pwm")
		# center line
		center1 = 7.226
		center2 = 7.618
		ax2.axhline(center1, ls = "--",color = "black",  label = "center")
		ax2.axhline(center2, ls = "--",color = "black",  label = "center")
		ax2.legend(loc="upper right")
		"""
		fig.tight_layout()

		#fig.savefig("/home/wanglab/catkin_ws/src/gimbal/Image/2022.11.18/metro120.png", bbox_inches='tight')
		fig.savefig("/home/wanglab/catkin_ws/src/gimbal/Image/2022.11.21/Yaw " + "P_%1.5f"%p + "I_%1.5f"%i + "D_%1.5f"%d + ".png", bbox_inches='tight')
		#fig.savefig("/home/wanglab/catkin_ws/src/gimbal/Image/2022.11.18/sleeptime%1.7f"%self.sleep_time + ".png", bbox_inches='tight')
		self.flag_yaw_graph = 1




	def image_callback(self, ros_image,camera_info):
		
		#TIME
		if self.time_start == 0:
			self.time_start = rospy.get_time()
		else:
			self.time = rospy.get_time()-self.time_start

		# get_yaw_graph
		if int(self.time) == self.save_time:
			if self.flag_yaw_graph == 0:
				self.get_yaw_graph()
			
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
		#print(self.yaw_input_pwm)


	def image_process(self, input_image):
		mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.Wide_Mask()
		mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.Wide_Tag(mask0_u0,mask0_u1,mask0_v0,mask0_v1)


		mask_image = cv2.rectangle(input_image,(0,0),(1280,int(mask0_v0)),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(0,int(mask0_v1)),(1280,720),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(0,0),(int(mask0_u0),720),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(int(mask0_u1),0),(1280,720),color=0, thickness=-1)
		
		output_image = self.bridge.cv2_to_imgmsg(np.array(mask_image), "bgr8")

		return output_image






	
	def Wide_Mask(self):
		center_u = self.Position_predicted_image[0]
		center_v = self.Position_predicted_image[1]

		f = 1449
		z = self.Position_predicted_camera[2]
		Length_Tag_world = 0.035

		Length_Tag_image = f * Length_Tag_world / z
		alpha = 1.2
		
		mask0_u0 = center_u - Length_Tag_image * alpha 
		mask0_u1 = center_u + Length_Tag_image * alpha
		mask0_v0 = center_v - Length_Tag_image * alpha
		mask0_v1 = center_v + Length_Tag_image * alpha
	
		return mask0_u0,mask0_u1,mask0_v0,mask0_v1





	def Wide_Tag(self,mask0_u0,mask0_u1,mask0_v0,mask0_v1):
		alpha = 2
		#alpha = 10000
		delta_Position_Tag = self.delta_Position_image
		# x
		if self.delta_Position_image[0] >= 0:
			mask0_u1 = mask0_u1 + self.delta_Position_image[0]*alpha
		else:
			mask0_u0 = mask0_u0 + self.delta_Position_image[0]*alpha

		# y
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
				self.flag_image = 1
				self.Position_old_image = data_image.detect_positions[0]
				self.TagPosImg_data[0].append(self.time)

				self.TagPosImg_data[1].append(640 - self.Position_old_image.x)
				self.TagPosImg_data[2].append(360 - self.Position_old_image.y)

			else:
				Position_now_image = data_image.detect_positions[0]
				self.Position_predicter_image(Position_now_image)
				self.pixel_error()

				# gimbal_controller
				#self.pitch_pid_controller()
				#self.yaw_pid_controller()
				self.TagPosImg_data[0].append(self.time)
				self.TagPosImg_data[1].append(640 - Position_now_image.x)
				self.TagPosImg_data[2].append(360 - Position_now_image.y)

				
				self.Position_old_image = Position_now_image

		else:
			self.pitch_input_pwm = 7.422
			#self.pitch.start(self.pitch_input_pwm)
			self.pitch_error = [0.00, 0.00, 0.00]

			self.yaw_input_pwm = 60.156#7.422#60.156
			#self.yaw.start(self.yaw_input_pwm)
			#time.sleep(0.0001)
			self.yaw_error = [0.00, 0.00, 0.00]


			self.Position_old_image = [0, 0, 0]
			self.Position_predicted_image = [640, 360]
			self.flag_image = 0








	def pitch_pid_controller(self):
		P = 0.07
		I = 0.001#3
		#I = 0.0009
		D = 0.00025#25

		P = P*(self.pitch_error[0]-self.pitch_error[1])
		I = I*self.pitch_error[0]
		D = D*((self.pitch_error[0]-self.pitch_error[1])-(self.pitch_error[1]-self.pitch_error[2]))

		self.pitch_input_pwm =  self.pitch_input_pwm + P + I + D
					
		if self.pitch_input_pwm >= 10.742:
			self.pitch_input_pwm = 10.742
			self.pitch.start(self.pitch_input_pwm)
			time.sleep(0.0000001)


		elif self.pitch_input_pwm <= 4.5:
			self.pitch_input_pwm = 4.5
			self.pitch.start(self.pitch_input_pwm)
			time.sleep(0.0000001)

		else:
			self.pitch.start(self.pitch_input_pwm)
			time.sleep(0.0000001)

		self.pitch_error[2] = self.pitch_error[1]
		self.pitch_error[1] = self.pitch_error[0]







	def yaw_pid_controller(self,event=None):
		#P = 0.00582
		#I = 0.0005
		#D = 0.00065
		P = self.yaw_P
		I = self.yaw_I
		D = self.yaw_D

		
		P = P*(self.yaw_error[0]-self.yaw_error[1])
		I = I*self.yaw_error[0]
		D = D*((self.yaw_error[0]-self.yaw_error[1])-(self.yaw_error[1]-self.yaw_error[2]))

		self.yaw_input_pwm = self.yaw_input_pwm + P + I + D
		
		if self.yaw_input_pwm >= 86.523:#10.742#21.289#43.164#86.523
			self.yaw_input_pwm = 86.523
			self.yaw.ChangeDutyCycle(self.yaw_input_pwm)

			
		elif self.yaw_input_pwm <= 4.5:#4.5#8.79#17.774#35.743
			self.yaw_input_pwm = 35.743
			self.yaw.ChangeDutyCycle(self.yaw_input_pwm)

		else:
			self.yaw.ChangeDutyCycle(self.yaw_input_pwm)
		"""
		self.yaw_input_pwm = 15.04
		self.yaw.ChangeDutyCycle(self.yaw_input_pwm)
		#rospy.sleep(1.0/50)
		"""
		
		self.yaw_error[2] = self.yaw_error[1]
		self.yaw_error[1] = self.yaw_error[0]




	def Position_predicter_camera(self,Position_now_camera):
		self.delta_Position_camera[0] = Position_now_camera.x - self.Position_old_camera.x
		self.delta_Position_camera[1] = Position_now_camera.y - self.Position_old_camera.y
		self.delta_Position_camera[2] = Position_now_camera.z - self.Position_old_camera.z

		self.Position_predicted_camera[0] = Position_now_camera.x + self.delta_Position_camera[0]
		self.Position_predicted_camera[1] = Position_now_camera.y + self.delta_Position_camera[1]
		self.Position_predicted_camera[2] = Position_now_camera.z + self.delta_Position_camera[2]






	def Position_predicter_image(self,Position_now_image):
		self.delta_Position_image[0] = Position_now_image.x - self.Position_old_image.x
		self.delta_Position_image[1] = Position_now_image.y - self.Position_old_image.y

		self.Position_predicted_image[0] = Position_now_image.x + self.delta_Position_image[0]
		self.Position_predicted_image[1] = Position_now_image.y + self.delta_Position_image[1]




	def pixel_error(self):
		error_pitch = -(360 - self.Position_predicted_image[1])
		error_yaw = 640 - self.Position_predicted_image[0]
		safe_pix = 0

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
		self.yaw.stop()
		GPIO.cleanup()


	def timer(self,event=None):
		#TIME
		if self.time_start == 0:
			self.time_start = rospy.get_time()
		else:
			#self.Hz = 1 / (self.time - (rospy.get_time()-self.time_start))
			#print(self.Hz)
			self.time = rospy.get_time()-self.time_start
		msg = Float64()
		msg.data = self.time
		self.timer_pub.publish(msg)

		



if __name__ == "__main__":
	ts = tracking_apriltag()
	rospy.Timer(rospy.Duration(1.0/500), ts.yaw_pid_controller)
	#rospy.Timer(rospy.Duration(1.0/10000), ts.timer)

	rospy.spin()

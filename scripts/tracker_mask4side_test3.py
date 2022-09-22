#!/usr/bin/env python
# -*- coding: utf-8 -*-

#pwm: 7.25 +- 2.5

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
import RPi.GPIO as GPIO
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
		self.Position_predicted_camera = [0, 0, 0]
		self.delta_Position_camera = [0, 0, 0]
		self.flag_camera = 0

		#Tag_image
		self.Position_old_image = [0, 0]
		self.Position_predicted_image = [0, 0]
		self.delta_Position_image = [0, 0]
		self.flag_image = 0		

		#gimbal_init
		pitch_pin = 32
		yaw_pin = 33
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.pitch = GPIO.PWM(pitch_pin, 50)
		GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.yaw = GPIO.PWM(yaw_pin, 50)


		#pwm_input_value, [0]=t, [1]=t-1, center_value=7.422
		self.pitch_input_pwm = 7.422
		self.yaw_input_pwm = 7.422

		#error_value_deg, [0]=t, [1]=t-1, [2]=t-2
		self.pitch_error = [0.00, 0.00, 0.00]
		self.yaw_error = [0.00, 0.00, 0.00]

		#mask_tag
		self.mask_size = 1280
		self.xyz_0 = np.float64([0.0, 0.0, 0.0])
		self.xyz_1 = np.float64([0.0, 0.0, 0.0])
		self.uv_0 = np.float64([640, 360])
		self.uv_1 = np.float64([640, 360])
		
		#camera
		with open('/home/wanglab/catkin_ws/src/gimbal/config/c270_gimbal.yaml', 'r') as yml:
    			cam_para = yaml.load(yml)
		self.rvec = np.float64([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
		self.tvec = np.float64([[0.0], [0.0], [0.0]])
		self.mtx = np.float64(cam_para['camera_matrix']['data']).reshape([3,3])
		self.dist = np.float64(cam_para['distortion_coefficients']['data'])
		self.f_y = self.mtx[1, 1]
		self.color = (0,0,0)

		#fps
		self.d = deque()
		self.delta_t = 0

		#data
		self.data = [["time_uv"],["data_u"],["data_v"],["rate_detection"],["area"],["delta_Pu"],["delta_Pv"]]
		self.i = 0
		self.time = 0
		self.time_start = 0
		
	



	



	def image_callback(self, ros_image,camera_info):
		
		#self.time = time.time()
		input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		output_image = self.image_process(input_image)
		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)








	def image_process(self, input_image):
		
		mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.Wide_Mask()
		#mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.Wide_Tag(mask0_u0,mask0_u1,mask0_v0,mask0_v1)
		#mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.Wide_Gimbal(mask0_u0,mask0_u1,mask0_v0,mask0_v1)
		
		mask0_u0 = int(mask0_u0)
		mask0_v0 = int(mask0_v0)
		mask0_u1 = int(mask0_u1)
		mask0_v1 = int(mask0_v1)
		
		mask_image = cv2.rectangle(input_image,(0,0),(1280,mask0_v0),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(0,mask0_v1),(1280,720),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(0,0),(mask0_u0,720),color=0, thickness=-1)
		mask_image = cv2.rectangle(input_image,(mask0_u1,0),(1280,720),color=0, thickness=-1)
		
		#mask_image = input_image

		output_image = self.bridge.cv2_to_imgmsg(np.array(mask_image), "bgr8")
		
		return output_image






	
	def Wide_Mask(self):
		center_u = self.Position_predicted_image[0]
		center_v = self.Position_predicted_image[1]
		Length_Tag = self.mask_size / 2
		alpha = 1
		z = self.Position_predicted_camera[2]
		
		mask0_u0 = center_u - Length_Tag*alpha 
		mask0_u1 = center_u + Length_Tag*alpha
		mask0_v0 = center_v - Length_Tag*alpha
		mask0_v1 = center_v + Length_Tag*alpha
		
		mask0_u0 = mask0_u0 - z*10
		mask0_u1 = mask0_u1 + z*10
		mask0_v0 = mask0_v0 - z*10
		mask0_v1 = mask0_v1 + z*10
		
		
		return mask0_u0,mask0_u1,mask0_v0,mask0_v1





	def Wide_Tag(self,mask0_u0,mask0_u1,mask0_v0,mask0_v1):
		alpha = 10000
		#alpha = 10000
		delta_Position_Tag = self.delta_Position_image
		#print(self.delta_Position_camera)
		#x
		if self.delta_Position_camera[0] >= 0:
			mask0_u1 = mask0_u1 + self.delta_Position_camera[0]*alpha
		else:
			mask0_u0 = mask0_u0 + self.delta_Position_camera[0]*alpha

		#y
		if self.delta_Position_camera[1] >= 0:
			mask0_v1 = mask0_v1 + self.delta_Position_camera[1]*alpha
		else:
			mask0_v0 = mask0_v0 + self.delta_Position_camera[1]*alpha
		
		return mask0_u0,mask0_u1,mask0_v0,mask0_v1





	def Wide_Gimbal(self,mask0_u0,mask0_u1,mask0_v0,mask0_v1):
		yaw = self.yaw_input_pwm
		pitch = self.pitch_input_pwm
		pix = 10
		#pix = 500
		yaw_rate = 0

		#yaw
		if yaw <= 7.226:
			yaw_rate = (7.226 - yaw) / 2.726
			mask0_u0 = mask0_u0 - yaw_rate*pix
			

		elif 7.618 <= yaw:
			yaw_rate = (yaw - 7.618) / 3.124
			mask0_u1 = mask0_u1 + yaw_rate*pix
		else:
			pass
		
		
		#pitch
		
		if pitch <= 7.226:
			pitch_rate = (7.226 - pitch) / 2.726
			mask0_v0 = mask0_v0 - pitch_rate*pix

		elif 7.618 <= pitch:
			pitch_rate = (pitch - 7.618) / 3.124
			mask0_v1 = mask0_v1 + pitch_rate*pix
		else:
			pass

		return mask0_u0,mask0_u1,mask0_v0,mask0_v1
	





	def tag_camera_callback(self,data_camera):

		if len(data_camera.detections) >= 1:

			self.data[3].append(1/)

			if self.flag_camera == 0:
				#fps
				self.d.append(time.time())
				#tag
				self.Position_old_camera = data_camera.detections[0].pose.pose.pose.position
				self.flag_camera = 1

			else:
				#fps
				self.d.append(time.time())
				self.delta_t = self.d[-1] - self.d[0]
				self.d.popleft()
				#tag
				Position_now_camera = data_camera.detections[0].pose.pose.pose.position
				self.Position_predicter_camera(Position_now_camera)

				self.Position_old_camera = Position_now_camera

		else:
			#self.uv_0 = np.float64([640, 360])
			#self.uv_1 = np.float64([640, 360])
			self.mask_size = 1280
			

			self.Position_old_camera = [0, 0, 0]
			self.flag_camera = 0
			self.data[3].append(0)






	def tag_image_callback(self, data_image):

		if len(data_image.detect_positions) >= 1:
			
			if self.flag_image == 0:
				#tag
				self.Position_old_image = data_image.detect_positions[0]
				self.flag_image = 1

			else:
				#tag
				Position_now_image = data_image.detect_positions[0]

				self.data[0].append(self.time)
				self.data[1].append(640 - Position_now_image.x)
				self.data[2].append(360 - Position_now_image.y)


				self.Position_predicter_image(Position_now_image)
				self.pixel_error()

				#gimbal_controller
				self.pitch_pid_controller()
				self.yaw_pid_controller()
				
				#TIME
				if self.time_start == 0:
					self.time_start = time.time()
				else:
					self.time = time.time()-self.time_start
					
					
				
				if int(self.time) == 2:
					self.get_data()
				else:
					pass

				
				self.Position_old_image = Position_now_image

		else:
			
			#self.uv_0 = np.float64([640, 360])
			#self.uv_1 = np.float64([640, 360])
			self.mask_size = 1280
			
			self.pitch_input_pwm = 7.422
			self.pitch.start(self.pitch_input_pwm)
			self.pitch_error = [0.00, 0.00, 0.00]

			self.yaw_input_pwm = 7.422
			self.yaw.start(self.yaw_input_pwm)
			self.yaw_error = [0.00, 0.00, 0.00]

			self.Position_old_image = [0, 0, 0]
			self.Position_predicted_image = [640, 360]
			self.flag_image = 0








	def pitch_pid_controller(self):
		P = 0.006
		I = 0.00#3
		#I = 0.0009
		D = 0.00#25

		P = P*(self.pitch_error[0]-self.pitch_error[1])
		I = I*self.pitch_error[0]
		D = D*((self.pitch_error[0]-self.pitch_error[1])-(self.pitch_error[1]-self.pitch_error[2]))

		self.pitch_input_pwm =  self.pitch_input_pwm + P + I + D
					
		if self.pitch_input_pwm >= 10.742:
			self.pitch_input_pwm = 10.742
			self.pitch.start(self.pitch_input_pwm)
			time.sleep(0.0001)

		elif self.pitch_input_pwm <= 4.5:
			self.pitch_input_pwm = 4.5
			self.pitch.start(self.pitch_input_pwm)
			time.sleep(0.0001)

		else:
			self.pitch.start(self.pitch_input_pwm)
			time.sleep(0.0001)

		
		self.pitch_error[2] = self.pitch_error[1]
		self.pitch_error[1] = self.pitch_error[0]







	def yaw_pid_controller(self):
		P = 0.00582
		I = 0.0005
		D = 0.00065

		P = P*(self.yaw_error[0]-self.yaw_error[1])
		I = I*self.yaw_error[0]
		D = D*((self.yaw_error[0]-self.yaw_error[1])-(self.yaw_error[1]-self.yaw_error[2]))

		self.yaw_input_pwm = self.yaw_input_pwm + P + I + D
		
		if self.yaw_input_pwm >= 10.742:
			self.yaw_input_pwm = 10.742
			self.yaw.start(self.yaw_input_pwm)
			time.sleep(0.0000001)
			
		elif self.yaw_input_pwm <= 4.5:
			self.yaw_input_pwm = 4.5
			self.yaw.start(self.yaw_input_pwm)
			time.sleep(0.0000001)

		else:
			self.yaw.start(self.yaw_input_pwm)
			time.sleep(0.0000001)
		#print(self.yaw_error)
		#print(self.yaw_input_pwm)
		self.yaw_error[2] = self.yaw_error[1]
		self.yaw_error[1] = self.yaw_error[0]






	def Position_predicter_camera(self,Position_now_camera):
		self.delta_Position_camera[0] = Position_now_camera.x - self.Position_old_camera.x
		self.delta_Position_camera[1] = Position_now_camera.y - self.Position_old_camera.y
		self.delta_Position_camera[2] = Position_now_camera.z - self.Position_old_camera.z

		self.Position_predicted_camera[0] = Position_now_camera.x + self.delta_Position_camera[0]
		self.Position_predicted_camera[1] = Position_now_camera.y + self.delta_Position_camera[1]
		self.Position_predicted_camera[2] = Position_now_camera.z + self.delta_Position_camera[2]

		self.mask_size = 120*(1/self.Position_predicted_camera[2])







	def Position_predicter_image(self,Position_now_image):
		self.delta_Position_image[0] = Position_now_image.x - self.Position_old_image.x
		self.delta_Position_image[1] = Position_now_image.y - self.Position_old_image.y

		self.Position_predicted_image[0] = Position_now_image.x + self.delta_Position_image[0]
		self.Position_predicted_image[1] = Position_now_image.y + self.delta_Position_image[1]
		
		self.data[5].append(self.delta_Position_image[0])
		self.data[6].append(self.delta_Position_image[1])






	def pixel_error(self):
		self.pitch_error[0] = (360 - self.Position_predicted_image[1])
		self.yaw_error[0] = (640 - self.Position_predicted_image[0])
		
		





	"""
	def z_change(self):
		self.uv_0, jac = cv2.projectPoints(self.xyz_0, self.rvec, self.tvec, self.mtx, self.dist)
		self.uv_0 = self.uv_0[0][0]

		self.uv_1, jac = cv2.projectPoints(self.xyz_1, self.rvec, self.tvec, self.mtx, self.dist)
		self.uv_1 = self.uv_1[0][0]




	
	def gimbal_callback(self,Quaternion):
		self.gimbal_euler = self.quaternion_to_euler(Quaternion.quaternion.x, Quaternion.quaternion.y, Quaternion.quaternion.z, Quaternion.quaternion.w)






	def quaternion_to_euler(self,q_x,q_y,q_z,q_w):
		euler = tf.transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))
		euler = [euler[0], euler[1], euler[2]]
		return euler




	"""
	
	def get_data(self):
			f = open('/home/wanglab/catkin_ws/src/gimbal/data/2022.09.20_data/data0.txt', 'w')
			f.write(str(self.data))
			f.close()
			print("finish!!!!")





		



	def cleanup(self):
		cv2.destroyAllWindows()



if __name__ == "__main__":
	tracking_apriltag()
	rospy.spin()

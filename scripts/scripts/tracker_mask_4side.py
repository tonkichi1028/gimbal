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
from geometry_msgs.msg import QuaternionStamped, Quaternion

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
		self.tag_det = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_det_callback)
		self.tag_pos = rospy.Subscriber('/tag_position',AprilTagDetectionPositionArray,self.tag_pos_callback)
		self.gimbal_msg = rospy.Subscriber('/storm32_node/camera_orientation',QuaternionStamped,self.gimbal_callback)
		sub1 = message_filters.Subscriber('/usb_cam/image_raw', Image)
		sub2 = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		#Tag
		self.P_Tnow = [0, 0, 0]
		self.P_Told = [0, 0, 0]
		self.P_Tnew = [0, 0, 0]
		self.delta_P_Tnow = [0, 0, 0]
		
		#gimbal_init
		pitch_pin = 32
		yaw_pin = 33
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.pitch = GPIO.PWM(pitch_pin, 50)
		GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.yaw = GPIO.PWM(yaw_pin, 50)

		#PID_parameter
		#self.pitch_PID_parameter = [0.1, 0.0005, 0.0009]
		#self.yaw_PID_parameter = [0.03, 0.0005, 0.0001]
		self.pitch_PID_parameter = [0.1, 0.0005, 0.0009]
		self.yaw_PID_parameter = [0.06, 0.0005, 0.0001]

		#pwm_input_value, [0]=t, [1]=t-1, center_value=7.422
		self.pitch_input_pwm = [7.422, 7.422]
		self.yaw_input_pwm = [7.422, 7.422]

		#error_value_deg, [0]=t, [1]=t-1, [2]=t-2
		self.pitch_error = [0.00, 0.00, 0.00]
		self.yaw_error = [0.00, 0.00, 0.00]

		#PID_calculate
		self.pitch_P = 0.00
		self.pitch_I = 0.00
		self.pitch_D = 0.00
		self.yaw_P = 0.00
		self.yaw_I = 0.00
		self.yaw_D = 0.00

		#mask_tag
		self.mask_tag = [0, 0, 0]
		self.mask_size = 1280
		self.mask_r = 800
		self.safe_rate_p = 1.2
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

		#mask
		self.mask = 0

		#data
		self.data = [["time_uv"],["data_u"],["data_v"],["time_maskuv"],["data_masku"],["data_maskv"]]
		self.tag_pos_data_x = 0
		self.tag_pos_data_y = 0
		self.i = 0
		self.time = 0
	



	



	def image_callback(self, ros_image,camera_info):
		
		self.time = time.time()
		input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		output_image = self.image_process(input_image)
		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)








	def image_process(self, input_image):
		
		mask = np.zeros((720, 1280), dtype=np.uint8)

		mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.mask_margin()
		#mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.tag_margin(mask0_u0,mask0_u1,mask0_v0,mask0_v1)
		mask0_u0,mask0_u1,mask0_v0,mask0_v1 = self.gimbal_margin(mask0_u0,mask0_u1,mask0_v0,mask0_v1)
		

		mask0_u0 = int(mask0_u0)
		mask0_v0 = int(mask0_v0)
		mask0_u1 = int(mask0_u1)
		mask0_v1 = int(mask0_v1)

		mask = cv2.rectangle(mask, (mask0_u0, mask0_v0), (mask0_u1, mask0_v1), color=255, thickness=-1)

		input_image[mask==0] = [0, 0, 0]
		output_image = self.bridge.cv2_to_imgmsg(np.array(input_image), "bgr8")
		
		return output_image







	def mask_margin(self):
		mask_size = self.mask_size /2
		
		mask0_u0 = self.uv_0[0] - mask_size
		mask0_u1 = self.uv_0[0] + mask_size
		mask0_v0 = self.uv_0[1] - mask_size
		mask0_v1 = self.uv_0[1] + mask_size
		
		return mask0_u0,mask0_u1,mask0_v0,mask0_v1





	def tag_margin(self,mask0_u0,mask0_u1,mask0_v0,mask0_v1):
		pix = 20000
		#x
		if self.delta_P_Tnow[0] >= 0:
			mask0_u1 = mask0_u1 + self.delta_P_Tnow[0]*pix
		else:
			mask0_u0 = mask0_u0 + self.delta_P_Tnow[0]*pix

		#y
		if self.delta_P_Tnow[1] >= 0:
			mask0_v1 = mask0_v1 + self.delta_P_Tnow[1]*pix
		else:
			mask0_v0 = mask0_v0 + self.delta_P_Tnow[1]*pix
		
		return mask0_u0,mask0_u1,mask0_v0,mask0_v1





	def gimbal_margin(self,mask0_u0,mask0_u1,mask0_v0,mask0_v1):
		yaw = self.yaw_input_pwm[0]
		pitch = self.pitch_input_pwm[0]
		pix = 1000
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
			mask0_v1 = mask0_v1 + pitch_rate*pix

		elif 7.618 <= pitch:
			pitch_rate = (pitch - 7.618) / 3.124
			mask0_v0 = mask0_v0 - pitch_rate*pix
		else:
			pass

		return mask0_u0,mask0_u1,mask0_v0,mask0_v1






	def tag_det_callback(self,data):

		if len(data.detections) >= 1:
			
			if type(self.P_Told) is list:
				#fps
				self.d.append(time.time())
				#tag
				self.P_Tnow = data.detections[0].pose.pose.pose.position
				self.P_Told = data.detections[0].pose.pose.pose.position


			else:
				#fps
				self.d.append(time.time())
				self.delta_t = self.d[-1] - self.d[0]
				self.d.popleft()
				#tag
				self.P_Tnow = data.detections[0].pose.pose.pose.position
				self.Tag_position_predicter()
				self.angle_error()

				#gimbal_controller
				self.pitch_pid_controller()
				self.yaw_pid_controller()
				
				#camera_to_image
				self.xyz_0 = np.float64(self.P_Tnew)
				self.z_change()

				self.P_Told = self.P_Tnow

		else:
			self.tag_p_old = [0, 0, 0]
			self.uv_0 = np.float64([640, 360])
			self.uv_1 = np.float64([640, 360])
			self.mask_r = 700
			self.mask_size = 1280

			self.pitch_input_pwm[0] = 7.422
			self.pitch.start(self.pitch_input_pwm[0])
			self.yaw_input_pwm[0] = 7.422
			self.yaw.start(self.yaw_input_pwm[0])

			self.P_Told = [0, 0, 0]







	def tag_pos_callback(self, data_pos):
		if len(data_pos.detect_positions) >= 1:
			self.i += 1
			
			#self.data[0].append(time.time())
			#self.data[1].append(data_pos.detect_positions[0].x)
			#self.data[2].append(data_pos.detect_positions[0].y)
		else:
			pass






	def gimbal_callback(self,Quaternion):
		self.gimbal_euler = self.quaternion_to_euler(Quaternion.quaternion.x, Quaternion.quaternion.y, Quaternion.quaternion.z, Quaternion.quaternion.w)







	def pitch_pid_controller(self):
		self.pitch_P = self.pitch_PID_parameter[0]*(self.pitch_error[0]-self.pitch_error[1])
		self.pitch_I = self.pitch_PID_parameter[1]*self.pitch_error[0]
		self.pitch_D = self.pitch_PID_parameter[2]*((self.pitch_error[0]-self.pitch_error[1])-(self.pitch_error[1]-self.pitch_error[2]))

		self.pitch_input_pwm[0] =  self.pitch_input_pwm[1] + self.pitch_P + self.pitch_I + self.pitch_D
					
		if self.pitch_input_pwm[0] >= 10.742:
			self.pitch_input_pwm[0] = 10.742
			self.pitch.start(self.pitch_input_pwm[0])
			time.sleep(0.0000001)

		elif self.pitch_input_pwm[0] <= 4.5:
			self.pitch_input_pwm[0] = 4.5
			self.pitch.start(self.pitch_input_pwm[0])
			time.sleep(0.0000001)

		else:
			self.pitch.start(self.pitch_input_pwm[0])
			time.sleep(0.0000001)

		self.pitch_error[2] = self.pitch_error[1]
		self.pitch_error[1] = self.pitch_error[0]
		self.pitch_input_pwm[1] = self.pitch_input_pwm[0]







	def yaw_pid_controller(self):
		self.yaw_P = self.yaw_PID_parameter[0]*(self.yaw_error[0]-self.yaw_error[1])
		self.yaw_I = self.yaw_PID_parameter[1]*self.yaw_error[0]
		self.yaw_D = self.yaw_PID_parameter[2]*((self.yaw_error[0]-self.yaw_error[1])-(self.yaw_error[1]-self.yaw_error[2]))

		self.yaw_input_pwm[0] = self.yaw_input_pwm[1] + self.yaw_P + self.yaw_I + self.yaw_D
					
		if self.yaw_input_pwm[0] >= 10.742:
			self.yaw_input_pwm[0] = 10.742
			self.yaw.start(self.yaw_input_pwm[0])
			time.sleep(0.0000001)
			
		elif self.yaw_input_pwm[0] <= 4.5:
			self.yaw_input_pwm[0] = 4.5
			self.yaw.start(self.yaw_input_pwm[0])
			time.sleep(0.0000001)

		else:
			self.yaw.start(self.yaw_input_pwm[0])
			time.sleep(0.0000001)

		self.yaw_error[2] = self.yaw_error[1]
		self.yaw_error[1] = self.yaw_error[0]
		self.yaw_input_pwm[1] = self.yaw_input_pwm[0]






	def Tag_position_predicter(self):
		self.delta_P_Tnow[0] = self.P_Tnow.x - self.P_Told.x
		self.delta_P_Tnow[1] = self.P_Tnow.y - self.P_Told.y
		self.delta_P_Tnow[2] = self.P_Tnow.z - self.P_Told.z

		self.P_Tnew[0] = self.P_Tnow.x + self.delta_P_Tnow[0]
		self.P_Tnew[1] = self.P_Tnow.y + self.delta_P_Tnow[1]
		self.P_Tnew[2] = self.P_Tnow.z + self.delta_P_Tnow[2]


		#self.mask_size = 120*(1/self.P_Tnew[2])
		self.mask_size = 120*(1/self.P_Tnew[2])





	def angle_error(self):
		self.pitch_error[0] = (90 - int(math.degrees(math.atan2(self.P_Tnew[2], self.P_Tnew[1]))))
		self.yaw_error[0] = -(90 - int(math.degrees(math.atan2(self.P_Tnew[2], self.P_Tnew[0]))))
		






	def z_change(self):
		self.uv_0, jac = cv2.projectPoints(self.xyz_0, self.rvec, self.tvec, self.mtx, self.dist)
		self.uv_0 = self.uv_0[0][0]

		self.uv_1, jac = cv2.projectPoints(self.xyz_1, self.rvec, self.tvec, self.mtx, self.dist)
		self.uv_1 = self.uv_1[0][0]





	def quaternion_to_euler(self,q_x,q_y,q_z,q_w):
		euler = tf.transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))
		euler = [euler[0], euler[1], euler[2]]
		return euler






	def get_data(self):
			f = open('/home/wanglab/catkin_ws/src/gimbal/data/2022.08.08_data/tagpos_maskpos.txt', 'w')
			f.write(str(self.data))
			f.close()
			print("finish!!!!")





		



	def cleanup(self):
		cv2.destroyAllWindows()



if __name__ == "__main__":
	tracking_apriltag()
	rospy.spin()

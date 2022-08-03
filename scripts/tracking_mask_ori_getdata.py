#!/usr/bin/env python

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

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetectionPositionArray
from geometry_msgs.msg import Vector3
from collections import deque
pitch_pin = 32
yaw_pin = 33
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
pitch = GPIO.PWM(pitch_pin, 50)
GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
yaw = GPIO.PWM(yaw_pin, 50)


class tag_mask:
	def __init__(self):
		self.node_name = "masking_image"
		rospy.init_node(self.node_name)
		rospy.on_shutdown(self.cleanup)

		#tag
		self.tag_p_old = [0, 0, 0]
		self.tag_p_now = [0, 0, 0]
		self.tag_d = [0, 0, 0]
		self.tag_q = [0, 0, 0, 0]
		self.tag_euler = [0, 0, 0]
		self.P_T = np.array([0.00, 0.00, 0.00])

		#mask_tag
		self.mask_tag = [0, 0, 0]
		self.mask_r = 1000
		self.safe_rate_p = 1.5
		self.xyz_0 = np.float64([0.0, 0.0, 0.0])
		self.uv_0 = np.float64([640, 360])

		#camera
		with open('/home/wanglab/catkin_ws/src/gimbal/config/c270_gimbal.yaml', 'r') as yml:
    			cam_para = yaml.load(yml)
		self.rvec = np.float64([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
		self.tvec = np.float64([[0.0], [0.0], [0.0]])
		self.mtx = np.float64(cam_para['camera_matrix']['data']).reshape([3,3])
		self.dist = np.float64(cam_para['distortion_coefficients']['data'])
		self.f_y = self.mtx[1, 1]
		self.color = (0,0,0)

		#ros
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/masking_image", Image, queue_size=1)
		self.info_pub = rospy.Publisher("/masking_info", CameraInfo, queue_size=1)
		sub1 = message_filters.Subscriber('/usb_cam/image_raw', Image)
		sub2 = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
		self.tag_det = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_det_callback)
		self.tag_pos = rospy.Subscriber('/tag_position',AprilTagDetectionPositionArray,self.tag_pos_callback)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		#gimbal
		pitch_pin = 32
		yaw_pin = 33
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.pitch = GPIO.PWM(pitch_pin, 50)
		GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.yaw = GPIO.PWM(yaw_pin, 50)

		self.M_py = [7.422, 7.422]
		self.M_py1 =  [7.422, 7.422]
		self.e_py = [0.00, 0.00]
		self.e_py1 = [0.00, 0.00]
		self.e_py2 = [0.00, 0.00]
		self.Kp_py = [0.115, 0.170]#0.125
		self.Ki_py = [0.000, 0.0005]#0.155
		self.Kd_py = [0, 0.0001]#0.9
		self.goal_py = [0.000, 0.000]
		#gmbl
		self.w = [0,0]
		self.deg_d = [0,0]
		self.R = 0
		self.gmbl_d = [0, 0, 0]
		#fps
		self.d = deque()
		self.delta_t = 0
		#mask
		self.mask = 0

		#data
		self.i = 0
		self.data = [["time_uv"],["data_u"],["data_v"],["time_maskuv"],["data_masku"],["data_maskv"]]
		self.data_uv = 0
		self.data_maskuv = 0

	def image_callback(self, ros_image,camera_info):
		input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		output_image = self.image_tf(input_image)
		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)

		
	def tag_det_callback(self,data):
		
		if len(data.detections) >= 1:
			
			if type(self.tag_p_old) is list:
				#tag
				self.tag_p_now = data.detections[0].pose.pose.pose.position
				self.tag_p_old = data.detections[0].pose.pose.pose.position
				#fps
				self.d.append(time.time())
				
			else:
				#fps
				self.d.append(time.time())
				self.delta_t = self.d[-1] - self.d[0]
				self.d.popleft()

				#tag
				self.tag_p_now = data.detections[0].pose.pose.pose.position
				self.tag_q = data.detections[0].pose.pose.pose.orientation
				self.tag_euler = self.quaternion_to_euler(self.tag_q.x, self.tag_q.y, self.tag_q.z, self.tag_q.w)

				
				#tag_d
				self.tag_d[0] = self.tag_p_now.x - self.tag_p_old.x
				self.tag_d[1] = self.tag_p_now.y - self.tag_p_old.y
				self.tag_d[2] = self.tag_p_now.z - self.tag_p_old.z
				
				#tag_P_T
				self.P_T[0] = self.tag_p_now.x	+ self.tag_d[0]
				self.P_T[1] = self.tag_p_now.y	+ self.tag_d[1]
				self.P_T[2] = self.tag_p_now.z	+ self.tag_d[2]

				
				#self.mask_r = (data.detections[0].size * self.f_y / self.mask_tag[2]) * self.safe_rate_p
				self.mask_r = 200

				#error_deg
				self.e_py[0] = -(90 - math.degrees(math.atan2(self.P_T[2], self.P_T[1])))
				self.e_py[1] = -(90 - math.degrees(math.atan2(self.P_T[2], self.P_T[0])))
				
				#gimbal_pitch_pwm
				self.M_py[0] = self.M_py1[0] + self.Kp_py[0]*(self.e_py[0]-self.e_py1[0]) + self.Ki_py[0]*self.e_py[0] + self.Kd_py[0]*((self.e_py[0]-self.e_py1[0])-(self.e_py1[0]-self.e_py2[0]))
				
				if self.M_py[0] >= 10.742:
					self.M_py[0] = 10.742
					self.pitch.start(self.M_py[0])
					time.sleep(0.0000001)

				elif self.M_py[0] <= 4.5:
					self.M_py[0] = 4.5
					self.pitch.start(self.M_py[0])
					time.sleep(0.0000001)
				else:
					self.pitch.start(self.M_py[0])
					time.sleep(0.0000001)


				#gimbal_yaw_pwm
				self.M_py[1] = self.M_py1[1] + self.Kp_py[1]*(self.e_py[1] - self.e_py1[1]) + self.Ki_py[1]*self.e_py[1] + self.Kd_py[1]*((self.e_py[1] - self.e_py1[1]) - (self.e_py1[1] - self.e_py2[1]))

				if self.M_py[1] >= 10.742:
					self.M_py[1] = 10.742
					self.yaw.start(self.M_py[1])
					time.sleep(0.0000001)

				elif self.M_py[1] <= 4.5:
					self.M_py[1] = 4.5
					self.yaw.start(self.M_py[1])
					time.sleep(0.0000001)
				else:
					self.yaw.start(self.M_py[1])
					time.sleep(0.0000001)
				

				#mask_cam_to_img
				self.xyz_0 = self.P_T
				self.z_change()

				#init
				self.M_py1[0] = self.M_py[0]
				self.e_py1[0] = self.e_py[0]
				self.e_py2[0] = self.e_py1[0]
				self.M_py1[1] = self.M_py[1]
				self.e_py1[1] = self.e_py[1]
				self.e_py2[1] = self.e_py1[1]

				self.tag_p_old = self.tag_p_now


		else:
			#init
			self.tag_p_old = [0, 0, 0]
			self.uv_0 = np.float64([640, 360])
			self.mask_r = 1000
			#gimbal
			self.M_py[0] = 7.422
			self.pitch.start(self.M_py[0])
			self.M_py[1] = 7.422
			self.yaw.start(self.M_py[1])
			#fps
			self.d.clear()


	def quaternion_to_euler(self,q_x,q_y,q_z,q_w):
		euler = tf.transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))
		euler = [euler[0], euler[1], euler[2]]
		return euler

		
	def z_change(self):
		
		self.uv_0, jac = cv2.projectPoints(self.xyz_0, self.rvec, self.tvec, self.mtx, self.dist)
		self.uv_0 = self.uv_0[0][0]

		
	def tag_pos_callback(self, data_pos):
		
		if len(data_pos.detect_positions) >= 1:
			self.i += 1
			self.data[0].append(time.time())
			self.data[1].append(data_pos.detect_positions[0].x)
			self.data[2].append(data_pos.detect_positions[0].y)	
		else:
			pass
		
	def image_tf(self, image_opcv):
		mask = np.zeros((720, 1280), dtype=np.uint8)
		mask_p_u = int(self.uv_0[0])
		mask_p_v = int(self.uv_0[1])
		mask_r = int(self.mask_r)

		self.data[3].append(time.time())
		self.data[4].append(mask_p_u)
		self.data[5].append(mask_p_v)
		

		if self.i == 100:
			f = open('/home/wanglab/catkin_ws/src/gimbal/data/2022.07.22_data/tagpos_maskpos.txt', 'w')
			f.write(str(self.data))
			print("finish!!!!!!")
			f.close()

		mask = cv2.circle(mask, center=(640, 360), radius=1000, color=255, thickness=-1)
		image_opcv[mask==0] = [0, 0, 0]
		result = self.bridge.cv2_to_imgmsg(np.array(image_opcv), "bgr8")
		
		return result

	def cleanup(self):
		cv2.destroyAllWindows()

if __name__ == '__main__':
	tag_mask()
	rospy.spin()

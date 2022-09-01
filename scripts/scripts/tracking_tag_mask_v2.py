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
		self.i = 0
		#fps
		self.d = deque()
		self.delta_t = 0
		#mask
		self.mask = 0

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
			
			if self.i == 0:
				#tag
				self.tag_p_now = data.detections[0].pose.pose.pose.position
				self.tag_p_old = data.detections[0].pose.pose.pose.position
				#fps
				self.d.append(time.time())

				self.i = 1
				
			else:
				
				#fps
				self.d.append(time.time())
				self.delta_t = self.d[-1] - self.d[0]
				print("self.delta_t")
				print(self.delta_t)
				self.d.popleft()

				#tag
				self.tag_p_now = data.detections[0].pose.pose.pose.position
				self.tag_q = data.detections[0].pose.pose.pose.orientation
				self.tag_euler = self.quaternion_to_euler(self.tag_q.x, self.tag_q.y, self.tag_q.z, self.tag_q.w)

				self.P_T[0] = self.tag_p_now.x	
				self.P_T[1] = self.tag_p_now.y
				self.P_T[2] = self.tag_p_now.z
				#tag_d
				self.tag_d[0] = self.tag_p_now.x - self.tag_p_old.x
				self.tag_d[1] = self.tag_p_now.y - self.tag_p_old.y
				self.tag_d[2] = self.tag_p_now.z - self.tag_p_old.z
				
				#self.mask_r = (data.detections[0].size * self.f_y / self.mask_tag[2]) * self.safe_rate_p
				self.mask_r = 200
				print("self.P_T")
				print(self.P_T)
				#error_deg
				self.e_py[0] = -(90 - math.degrees(math.atan2(self.P_T[2], self.P_T[1])))
				self.e_py[1] = -(90 - math.degrees(math.atan2(self.P_T[2], self.P_T[0])))
				print("self.e_py")
				print(self.e_py)
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

				#angular_velocity
				
				self.w = np.array(self.angular_velocity(*self.M_py))
				print("self.w")
				print(self.w)

				#gmbl_d
				self.deg_d = self.delta_t * self.w
				print("self.deg_d")
				print(self.deg_d)

				self.R = self.rotM(self.deg_d).T
				
				#mask_tag
				self.mask_tag =self.R.dot(self.P_T + self.tag_d)
				
				print("self.mask_tag")
				print(self.mask_tag)
				print("self.P_T - self.mask_tag")
				print(self.P_T - self.mask_tag)
				#mask_cam_to_img
				self.xyz_0 = self.mask_tag
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
			
			self.i = 0



	def quaternion_to_euler(self,q_x,q_y,q_z,q_w):
		euler = tf.transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))
		euler = [euler[0], euler[1], euler[2]]
		return euler

	def angular_velocity(self, u_pitch, u_yaw ):
		w = [0,0]
		#pitch_w
		if u_pitch < 7.227:
			w[0] = -1.4893*u_pitch**3 + 27.002*u_pitch**2 -155.5*u_pitch + 272.33
		elif  7.227 <= u_pitch < 7.617:
			w[0] = 0
		else:
			w[0] = -7.8396*u_pitch**4 + 174.19*u_pitch**3 - 1426*u_pitch**2 + 5128.4*u_pitch + 6966.1

		#yaw_w
		if u_yaw < 7.227:
			w[1] = -1.4893*u_yaw**3 + 27.002*u_yaw**2 -155.5*u_yaw + 272.33
		elif  7.227 <= u_yaw < 7.617:
			w[1] = 0
		else:
			w[1] = -7.8396*u_yaw**4 + 174.19*u_yaw**3 - 1426*u_yaw**2 + 5128.4*u_yaw + 6966.1


		return w		
		
	def rotM(self,deg_d):
	
		deg_p = deg_d[0]
		deg_y = deg_d[1]

		C_p = round(math.cos(math.radians(deg_p)), 10)
		S_p = round(math.sin(math.radians(deg_p)), 10)
		C_y = round(math.cos(math.radians(deg_y)), 10)
		S_y = round(math.sin(math.radians(deg_y)), 10)

		R = np.array([[C_y*C_p, -S_y, C_y*S_p],
				[S_y*C_p, C_y, S_y*S_p],
				[-S_p, 0, C_p]])
		return R
		
	def z_change(self):
		
		self.uv_0, jac = cv2.projectPoints(self.xyz_0, self.rvec, self.tvec, self.mtx, self.dist)
		self.uv_0 = self.uv_0[0][0]

		
		
		
		
	def image_tf(self, image_opcv):
		mask = np.zeros((720, 1280), dtype=np.uint8)
		mask_p_u = int(self.uv_0[0])
		mask_p_v = int(self.uv_0[1])
		mask_r = int(self.mask_r)
		print("self.uv_0")
		print(self.uv_0)
		mask = cv2.circle(mask, center=(mask_p_u,mask_p_v), radius=mask_r, color=255, thickness=-1)
		image_opcv[mask==0] = [0, 0, 0]
		result = self.bridge.cv2_to_imgmsg(np.array(image_opcv), "bgr8")
		
		return result

	def cleanup(self):
		cv2.destroyAllWindows()

if __name__ == '__main__':
	tag_mask()
	rospy.spin()

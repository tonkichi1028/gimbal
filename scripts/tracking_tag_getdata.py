#!/usr/bin/env python
# -*- coding: utf-8 -*-

#pwm: 7.25 +- 2.5

import time
import RPi.GPIO as GPIO
import sys
import tf2_ros
import rospy
import numpy as np
import math
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetectionPositionArray

pitch_pin = 32
yaw_pin = 33
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
pitch = GPIO.PWM(pitch_pin, 50)
GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
yaw = GPIO.PWM(yaw_pin, 50)



class tracking_apriltag(object):


	def __init__(self):
		pitch_pin = 32
		yaw_pin = 33
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.pitch = GPIO.PWM(pitch_pin, 50)
		GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.yaw = GPIO.PWM(yaw_pin, 50)


		rospy.init_node("tracking_apriltag")
		self.tag_det = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_det_callback)
		self.tag_pos = rospy.Subscriber('/tag_position' ,AprilTagDetectionPositionArray, self.tag_pos_callback)
		
		self.tag_p_now = [0, 0, 0]
		self.tag_p_old = [0, 0, 0]
		self.tag_d = [0, 0, 0]
		self.tag_p = [0, 0, 0]

		self.rate = 2.0

		self.M_py = [7.5, 7.5]
		self.M_py1 =  [7.5, 7.5]
		self.e_py = [0.00, 0.00]
		self.e_py1 = [0.00, 0.00]
		self.e_py2 = [0.00, 0.00]
		self.Kp_py = [0.170, 0.170]#0.125 0.095
		self.Ki_py = [0.0005, 0.0005]#0.155
		self.Kd_py = [0.0001, 0.0001]#0.001
		self.goal_py = [0.000, 0.000]

		self.pitch_list = []
		self.pitch_list.append(0.00)
		self.yaw_list = []
		self.yaw_list.append(0.00)

		self.data = [["PWMin_time"], ["tagmove_time"],["tagmove_v"]]
		self.k = 0


	def tag_det_callback(self,data):

		if len(data.detections) >= 1:

			if type(self.tag_p_old) is list:

				self.tag_p_now = data.detections[0].pose.pose.pose.position
				self.tag_p_old = data.detections[0].pose.pose.pose.position

			else:

				self.tag_p_now = data.detections[0].pose.pose.pose.position

				self.tag_d[0] = self.tag_p_now.x - self.tag_p_old.x
				self.tag_d[1] = self.tag_p_now.y - self.tag_p_old.y
				self.tag_d[2] = self.tag_p_now.z - self.tag_p_old.z

				self.tag_p[0] = self.tag_p_now.x + self.tag_d[0]
				self.tag_p[1] = self.tag_p_now.y + self.tag_d[1]
				self.tag_p[2] = self.tag_p_now.z + self.tag_d[2]

				self.e_py[0] = -(90 - int(math.degrees(math.atan2(self.tag_p[2], self.tag_p[1]))))
				self.e_py[1] = -(90 - int(math.degrees(math.atan2(self.tag_p[2], self.tag_p[0]))))

				self.pitch_list.append(self.e_py[0])
				
				
				self.M_py[0] = self.M_py1[0] + self.Kp_py[0]*(self.e_py[0]-self.e_py1[0]) + self.Ki_py[0]*self.e_py[0] + self.Kd_py[0]*((self.e_py[0]-self.e_py1[0])-(self.e_py1[0]-self.e_py2[0]))
				
				if self.M_py[0] >= 10.742:
					self.M_py[0] = 10.742
					self.pitch.start(self.M_py[0])
					self.data[0].append(time.time())
					time.sleep(0.0000001)

				elif self.M_py[0] <= 4.5:
					self.M_py[0] = 4.5
					self.pitch.start(self.M_py[0])
					self.data[0].append(time.time())
					time.sleep(0.0000001)

				else:
					self.pitch.start(self.M_py[0])
					self.data[0].append(time.time())
					time.sleep(0.0000001)

				
				
				
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
					
					
				

				self.M_py1[0] = self.M_py[0]
				self.e_py1[0] = self.e_py[0]
				self.e_py2[0] = self.e_py1[0]
				self.M_py1[1] = self.M_py[1]
				self.e_py1[1] = self.e_py[1]
				self.e_py2[1] = self.e_py1[1]
				
				self.tag_p_old = self.tag_p_now
				self.yaw_list.append(self.e_py[1])
				

		else:
			self.M_py[0] = 7.5
			self.pitch.start(self.M_py[0])
			self.M_py[1] = 7.5
			self.yaw.start(self.M_py[1])
			self.tag_p_old = [0, 0, 0]



	def tag_pos_callback(self, pos):
		if len(pos.detect_positions) >= 1:
			self.data[1].append(time.time())
			self.data[2].append(pos.detect_positions[0].y)
			self.k += 1
			


		if self.k == 50:
			f = open('/home/wanglab/catkin_ws/src/gimbal/data/2022.07.12_data/mudatime_data.txt', 'w')
			f.write(str(self.data))
			f.close()
			print("finish!!")

			
		else:
			pass


if __name__ == "__main__":
	tracking_apriltag()
	rospy.spin()

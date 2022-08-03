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



class tracking_apriltag(object):


	def __init__(self):
		#ROS
		rospy.init_node("tracking_apriltag")
		self.tag_det = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_det_callback)

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
		self.pitch_PID_parameter = [0.17, 0.0005, 0.0004]
		self.yaw_PID_parameter = [0.17, 0.0005, 0.0001]

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


	def tag_det_callback(self,data):

		if len(data.detections) >= 1:

			if type(self.P_Told) is list:
				self.P_Tnow = data.detections[0].pose.pose.pose.position
				self.P_Told = data.detections[0].pose.pose.pose.position

			else:
				self.P_Tnow = data.detections[0].pose.pose.pose.position

				self.Tag_position_predicter()
				self.angle_error()
				self.pitch_pid_controller()
				self.yaw_pid_controller()
				
				self.P_Told = self.P_Tnow

		else:
			self.pitch_input_pwm[0] = 7.422
			self.pitch.start(self.pitch_input_pwm[0])
			self.yaw_input_pwm[0] = 7.422
			self.yaw.start(self.yaw_input_pwm[0])
			self.P_Told = [0, 0, 0]


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

	def angle_error(self):
		self.pitch_error[0] = -(90 - int(math.degrees(math.atan2(self.P_Tnew[2], self.P_Tnew[1]))))
		self.yaw_error[0] = -(90 - int(math.degrees(math.atan2(self.P_Tnew[2], self.P_Tnew[0]))))	

if __name__ == "__main__":
	tracking_apriltag()
	rospy.spin()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import cv2
import message_filters
import numpy as np
import math
import tf
import rosparam
import time
import matplotlib as mpl
import matplotlib.pyplot as plt

from collections import deque
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Vector3

font = {'family':'monospace', 'size':'9'}
mpl.rc('font', **font)

class PlottingXY:

	def __init__(self):
	
		self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
		
		
		#tag
		self.tag_p_old = [0, 0, 0]
		self.tag_p_now = [0, 0, 0]
		self.tag_d = [0, 0, 0]
		self.tag_size = 0.076
		self.tag_q = [0, 0, 0, 0]
		self.tag_euler = [0, 0, 0]
		self.tag_p = [0, 0, 0]
		
		self.safe_rate_p = 3
		
		self.xyz_0 = np.float64([0.0, 0.0, 0.0])
		self.xyz_1 = np.float64([0.0, 0.0, 0.0])
		self.uv_0 = np.float64([320, 240])
		self.uv_1 = np.float64([320, 240])
		
		self.x_list = [0]
		self.y_list = [0]
		
		self.y_speed_list = [0]
		
		#camera
		self.rvec = np.float64([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
		self.tvec = np.float64([[0.0], [0.0], [0.0]])
		self.mtx = np.float64([[823.01442,   0.     , 332.287], [0.     , 823.27259, 238.87638], [0.0, 0.0, 1.0]])
		self.dist = np.float64([-0.010927, 0.024764, -0.001169, 0.002334, 0.000000])
		self.f_y = 823.27259
		self.color = (0,0,0)
		
		#fps
		self.d = deque()
		self.fps_list = [0]
		
		#graph
		self.fig = plt.figure(figsize=(10.0, 10.0))
		
		self.ax0 = self.fig.add_subplot(2, 2, 1)
		self.ax1 = self.fig.add_subplot(2, 2, 3)
		self.ax2 = self.fig.add_subplot(2, 2, 2)
		self.ax3 = self.fig.add_subplot(2, 2, 4)
		
		self.ax0.grid(True)
		self.ax1.grid(True)
		self.ax3.grid(True)
		
		self.ax2.axis("off")
		
		self.ax0.set_title('y data')
		self.ax1.set_title('x data')
		self.ax3.set_title('y speed data')
		
		self.ax0.set_ylabel('y[m]')
		
		self.ax1.set_xlabel('t[s]')
		self.ax1.set_ylabel('x[m]')
		
		self.ax3.set_xlabel('t[s]')
		self.ax3.set_ylabel('y speed[m/s]')
		
		self.ax0.set_ylim((-1, 1))
		self.ax1.set_ylim((-2, 2))
		self.ax3.set_ylim((-6, 6))
		
		#ラインの取得
		self.lines0, = self.ax0.plot(self.fps_list, self.y_list)
		self.lines1, = self.ax1.plot(self.fps_list, self.x_list)
		self.lines3, = self.ax3.plot(self.fps_list, self.y_speed_list)
		
	#tag sub
	def callback(self,data):
		
		if len(data.detections) >= 1:
			
			if type(self.tag_p_old) is list:
				#tag
				self.tag_p_now = data.detections[0].pose.pose.pose.position
				self.tag_p_old = data.detections[0].pose.pose.pose.position
				#self.x_list.append(self.tag_p_now.x)
				#self.y_list.append(self.tag_p_now.y)
				
				#fps
				self.d.append(time.perf_counter())
				
				#graph
				self.lines0.set_data(self.fps_list, self.y_list)
				self.ax0.set_xlim((0, self.fps_list[-1]))
		
				self.lines1.set_data(self.fps_list, self.x_list)
				self.ax1.set_xlim((0, self.fps_list[-1]))
				
				self.lines3.set_data(self.fps_list, self.y_speed_list)
				self.ax3.set_xlim((0, self.fps_list[-1]))
				
			else:
				#fps
				self.d.append(time.perf_counter())
				self.fps_list.append(self.fps_list[-1] + (self.d[-1] - self.d[0]))
				
				#tag
				self.tag_q = data.detections[0].pose.pose.pose.orientation
				self.tag_p_now = data.detections[0].pose.pose.pose.position
				self.x_list.append(self.tag_p_now.x)
				self.y_list.append(self.tag_p_now.y)
				
				#tag speed
				self.y_speed_list.append((self.y_list[-1] - self.y_list[-2]) / (self.d[-1] - self.d.popleft()))
				
				#graph
				self.lines0.set_data(self.fps_list, self.y_list)
				self.ax0.set_xlim((0, self.fps_list[-1]))
		
				self.lines1.set_data(self.fps_list, self.x_list)
				self.ax1.set_xlim((0, self.fps_list[-1]))
				
				
				self.lines3.set_data(self.fps_list[2:], self.y_speed_list[2:])
				self.ax3.set_xlim((0, self.fps_list[-1]))
				self.ax3.set_xlabel(f"max speed: {max(self.y_speed_list[2:]*100):0.1f} [cm/s]", fontsize=18)
				
				#print(max(self.y_speed_list[2:]))
				
				self.tag_p_old = self.tag_p_now
				
		else:
			pass#print("There are no Tags.")
			
def main():
	rospy.init_node("masking_image")
	plotting_xy = PlottingXY()
	while not rospy.is_shutdown():
		plt.pause(.01)

if __name__ == '__main__':
	main()

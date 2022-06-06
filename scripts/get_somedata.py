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
import time

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetectionPositionArray
from geometry_msgs.msg import Vector3

class get_pos_det_hz:
	def __init__(self):
		self.tag_pos = rospy.Subscriber('/tag_position' ,AprilTagDetectionPositionArray, self.tag_pos_callback)
		self.tag_det = rospy.Subscriber('/tag_detections' ,AprilTagDetectionArray ,self.tag_det_callback)
		self.data = [["Hz"], ["pos_u"], ["pos_v"], ["det_x"], ["det_y"], ["det_z"], ["deg_pitch"], ["deg_yaw"]]


	def tag_pos_callback(self, pos):
		if len(pos.detect_positions) >= 1:
			self.data[1].append(pos.detect_positions[0].x)
			self.data[2].append(pos.detect_positions[0].y)

			print(self.data[7])
		else:
			pass

	def tag_det_callback(self, det):
		if len(det.detections) >= 1:
			self.data[3].append(det.detections[0].pose.pose.pose.position.x)
			self.data[4].append(det.detections[0].pose.pose.pose.position.y)
			self.data[5].append(det.detections[0].pose.pose.pose.position.z)

			self.data[6].append(-(90 - float(math.degrees(math.atan2(det.detections[0].pose.pose.pose.position.z, det.detections[0].pose.pose.pose.position.y)))))
			self.data[7].append(-(90 - float(math.degrees(math.atan2(det.detections[0].pose.pose.pose.position.z, det.detections[0].pose.pose.pose.position.x)))))

		else:
			pass

if __name__ == "__main__":
	rospy.init_node("get_pos_det_hz")
	tag_subscriber = get_pos_det_hz()
	rospy.spin()

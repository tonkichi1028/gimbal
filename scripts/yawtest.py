#!/usr/bin/env python
# -*- coding: utf-8 -*-
                                                                    
#pwm: 4.5 7.226 7.618 10.742

import Jetson.GPIO as GPIO
import time

yaw_pin = 33

def main():
	GPIO.setmode(GPIO.BOARD)

	GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
	yaw = GPIO.PWM(yaw_pin, 430)
	
	for i in range(100):
		n = i
		yaw.start(59.1793)
		#time.sleep(0.0025)
		print(n)
	#time.sleep(1)
	yaw.stop()
	GPIO.cleanup()
	"""
	yaw.start(63.479)
	time.sleep(1)
	yaw.stop()
	GPIO.cleanup()
	"""
if __name__ == '__main__':
	main()

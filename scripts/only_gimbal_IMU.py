#!/usr/bin/env python
# -*- coding: utf-8 -*-
                                                                    
#pwm: 7.25 +- 2.5

import time
import RPi.GPIO as GPIO

pitch_pin = 32
yaw_pin = 33



def main():


	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
	pitch = GPIO.PWM(pitch_pin, 50)
	GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
	yaw = GPIO.PWM(yaw_pin, 50)
	
	pwm_r = 7.5
	pwm_l = 7.5
	
	yaw.start(7.226)
	time.sleep(1)
	yaw.start(7.618)
	time.sleep(1)
	
	for i in range(6):
		#print(pwm_r)
		#print(pwm_l)
		pwm_r -= 0.5
		pwm_l += 0.5
		yaw.start(pwm_r)
		time.sleep(1)
		yaw.start(pwm_l)
		time.sleep(1)
		
	yaw.start(4.5)
	time.sleep(1)
	yaw.start(10.742)
	time.sleep(1)
	
	yaw.stop()
	GPIO.cleanup()
if __name__ == '__main__':
	main()

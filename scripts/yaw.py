#!/usr/bin/env python
# -*- coding: utf-8 -*-
                                                                    
#pwm: 4.5 7.226 7.618 10.742


import time
import RPi.GPIO as GPIO

pitch_pin = 32
yaw_pin = 33



def main():
	GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
	yaw = GPIO.PWM(yaw_pin, 50)
	for i in range(100):
		pwm = 1*i
		yaw.start(pwm)
		time.sleep(1)
		print(pwm)

	#time.sleep(1)
	yaw.stop()
	GPIO.cleanup()

if __name__ == '__main__':
	main()

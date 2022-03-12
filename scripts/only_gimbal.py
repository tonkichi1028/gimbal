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

	yaw.start(5.1)
	time.sleep(1)
	yaw.stop()
	GPIO.cleanup()
if __name__ == '__main__':
	main()

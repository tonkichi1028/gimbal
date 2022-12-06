#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 35.743 ~ 59.179 -- 61.133 ~ 86.523
import time
import RPi.GPIO as GPIO

def main():
	pitch_pin = 32

	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
	pitch = GPIO.PWM(pitch_pin, 400)

	pitch.start(59.179)
	time.sleep(2)
	pitch.stop()
	GPIO.cleanup()

if __name__ == '__main__':
	main()

import RPi.GPIO as GPIO
import time

def gimbal_init():

	#gimbal_init
	pitch_pin = 32
	yaw_pin = 33
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(pitch_pin, GPIO.OUT, initial=GPIO.HIGH)
	self.pitch = GPIO.PWM(pitch_pin, 50)
	GPIO.setup(yaw_pin, GPIO.OUT, initial=GPIO.HIGH)
	self.yaw = GPIO.PWM(yaw_pin, 50)

	#pid_parameter
	self.pitch_PID_parameter = [0.17, 0.0005, 0.0004]
	self.yaw_PID_parameter = [0.17, 0.0005, 0.0001]

	#pwm_input_value, [0]=t, [1]=t-1, center_value=7.422
	self.pitch_input_pwm = [7.422, 7.422]
	self.yaw_input_pwm = [7.422, 7.422]

	#error_value_deg, [0]=t, [1]=t-1, [2]=t-2
	self.pitch_error = [0.00, 0.00, 0.00]
	self.yaw_error = [0.00, 0.00, 0.00]

	#pid_calculate
	self.pitch_P = 0.00
	self.pitch_I = 0.00
	self.pitch_D = 0.00
	self.yaw_P = 0.00
	self.yaw_I = 0.00
	self.yaw_D = 0.00


def pitch_pid_controller(pitch_error_angle_deg):
	
	self.pitch_error[0] = pitch_error_angle_deg

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


def yaw_pid_controller(yaw_error_angle_deg):

	self.yaw_error[0] = yaw_error_angle_deg
	
	self.yaw_P = self.yaw_PID_parameter[0]*(self.yaw_error[0] - self.yaw_error[1])
	self.yaw_I = self.yaw_PID_parameter[1]*self.yaw_error[0]
	self.yaw_D = self.yaw_PID_parameter[2]*((self.yaw_error[0] - self.yaw_error[1]) - (self.yaw_error[1] - self.yaw_error[2]))

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




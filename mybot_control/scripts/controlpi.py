#! /usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
from gpiozero import PWMOutputDevice
from time import sleep
from enum import Enum
import RPi.GPIO as GPIO
# [START pin_config]
class Pin(Enum):
	# Motor A
	pwm_forward_left_pin = 26
	pwm_reverse_left_pin = 19
	# Motor B
	pwm_forward_right_pin = 13
	pwm_reverse_right_pin = 6
	# Pin encoder Motor A
	encoder_channel_a_left_pin = 23
	encoder_channel_b_left_pin = 24
	# Pin encoder Motor B
	encoder_channel_a_right_pin = 25
	encoder_channel_b_right_pin = 8
# [END pin_config]

# [START gpio setup]
GPIO.setmode(GPIO.BCM)
GPIO.setup(Pin.encoder_channel_a_left_pin.value, GPIO.IN)
GPIO.setup(Pin.encoder_channel_b_left_pin.value, GPIO.IN)
# [END gpio setup]

# [START initialise objects for H-Bride PWM pins
forward_left = PWMOutputDevice(Pin.pwm_forward_left_pin.value, True, 0, 500)
reverse_left = PWMOutputDevice(Pin.pwm_reverse_left_pin.value, True, 0, 500)

forward_right = PWMOutputDevice(Pin.pwm_forward_right_pin.value, True, 0, 500)
reverse_right = PWMOutputDevice(Pin.pwm_reverse_right_pin.value, True, 0, 500)
# [END initialise objects for H-Bride PWM pins]

# [START pid_config_parameters]
pulse = 0
pre_pulse = 0
kp = 0.000152941; ki = 0.000027451; kd = 0.000011765
P = 0; I = 0; D = 0
error = 0
pre_error = 0
des_rSpeed = float(sys.argv[1])
dir = int(sys.argv[2])
sp_time = 50 #(ms)
res_encoder = 334
rate = 34 #1:34
output = 0
# [END pid_config_parameters]

# [START class_control]
class ControlPi(object):
	def __init__(self):
		pass

	def interruptLeftFunc(self, channel):
		global pulse
		if GPIO.input(Pin.encoder_channel_b_left_pin.value):
			pulse += 1
		else:
			pulse -= 1

	def callback(self, data):
		rospy.loginfo(rospy.get_calller_id(), data.data)

	def listener(self):
		rospy.init_node("control_pi_node", anonymous=True)
		rospy.Subscriber("wheel_angular_speed", String, callback)
		self.node_name = rospy.get_name()
		rospy.loginfo("{0} started".format(self.node_name))
		rospy.spin()

	def main(self):
		GPIO.add_event_detect(Pin.encoder_channel_a_left_pin.value, GPIO.RISING, callback=interruptLeftFunc)
		while True:
			des_pSpeed = ConvertRpmToPulse(des_rSpeed * rate)
			Rotate(MotorPID(des_pSpeed), dir)
			sleep(sp_time/1000.0)

	def ConvertRpmToPulse(self, rpm_value):
		return (rpm_value * sp_time * res_encoder * 0.001) / 60.0

	def ConvertPulseToRpm(self, pulse_value):
		return (pulse_value * 60.0) / (sp_time * res_encoder * 0.001)


	def MotorPID(self, des_pSpeed):
		global P, I, D, error, pre_error, pulse, pre_pulse, output
		act_pSpeed = pulse - pre_pulse
		pre_pulse = pulse
		error = des_pSpeed - abs(act_pSpeed)
		P = kp * error
		I += ki * error * sp_time / 1000.0
		D = kd * (error - pre_error) * 1000.0 / sp_time
		output += P + I + D
		if output >= 1.0:
			output = 1.0
		elif output <= 0:
			output = 0.0
		pre_error = error
		return output

	def Rotate(self, pwm, dir):
		forward_left.value = pwm if dir else 0
		reverse_left.value = 0 if dir else pwm

if __name__ == "__main__":
	try:
		node = ControlPi()
		node.listener()
	except rospy.ROSInterruptException:
		pass

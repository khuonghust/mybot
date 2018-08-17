#!/usr/bin/python
import rospy
from math import pi
# import numpy
from std_msgs.msg import Float32,Int32MultiArray

class WheelEncoderPublisher(object):
	"""docstring for WheelEncoderPublisher"""
	def __init__(self):
		super(WheelEncoderPublisher, self).__init__()
		rospy.init_node('mybot_wheel_encoder_publisher')
		# Read in tangential velocity targets
		self.wheel_enc_sub = rospy.Subscriber('wheel_enc', Int32MultiArray, self.wheel_enc_sub_callback)
		self.lwheel_angular_vel_enc_pub = rospy.Publisher('lwheel_angular_vel_enc', Float32, queue_size=10)
		self.rwheel_angular_vel_enc_pub = rospy.Publisher('rwheel_angular_vel_enc', Float32, queue_size=10)
		self.lwheel_tangent_vel_enc_pub = rospy.Publisher('lwheel_tangent_vel_enc', Float32, queue_size=10)
		self.rwheel_tangent_vel_enc_pub = rospy.Publisher('rwheel_tangent_vel_enc', Float32, queue_size=10)
		
		self.rate = rospy.get_param('~rate', 50)
		self.rate_gear = rospy.get_param('~rate_gear', 55)
		self.timeout = float(rospy.get_param('~timeout', 1))
		self.time_prev_update = rospy.Time.now()

		self.R = rospy.get_param('~robot_wheel_radius', 0.1)
		self.res_encoder = rospy.get_param('~res_encoder', 13)

		# Need a little hack to incorporate direction wheels are spinning
		self.lwheel_dir = 1;
		self.rwheel_dir = 1;

		self.wheel_enc = [0,0,0,0]

	def wheel_enc_sub_callback(self, msg):
		self.wheel_enc = msg.data

	def angular_to_tangent(self, angular):
		return angular*self.R

	def enc_2_rads(self, enc):
		rads = enc * 2.0 * pi / (self.res_encoder * self.rate_gear)
		return rads

	def update(self):
		# History of past three encoder reading
		time_curr_update = rospy.Time.now()
		dt = (time_curr_update - self.time_prev_update).to_sec()

		#Compute angular velocity in rad/s
		lwheel_enc_delta = self.wheel_enc[0] - self.wheel_enc[1]
		rwheel_enc_delta = self.wheel_enc[2] - self.wheel_enc[3]
		lwheel_angular_vel_enc = self.enc_2_rads(lwheel_enc_delta) / dt
		rwheel_angular_vel_enc = self.enc_2_rads(rwheel_enc_delta) / dt
		#print(dt)qw	
		#Update time
		self.time_prev_update = time_curr_update

		#Publish data
		self.lwheel_angular_vel_enc_pub.publish(lwheel_angular_vel_enc)
		self.rwheel_angular_vel_enc_pub.publish(rwheel_angular_vel_enc)
		self.lwheel_tangent_vel_enc_pub.publish(self.angular_to_tangent(lwheel_angular_vel_enc))
		self.rwheel_tangent_vel_enc_pub.publish(self.angular_to_tangent(rwheel_angular_vel_enc))


	def spin(self):
		rospy.loginfo("Start mybot_wheel_encoder_publisher")
		rate = rospy.Rate(self.rate)
		time_curr_update = rospy.Time.now()
		rospy.on_shutdown(self.shutdown)

		while not rospy.is_shutdown():
			time_diff_update = (time_curr_update - self.time_prev_update).to_sec()
			if time_diff_update < self.timeout:
				self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Stop mybot_wheel_encoder_publisher")
		self.lwheel_angular_vel_enc_pub.publish(0)
		self.rwheel_angular_vel_enc_pub.publish(0)
		self.lwheel_tangent_vel_enc_pub.publish(0)
		self.rwheel_tangent_vel_enc_pub.publish(0)
		rospy.sleep(0.5)

def main():
	encoder_publisher = WheelEncoderPublisher()
	encoder_publisher.spin()

if __name__ == '__main__':
	main()

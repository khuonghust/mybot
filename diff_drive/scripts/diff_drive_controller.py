#! /usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

import controller

class ControllerNode:

	def __init__(self):
		self.controller = controller.Controller()
		self.linear_velocity = 0.0
		self.angular_velocity = 0.0

	def main(self):
		self.linear_speed_pub = rospy.Publisher('wheel_linear_speed',
										String, queue_size=1)
		self.angular_speed_pub = rospy.Publisher('wheel_angular_speed', 
										String, queue_size=1)
		rospy.init_node('diff_drive_controller')
		self.node_name = rospy.get_name()
		rospy.loginfo("{0} started".format(self.node_name))

		rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

		self.wheel_separation = float(rospy.get_param('~wheel_separation'))
		self.wheel_radius = float(rospy.get_param('~wheel_radius'))
		self.max_motor_speed = float(rospy.get_param('~max_motor_speed'))
		self.rate = float(rospy.get_param('~rate', 10.0))
		self.timeout = float(rospy.get_param('~timeout', 0.2))

		self.controller.setWheelSeparation(self.wheel_separation)
		self.controller.setWheelRadius(self.wheel_radius)
		self.controller.setMaxMotorSpeed(self.max_motor_speed)

		rate = rospy.Rate(self.rate)
		self.last_twist_time = rospy.get_time()
		while not rospy.is_shutdown():
			self.publish()
			rate.sleep()

	def publish(self):
		if rospy.get_time() - self.last_twist_time < self.timeout:
			linear_speeds = self.controller.getLinearSpeeds(self.linear_velocity,
											   self.angular_velocity)
			angular_speeds = self.controller.getAngularSpeeds(self.linear_velocity,
												self.angular_velocity)

			linear_speed_data_pub = json.dumps(
				{'left': linear_speeds.left, 'right' : linear_speeds.right})
			angular_speed_data_pub = json.dumps(
				{'left': angular_speeds.left, 'right' : angular_speeds.right})
		else:
			linear_speed_data_pub = json.dumps(
				{'left': 0, 'right' : 0})
			angular_speed_data_pub = json.dumps(
				{'left': 0, 'right' : 0})
		self.linear_speed_pub.publish(linear_speed_data_pub)
		self.angular_speed_pub.publish(angular_speed_data_pub)


	def twistCallback(self, twist):
		self.linear_velocity = twist.linear.x
		self.angular_velocity = twist.angular.z
		self.last_twist_time = rospy.get_time()

if __name__ == '__main__':
	try:
		node = ControllerNode()
		node.main()
	except rospy.ROSInterruptException:
		pass

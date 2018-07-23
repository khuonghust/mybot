from __future__ import division

class MotorCommand:
	"""Holds motor control commands for a differential-drive robot.
	"""

	def __init__(self):
		self.left = 0
		self.right = 0
		

class Controller:
	"""Determines motor speeds to accomplish a desired motion.
	"""

	def __init__(self):
		# Set the max motor speed to a very large value so that it
		# is, essentially, unbound.
		self.max_motor_speed = 100 # (rpm)
		self.wheel_separation = 0.2 #(m)
		self.wheel_radius = 0.065 #(m)

	def getLinearSpeeds(self, linearSpeed, angularSpeed):
		linear_speeds = MotorCommand()
		linear_speeds.left = linearSpeed + angularSpeed * self.wheel_separation/2.0
		linear_speeds.right = linearSpeed - angularSpeed * self.wheel_separation/2.0

		# Adjust speeds if they exceed the maximum.
		if max(linear_speeds.left, linear_speeds.right) > self.max_motor_speed:
			factor = self.max_motor_speed / max(linear_speeds.left, linear_speeds.right)
			linear_speeds.left *= factor
			linear_speeds.right *= factor

		return linear_speeds

	def getAngularSpeeds(self, linearSpeed, angularSpeed):
		linear_speeds = self.getLinearSpeeds(linearSpeed, angularSpeed)
		angular_speeds = MotorCommand()
		angular_speeds.left = linear_speeds.left / self.wheel_radius
		angular_speeds.right = linear_speeds.right / self.wheel_radius
		return angular_speeds

	def setWheelSeparation(self, separation):
		self.wheel_separation = separation

	def setMaxMotorSpeed(self, limit):
		self.max_motor_speed = limit

	def setWheelRadius(self, radius):
		self.wheel_radius = radius

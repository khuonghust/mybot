#!/usr/bin/python
import rospy
import roslib
from std_msgs.msg import String
import json

from dynamic_reconfigure.server import Server
from mybot_control.cfg import ParamConfig

param = None
def dynamic_reconfigure_callback(config, level):
	data = {
		'kp': config.kp,
		'ki': config.ki,
		'kd': config.kd
	}
	data_string = json.dumps(data)
	param.publish(data_string)
	rospy.loginfo(data_string)
	return config

def main():
	global param
	rospy.init_node("mybot_param")
	param = rospy.Publisher('param_pid', String, queue_size=10)
	srv = Server(ParamConfig, dynamic_reconfigure_callback)
	rospy.spin()
if __name__ == '__main__':
	main()
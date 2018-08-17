#!/usr/bin/python
import rospy
import roslib
from std_msgs.msg import Float32, String
import json
from dynamic_reconfigure.server import Server
import mybot_control.cfg.ParamConfig as ConfigType
class ControlsToMotors:
    def __init__(self):
        rospy.init_node('mybot_controller')
        self.dyn_reconf_server = Server(ConfigType, self.reconfigure)
        self.rate = rospy.get_param('~rate', 50)
        self.Kp = 2.0
        self.Ki = 0.0
        self.Kd = 0.0

        # approx 5.34 rad / s when motor_cmd = 255
        self.motor_max_angular_vel = rospy.get_param('~motor_max_angular_vel', 3.97)
        self.motor_min_angular_vel = rospy.get_param('~motor_min_angular_vel', 0)

        self.wheel_radius = rospy.get_param('~wheel_radius', 0.1)

        # Corresponding motor commands
        self.motor_cmd_max = rospy.get_param('~motor_cmd_max', 1.0)
        self.motor_cmd_min = rospy.get_param('~motor_cmd_min', 0.0)

        # # Publish the computed angular velocity motor command
        # self.lwheel_angular_vel_motor_pub = rospy.Publisher('lwheel_angular_vel_motor', Float32, queue_size=10)
        # self.rwheel_angular_vel_motor_pub = rospy.Publisher('rwheel_angular_vel_motor', Float32, queue_size=10)

        # Publish motor command
        self.lwheel_motor_cmd_pub = rospy.Publisher('lwheel_motor_cmd_pub', Float32, queue_size=10)
        self.rwheel_motor_cmd_pub = rospy.Publisher('rwheel_motor_cmd_pub', Float32, queue_size=10)

        self.param_pid = rospy.Subscriber('param_pid', String, self.param_pid_callback)

        # Read in encoders for PID control
        self.lwheel_angular_vel_enc_sub = rospy.Subscriber(
            'lwheel_angular_vel_enc',
            Float32,
            self.lwheel_angular_vel_enc_callback)
        self.rwheel_angular_vel_enc_sub = rospy.Subscriber(
            'rwheel_angular_vel_enc',
            Float32,
            self.rwheel_angular_vel_enc_callback)

        # Read in tangential velocity targets
        self.lwheel_tangent_vel_target_sub = rospy.Subscriber(
            'lwheel_tangent_vel_target',
            Float32,
            self.lwheel_tangent_vel_target_callback)
        self.rwheel_tangent_vel_target_sub = rospy.Subscriber(
            'rwheel_tangent_vel_target',
            Float32,
            self.rwheel_tangent_vel_target_callback)

        # Tangential velocity target
        self.lwheel_tangent_vel_target = 0
        self.rwheel_tangent_vel_target = 0

        # Angular velocity target
        self.lwheel_angular_vel_target = 0
        self.rwheel_angular_vel_target = 0

        # Angular velocity encoder readings
        self.lwheel_angular_vel_enc = 0
        self.rwheel_angular_vel_enc = 0

        # Value motor command
        self.lwheel_motor_cmd = 0
        self.rwheel_motor_cmd = 0

        # PID control variables
        self.lwheel_pid = {}
        self.rwheel_pid = {}

    # # Motor cmd
    # self.motor_left = MotorCommand(Pin.pwm_forward_left_pin.value, Pin.pwm_reverse_left_pin.value)
    # self.motor_right = MotorCommand(Pin.pwm_forward_right_pin.value, Pin.pwm_reverse_right_pin.value)

    def param_pid_callback(self, msg):
        param_data = json.loads(msg.data)
        self.Kp = param_data['kp']
        self.Ki = param_data['ki']
        self.Kd = param_data['kd']
        rospy.loginfo("Pid changed! [{} {} {}]".format(self.Kp, self.Ki, self.Kd))

    # ==================================================
    # Read in encoder readings for PID
    # ==================================================
    def lwheel_angular_vel_enc_callback(self, msg):
        self.lwheel_angular_vel_enc = msg.data

    def rwheel_angular_vel_enc_callback(self, msg):
        self.rwheel_angular_vel_enc = msg.data

    # ==================================================
    # Read in tangential velocity targets
    # ==================================================
    def lwheel_tangent_vel_target_callback(self, msg):
        self.lwheel_tangent_vel_target = msg.data

    def rwheel_tangent_vel_target_callback(self, msg):
        self.rwheel_tangent_vel_target = msg.data

    # ==================================================
    # Update motor commands
    # ==================================================

    # Compute angular velocity target
    def tangentVelToAngularVel(self, tangent_vel):
        # v = wr
        # v - tangential velocity (m/s)
        # w - angular velocity (rad/s)
        # r - radius of wheel (m)
        angular_vel = tangent_vel / self.wheel_radius;
        return angular_vel

    def pidControl(self, wheel_pid, target, state):
        # Initialize pid dictionary
        if len(wheel_pid) == 0:
            wheel_pid.update({'time_prev': rospy.Time.now(), 'derivative': 0, 'integral': [0] * 10, 'error_prev': 0,
                              'error_curr': 0})

        wheel_pid['time_curr'] = rospy.Time.now()
        # PID control
        wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
        if wheel_pid['dt'] == 0: return 0

        wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr'] * wheel_pid['dt'])]
        wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev']) / wheel_pid['dt']
        wheel_pid['error_prev'] = wheel_pid['error_curr']
        control_signal = (
                    self.Kp * wheel_pid['error_curr'] + self.Ki * sum(wheel_pid['integral']) + self.Kd * wheel_pid[
                'derivative'])
        # print(self.Kp)
        target_new = target + control_signal

        if target > 0 and target_new < 0: target_new = target
        if target < 0 and target_new > 0: target_new = target

        if (target == 0):  # Not moving
            target_new = 0
            return target_new

        wheel_pid['time_prev'] = wheel_pid['time_curr']
        return target_new

    # Mapping angular velocity targets to motor commands
    # Note: motor commands are ints between 0 - 255
    # We also assume motor commands are issues between motor_min_angular_vel and motor_max_angular_vel
    def angularVelToMotorCmd(self, angular_vel_target):
        if angular_vel_target == 0: return 0
        slope = (self.motor_cmd_max - self.motor_cmd_min) / (self.motor_max_angular_vel - self.motor_min_angular_vel)
        intercept = self.motor_cmd_max - slope * self.motor_max_angular_vel

        if angular_vel_target > 0:  # positive angular velocity
            motor_cmd = slope * angular_vel_target + intercept
            if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
            if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
        else:  # negative angular velocity
            motor_cmd = slope * abs(angular_vel_target) + intercept
            if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
            if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
            motor_cmd = -motor_cmd

        return motor_cmd

    def lwheelUpdate(self):
        # Compute target angular velocity
        self.lwheel_angular_vel_target = self.tangentVelToAngularVel(self.lwheel_tangent_vel_target)

        # If we want to adjust target angular velocity using PID controller to incorporate encoder readings
        self.lwheel_angular_vel_target = self.pidControl(self.lwheel_pid, self.lwheel_angular_vel_target,
                                                         self.lwheel_angular_vel_enc)

        # Compute motor command
        lwheel_motor_cmd = self.angularVelToMotorCmd(self.lwheel_angular_vel_target)
        self.lwheel_motor_cmd_pub.publish(lwheel_motor_cmd)

    # # Send motor command
    # self.motorCmdToRobot('left',lwheel_motor_cmd)

    def rwheelUpdate(self):
        # Compute target angular velocity
        self.rwheel_angular_vel_target = self.tangentVelToAngularVel(self.rwheel_tangent_vel_target)
        # self.rwheel_angular_vel_target_pub.publish(self.rwheel_angular_vel_target)

        # If we want to adjust target angular velocity using PID controller to incorporate encoder readings
        self.rwheel_angular_vel_target = self.pidControl(self.rwheel_pid, self.rwheel_angular_vel_target,
                                                         self.rwheel_angular_vel_enc)
        # self.rwheel_angular_vel_control_pub.publish(self.rwheel_angular_vel_target)

        # Compute motor command
        rwheel_motor_cmd = self.angularVelToMotorCmd(self.rwheel_angular_vel_target)
        self.rwheel_motor_cmd_pub.publish(rwheel_motor_cmd)

    # # Send motor command
    # self.motorCmdToRobot('right',rwheel_motor_cmd)

    # When given no commands for some time, do not move

    def reconfigure(self, config, level):
        self.Kp = config.kp
        self.Ki = config.ki
        self.Kd = config.kd
        rospy.loginfo("Gains pid changed![{} {} {}".format(self.Kp, self.Ki, self.Kd))
        return config

    def spin(self):
        rospy.loginfo("Start mybot_controller")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.rwheelUpdate()
            self.lwheelUpdate()
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop mybot_controller")
        self.lwheel_motor_cmd_pub.publish(0);
        self.rwheel_motor_cmd_pub.publish(0);
        rospy.sleep(0.5)


def main():
    controls_to_motors = ControlsToMotors();
    controls_to_motors.spin()


if __name__ == '__main__':
    main()

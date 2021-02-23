#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float32
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.srv import ControlMode, ControlModeRequest
from com_utils import Topics, Services


CENTER = 450
AMPLITUD = 350
TENDON = 8
PERIOD = 5
FREQUENCY = 100

target_pub = rospy.Publisher("/target_pwm", Float32, queue_size=1)
pub = rospy.Publisher(Topics.MOTOR_COMMAND, MotorCommand, queue_size=1)
srv = rospy.ServiceProxy(Services.CONTROL_MODE, ControlMode)
rospy.wait_for_service(Services.CONTROL_MODE, timeout=1.0)
rospy.init_node("test_node")
rate = rospy.Rate(FREQUENCY)
t = 0

mode_req = ControlModeRequest()
mode_req.legacy = False
mode_req.control_mode = 3
mode_req.motor_id = [TENDON]
mode_req.set_points = [0]

srv(mode_req)

command_msg = MotorCommand()
command_msg.legacy = False
command_msg.motor = [TENDON]

while not rospy.is_shutdown():
	t += 1
	pwm = math.sin(t*2*math.pi/FREQUENCY/PERIOD)*AMPLITUD + CENTER
	command_msg.setpoint = [pwm]
	pub.publish(command_msg)
	target_pub.publish(pwm/10)
	rate.sleep()



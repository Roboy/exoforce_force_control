#!/usr/bin/env python3

import math
import rospy
from roboy_simulation_msgs.msg import TendonUpdate
from com_utils import Topics

CENTER = 10
AMPLITUD = 5
TENDON = 8
PERIOD = 5
FREQUENCY = 100

pub = rospy.Publisher(Topics.TENDON_FORCE, TendonUpdate, queue_size=1)
rospy.init_node("test_node")
rate = rospy.Rate(FREQUENCY)
t = 0

while not rospy.is_shutdown():
	t += 1
	force = math.sin(t*2*math.pi/FREQUENCY/PERIOD)*AMPLITUD + CENTER
	msg = TendonUpdate()
	msg.tendon_id = TENDON
	msg.force = force
	pub.publish(msg)
	rate.sleep()



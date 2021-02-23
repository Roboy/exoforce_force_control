#!/usr/bin/env python3

from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *


class ConnectionError(Exception):
	"""Raised when connection to load cell channels fails."""
	def __init__(self, message):
		self.message = message


class LoadCell(VoltageRatioInput):
	"""This class handles a single load cell channel.
	
	"""
	def __init__(self, conf):
		"""
		Args:
			conf (dict): Dictionary with initial channel configuration:
							'tendon_id' 	(int): Tendon id to which the cell is attached.
							'cal_offset'  (float): Calibration offset value of the load cell.
							'cal_factor'  (float): Calibration factor value of the load cell.
							'serial' 		(int): Serial number of the phidget to which the cell is connected.
							'channel' 		(int): Number of the phidget channel to which the cell is connected.
		
		"""
		super().__init__()
		self.id = conf['tendon_id']
		self.cal_offset = conf['cal_offset']
		self.cal_factor = conf['cal_factor']
		self.setDeviceSerialNumber(conf['serial'])
		self.setChannel(conf['channel'])

	@property
	def ready(self):
		return self.cal_offset is not None and self.cal_factor is not None

	def getAddress(self):
		return f"{self.getDeviceSerialNumber()}/{self.getChannel()}"

	def openChannel(self):
		"""Opens and attaches to phidget channel.
		
		Args:
			-

		Returns:
		   	-

		"""
		try:
			self.openWaitForAttachment(Phidget.DEFAULT_TIMEOUT)
		except PhidgetException as e:
			raise ConnectionError(e.details)

	def closeChannel(self):
		"""Closes phidget channel.
		
		Args:
			-

		Returns:
		   	-

		"""
		self.close()

	def readForce(self):
		"""Reads force value from load cell in Newtons.
		
		Args:
			-

		Returns:
		   	float: Force value in Newtons.

		"""
		force = 0
		if self.ready:
			force = self.cal_factor * (self.getVoltageRatio() + self.cal_offset)
		return force * 9.81



# provisional code for class testing

import time

phidget_serial = 586100
cal_offsets = [-1.433305e-06, -2.652407e-06, -4.034489e-06, 2.57045e-06]
cal_factors = [76946.05648001497, 79410.51153687084, 79372.69350821163, 81081.8446537218]
cal_offsets = [-4.806556e-06]
cal_factors = [-7781.41450138835]

configuration = [{'tendon_id': i, 'cal_offset': o, 'cal_factor': f, 'serial': phidget_serial, 'channel': i} for i, (o, f) in enumerate(zip(cal_offsets, cal_factors))]

if __name__ == "__main__":
	import rospy
	from std_msgs.msg import Float32

	pub = rospy.Publisher("/actual_force", Float32, queue_size=1)
	rospy.init_node("load_cell_test")
	rate = rospy.Rate(100)

	channels = []
	for i, conf in enumerate(configuration):
		new_channel = LoadCell(conf)
		try:
			new_channel.openChannel()
			new_channel.setDataInterval(10)
			channels.append(new_channel)
		except ConnectionError as e:
			print(f"Failed to open load cell {i}: {e.message}")
			
	try:
		while not rospy.is_shutdown():
			print([f"{channel.id}: [{channel.readForce():4.2f} N]" for channel in channels])
			pub.publish(channels[0].force)
			rate.sleep()
	except (Exception, KeyboardInterrupt):
		pass

	# Close your Phidgets once the program is done.
	for channel in channels:
		channel.closeChannel()

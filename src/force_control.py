#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

from simple_pid import PID

from roboy_simulation_msgs.msg import TendonUpdate
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.srv import ControlMode, ControlModeRequest

from load_cell import LoadCell, ConnectionError
from com_utils import Topics, Services

class TendonForceController:
	"""This class handles the force controller for one individual tendon.

	"""
	def __init__(self, conf):
		"""
		Args:
			conf (dict): Dictionary with controller configuration:
							'controller_id' (int)	: Id of the corresponding motor.
							'kp' (float)			: Proportional gain.
							'ki' (float)			: Integral gain.
							'kd' (float)			: Derivative gain.
							'direction' (int)		: Direction of rotation of motor on cage.
							'pwm_limit' (float)		: Maximum PWM value for the motors.
							'freq' (int)			: Frequency of the control loop.
							'load_cell_conf' (dict)	: Load cell configuration.
		
		"""
		self.id = conf['controller_id']
		self.kp = conf['kp']
		self.ki = conf['ki']
		self.kd = conf['kd']
		self.direction = conf['direction']
		self.pwm_limit = conf['pwm_limit']
		self.frequency = conf['freq']

		self.pid = PID(self.kp, self.ki, self.kd, sample_time=1/self.frequency, output_limits=(-self.pwm_limit, self.pwm_limit))

		self.target_force = 10
		self.ready = False
		self.detached = False

		self.pwm_set_point = 0

		self.force_sensor = LoadCell(conf['load_cell_conf'])
		self.force_sensor.setOnAttachHandler(self.onAttach)
		self.force_sensor.setOnDetachHandler(self.onDetach)

	@property
	def target_force(self):
		return self._target_force

	@target_force.setter
	def target_force(self, value):
		self.pid.setpoint = value
		self._target_force = value

	def onAttach(self, channel):
		"""Event handler for the attach event of the LoadCell object.

		Args:
			-

		Returns:
			-
		
		"""
		self.force_sensor.setDataInterval(int(1000/self.frequency))
		self.ready = self.force_sensor.ready
		self.detached = False
		rospy.loginfo(f"Load Cell [{self.id}] atached!")
	
	def onDetach(self, channel):
		"""Event handler for the detach event of the LoadCell object.

		Args:
			-

		Returns:
			-
		
		"""
		self.detached = True
		rospy.loginfo(f"Load Cell [{self.id}] detached!")

	def connectToSensor(self):
		"""Opens and attaches the LoadCell object to a load cell channel.

		Args:
			-

		Returns:
			-
		
		"""
		self.force_sensor.openChannel()

	def getPwmSetPoint(self):
		"""Returns pwm control value calculated from current error.

		Args:
			-

		Returns:
			(float): PWM set point.
		
		"""
		if self.detached:
			self.ready = False
			return 0.0

		current_force = self.force_sensor.readForce()

		self.pwm_set_point += self.pid(current_force) * self.direction
		
		if self.pwm_set_point > self.pwm_limit: self.pwm_set_point = self.pwm_limit
		if self.pwm_set_point < -self.pwm_limit: self.pwm_set_point = -self.pwm_limit

		rospy.loginfo(f"Tendon [{self.id}] target [{self.target_force:4.2f}] actual [{current_force: 4.2f}] pwm [{self.pwm_set_point:.0f}]")

		# return pwm_set_point
		return self.target_force, current_force, self.pwm_set_point, self.pid.components

class ForceControl:
	"""This class handles the force control ROS node.

	"""
	def __init__(self):
		"""
		Args:
			-
		
		"""
		self.refresh_rate = rospy.get_param('refresh_rate', None)
		self.rate = None if self.refresh_rate is None else rospy.Rate(self.refresh_rate)

		controllers_conf = self.get_controllers_conf()

		self.controllers = []
		for conf in controllers_conf:
			self.controllers.append(TendonForceController(conf))

		rospy.Subscriber(Topics.TENDON_FORCE, TendonUpdate, self.set_target_force)

		self.start_controllers()
		self.init_roboy_plexus()
		self.start_node()

	def get_controllers_conf(self):
		"""Reads node's parameters and builds list of configuration dictionaries.

		Args:
			-

		Returns:
			(list[dict]): List of controllers configurations.
		
		"""
		controllers_id = rospy.get_param('controllers_id')
		cal_offsets = rospy.get_param('cal_offset')
		cal_factors = rospy.get_param('cal_factor')
		serials = rospy.get_param('serials')
		channels = rospy.get_param('channels')
		kp = rospy.get_param('kp')
		ki = rospy.get_param('ki')
		kd = rospy.get_param('kd')
		direction = rospy.get_param('direction')
		pwm_limit = rospy.get_param('pwm_limit')
		if len(kp) == 1: kp *= len(controllers_id)
		if len(ki) == 1: ki *= len(controllers_id)
		if len(kd) == 1: kd *= len(controllers_id)
		if len(direction) == 1: direction *= len(controllers_id)
		if len(pwm_limit) == 1: pwm_limit *= len(controllers_id)

		load_cells_conf = [{'tendon_id': i, 'cal_offset': o, 'cal_factor': f, 'serial': s, 'channel': c} for i, o, f, s, c in zip(controllers_id, cal_offsets, cal_factors, serials, channels)]

		return [{'controller_id': index, 'kp': p, 'ki': i , 'kd': d, 'direction': di, 'pwm_limit': l,'load_cell_conf': conf, 'freq': self.refresh_rate} for index, p, i, d, di, l, conf in zip(controllers_id, kp, ki, kd, direction, pwm_limit, load_cells_conf)]
		
	def init_roboy_plexus(self):
		"""Initiliazes objects to interface with the roboy plexus.

		Args:
			-

		Returns:
			-
		
		"""
		self.motor_command_publisher = rospy.Publisher(Topics.MOTOR_COMMAND, MotorCommand, queue_size=1)

		self.control_mode_client = rospy.ServiceProxy(Services.CONTROL_MODE, ControlMode)

		self.control_mode_req = ControlModeRequest()
		self.motor_command_msg = MotorCommand()

	def start_controllers(self):
		"""Connects controller with load cells.

		Args:
			-

		Returns:
			-
		
		"""
		for controller in self.controllers:
			try:
				controller.connectToSensor()
			except ConnectionError as e:
				rospy.logwarn(f"Failed to connect to load cell {controller.id}")
				rospy.logdebug(f"Connection error to load cell {controller.id}: {e.message}")

	def start_node(self):
		"""Starts control loop.

		Args:
			-

		Returns:
			-
		
		"""
		if self.rate is None:
			rospy.logerr("refresh_rate not set")
		else:
			self.set_control_mode(3)
			self.control_loop()

	def get_controller(self, id):
		"""Returns controller with requested id.

		Args:
			id (int): Id of desired controller.

		Returns:
			(TendonForceController): Contoller object.
		
		"""
		for controller in self.controllers:
			if controller.id == id:
				return controller
		return None

	def set_target_force(self, msg):
		"""Set tendon target force callback.

		Args:
			msg (TendonUpdate): Received ROS Message.

		Returns:
			-
		
		"""
		controller = self.get_controller(msg.tendon_id)
		if controller is None:
			rospy.logwarn(f"Trying to set target force for tendon ({msg.tendon_id}) without controller.")
		else:
			if msg.force < 0:
				rospy.logwarn(f"Trying to set negative target force for tendon ({msg.tendon_id}).")
			else:
				controller.target_force = msg.force

	def control_loop(self):
		"""Control loop.

		Args:
			-

		Returns:
			-
		
		"""
		target_pub = rospy.Publisher("/target_force", Float32, queue_size=1)
		actual_pub = rospy.Publisher("/actual_force", Float32, queue_size=1)
		control_pub = rospy.Publisher("/control_value", Float32, queue_size=1)
		p_value_pub = rospy.Publisher("/p_value", Float32, queue_size=1)
		i_value_pub = rospy.Publisher("/i_value", Float32, queue_size=1)
		d_value_pub = rospy.Publisher("/d_value", Float32, queue_size=1)
		while not rospy.is_shutdown():
			motor_ids = []
			set_points = []
			for controller in self.controllers:
				if controller.ready:
					motor_ids.append(controller.id)
					target, actual, control, (p_value, i_value, d_value) = controller.getPwmSetPoint()
					set_points.append(control)

					target_pub.publish(target)
					actual_pub.publish(actual)
					control_pub.publish(control)
					p_value_pub.publish(p_value)
					i_value_pub.publish(i_value)
					d_value_pub.publish(d_value)

			self.send_motor_commands(motor_ids, set_points)
			
			self.rate.sleep()

	def set_control_mode(self, mode, motor_ids=None, set_points=[]):
		"""Set motors control mode.

		Args:
			mode (int)				: Control mode:
										0: ENCODER0_POSITION: position controller using encoder0
										1: ENCODER1_POSITION: position controller using encoder1
										2: DISPLACEMENT: position controller using displacement
										3: DIRECT_PWM: directly controlling the pwm value of the motor
			motor_ids (list[int]) 	: Motor list to change mode, if None, all motors will be updated.
			set_points (list[float]): Set points for the corresponding motor list, if empty all motors will be set to 0.

		Returns:
			-
		
		"""
		self.control_mode_req.legacy = False
		self.control_mode_req.control_mode = mode
		self.control_mode_req.motor_id = motor_ids if motor_ids is not None else [controller.id for controller in self.controllers]
		self.control_mode_req.set_points = set_points if motor_ids is not None else [0] * len(self.control_mode_req.motor_id)

		try:
			rospy.wait_for_service(Services.CONTROL_MODE, timeout=1.0)
			self.control_mode_client(self.control_mode_req)
		except rospy.exceptions.ROSException as e:
			rospy.logerr(e)
			rospy.signal_shutdown(e)

	def send_motor_commands(self, motor_ids=None, set_points=None):
		"""Set motors set point.

		Args:
			motor_ids (list[int]) 	: Motor list to set, if None, all motors will be updated.
			set_points (list[float]): Set points for the corresponding motor list, if None all motors will be set to 0.

		Returns:
			-
		
		"""
		if motor_ids is None:
			motor_ids = [controller.id for controller in self.controllers]
		if set_points is None:
			set_points = [0.0] * len(motor_ids)

		self.motor_command_msg.legacy = False
		self.motor_command_msg.motor = motor_ids
		self.motor_command_msg.setpoint = set_points

		self.motor_command_publisher.publish(self.motor_command_msg)


if __name__ == "__main__":
	rospy.init_node("force_control", disable_signals=True)
	force_control = ForceControl()

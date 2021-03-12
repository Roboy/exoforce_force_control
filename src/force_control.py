#!/usr/bin/env python3

import argparse
import sys

import rospy
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.srv import ControlMode, ControlModeRequest
from roboy_simulation_msgs.msg import TendonUpdate
from simple_pid import PID
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse

from com_utils import Services, Topics
from load_cell import ConnectionError, LoadCell

class TendonForceController:
	"""This class handles the force controller for one individual tendon.

	"""
	def __init__(self, conf, init_force=0.0, test=False):
		"""
		Args:
			conf (dict): Dictionary with controller configuration:
							'controller_id' (int)	: Id of the corresponding motor.
							'kp' (float)			: Proportional gain.
							'ki' (float)			: Integral gain.
							'kd' (float)			: Derivative gain.
							'direction' (int)		: Direction of rotation of motor on cage.
							'pwm_limit' (float)		: Maximum PWM value for the motors.
							'min_force' (float)		: Minimal target force.			
							'max_force' (float)		: Maximal target force.			
							'freq' (int)			: Frequency of the control loop.
							'load_cell_conf' (dict)	: Load cell configuration.
			init_force (float): Initial target force.
			test (bool): Test mode flag.
		
		"""
		self.id = conf['controller_id']
		self.kp = conf['kp']
		self.ki = conf['ki']
		self.kd = conf['kd']
		self.direction = conf['direction']
		self.pwm_limit = conf['pwm_limit']
		self.frequency = conf['freq']
		self.min_force = conf['min_force']
		self.max_force = conf['max_force']
		self.test = test
		self.init_force = init_force

		self.ready = False
		self.detached = False

		self.force_sensor = LoadCell(conf['load_cell_conf'])
		self.force_sensor.setOnAttachHandler(self.onAttach)
		self.force_sensor.setOnDetachHandler(self.onDetach)

		self.reset()

	@property
	def target_force(self):
		return self._target_force

	@target_force.setter
	def target_force(self, value):
		if value > self.max_force:
			value = self.max_force
		elif value < self.min_force:
			value = self.min_force
		self.pid.setpoint = value
		self._target_force = value

	def reset(self):
		self.pid = PID(self.kp, self.ki, self.kd, sample_time=1/self.frequency, output_limits=(-self.pwm_limit, self.pwm_limit))

		self.target_force = self.init_force
		self.pwm_set_point = 0

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

		if self.target_force < 1 and current_force < 1:
			return 0.0

		self.pwm_set_point += self.pid(current_force) * self.direction
		
		if self.pwm_set_point > self.pwm_limit:
			self.pwm_set_point = self.pwm_limit
		elif self.pwm_set_point < -self.pwm_limit:
			self.pwm_set_point = -self.pwm_limit

		if self.test:
			rospy.loginfo(f"Tendon [{self.id}] target [{self.target_force:4.2f}] actual [{current_force: 4.2f}] pwm [{self.pwm_set_point:.0f}]")

		return self.target_force, current_force, self.pwm_set_point, self.pid.components
		
class ForceControl:
	"""This class handles the force control ROS node.

	"""
	def __init__(self, test_tendon=None, test_force=None):
		"""
		Args:
			test_tendon (int): If given only that tendon will be started for test.
			test_force (float): If test_tendon is given this will be the force to test.
		
		"""
		self.refresh_rate = rospy.get_param('refresh_rate', None)
		
		if self.refresh_rate is None:
			error = "refresh_rate not set"
			rospy.logerr(f"Cannot start node, {error}!")
			rospy.signal_shutdown(error)
		else:
			self.started = False

			if test_tendon is not None:
				self.controllers = [TendonForceController(conf, init_force=test_force, test=True) for conf in self.get_controllers_conf() if conf['controller_id'] == test_tendon]
				assert len(self.controllers) == 1, f"{len(self.controllers)} tendons with id {test_tendon} found in parameter file."

				# testing publisher
				self.target_pub = rospy.Publisher("/target_force", Float32, queue_size=1)
				self.actual_pub = rospy.Publisher("/actual_force", Float32, queue_size=1)
				self.control_pub = rospy.Publisher("/control_value", Float32, queue_size=1)
				self.p_value_pub = rospy.Publisher("/p_value", Float32, queue_size=1)
				self.i_value_pub = rospy.Publisher("/i_value", Float32, queue_size=1)
				self.d_value_pub = rospy.Publisher("/d_value", Float32, queue_size=1)
			else:
				self.controllers = [TendonForceController(conf) for conf in self.get_controllers_conf()]

			rospy.Subscriber(Topics.TARGET_FORCE, TendonUpdate, self.set_target_force)

			self.start_controllers()
			self.init_roboy_plexus()

			rospy.Service(Services.START_FORCE_CONTROL, Trigger, self.start_node)
			rospy.Service(Services.STOP_FORCE_CONTROL, Trigger, self.stop_node)
			
			if test_tendon is not None:
				self.start_node(None)

	def get_controllers_conf(self):
		"""Reads node's parameters and builds list of configuration dictionaries.

		Args:
			-

		Returns:
			(list[dict]): List of controllers configurations.
		
		"""
		controllers_id = rospy.get_param('controllers_id')
		serials = rospy.get_param('serials')
		assert len(serials) == len(controllers_id)
		channels = rospy.get_param('channels')
		assert len(channels) == len(controllers_id)
		direction = rospy.get_param('direction')
		assert len(direction) == len(controllers_id)
		cal_offsets = rospy.get_param('cal_offset')
		cal_factors = rospy.get_param('cal_factor')
		kp = rospy.get_param('kp')
		ki = rospy.get_param('ki')
		kd = rospy.get_param('kd')
		pwm_limit = rospy.get_param('pwm_limit')
		min_force = rospy.get_param('min_force')
		max_force = rospy.get_param('max_force')
		if len(kp) == 1: kp *= len(controllers_id)
		if len(ki) == 1: ki *= len(controllers_id)
		if len(kd) == 1: kd *= len(controllers_id)
		if len(direction) == 1: direction *= len(controllers_id)
		if len(pwm_limit) == 1: pwm_limit *= len(controllers_id)
		if len(min_force) == 1: min_force *= len(controllers_id)
		if len(max_force) == 1: max_force *= len(controllers_id)

		load_cells_conf = [{'tendon_id': i, 'cal_offset': cal_offsets[f"p{s}"][c], 'cal_factor': cal_factors[f"p{s}"][c], 'serial': s, 'channel': c} for i, s, c in zip(controllers_id, serials, channels)]

		return [{'controller_id': index, 'kp': p, 'ki': i , 'kd': d, 'direction': di, 'pwm_limit': l, 'min_force': minf, 'max_force': maxf, 'load_cell_conf': conf, 'freq': self.refresh_rate} for index, p, i, d, di, l, minf, maxf, conf in zip(controllers_id, kp, ki, kd, direction, pwm_limit, min_force, max_force, load_cells_conf)]
		
	def init_roboy_plexus(self):
		"""Initiliazes objects to interface with the roboy plexus.

		Args:
			-

		Returns:
			-
		
		"""
		self.motor_command_publisher = rospy.Publisher(Topics.MOTOR_COMMAND, MotorCommand, queue_size=1)
		self.control_mode_client = rospy.ServiceProxy(Services.CONTROL_MODE, ControlMode)
		self.set_control_mode(3)

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

	def start_node(self, _):
		"""Starts control loop.

		Args:
			-

		Returns:
			-
		
		"""
		if not self.started:
			rospy.loginfo("Starting Force Control node...")
			self.started = True
			self.start_controllers()
			self.loop_timer = rospy.Timer(rospy.Duration(1/self.refresh_rate), self.control_loop)
			rospy.loginfo("Force Control node started!")

		return TriggerResponse(True, "Force Control node started")

	def stop_node(self, _):
		"""Stops control loop.

		Args:
			-

		Returns:
			-
		
		"""
		if self.started:
			rospy.loginfo("Stopping Force Control node...")
			self.started = False
			self.loop_timer.shutdown()

			for controller in self.controllers: controller.reset()
			self.send_motor_commands()
			rospy.loginfo("Force Control node stopped!")

		return TriggerResponse(True, "Force Control node stopped")

	def get_controller(self, controller_id):
		"""Returns controller with requested id.

		Args:
			controller_id (int): Id of desired controller.

		Returns:
			(TendonForceController): Contoller object.
		
		"""
		controller = [cont for cont in self.controllers if cont.id == controller_id]
		return controller[0] if controller else None

	def set_target_force(self, msg):
		"""Set tendon target force callback.

		Args:
			msg (TendonUpdate): Received ROS Message.

		Returns:
			-
		
		"""
		if not self.started:
			return
		
		controller = self.get_controller(msg.tendon_id)
		if controller is None:
			rospy.logwarn(f"Trying to set target force for tendon ({msg.tendon_id}) without controller.")
		else:
			if msg.force < 0:
				rospy.logwarn(f"Trying to set negative target force for tendon ({msg.tendon_id}).")
			else:
				controller.target_force = msg.force

	def control_loop(self, event):
		"""Control loop.

		Args:
			-

		Returns:
			-
		
		"""
		motor_ids = []
		set_points = []
		for controller in self.controllers:
			if controller.ready:
				motor_ids.append(controller.id)
				target, actual, control, (p_value, i_value, d_value) = controller.getPwmSetPoint()

				if controller.test:
					self.target_pub.publish(target)
					self.actual_pub.publish(actual)
					self.control_pub.publish(control)
					self.p_value_pub.publish(p_value)
					self.i_value_pub.publish(i_value)
					self.d_value_pub.publish(d_value)

				set_points.append(control)

		self.send_motor_commands(motor_ids, set_points)

	def set_control_mode(self, mode, motor_ids=None, set_points=None):
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
		if motor_ids is None:
			motor_ids = [controller.id for controller in self.controllers]
		if set_points is None:
			set_points = [0] * len(motor_ids)

		assert len(motor_ids) == len(set_points)

		try:
			rospy.wait_for_service(Services.CONTROL_MODE, timeout=1.0)
			self.control_mode_client(ControlModeRequest(False, mode, set_points, motor_ids))
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

		assert len(motor_ids) == len(set_points)

		self.motor_command_publisher.publish(MotorCommand(False, motor_ids, set_points))


if __name__ == "__main__":

	parser = argparse.ArgumentParser(description="Force control node.")
	parser.add_argument('-t', '--test', dest='test_tendon', help='Id of tendon to test', type=int)
	parser.add_argument('-f', '--force', dest='test_force', default=0.0, help='Initial tendon force to test', type=float)
	args = parser.parse_args()

	rospy.init_node("force_control", disable_signals=True)
	force_control = ForceControl(args.test_tendon, args.test_force)
	rospy.spin()

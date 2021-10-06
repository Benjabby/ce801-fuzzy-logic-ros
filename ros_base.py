#!/usr/bin/env python

#### ROS imports ####
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
### Other imports ###
from functools import partial


class RosBase(object):
	'''
	A base class for controllers
	
	Parameters:
		name:		The name of the ros node
		suppress:	If true, this controller only calculates an output without publishing
		rate:		Rate in Hz to execute the controll loop 
	'''
	def __init__(self, name, suppress=False, rate=10):
		if not suppress:
			rospy.init_node(name, anonymous=True)
		self.rate = rate
		self.suppress = suppress
		self.reset()
		
	def reset(self):
		'''
		Resets the controller
		'''
		# Reset/init subscriber and publisher
		self.RATE = rospy.Rate(self.rate)
		self.PUB = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.SUB = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
		# Reset/init current message
		self.processed_inputs = {}
		self.outputs = {'move':0,'turn':0}
		self.new_msg = False
		self.end = False
		

	def _reset_world(self):
		'''
		Stops the robot and resets the environment
		'''
		# Stop wheels if they're running
		wheels = Twist()
		self.PUB.publish(wheels)
		# Reset Gazebo
		rospy.wait_for_service('/gazebo/reset_world')
		reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
		reset_world()

	def laser_callback(self, msg):	
		'''
		callback for the LaserScan subscriber.
		Calls _process_inputs and sets new_msg to true
		
		Inputs:
			msg (LaserScan):	The message from LaserScan
		'''
		self.processed_inputs = self._process_inputs(msg)
		self.new_msg = True
		
		

	def start(self, reset=False):
		'''
		Starts the controller running.
		The world resets don't work.
		'''
		# Resets the world
		if reset:
			self._reset_world()
		try:
			while not (rospy.is_shutdown() or self.end):
				if self.processed_inputs:
					self._control_loop()
					self.new_msg = False
					if not self.suppress:
						self._publish()
				else:
					self.RATE.sleep()
			#self._reset_world()
		except rospy.ROSInterruptException:
			#self._reset_world()
			pass
		
	
	def _process_inputs(self, msg):
		'''
		Abstract function that should be overridded to process laser scan inputs
		
		Inputs:
			msg (LaserScan): 	The message from LaserScan
		Outputs:
			(dict):			The processed outputs
		'''
		return None

	def _control_loop(self):
		'''
		Abstract function that should be overridded to run the main process loop
		'''
		pass
	
	def _publish(self):
		'''
		Function that could be overridded. publishes output to publisher node
		'''
		wheels = Twist()
		wheels.linear.x = self.outputs['move']
		wheels.angular.z = self.outputs['turn']
		self.PUB.publish(wheels)

#!/usr/bin/env python

import rospy
from ros_base import *
from fuzzy_logic import *
import numpy as np
from functools import partial
import sys

class FuzzyWall(RosBase):
	def __init__(self, name, **kwargs):
		super(self.__class__, self).__init__(name, **kwargs)
		# Create fuzzy logic system
		self.logic = FuzzyLogicSystem()

		# Define input variable 1
		self.logic.add_input('distance', 0.2, 12)
		# Define input sets for variable 1
		self.logic.add_input_set('distance', 'near',	m_linear,	0.0,	0.0,	0.02,	0.035)
		self.logic.add_input_set('distance', 'okay',	m_linear,	0.02,	0.05,	0.07,	0.1)
		self.logic.add_input_set('distance', 'far',	m_linear,	0.085,	0.1,	1,	1)

		# Define input variable 2
		self.logic.add_input('angle', 0, np.pi)
		# Define input sets for variable 2
		self.logic.add_input_set('angle', 'behind', 	m_linear,	0.0,	0.0,	0.3,	0.5)
		self.logic.add_input_set('angle', 'right', 	m_linear,	0.3,	0.4,	0.6,	0.7)
		self.logic.add_input_set('angle', 'infront', 	m_linear,	0.5,	0.7,	1,	1)
		
	# Define output variable 1
		self.logic.add_output('turn', -2,2)
		# Define output sets for variable 1
		self.logic.add_output_set('turn', 'left',		m_linear,	0,	0,	0.2,	0.45)
		self.logic.add_output_set('turn', 'straight',		m_linear,	0.35,	0.5,	0.5,	0.65)
		self.logic.add_output_set('turn', 'right',		m_linear,	0.55,	0.8,	1,	1)


		# Define rule base
		self.logic.add_rule("If distance is near or angle is infront then turn left")
		self.logic.add_rule("If distance is okay or angle is right then turn straight")
		self.logic.add_rule("If distance is slightly far or angle is slightly behind then turn right")
		
		

	# Override from RosBase
	def _process_inputs(self, msg):
		
		# All laser readings from the right are used
		readings = np.array(msg.ranges[360:])
		
		# To mitigate noise, include the two readings either side
		#where = 0 if np.min(readings) == np.inf else np.argmin(readings)
		#left, right = msg.ranges[(360+where+1)%720], msg.ranges[360+where-1]
		#dist = (dist+left+right)/3.0
		
		# Softmin of distances
		er = np.exp(-readings)
		soft = er/np.sum(er)
		readings = np.clip(readings, msg.range_min, msg.range_max)
		
		dist, angle = self._methodA(soft, readings)
	
		return {'distance':dist,'angle':angle}

	#### Softmin methods
	def _methodA(self, soft, readings):
		dist = np.sum(soft*readings)
		angles = np.linspace(0,np.pi,360,endpoint=False)
		angle = np.sum(soft*angles)
		return dist, angle

	def _methodB(self, soft, readings):
		angles = np.linspace(0,np.pi,360,endpoint=False)
		dist = np.sum(soft*readings)
		cosa = np.sum(np.cos(angles)*soft)
		sina = np.sum(np.sin(angles)*soft)
		angle = np.atan2(sina,cosa)
		
		return dist, angle
		
	def _methodC(self, soft, readings):
		angles = np.linspace(0,np.pi,360,endpoint=False)
		dist = np.sum(soft*readings)
		cosa = np.sum(np.cos(angles)*soft)
		sina = np.sum(np.sin(angles)*soft)
		x = cosa*dist
		y = sina*dist
		angle = np.atan2(y,x)
		dist = np.sqrt(x*x+y*x)
		return dist, angle

	def _methodD(self, soft, readings):
		angles = np.linspace(0,np.pi,360,endpoint=False)
		dist = np.sum(soft*readings)
		cosa = np.sum(np.cos(angles)*soft)
		angle = np.arccos(cosa)
		return dist, angle

	def _methodE(self, soft, readings):
		angles = np.linspace(0,np.pi,360,endpoint=False)
		x = np.sum(np.cos(angles)*readings*soft)
		y = np.sum(np.sin(angles)*readings*soft)
		angle = np.atan2(y,x)
		dist = np.sqrt(x*x+y*x)
		return dist, angle

	# Override from RosBase
	def _control_loop(self):
		self.outputs = self.logic.infer(self.processed_inputs)
		self.outputs['turn'] = -self.outputs['turn']
		self.outputs['move'] = 0.6
		

if __name__ == '__main__':
	try:
		controller = FuzzyWall("fuzzy_wall_follow")
		if len(sys.argv)>1 and sys.argv[1]=='di':
			controller.logic.draw_inputs()
		elif len(sys.argv)>1 and sys.argv[1]=='do':
			controller.logic.draw_outputs()
		else:	
			controller.start(reset=(len(sys.argv)>1 and sys.argv[1]=='reset'))
	except rospy.ROSInterruptException:		
		pass


'''
##### Distance
		# Front right quadrant is used for distance so that corners are properly accounted for distance-wise
		fr = np.array(msg.ranges[539:])
		dist = np.min(fr)
		# To mitigate noise, include the two readings either side
		where = np.argmin(fr)
		left, right = msg.ranges[(539+where+1)%720], msg.ranges[539+where-1]
		dist = (dist+left+right)/3.0
		
		##### Angle
		#All laser readings from the right are used for the angle
		r = np.array(msg.ranges[360:])
		where = 0 if np.min(r) == np.inf else np.argmin(r)
		# Get angle
		angle = (np.pi)*(where/float(len(r)))-np.pi/2
'''

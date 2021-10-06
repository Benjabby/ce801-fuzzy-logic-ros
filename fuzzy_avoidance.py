#!/usr/bin/env python

import rospy
from ros_base import *
from fuzzy_logic import *
import numpy as np
from functools import partial
import sys

class FuzzyAvoid(RosBase):
	def __init__(self, name, **kwargs):
		super(self.__class__, self).__init__(name, **kwargs)
		# Create fuzzy logic system
		self.logic = FuzzyLogicSystem()

		# Define input variable 1
		self.logic.add_input('front right', 0.2, 12)
		# Define input sets for variable 1
		self.logic.add_input_set('front right', 'near',	m_linear,	0.0,	0.0,	0.03,	0.05)
		self.logic.add_input_set('front right', 'okay',	m_linear,	0.04,	0.06,	1,	1)

		# Define input variable 2
		self.logic.add_input('front left', 0.2, 12)
		# Define input sets for variable 2
		self.logic.add_input_set('front left', 'near',	m_linear,	0.0,	0.0,	0.03,	0.05)
		self.logic.add_input_set('front left', 'okay',	m_linear,	0.04,	0.06,	1,	1)

		# Define input variable 3
		self.logic.add_input('behind', 0.2, 12)
		# Define input sets for variable 3
		self.logic.add_input_set('behind', 'near',	m_linear,	0.0,	0.0,	0.02,	0.04)
		self.logic.add_input_set('behind', 'okay',	m_linear,	0.03,	0.06,	1,	1)

		# Define output variable 1
		self.logic.add_output('turn', -2,2)
		# Define output sets for variable 1
		self.logic.add_output_set('turn', 'left',		m_linear,	0,	0,	0.2,	0.45)
		self.logic.add_output_set('turn', 'straight',		m_linear,	0.35,	0.5,	0.5,	0.65)
		self.logic.add_output_set('turn', 'right',		m_linear,	0.55,	0.8,	1,	1)


		# Define output variable 2
		self.logic.add_output('move', -0.4,0.8)
		# Define output sets for variable 2
		self.logic.add_output_set('move', 'backward',	m_linear,	-0.4,	-0.4,	-0.3,	0.0, 	pre_ranged=True)
		self.logic.add_output_set('move', 'still',	m_linear,	-0.1,	0.0,	0.0,	0.1, 	pre_ranged=True)
		self.logic.add_output_set('move', 'slow',	m_linear,	0,	0.3,	0.4,	0.7, 	pre_ranged=True)
		self.logic.add_output_set('move', 'fast',	m_linear,	0.6,	0.7,	0.8,	0.8, 	pre_ranged=True)


		# Define rule base
		self.logic.add_rule("if front right is near and front left is near and behind is okay then move backward and turn left")
		self.logic.add_rule("if front right is near and front left is near and behind is near then move still and turn left")
		self.logic.add_rule("if front right is okay and front left is okay then move fast and turn straight")
		self.logic.add_rule("if front right is near and front left is okay then move slow and turn left")
		self.logic.add_rule("if front right is okay and front left is near then move slow and turn right")

	# Override from RosBase
	def _process_inputs(self, msg):
		# Front Right
		fr = np.array(msg.ranges[539:])
		er = np.exp(-fr)
		s = np.sum(er) 
		soft = er/s if s !=0 else 0
		fr = np.clip(fr, msg.range_min, msg.range_max)
		fr = np.sum(fr*soft)
		# Front Left
		fl = np.array(msg.ranges[:181])
		er = np.exp(-fl)
		s = np.sum(er) 
		soft = er/s if s !=0 else 0
		fl = np.clip(fl, msg.range_min, msg.range_max)
		fl = np.sum(fl*soft)
		# Behind
		b = np.array(msg.ranges[270:450])
		er = np.exp(-b)
		s = np.sum(er) 
		soft = er/s if s !=0 else 0
		b = np.clip(b, msg.range_min, msg.range_max)
		b = np.sum(b*soft)

		return {'front right':fr, 'front left':fl, 'behind':b}
		
		'''
		# right readings
		right = np.array(msg.ranges[360:])
		
		# Softmin of right
		er = np.exp(-right)
		soft = er/np.sum(er)
		right = np.clip(right, msg.range_min, msg.range_max)
		
		rdist, rangle = self._methodA(soft, right)


		# left readings
		left = np.array(msg.ranges[:360])
		
		# Softmin of left
		er = np.exp(-left)
		soft = er/np.sum(er)
		left = np.clip(left, msg.range_min, msg.range_max)

		ldist, langle = self._methodA(soft, left)

		return {'left distance':ldist,'right distance':rdist, 'left angle':langle, 'right angle':rangle}
		'''
	
	'''
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
	'''

	# Override from RosBase
	def _control_loop(self):
		self.outputs = self.logic.infer(self.processed_inputs)
		self.outputs['turn'] = -self.outputs['turn'] 
		
		

if __name__ == '__main__':
	try:
		controller = FuzzyAvoid("fuzzy_avoidance")
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

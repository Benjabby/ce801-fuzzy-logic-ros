#!/usr/bin/env python

import rospy
from ros_base import *
from fuzzy_logic import *
from fuzzy_avoidance import FuzzyAvoid
from fuzzy_wall_following import FuzzyWall
import numpy as np
from functools import partial
import sys

class FuzzyUltimate(RosBase):
	def __init__(self, name):
		super(self.__class__, self).__init__(name)

		self.wf = FuzzyWall("fuzzy_wall_follow",suppress=True)
		self.wf_func = lambda x: np.maximum(0.0, m_linear(1,2,12,12,x))
		self.av = FuzzyAvoid("fuzzy_avoidance",suppress=True)
		self.av_func = lambda x: m_linear(0,0,1,2,x)


	# Override from RosBase
	def _process_inputs(self, msg):
		
		self.wf._process_inputs(msg)
		self.av._process_inputs(msg)
		
		front = np.concatenate([np.array(msg.ranges[629:]),np.array(msg.ranges[:91])])
		er = np.exp(-front)
		s = np.sum(er) 
		soft = er/s if s !=0 else 0
		front = np.clip(front, msg.range_min, msg.range_max)
		front = np.sum(front*soft)

		fwf = self.wf_func(front)
		fav = self.av_func(front)

		return {'fwf':fwf, 'fav':fav}

	# Override from RosBase
	def _control_loop(self):
		self.wf._control_loop()
		wf_out = self.wf.outputs
		self.av._control_loop()
		av_out = self.av.outputs
		
		fwf = self.processed_inputs['fwf']
		fav = self.processed_inputs['fav']
		
		self.outputs['move'] = (fwf*wf_out['move']+fav*av_out['move'])/(fwf+fav)
		self.outputs['turn'] = (fwf*wf_out['turn']+fav*av_out['turn'])/(fwf+fav)

		#print("FWF: {}".format(fwf/(fwf+fav)))
		#print("FAV: {}".format(fav/(fwf+fav)))
		

if __name__ == '__main__':
	try:
		controller = FuzzyUltimate("fuzzy_ultimate")
		if len(sys.argv)>1 and sys.argv[1]=='di':
			controller.logic.draw_inputs()
		elif len(sys.argv)>1 and sys.argv[1]=='do':
			controller.logic.draw_outputs()
		else:	
			controller.start(reset=(len(sys.argv)>1 and sys.argv[1]=='reset'))
	except rospy.ROSInterruptException:		
		pass



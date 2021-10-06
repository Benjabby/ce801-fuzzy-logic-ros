#!/usr/bin/env python

import rospy
from ros_base import *
from fuzzy_logic import *
import numpy as np
from functools import partial
import warnings
import sys

SHOW_TIME = True

class PIDController(RosBase):
	def __init__(self, ID='', P=0.0, I=0.0, D=0.0, turn_rate=4.0, dist_mode='linear', d_ignore=0.05, integral_cap=1, integral_reset=False, only_new_d=True, slow_base_speed=False, desired_distance=0.06779661016, base_speed=0.6, max_speed=0.8, test_mode=False):
		super(self.__class__, self).__init__("PID_controller"+'_'+str(ID))
		self.ID = ID
		self.P = P
		self.I = I
		self.D = D
		self.turn_rate = turn_rate
		self.dist_mode = dist_mode
		self.d_ignore = d_ignore
		self.integral_cap = integral_cap
		self.integral_reset = integral_reset
		self.only_new_d = only_new_d
		self.slow_base_speed = slow_base_speed	
		self.desired_distance = desired_distance
		self.base_speed = base_speed
		self.max_speed = max_speed

		self.e = {"P":0,"I":0,"D":0,"prev":0}

		self.test_mode = test_mode
		self.ready = False

		if test_mode:
			self.plot = {'dist':[], 'sp':[], 'P':[],'I':[],'D':[],'total':[]}

	# Override from RosBase
	def _process_inputs(self, msg):
		
		# All laser readings from the right to the front (front right quadrant)
		readings = np.array(msg.ranges[539:])
		
		ndist = np.min(readings)
		# To mitigate noise, include the two readings either side
		where = np.argmin(readings)
		left = msg.ranges[(539+where+1)%720]
		right = msg.ranges[539+where-1]

		ndist = (ndist+left+right)/3

		# scale between laser's max and min values
		# doesn't matter that this is done after finding the minimum as its a linear transformation
		ndist = (ndist-msg.range_min)/(msg.range_max-msg.range_min)
		# clip
		ndist = np.clip(ndist,0.0,1.0)

		if self.test_mode:
			self.plot['dist'].append(ndist) # Add distance to plot before applying any non-linear scaling

		
		if self.dist_mode == 'angular':
			# Adjusts for the angle that the shortest distance was detected on. 
			# If  the shortest distance is in front of the robot it needs to have a greater response.
			# multiplies the distance by the cosine of the angle starting at the rightmost sensor.
			angle = (np.pi/2.0)*(where/float(len(readings)))
			ndist = np.cos(angle)*ndist


		speed = np.cos(angle)*self.max_speed if self.slow_base_speed else self.base_speed

		#print(ndist)
		
		return {'distance':ndist, 'speed':speed}

	
	# Override from RosBase
	def _control_loop(self):

		self.e['P'] = self.desired_distance-self.processed_inputs['distance']
		#print(self.new_msg)

		if self.dist_mode == 'piecewise':
			if self.e['P']<0: 
				self.e['P'] /= 1-self.desired_distance
			else:
				self.e['P'] /= self.desired_distance
			
		self.e['I'] = np.clip(self.e['I']+self.e['P'],-self.integral_cap,self.integral_cap) # Clamp integral error
		
		# reset integral on zero error crossing
		if self.integral_reset and (np.sign(self.e['P']) != np.sign(self.e['prev']) or self.e['P']==0):
			self.e['I'] = 0
		
		if (self.new_msg or not self.only_new_d) and self.ready:
			self.e['D'] = self.e['P']-self.e['prev']
			if np.abs(self.e['D'])>=self.d_ignore:
				self.e['D'] = 0

		self.e['prev'] = self.e['P']


		epk = self.e['P']*self.P
		eik = self.e['I']*self.I
		edk = self.e['D']*self.D
		total = epk+eik+edk
		total = max(-1,min(1,total)) # Clamp the final output

		if self.test_mode:
			self.plot['P'].append(epk)
			self.plot['I'].append(eik)
			self.plot['D'].append(edk)
			self.plot['total'].append(total)
		
		self.outputs = {'move':self.processed_inputs['speed'], 'turn':total*self.turn_rate}
		

	def _publish(self):
		if self.ready:
			super(self.__class__, self)._publish()
		self.ready = True

tests = [
{'ID':0,'P':0.05},
{'ID':1,'P':0.1},
{'ID':2,'P':0.2},
{'ID':3,'P':0.4},
{'ID':4,'P':0.6},
{'ID':5,'P':0.8},
{'ID':6,'P':1.0},
{'ID':7,'P':1.2},
{'ID':8,'P':1.4},
{'ID':9,'P':1.6},
{'ID':10,'P':1.8},
{'ID':11,'P':2.0},
{'ID':12,'P':2.2},
{'ID':13,'P':2.4},
{'ID':14,'P':2.6},
{'ID':15,'P':2.8},
{'ID':16,'P':3.0},
{'ID':17,'P':3.2},
{'ID':18,'P':3.4},
{'ID':19,'P':3.6},
{'ID':20,'P':3.8},
{'ID':21,'P':4.0},
{'ID':22,'dist_mode':'piecewise', 'P':0.02}, # piecewise not great
{'ID':23,'dist_mode':'piecewise', 'P':0.05}, # piecewise not great
{'ID':24,'dist_mode':'piecewise', 'P':0.1}, # piecewise not great
{'ID':25,'dist_mode':'angular', 'P':0.05},
{'ID':26,'dist_mode':'angular', 'P':0.1},
{'ID':27,'dist_mode':'angular', 'P':0.2},
{'ID':28,'dist_mode':'angular','P':0.4},
{'ID':29,'dist_mode':'angular','P':0.6},
{'ID':30,'dist_mode':'angular','P':0.8},
{'ID':31,'dist_mode':'angular','P':1.0},
{'ID':32,'dist_mode':'angular','P':1.2},
{'ID':33,'dist_mode':'angular','P':1.4},
{'ID':34,'dist_mode':'angular','P':1.6},
{'ID':35,'dist_mode':'angular','P':1.8},
{'ID':36,'dist_mode':'angular','P':2.0},
{'ID':37,'dist_mode':'angular','P':2.2},
{'ID':38,'dist_mode':'angular','P':2.4},
{'ID':39,'dist_mode':'angular','P':2.6},
{'ID':40,'dist_mode':'angular','P':2.8},
{'ID':41,'dist_mode':'angular','P':3.0},
{'ID':42,'dist_mode':'angular','P':3.2},
{'ID':43,'dist_mode':'angular','P':3.4},
{'ID':44,'dist_mode':'angular','P':3.6},
{'ID':45,'dist_mode':'angular','P':3.8},
{'ID':46,'dist_mode':'angular','P':4.0},
{'ID':47,'dist_mode': 'linear','P':1.92,'I':0.265,'D':3.48,'integral_reset':False},
{'ID':48,'dist_mode': 'linear','P':1.92,'I':0.265,'D':3.48, 'integral_reset':True},
{'ID':49,'dist_mode': 'linear','P':1.92,'I':0.265,'D':3.48, 'integral_cap':0.2,'integral_reset':False},
{'ID':50,'dist_mode': 'linear','P':1.92,'I':0.265,'D':3.48, 'integral_cap':0.2, 'integral_reset':True},
{'ID':51,'dist_mode': 'angular','P':1.08,'I':0.036,'D':8.1,'integral_reset':False},
{'ID':52,'dist_mode': 'angular','P':1.08,'I':0.036,'D':8.1, 'integral_reset':True}, 				# Good!
{'ID':53,'dist_mode': 'angular','P':1.08,'I':0.036,'D':8.1, 'integral_cap':0.2,'integral_reset':False},
{'ID':54,'dist_mode': 'angular','P':1.08,'I':0.036,'D':8.1, 'integral_cap':0.2, 'integral_reset':True}, 	# Good!
{'ID':55,'dist_mode': 'linear','P':0.1,'I':0.05,'D':0.0,'integral_reset':True},
{'ID':56,'dist_mode': 'linear','P':0.1,'I':0.1,'D':0.0,'integral_reset':True},
{'ID':57,'dist_mode': 'linear','P':0.1,'I':0.2,'D':0.0,'integral_reset':True},
{'ID':58,'dist_mode': 'linear','P':0.1,'I':0.05,'D':0.0,'integral_reset':False},
{'ID':59,'dist_mode': 'linear','P':0.1,'I':0.1,'D':0.0,'integral_reset':False},
{'ID':60,'dist_mode': 'linear','P':0.1,'I':0.2,'D':0.0,'integral_reset':False},
{'ID':61,'dist_mode': 'angular','P':2.16,'I':0.072,'D':16.2, 'integral_cap':0.2, 'integral_reset':True, 'turn_rate':2.0},
{'ID':62,'dist_mode': 'linear', 'P':3.2,'I':0.01,'D':1.0, 'integral_cap':0.2, 'integral_reset':True, 'turn_rate':4.0},
{'ID':63,'dist_mode': 'angular','P':4.32,'I':0.144,'D':32.4, 'integral_cap':0.2, 'integral_reset':True, 'turn_rate':1.0},
{'ID':64,'dist_mode': 'linear', 'P':3.0,'I':0.1,'D':4.0, 'integral_cap':0.2, 'integral_reset':True, 'turn_rate':4.0},
{'ID':65,'dist_mode': 'linear','P':1.5,'I':0.05,'D':10,'integral_cap':1, 'turn_rate':4.0,'integral_reset':True}, # Good! Still takes corners very wide
{'ID':66,'dist_mode': 'linear','P':1.08,'I':0.036,'D':8.1, 'integral_cap':0.2, 'integral_reset':True, 'turn_rate':4.0}, # Good!
{'ID':67,'dist_mode': 'angular','P':0.98,'I':0.036,'D':8.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':68,'dist_mode': 'angular','P':1.18,'I':0.036,'D':8.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':69,'dist_mode': 'angular','P':1.08,'I':0.026,'D':8.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':70,'dist_mode': 'angular','P':1.08,'I':0.046,'D':8.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':71,'dist_mode': 'angular','P':1.08,'I':0.036,'D':7.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':72,'dist_mode': 'angular','P':1.08,'I':0.036,'D':9.1, 'integral_cap':0.2, 'integral_reset':True}, # Good!
{'ID':73,'dist_mode': 'linear','P':2.0,'I':0.05,'D':10,'integral_cap':1, 'turn_rate':4.0,'integral_reset':True},
{'ID':74,'dist_mode': 'linear','P':3.0,'I':0.05,'D':10,'integral_cap':1, 'turn_rate':4.0,'integral_reset':True},
{'ID':75,'dist_mode': 'linear','P':1.5,'I':0.1,'D':10,'integral_cap':1, 'turn_rate':4.0,'integral_reset':True},
{'ID':76,'dist_mode': 'linear','P':0.98,'I':0.036,'D':8.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':77,'dist_mode': 'linear','P':1.18,'I':0.036,'D':8.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':78,'dist_mode': 'linear','P':1.08,'I':0.026,'D':8.1, 'integral_cap':0.2, 'integral_reset':True}, # Very Good!
{'ID':79,'dist_mode': 'linear','P':1.08,'I':0.046,'D':8.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':80,'dist_mode': 'linear','P':1.08,'I':0.036,'D':7.1, 'integral_cap':0.2, 'integral_reset':True},
{'ID':81,'dist_mode': 'linear','P':1.08,'I':0.036,'D':9.1, 'integral_cap':0.2, 'integral_reset':True}, # Very Good!
{'ID':82,'dist_mode': 'angular','P':1.1,'I':0.03,'D':10, 'integral_cap':0.2, 'integral_reset':True}, # Veryish Good!
{'ID':83,'dist_mode': 'linear','P':0.98,'I':0.036,'D':8.1, 'integral_cap':1, 'integral_reset':True},
{'ID':84,'dist_mode': 'linear','P':1.18,'I':0.036,'D':8.1, 'integral_cap':1, 'integral_reset':True},
{'ID':85,'dist_mode': 'linear','P':1.08,'I':0.026,'D':8.1, 'integral_cap':1, 'integral_reset':True},
{'ID':86,'dist_mode': 'linear','P':1.08,'I':0.046,'D':8.1, 'integral_cap':1, 'integral_reset':True},
{'ID':87,'dist_mode': 'linear','P':1.08,'I':0.036,'D':7.1, 'integral_cap':1, 'integral_reset':True},
{'ID':88,'dist_mode': 'linear','P':1.08,'I':0.036,'D':9.1, 'integral_cap':1, 'integral_reset':True},
{'ID':89,'dist_mode': 'angular','P':1.1,'I':0.03,'D':10, 'integral_cap':1, 'integral_reset':True},
{'ID':90,'dist_mode': 'linear','P':1.1,'I':0.026,'D':10, 'integral_cap':0.25, 'integral_reset':True}, # Very Good!
{'ID':91,'dist_mode': 'linear','P':0.9,'I':0.046,'D':9, 'integral_cap':1, 'integral_reset':True},
]

if __name__ == '__main__':
	try:
		reset = "reset" in sys.argv
		if len(sys.argv)<=1:
			controller = PIDController(P=1.1,I=0.026,D=10,integral_cap=0.25, integral_reset=True)
			controller.start(reset=reset)
		elif len(sys.argv)==2:
			controller = PIDController(**tests[int(sys.argv[1])])
			controller.start(reset=reset)
		elif len(sys.argv)==3:
			start = int(sys.argv[1])
			end = None if sys.argv[2]=='None' else int(sys.argv[2])
			T = tests[START:END]
			i = 0
			t = len(T)
			print("Running {} tests, ID's {} to {}".format(t,START,START+t))
			for test in T:
				controller = PIDController(**test)
				controller.start(reset=reset)
				i += 1
				print("Test {} of {} done".format(i,t))

	except rospy.ROSInterruptException:		
		pass



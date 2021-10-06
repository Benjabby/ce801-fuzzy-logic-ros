#!/usr/bin/env python

from ros_base import * 
import numpy as np
import matplotlib.pyplot as plt 
import string
import warnings
import re

#from inspect import signature # Not here since this is python 2
from inspect import getargspec
from functools import partial
from mpl_toolkits.mplot3d import Axes3D

########### MEMBERSHIP FUNCTIONS ###########
# x is last argument so other arguments can be bound
############################################
def m_linear(start, left_peak, right_peak, end, x): # Trapezoidal membership function
	if type(x) is np.ndarray:
		return np.piecewise(x, [x<start, (start<=x) & (x<left_peak), (left_peak<=x) & (x<=right_peak), (right_peak<x) & (x<end)], [lambda x: np.zeros_like(x), lambda x: (x-start)/float(left_peak-start), lambda x: np.ones_like(x), lambda x: (end-x)/float(end-right_peak), lambda x: np.zeros_like(x)])

	if x<start:
		return 0.0
	elif x<left_peak:
		return (x-start)/float(left_peak-start)
	elif x<=right_peak:
		return 1.0
	elif x<end:
		return (end-x)/float(end-right_peak)
	else:
		return 0.0

def m_bump(start, left_peak, right_peak, end, x):
	if type(x) is np.ndarray:
		return np.piecewise(x, [(x==0) & (left_peak<=0), x<=start, (start<x) & (x<=left_peak), (left_peak<x) & (x<=right_peak), (right_peak<x) & (x<=end)], [lambda x: np.ones_like(x), lambda x: np.zeros_like(x), lambda x: np.exp(1/(np.square((x-left_peak)/float(left_peak-start))-1))*np.e, lambda x: np.ones_like(x), lambda x: np.exp(1/(np.square((x-right_peak)/float(end-right_peak))-1))*np.e, lambda x: np.zeros_like(x)])

	if x==0 and left_peak<=0:
		return 1.0
	elif x<=start:
		return 0.0
	elif x<=left_peak:
		return np.exp(1/(np.square((x-left_peak)/float(left_peak-start))-1))*np.e
	elif x<=right_peak:
		return 1.0
	elif x<end:
		return np.exp(1/(np.square((x-right_peak)/float(end-right_peak))-1))*np.e
	else:
		return 0.0

#### Simple versions with 0 core ####
def m_simple_linear(start, peak, end, x): # Triangular membership function
	return m_linear(start,peak,peak,end,x)

def m_simple_bump(start, peak, end, x):
	return m_bump(start, peak, peak, end, x)

#### Symmetric versions ####
def m_sym_bump(peak, core_width, support_width, x):
	bottom = support_width/2.0
	top = core_width/2.0
	return m_bump(peak-bottom, peak-top, peak+top, peak+bottom, x)

def m_sym_linear(peak, peak_width, base_width, x):
	bottom = base_width/2.0
	top = peak_width/2.0
	return m_linear(peak-bottom, peak-top, peak+top, peak+bottom, x)
############################################
############ OPERATOR FUNCTIONS ############
def NOT(x):
	return 1.0-x

#### HEDGE OPERATORS ####
def VERY(x):
	return np.square(x)

def SLIGHTLY(x):
	return np.sqrt(x)

#### AND operators (t-norms) ####
def AND_min(x,y):
	return np.minimum(x,y)
def AND_prob(x,y):
	return x*y

#### OR operators (t-conorms) ####
def OR_max(x, y):
	return np.maximum(x,y)
def OR_prob(x, y):
	return x+y-x*y

#### THEN operators (Implication) ####
def THEN_min(function, strength, x):
	return np.minimum(function(x),strength)
def THEN_prod(function, strength, x):
	return function(x)*strength

#### Aggregate operators ####
def AGG_max(func_array):
	return np.amax(func_array,axis=0)
def AGG_sum(func_array):
	return np.sum(np.array(func_array),axis=0)

NP_PROBOR = np.frompyfunc(OR_prob,2,1)
def AGG_prob(func_array):
	return NP_PROBOR.reduce(func_array, axis=0)

#### Defuzzifiers ####
def DEFUZZ_centroid(x, y):
	numer = np.sum(x*y)
	denom = np.sum(y)
	if denom==0:
		return np.mean(x)
	return numer/denom
def DEFUZZ_bisector(x,y):
	t = np.sum(y)
	cs = np.cumsum(y)
	return x[np.argmax(y>=t/2.0)]
def DEFUZZ_midmax(x, y):
	return np.mean(x[x==np.max(x)])

############################################
############ FUZZY LOGIC SYSTEM ############
class FuzzyLogicSystem():
	def __init__(self, and_method=AND_min, or_method=OR_max, then_method=THEN_min, agg_method=AGG_max, defuzz_method=DEFUZZ_centroid, default_resolution=201, auto_clip=True):
		self.AND = and_method
		self.OR = or_method
		self.THEN = then_method
		self.AGG = agg_method
		self.DEFUZZ = defuzz_method
		self.resolution = default_resolution
		self.auto_clip = auto_clip

		self.inputs = {}
		self.outputs = {}
		self.rules = []
		self.linguistic_rules = []
		
	def print_rules(self,linguistic=True):
		R = self.linguistic_rules if linguistic else self.rules
		i = 1
		for r in R:
			print("Rule {}: {}".format(i,r))
			i+=1

	def draw_inputs(self, test_values=None):
		if len(self.inputs)==0:
			warnings.warn("draw_inputs called on FuzzyController with no inputs defined")
			return
		fig, axs = plt.subplots(len(self.inputs), 1)
		if not isinstance(axs,(list,np.ndarray)):
			axs = [axs]
		fig.canvas.set_window_title('Input Sets')
		a = 0
		for i in self.inputs:
			self.inputs[i].draw(axs[a], None if test_values is None else test_values[i])
			a += 1
		
		plt.show()

	def draw_outputs(self):
		if len(self.outputs)==0:
			warnings.warn("draw_outputs called on FuzzyController with no inputs defined")
			return
		fig, axs = plt.subplots(len(self.outputs), 1)
		if not isinstance(axs,(list,np.ndarray)):
			axs = [axs]
		fig.canvas.set_window_title('Output Sets')
		a = 0
		for i in self.outputs:
			self.outputs[i].draw(axs[a])
			a += 1
		plt.show()

	def draw_rule_strength_surface(self, rule_num):
		rule_num = rule_num-1
		if rule_num>=len(self.rules):
			warnings.warn("Rule {} out of range".format(rule_num))
			return

		rule = self.rules[rule_num]
		op, antecedent, consequent = rule
		
		if len(antecedent)==1:
			a, = antecedent
			x, y = self.inputs[a[0]].mapping(a[1])
			plt.plot(x,y)
			plt.show()
		elif len(antecedent)==2:
			a1, a2 = antecedent
			x, z1 = self.inputs[a1[0]].mapping(a1[1])
			z1 = np.repeat(z1[:,None], resolution, axis=1)

			y, z2 = self.inputs[a2[0]].mapping(a2[1])
			z2 = z2.T
			Z = op(z1, z2)
			
			print(z.shape)
			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')
			X, Y = np.meshgrid(x, y)
			ax.plot_surface(X, Y, Z)
			plt.show()
		else:
			warnings.warn("Can't plot a rule surface with more than 2 parts to its antecedent")
			return

	def infer(self, input_vals):
		
		return_out = {}
		# Fuzzify inputs
		for i in input_vals:
			self.inputs[i].fuzzify(input_vals[i])
		
		# Calculate firing strength for each rule
		for rule in self.rules:
			op, antecedent, consequent = rule
			strength = None
			for var_label, set_label, hedge  in antecedent:
				v = self.inputs[var_label].get(set_label)
				if hedge is not None:
					v = hedge(v)
				strength = v if strength is None else op(strength, v)
		# Implication for any rule with a firing strength greater than 0
			if strength>0.0:
				for var_label, set_label in consequent:
					self.outputs[var_label].implicate(set_label, strength)
		# Aggregate & defuzzify
		for o in self.outputs:
			self.outputs[o].aggregate()
			return_out[o] = self.outputs[o].defuzzify()

		return return_out

	def add_input(self, var_label, *args, **kw_args):
		self.inputs[var_label] = self.InputVariable(self, var_label, *args, **kw_args)

	def add_input_set(self, var_label, *args, **kw_args):
		self.inputs[var_label].add_set(*args, **kw_args)

	def add_output(self, var_label, *args, **kw_args):
		self.outputs[var_label] = self.OutputVariable(self, var_label, *args, **kw_args)

	def add_output_set(self, var_label, *args, **kw_args):
		self.outputs[var_label].add_set(*args, **kw_args)

	def add_rule(self, linguistic_rule):
		'''
		Sidenote: 
			If 'X is A then Y is B and Z is C' is equivalent to 2 rules 'X is A then Y is B' and 'X is A then z is C', then 
			'X is A then Y is B and Y is C' must also be equivalent to using 2 rules 'X is A then Y is B' and 'X is A then Y is C',
			which would then be combined with the aggregate operation because they are the same variable: AGG(THEN(X is X_A, Y is B), THEN(X is A, Y is C)). 
			would 'X is A then Y is B or Y is C' be equivalent CAGG(THEN(X is A, Y is B), THEN(X is A, Y is C)) where CAGG is the co-norm of the aggregate opperation? 
			How would OR work in the consequent for two different variables? is OR just not used in the conseqeuent at all?
		'''
		if 'if' not in linguistic_rule.lower() or 'then' not in linguistic_rule.lower():
			warnings.warn("Rule's must be formed as 'if...then' sentences. Rule ignored")
			return
		linguistic_rule = linguistic_rule.translate(None, string.punctuation)
		antecedent, consequent = re.split(' then ',linguistic_rule[3:],flags=re.I)
		
		use_and = 'and' in antecedent
		use_or = 'or' in antecedent
		if use_and and use_or:
			warnings.warn("Rule antecedent can only contain 'or' or 'and' not both. Rule ignored")
			return

		spl = ' and ' if use_and else ' or '
		op = self.AND if use_and else self.OR
		parts = re.split(spl, antecedent, flags=re.I)
		
		try:
			antecedent = [(x1.strip(), x2.strip(), None) for x in parts for x1,x2 in [re.split(' is ',x,flags=re.I)]]
		except(ValueError):
			print(parts)

		for i,a in enumerate(antecedent):
			for s,m in zip(['not','very','slightly'], [NOT, VERY, SLIGHTLY]):
				if s in a[1]:
					antecedent[i] = (a[0], a[1].replace(s, '').strip(), m)
					break

		parts = re.split(' and ', consequent, flags=re.I)
		consequent = [(x1.strip(), x2.strip()) for x in parts for x1,x2 in [re.split(' is ' if 'is' in x.lower() else ' ',x.strip(),flags=re.I)]]
		
		if self._check_rule(antecedent, consequent):
			self.linguistic_rules.append(linguistic_rule)
			self.rules.append((op, antecedent, consequent))
	
	def _check_rule(self, antecedent, consequent):
		# Check antecedent
		for a in antecedent:
			if a[0] not in self.inputs:
				warnings.warn("Bad rule. '{}' is not defined as an input. Rule ignored".format(a[0]))
				return False
			else:
				if a[1] not in self.inputs[a[0]]:
					warnings.warn("Bad rule. '{}' is not defined as a set for input '{}'. Rule ignored".format(a[1],a[0]))
					return False
		# Check consequent
		for c in consequent:
			if c[0] not in self.outputs:
				warnings.warn("Bad rule. '{}' is not defined as an output. Rule ignored".format(c[0]))
				return False
			else:
				if c[1] not in self.outputs[c[0]]:
					warnings.warn("Bad rule. '{}' is not defined as a set for output '{}'. Rule ignored".format(c[1],c[0]))
					return False
		return True		
	
	##### NESTED CLASSES ######
	class FuzzyVariable(object):
		def __init__(self, logic, var_label, val_min, val_max):
			if not isinstance(var_label, str):
				raise RuntimeError("FuzzyVariable's var_label must be a string")
			self.var_label = var_label
			self.sets = {}
			self.logic = logic
			self.val_min = float(val_min)
			self.val_max = float(val_max)
			#self.external_func = external_func
			self.x = np.linspace(self.val_min, self.val_max, self.logic.resolution)

		def __contains__(self, set_label):
			return set_label in self.sets

		def add_set(self, set_label, membership_function, *membership_function_args, **kw_args):
			if not isinstance(set_label, str):
				raise RuntimeError("'set_labels' must be a string")
			# Make sure we're binding everything but the x
			#required = len(signature(function).parameters)-1
			params = getargspec(membership_function)[0]
			required = len(params)-1
			given = len(membership_function_args)
			if given != required:
				raise RuntimeError("Membership function binding must be complete: {} arguments given but {} required {}".format(given,required,params))
			
			if 'pre_ranged' not in kw_args:			
				scaled_args = []
				for m in membership_function_args:
					scaled_args.append(m*(self.val_max - self.val_min) + self.val_min)
			else:
				scaled_args = membership_function_args

			self.sets[set_label] = partial(membership_function, *scaled_args)
		
		def get_function(self, set_label):
			return self.sets[set_label]

		def mapping(self, set_label):
			return (self.x, self.sets[set_label](x))

		def all_mappings(self, set_labels=None):
			y = {}
			for s in self.sets:
				y[s] = self.sets[s](self.x)
			return (self.x, y)

		def draw(self, axis=None, test_value=None):
			if axis is None:
				axis = plt

			for s in self.sets:
				plot = self.sets[s](self.x)
				axis.set_title(self.var_label)
				axis.plot(self.x,plot,label=s)

			if test_value is not None:
				axis.plot([test_value,test_value],[0,1],'k--')
				axis.set_color_cycle(None)
				for s in self.sets:
					y = self.sets[s](test_value)
					axis.plot([test_value,self.val_max],[y,y],'--')
				
			axis.legend()

	class InputVariable(FuzzyVariable):
		'''
		Handles input sets.
		Must add input fuzzy sets to it to use.

		Parameters:
			var_label (str): 	linguistic name of the variable
		'''

		def __init__(self, logic, var_label, val_min, val_max):
			#params = getargspec(input_source)[0]
			#if len(params)!=0:
				#raise RuntimeError("InputVariable's external function (input_source) must be have just one argument for the parent logic")
			super(self.__class__, self).__init__(logic, var_label, val_min, val_max)
			self.current_memberships = {}
		
		def get(self, set_label):
			return self.current_memberships[set_label]

		def add_set(self, set_label, membership_function, *membership_function_args):
			super(self.__class__, self).add_set(set_label, membership_function, *membership_function_args)
			self.current_memberships[set_label] = 0.0

		def fuzzify(self, x):
			if not self.sets:
				warnings.warn("fuzzify called on InputVariable with no fuzzy sets defined")
				return

			#x = self.external_func()

			if self.logic.auto_clip:
				x = np.clip(x, self.val_min, self.val_max)
			
			fuzz = {}
			for s in self.sets:
				fuzz[s] = self.sets[s](x)
			
			self.current_memberships = fuzz


	class OutputVariable(FuzzyVariable):
		def __init__(self, logic, var_label, val_min, val_max, initial_output=0.0, THEN=None, AGG=None, DEFUZZ=None):
			#params = getargspec(output_function)[0]
			#if len(params)!=1:
				#raise RuntimeError("OutputVariables's external function (output_function) must take 1 argument only")
			super(self.__class__, self).__init__(logic, var_label, val_min, val_max)
			self.fired_outputs = []
			self.current_output = initial_output
			self.current_set = None
			self.updated = False
			## functionality for different output sets to use different implication and aggregation - not that it's ever needed.
			self.THEN = self.logic.THEN if THEN==None else THEN
			self.AGG = self.logic.AGG if AGG==None else AGG
			self.DEFUZZ = self.logic.DEFUZZ if DEFUZZ==None else DEFUZZ
		
		def implicate(self, set_label, strength):
			self.fired_outputs.append(self.THEN(self.sets[set_label], strength, self.x))

		def aggregate(self):
			if not self.fired_outputs:
				self.updated = False
				return
			self.updated = True
			fired_array = np.array(self.fired_outputs)
			self.fired_outputs = []
			self.current_set = self.AGG(fired_array)
			
		def defuzzify(self):
			if self.updated:
				self.current_output = self.DEFUZZ(self.x, self.current_set)
			
			#self.external_func(self.current_output)
			return self.current_output
		
	

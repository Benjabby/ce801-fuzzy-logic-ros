### Overview
This code for controllers is intended to be run on a ROS-bot test environment that is no longer available. 

The FuzzyLogicSystem class is very in-depth and has support for the following:
* Different membership functions
* Different OR methods: min, probor
* Different AND methods: max, prod
* The hedges 'very' and 'slightly' (AKA 'more or less')
* Different aggregate methods: max, sum, prod
* Different defuzzification methods: centroid, bisector, midmax (though not all have been thoroughly tested)
* Plotting input sets 
* Plotting output sets
* plotting the firing strength of rules with one or two antecedent parts for the range of input values. 
* Linguistic rule parsing

While the controllers themselves will need to be adapted for use outside of the unavailable test environment, FuzzyLogicSystem can be used in its current state to implement fuzzy logic systems.

### `ros_base.py`
Contains the base class for controllers

### `fuzzy_logic.py`
Contains the FuzzyLogicSystem class and membership functions etc.

### `fuzzy_avoidance.py`
Contains a fuzzy avoidance controller

### `fuzzy_wall_following.py`
Contains a fuzzy wall following controller

### `fuzzy_ultimate.py`
Contains a combined fuzzy controller

### `main_pid.py`
Contains a PID controller


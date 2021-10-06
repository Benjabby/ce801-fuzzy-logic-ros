ros_base.py 		contains the base class for controllers
fuzzy_logic.py 		contains the FuzzyLogicSystem class and membership functions etc.

fuzzy_avoidance.py 	contains the fuzzy avoidance controller
fuzzy_wall_following.py contains the fuzzy wall following controller
fuzzy_ultimate.py 	contains the combined fuzzy controller
main_pid.py		contains the PID controller. (be sure not to run it with any arguments as those are used for executing test runs)

Run any controllers with 'reset' as an argument to reset the world before running (Note this does not work on the demo environment)
Run any of the fuzzy controllers with the arguments 'di' or 'do' to instead show the input and output sets respectively for that controller.

Code is semi-documented...

Softmin was used because I figured if the smoothness of the control surface was important, the smoothness of the inputs over timeshould be too. If you use min(x) it would be discontinous.
Though realisticly it's discontinous anyway because the laser scans themselves have a non zero interval, but still, softmin works well.

Many many many other sensor methods were tested, far too much to properly explain in a few slides. 

FuzzyLogicSystem is extremely in depth, probably spent way more time on it than needed.
It has support for the following:
	Different membership functions
	Different OR methods: min, probor
	Different AND methods: max, prod
	The hedges 'very' and 'slightly' (AKA 'more or less')
	Different aggregate methods: max, sum, prod
	Different defuzzification methods: centroid, bisector, midmax (though not all have been thoroughly tested)
	Plotting input sets 
	Plotting output sets
	plotting the firing strength of rules with one or two antecedent parts for the range of input values. 
	Linguistic rule parsing

I might use it for something else in the future.






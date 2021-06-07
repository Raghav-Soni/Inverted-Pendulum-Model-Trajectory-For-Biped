# Inverted-Pendulum-Model-Trajectory-For-Biped
A Spring Loaded Inverted Pendulum (SLIP) model based framework for generating walking trajectories for bipeds.

The files are python script, each file has following functions -:
  1) slip.py - Contains the slip class which generate trajectories, call step function with time as a variable to execute the next step
  2) test.py - A file to load the slip class and print the output in form of base, right foot and left foot transformation matrices along with some other parameters
  3) plot.py - Plots the 2D motion with time of the centre of mass and feet of the model

This code is implemented based on the following papers - 

  1) [The Linear Inverted Pendulum Mode : A simple modeling for a biped walking pattern generation](http://users.dimi.uniud.it/~antonio.dangelo/Robotica/dissertations/helper/3D_Linear_Inverted_Pendulum_Model.pdf)
  2) [Real-time 3D walking pattern generation for a biped robot](http://users.dimi.uniud.it/~antonio.dangelo/Robotica/2012/helper/K0529.pdf)

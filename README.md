# README #

### Running Models ###

* Calculates multi body dynamics for models for the purpose of investigating the dynamics, energetics, and control of human running.  Uses a class library in Matlab to expedite the creation of new models based on simple models.

* Version 0.1

### How do I get set up? ###

* You need Mathematica, Dynamics Workbench, and a recent version of Matlab to use this repository.

* Human data is not stored on this repository, but is used in some of the analysis

* Animations and saved mat files are also not included in the repository, although some functions reference such files

* You may need to create the local folders: "Animations", "SavedGaits", and "ParameterStudies" in order for the saving functions & methods to work properly; else make changes to the code such that it saves elsewhere

### General Info ###
*Runner is an abstract class that is a superclass to all the other classes (e.g. SLIP, SoftParRunner, Swing, etc.)

* Each class also has an associated state definition class located in the folder State_Definitions which makes it easier to parse the states of each class.  For example, it converts a vector of states to a nicely labeled structure that tells you Knee Velocity, Pelvis Position, etc.

*Also includes some simulations of models that fall under the category of Differential Algebraic Equations, located in the folder DAE.  This is a result of setting unconstrained degrees of freedom to have zero mass, which can still be solved for if there are springs and/or dampers that act on those DOF.

### Important Methods  ###

*"test" is a static method used to test energy conservation, that constraints are working, that event functions are working, and to play with initial conditions and parameters to try to get close to a limit cycle.  It is basically a mess and a playground.  It might be better to divide "test" into multiple static methods that each contain a single test.

* All classes have the method "onestep" which simulates the model for a single step given initial conditions, as well as additional extra arguments.  The simulation stops based on event functions that are generally included as methods of the class and called within onestep

* "plot" plots the model at one state

* "anim" animates the model at the states given to its input.  If only initial conditions of the model are given, it calls onestep first, and then animates the results

### Who do I talk to? ###

* Contact riddryan on bitbucket or email at riddryan@umich.edu
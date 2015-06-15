# README #

## Running Models ##

* Calculates multi body dynamics for models for the purpose of investigating the dynamics, energetics, and control of human running.  Uses a class library in Matlab to expedite the creation of new models based on simple models.

## How do I get set up? ##

* You need Mathematica, Dynamics Workbench, and a recent version of Matlab to use this repository.  Created on Matlab 2014b, Mathematica 9.0.0.0, and [Dynamics Workbench 3.7] (http://www-personal.umich.edu/~artkuo/DynamicsWorkbench/).  Add folder State_Definitions to your Matlab Path.

* Human data is not stored on this repository, but is used in some of the analysis.  Animations and saved mat files are also not included in the repository, although some functions reference such files.  You may need to create the local folders: "Animations", "SavedGaits", "ParameterStudies", "ReturnMapStudies", and "Figures" in order for the saving functions & methods to work properly; else make changes to the code such that it saves elsewhere.

* The data & folders can be found at `\\hbcl-server.engin.umich.edu\hbcl\projects\RunModels`

## General Info ##
* `Runner` is an abstract class that is a superclass to all the other classes (e.g. `SLIP`, `SoftParRunner`, `Swing`, etc.)

* Each class also has an associated state definition class located in the folder State_Definitions which makes it easier to parse the states of each class.  For example, it converts a vector of states to a nicely labeled structure that tells you Knee Velocity, Pelvis Position, etc.

* Also includes some simulations of models that fall under the category of Differential Algebraic Equations, located in the folder DAE.  This is a result of setting unconstrained degrees of freedom to have zero mass, which can still be solved for if there are springs and/or dampers that act on those DOF.

## Tutorial ##

The process of describing a model, solving the equations of motion, constructing a class definition of that model, and then using the model to find and analyze limit cycles are described here.

### Outline ###

1. Use the Dynamics Workbench packages to build a 2D or 3D model, and then export the equations of motion into a .m file for use in Matlab.  It is also helpful to export the energy of each body & spring, the position & velocity vectors of each point, and the Constraint matrices and their derivatives as well.

2.  Copy an old class definition file that most resembles your desired model, rename it appropriately, and then go through the annoying process of copy/pasting and class-specific changes that are required in each method.

     * If desired, for your class `MyClass.m`, also create an m-file `MyClassState.m` in the State_Definitions folder to allow easier decoding of which state is which.
     * If you created your class from `OldClass.m`, replace each call to `OldClass` and `OldClassState` by `MyClass` and `MyClassState` respectively.
     * Copy and paste the appropiate output from Mathematica into the methods `getMMandRHS`, `getConstraints`, `getVelJacob`, `getEnergies`, `getPoints`, and `getcomWR`
     * You will probably have to change several other class methods by hand including: `onestep` , `plot`, methods that serve as event functions for simulations, methods that calculate special quantities such as      power going through a spring (could also export from Mathematica).  You may also have to change `getSpeed`, `getStepLength`, and `getAerialFraction`.

3.  Use the method `test` to simulate the model and test for energy conservation, that event functions are working properly, that constraints are working properly, and to get initial conditions that look like they could be close to a limit cycle.

4.  Use method `findLimitCycle` (of class Runner unless otherwise overridden) to take those initial conditons and find a limit cycle that meet the desired constraints of speed, step length, air time, and any other constraints specified.

5.  Use that limit cycle to find other limit cycles and analyze how parameters effect dynamics and energetics using `parmstudy1d.m` and `parmstudy2d.m`

### Example ###

This example will show you how to start with the SLIP model and then add a degree of freedom with a spring, mass, and damper above the pelvis to represent a soft stomach.  

### Build the Model ###

* Open SLIP.nb in Mathematica, located in the folder EOM Notebooks.  Save file as Tutorial.nb and change the line `savename = workdir <> "\\SLIP.m";` to `savename = workdir <> "\\Tutorial.m";` This is the name of the m-file Mathematica will export Matlab code into

* After the last line in `Set Up Bodies`, add the line `AddBody[stomach,pelvis,Slider,Axis->ground[2],Mass->mstomach];`

* Under `Gravity`, add the line `AppFrc[stomach,Mass[stomach] grav, 0];`

* Under `Springs`, add the line



## Important Methods  ##

* `test` is a static method used to test energy conservation, that constraints are working, that event functions are working, and to play with initial conditions and parameters to try to get close to a limit cycle.  It is basically a mess and a playground.  It might be better to divide `test` into multiple static methods that each contain a single test.

* All classes have the method `onestep` which simulates the model for a single step given initial conditions, as well as additional extra arguments.  The simulation stops based on event functions that are generally included as methods of the class and called within onestep

* `plot` plots the model at one state

* `anim` animates the model at the states given to its input.  If only initial conditions of the model are given, it calls onestep first, and then animates the results

## Who do I talk to? ##

* Contact riddryan on bitbucket or email at riddryan@umich.edu
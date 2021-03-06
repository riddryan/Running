[TOC]

# Running Models #

Calculates multi body dynamics for models for the purpose of investigating the dynamics, energetics, and control of human running.  Uses a class library in Matlab to expedite the creation of new models based on simple models.

## How do I get set up? ##

* You need Mathematica, Dynamics Workbench, and a recent version of Matlab to use this repository.  Created on Matlab 2014b, Mathematica 9.0.0.0, and [Dynamics Workbench 3.7](http://www-personal.umich.edu/~artkuo/DynamicsWorkbench/).  

* Add folder State_Definitions to your Matlab Path.

* Download the human data and folders that store model data & animations from `\\hbcl-server.engin.umich.edu\hbcl\projects\RunModels` and place these folders and files into the same folder as the repository.

## Outline ##

The following steps are the general workflow for building a new model and analyzing limit cycles with it.

1. Use the Dynamics Workbench packages to build a 2D or 3D model, and then export the equations of motion into a .m file for use in Matlab.  It is also helpful to export the energy of each body & spring, the position & velocity vectors of each point, and the Constraint matrices and their derivatives as well.

2.  Copy an old class definition file that most resembles your desired model, rename it appropriately, and then go through the annoying process of copy/pasting and class-specific changes that are required in each method.

     * If desired, for your class `MyClass.m`, also create an m-file `MyClassState.m` in the State_Definitions folder to allow easier decoding of which state is which.

     * If you created your class from `OldClass.m`, replace each call to `OldClass` and `OldClassState` by `MyClass` and `MyClassState` respectively.

     * Copy and paste the appropiate output from Mathematica into the methods `getMMandRHS`, `getConstraints`, `getVelJacob`, `getEnergies`, `getPoints`, and `getcomWR`

     * You will probably have to change several other class methods by hand including: `onestep` , `plot`, methods that serve as event functions for simulations, methods that calculate special quantities such as      power going through a spring (could also export from Mathematica).  You may also have to change `getSpeed`, `getStepLength`, and `getAerialFraction`.

3.  Use the static method `test` to simulate the model and test for energy conservation, that event functions are working properly, that constraints are working properly, and to get initial conditions that look like they could be close to a limit cycle.

4.  Use method `findLimitCycle` (of class Runner unless otherwise overridden) to take those initial conditons and find a limit cycle that meet the desired constraints of speed, step length, air time, and any other constraints specified.

5.  Use that limit cycle to find other limit cycles and analyze how parameters effect dynamics and energetics using `parmstudy1d.m` and `parmstudy2d.m`

## Tutorial Example ##

This example will show you how to start with the SLIP model and then add a degree of freedom with a spring and mass above the pelvis to represent a soft stomach.  You can look in the files Stomach.nb, and the class definition Stomach.m to check your work.

### Build the Model in Mathematica ###

* Open SLIP.nb in Mathematica, located in the folder EOM Notebooks.  Save file as MyStomach.nb and change the line `savename = workdir <> "\\SLIP.m";` to `savename = workdir <> "\\MyStomach.m";` This is the name of the m-file Mathematica will export Matlab code into

* In the `Set Up Bodies` section, add the line 
```mathematica
AddBody[stomach, pelvis, Slider, TAxis -> ground[2], Mass -> mstomach];
```

* In the `Gravity` Section, add the line 
```mathematica
AppFrc[stomach, Mass[stomach] grav, 0];
```

* Under `Springs`, add the lines
```mathematica
AppFrc[stomach, -kstomach ( q[5] - stomachl ) stomach[1], 0];
AppFrc[pelvis, kstomach (q[5] - stomachl) stomach[1], 0];
```

* Under `Energy`, replace the code with the lines:
```mathematica
gravE = -(Mass[pelvis] grav.PosCOM[pelvis]);
springE = 1/2 kstance (q[4] - stancel)^2 + 1/2kstomach (q[5] - stomachl)^2;
PE = gravE + springE // Simplify;
KE = 1/2(Mass[pelvis] VelCOM[pelvis].VelCOM[pelvis])+1/2(Mass[stomach] VelCOM[stomach].VelCOM[stomach])// Simplify;
```

* Under `Kinematic Vectors`, add the lines:
```mathematica
stomachpos = PosCOM[stomach];
stomachvel = VelCOM[stomach];
```

* Under Export, replace the list `expressionsToExport` with:
```mathematica
expressionsToExport = {{constraintJacobianStance, "constraintJacobianStance"}, {constraintJacobianStanceDot, "constraintJacobianStanceDot"}, {constraintJacobianAerial, "constraintJacobianAerial"}, {constraintJacobianAerialDot, "constraintJacobianAerialDot"}, {KE, "kineticEnergy"}, {PE, "potentialEnergy"}, {gravE, "PEgrav"}, {springE, "PEspring"},
    
    {inGroundFrame2D[stancefootpos], "points.stancefoot"}, {inGroundFrame2D[pelvpos], "points.pelvis"}, {inGroundFrame2D[COMpos], "points.COM"},{inGroundFrame2D[stomachpos], "points.stomach"},
    
    {inGroundFrame2D[stancefootvel], "vels.stancefoot"}, {inGroundFrame2D[pelvvel], "vels.pelvis"},{inGroundFrame2D[COMvel], "vels.COM"},{inGroundFrame2D[stomachvel], "vels.stomach"}}
```

### Construct the Class State Definition in Matlab ###

First make a class that interprets the states of the model that you will build in Matlab into readable words.  This is most useful when you are working with a lot of models or degrees of freedom and don't remember which state is which.  Furthermore, if you have many models but with common elements such as a "pelvis" body, then references within code to the states of that body do not need to be rewritten for the new model.

* Make a copy of SLIPState.m in the folder State_Definitions and rename it MyStomachState.m

* Open MyStomachState.m, and do a find "SLIPState" and replace all with "MyStomachState"

* Under properties, you need to define the structures with sensible names that correspond to each element of the vector.  For example, replace the code under properties with the following:
```matlab
        pelvis = struct('x', 0, 'y', 0, 'xDot', 0, 'yDot',0);
        stancefoot = struct('Angle', 0, 'Length',0,'AngleDot',0,'LengthDot',0);
        stomach = struct('Length',0,'LengthDot',0);
        order=[1 2 6 7 ...
               3 4 8 9 ...
               5 10]; 
```

### Develop the Class Definition in Matlab ###

* Make a copy of SLIP.m in the main directory and rename it MyStomach.m

* Open MyStomach.m and do a find "SLIP" and replace all with "MyStomach"

* Also open MyStomach.m in the "EOM Notebooks" folder to access the exported Mathematica code.  Note that this has the same file name as your class definition but is in a different folder.

* Copy and paste from the exported code into class methods:
    1. `MM` and `rhs` into the method `getMMandrhs`.
    2. All constraint matrices & their derivatives into method `getConstraints`.  Make sure you copy and paste the constraint matrices into the correct location in the if-else statement such that the stance constraint matrices are under the `strcmp(phase,'Stance')` section of the if-else statement.
    3. All fields of the structure `points` into method `getPoints`.
    4. All fields of the structure `vels` into method `getVels`.
    5. All potential & kinetic energies into `getEnergies`.

* In class properties default declarations:
    1. Set `statestomeasure = [3:5 8:10]`. `statestomeasure` tells the optimizer what initial conditions it is allowed to change to get a limit cycle.  The reason the states of the pelvis are not included is because there are two constraints acting on the system which negate the need to specify the IC of all states.  
    2. Set `statestovary = [3 5 8:10]`.  `statestomeasure` say which states need to be equal to the IC at the start of the next step to be a limit cycle.  State 4, leg length, is not included because we always want the leg to be equal to the rest length of the spring at the start of a step.
    3. Set `N=10` to tell it how many states are in the model.
    4. Create the following properties as arbitrary default values for the stomach parameters:
      ```matlab
         kstomach = 1;
         stomachl = 0.2;
         mstomach = 0.2;
      ```
    5. Set `mpelvis = 0.8`.  The reason to do this is so that the total mass of the system is equal to 1, which makes it easier to work with the numbers.

* In the method `plot`, add the following lines to draw the stomach mass and stomach spring during animations:
```matlab
stomachcoils = 3;
stomachspringwidth = 0.1;
plotter.plotSpring(points.pelvis(1),points.pelvis(2),points.stomach(1),points.stomach(2),stomachcoils,this.stomachl,stomachspringwidth);
plotter.plotMass(points.pelvis,'scaling',this.mstomach/this.mpelvis);
```

* Luckily, since we want `MyStomach` to have the same phases and event functions (detecting take off and heel strike) as `SLIP`, you do not need to change the `onestep` or any of the event methods.

### Test the Matlab Class ###
Since we are building off of the SLIP model, we will use the initial conditions of that model to make it easier to find a limit cycle.  A limit cycle just means dynamic behavior that repeats itself over a period of time.  In locomotion, the limit cycle we are most interested is that of a step.

- - -

* Firstly, delete all code in the section **Set Model IC & Parameters** within the static method `Test`

* To take applicable initial conditions and parameters of the SLIP model and use them as a starting point for your model.  To do this, add the lines:
```matlab
r = MyStomach;
[r,IC] = r.Inherit(SLIP);
```
The MyStomach model r has inherited all applicable parameters from the default SLIP model.  IC is of the class MyStomachState, and the properties for the pelvis and stance leg have also been set.

You may wonder why we don't make `MyStomach` a subclass of SLIP.  You could do that and it might make things a little easier in this case.  However, most models you make will not need all of the class properties and methods of the class that you built the model from.  In this case, you would have a bunch of unused properties and methods taking up space.  Having most of the property and method declarations within one file can also make things easier.

* Now we must choose initial conditions for the stomach states.  We will assume for now the stomach has no velocity at heel-strike, and the spring is unstretched:
```matlab
IC.stomach.Length = r.stomachl;
IC.stomach.LengthDot = 0;
```

* We are now ready to test the model by typing `MyStomach.test` into the Matlab console

You should see an animation of the model running, figures of the energies of the model, and numbers such as speed and limit cycle error printed to the Matlab console.  At this stage it is common to have bugs that you need to track down.  Often at this stage you will realize that you need to change more of the methods from the model you started from (SLIP in this case).

Since we have no dampers nor impulses in this model, the total energy should be constant across time.  If it isn't, there is something wrong.

## Matlab Class Structure and Methods ##
* `Runner` is an abstract class that is a superclass to all the other classes (e.g. `SLIP`, `SoftParRunner`, `Swing`, etc.)

* Each class also has an associated state definition class located in the folder State_Definitions which makes it easier to parse the states of each class.  For example, it converts a vector of states to a nicely labeled structure that tells you Knee Velocity, Pelvis Position, etc.


* `test` is a static method used to test energy conservation, that constraints are working, that event functions are working, and to play with initial conditions and parameters to try to get close to a limit cycle.  It is basically a mess and a playground.  It might be better to divide `test` into multiple static methods that each contain a single test.

* All classes have the method `onestep` which simulates the model for a single step given initial conditions, as well as additional extra arguments.  The simulation stops based on event functions that are generally included as methods of the class and called within onestep

* `plot` plots the model at one state

* `anim` animates the model at the states given to its input.  If only initial conditions of the model are given, it calls onestep first, and then animates the results

## Differential Algebraic Equations (DAE) ##

* Also includes some simulations of models that fall under the category of Differential Algebraic Equations, located in the folder DAE.  This is a result of setting unconstrained degrees of freedom to have zero mass, which can still be solved for if there are springs and/or dampers that act on those DOF.
* `MasslessAchillesRunner` is DAE system, but the DAE solvers in Matlab do a poor job of solving the dynamics because of the difficult kinematics.  Instead, an optimization is performed each step of the integrator to solve for the states that meet the kinematic constraints and minimize the Lagrangian.

## Who do I talk to? ##

* Contact riddryan on bitbucket or email at riddryan@umich.edu
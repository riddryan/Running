function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6); 
u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12); 

c5 = cos(q5); s5 = sin(q5); 

MM = zeros(6,6); rhs = zeros(6,1);

% Mass Matrix
MM(1,1) = mpelvis; MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = 0; MM(1,5) = 0; ...
MM(1,6) = 0; 
MM(2,1) = 0; MM(2,2) = mpelvis; MM(2,3) = 0; MM(2,4) = 0; MM(2,5) = 0; ...
MM(2,6) = 0; 
MM(3,1) = 0; MM(3,2) = 0; MM(3,3) = 0; MM(3,4) = 0; MM(3,5) = 0; MM(3,6) = 0; 
MM(4,1) = 0; MM(4,2) = 0; MM(4,3) = 0; MM(4,4) = 0; MM(4,5) = 0; MM(4,6) = 0; 
MM(5,1) = -(q6*s5); MM(5,2) = c5*q6; MM(5,3) = 0; MM(5,4) = 0; MM(5,5) = ...
q6*q6; MM(5,6) = 0; 
MM(6,1) = c5; MM(6,2) = s5; MM(6,3) = 0; MM(6,4) = 0; MM(6,5) = 0; MM(6,6) = ...
1; 

% righthand side terms
rhs(1) = g*mpelvis*sin(gslope); 
rhs(2) = -(g*mpelvis*cos(gslope)); 
rhs(3) = 0; 
rhs(4) = -(u4*cstance) - q4*kstance + kstance*stancel; 
rhs(5) = -2*q6*u5*u6 + (u3 - u5)*chip - (-q3 + q5 + hipl)*khip - q6*g*cos(q5 ...
- gslope); 
rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + u5*u5) - g*sin(q5 - ...
gslope); 

udot = MM\rhs;
xdot = [x(6+1:12); udot];

constraintJacobianStance(1,1) = 1; constraintJacobianStance(1,2) = 0; ...
constraintJacobianStance(1,3) = -(q4*sin(q3)); constraintJacobianStance(1,4) ...
= cos(q3); constraintJacobianStance(1,5) = 0; constraintJacobianStance(1,6) = ...
0; 
constraintJacobianStance(2,1) = 0; constraintJacobianStance(2,2) = 1; ...
constraintJacobianStance(2,3) = q4*cos(q3); constraintJacobianStance(2,4) = ...
sin(q3); constraintJacobianStance(2,5) = 0; constraintJacobianStance(2,6) = ...
0; 


constraintJacobianStanceDot(1,1) = 0; constraintJacobianStanceDot(1,2) = 0; ...
constraintJacobianStanceDot(1,3) = -(q4*u3*cos(q3)) - u4*sin(q3); ...
constraintJacobianStanceDot(1,4) = -(u3*sin(q3)); ...
constraintJacobianStanceDot(1,5) = 0; constraintJacobianStanceDot(1,6) = 0; 
constraintJacobianStanceDot(2,1) = 0; constraintJacobianStanceDot(2,2) = 0; ...
constraintJacobianStanceDot(2,3) = u4*cos(q3) - q4*u3*sin(q3); ...
constraintJacobianStanceDot(2,4) = u3*cos(q3); ...
constraintJacobianStanceDot(2,5) = 0; constraintJacobianStanceDot(2,6) = 0; 


kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (khip*((-q3 + q5 + hipl)*(-q3 + q5 + hipl)) + kstance*((-q4 ...
+ stancel)*(-q4 + stancel)) + kswing*((-q6 + swingl)*(-q6 + swingl)) + ...
2*g*mpelvis*(q2*cos(gslope) - q1*sin(gslope)))/2.;

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (khip*((q3 - q5 - hipl)*(q3 - q5 - hipl)))/2. + (kstance*((q4 - ...
stancel)*(q4 - stancel)))/2. + (kswing*((q6 - swingl)*(q6 - swingl)))/2.;

points.stancefoot(1) = q1 + q4*cos(q3); 
points.stancefoot(2) = q2 + q4*sin(q3); 


points.swingfoot(1) = q1 + c5*q6; 
points.swingfoot(2) = q2 + q6*s5; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 


vels.stancefoot(1) = u1 + u4*cos(q3) - q4*u3*sin(q3); 
vels.stancefoot(2) = u2 + q4*u3*cos(q3) + u4*sin(q3); 


vels.swingfoot(1) = u1 - q6*s5*u5 + c5*u6; 
vels.swingfoot(2) = u2 + c5*q6*u5 + s5*u6; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 


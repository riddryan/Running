function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6); 
u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12); 

c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5); 

MM = zeros(6,6); rhs = zeros(6,1);

% Mass Matrix
MM(1,1) = 2*mfoot + mpelvis; MM(1,2) = 0; MM(1,3) = -(q4*s3*mfoot); MM(1,4) = ...
c3*mfoot; MM(1,5) = -(q6*s5*mfoot); MM(1,6) = c5*mfoot; 
MM(2,1) = MM(1,2); MM(2,2) = 2*mfoot + mpelvis; MM(2,3) = c3*q4*mfoot; ...
MM(2,4) = s3*mfoot; MM(2,5) = c5*q6*mfoot; MM(2,6) = s5*mfoot; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mfoot*(q4*q4); MM(3,4) = 0; ...
MM(3,5) = 0; MM(3,6) = 0; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot; ...
MM(4,5) = 0; MM(4,6) = 0; 
MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
MM(5,5) = mfoot*(q6*q6); MM(5,6) = 0; 
MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
MM(6,5) = MM(5,6); MM(6,6) = mfoot; 

% righthand side terms
rhs(1) = 2*s3*u3*u4*mfoot + 2*s5*u5*u6*mfoot + c3*q4*mfoot*(u3*u3) + ...
c5*q6*mfoot*(u5*u5) + 2*g*mfoot*sin(gslope) + g*mpelvis*sin(gslope); 
rhs(2) = -2*c3*u3*u4*mfoot - 2*c5*u5*u6*mfoot - 2*g*mfoot*cos(gslope) - ...
g*mpelvis*cos(gslope) + q4*s3*mfoot*(u3*u3) + q6*s5*mfoot*(u5*u5); 
rhs(3) = -(u3*chip) + u5*chip - q3*khip + q5*khip + hipl*khip - ...
q4*mfoot*(2*u3*u4 + g*cos(q3 - gslope)); 
rhs(4) = -(u4*cstance) + kstance*stancel + q4*(-kstance + mfoot*(u3*u3)) - ...
g*mfoot*sin(q3 - gslope); 
rhs(5) = (u3 - u5)*chip - (-q3 + q5 + hipl)*khip - 2*q6*u5*u6*mfoot - ...
q6*g*mfoot*cos(q5 - gslope); 
rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + mfoot*(u5*u5)) - ...
g*mfoot*sin(q5 - gslope); 

udot = MM\rhs;
xdot = [x(6+1:12); udot];

constraintJacobianStance(1,1) = 1; constraintJacobianStance(1,2) = 0; ...
constraintJacobianStance(1,3) = -(q4*s3); constraintJacobianStance(1,4) = c3; ...
constraintJacobianStance(1,5) = 0; constraintJacobianStance(1,6) = 0; 
constraintJacobianStance(2,1) = 0; constraintJacobianStance(2,2) = 1; ...
constraintJacobianStance(2,3) = c3*q4; constraintJacobianStance(2,4) = s3; ...
constraintJacobianStance(2,5) = 0; constraintJacobianStance(2,6) = 0; 


constraintJacobianStanceDot(1,1) = 0; constraintJacobianStanceDot(1,2) = 0; ...
constraintJacobianStanceDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianStanceDot(1,4) = -(s3*u3); constraintJacobianStanceDot(1,5) ...
= 0; constraintJacobianStanceDot(1,6) = 0; 
constraintJacobianStanceDot(2,1) = 0; constraintJacobianStanceDot(2,2) = 0; ...
constraintJacobianStanceDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianStanceDot(2,4) = c3*u3; constraintJacobianStanceDot(2,5) = ...
0; constraintJacobianStanceDot(2,6) = 0; 


kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (khip*((-q3 + q5 + hipl)*(-q3 + q5 + hipl)) + kstance*((-q4 ...
+ stancel)*(-q4 + stancel)) + kswing*((-q6 + swingl)*(-q6 + swingl)) + ...
2*g*mpelvis*(q2*cos(gslope) - q1*sin(gslope)))/2.;

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (khip*((q3 - q5 - hipl)*(q3 - q5 - hipl)))/2. + (kstance*((q4 - ...
stancel)*(q4 - stancel)))/2. + (kswing*((q6 - swingl)*(q6 - swingl)))/2.;

points.stancefoot(1) = q1 + c3*q4; 
points.stancefoot(2) = q2 + q4*s3; 


points.swingfoot(1) = q1 + c5*q6; 
points.swingfoot(2) = q2 + q6*s5; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 


vels.stancefoot(1) = u1 - q4*s3*u3 + c3*u4; 
vels.stancefoot(2) = u2 + c3*q4*u3 + s3*u4; 


vels.swingfoot(1) = u1 - q6*s5*u5 + c5*u6; 
vels.swingfoot(2) = u2 + c5*q6*u5 + s5*u6; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 


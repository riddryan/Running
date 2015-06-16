function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); 
u1 = x(5); u2 = x(6); u3 = x(7); u4 = x(8); 

c3 = cos(q3); s3 = sin(q3); 

MM = zeros(4,4); rhs = zeros(4,1);

% Mass Matrix
MM(1,1) = mfoot + mpelvis; MM(1,2) = 0; MM(1,3) = -(q4*s3*mfoot); MM(1,4) = ...
c3*mfoot; 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + mpelvis; MM(2,3) = c3*q4*mfoot; MM(2,4) ...
= s3*mfoot; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mfoot*(q4*q4); MM(3,4) = 0; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot; 

% righthand side terms
rhs(1) = 2*s3*u3*u4*mfoot + c3*q4*mfoot*(u3*u3) + g*(mfoot + ...
mpelvis)*sin(gslope); 
rhs(2) = -2*c3*u3*u4*mfoot - g*(mfoot + mpelvis)*cos(gslope) + ...
q4*s3*mfoot*(u3*u3); 
rhs(3) = -(q4*mfoot*(2*u3*u4 + g*cos(q3 - gslope))); 
rhs(4) = kstance*stancel + q4*(-kstance + mfoot*(u3*u3)) - g*mfoot*sin(q3 - ...
gslope); 

udot = MM\rhs;
xdot = [x(4+1:8); udot];

constraintJacobianStance(1,1) = 1; constraintJacobianStance(1,2) = 0; ...
constraintJacobianStance(1,3) = -(q4*s3); constraintJacobianStance(1,4) = c3; 
constraintJacobianStance(2,1) = 0; constraintJacobianStance(2,2) = 1; ...
constraintJacobianStance(2,3) = c3*q4; constraintJacobianStance(2,4) = s3; 


constraintJacobianStanceDot(1,1) = 0; constraintJacobianStanceDot(1,2) = 0; ...
constraintJacobianStanceDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianStanceDot(1,4) = -(s3*u3); 
constraintJacobianStanceDot(2,1) = 0; constraintJacobianStanceDot(2,2) = 0; ...
constraintJacobianStanceDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianStanceDot(2,4) = c3*u3; 


constraintJacobianAerial(1,1) = 0; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 1; constraintJacobianAerial(1,4) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 0; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 1; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; 


kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kstance*((-q4 + stancel)*(-q4 + stancel)))/2. + ...
g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

points.stancefoot(1) = q1 + c3*q4; 
points.stancefoot(2) = q2 + q4*s3; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 


vels.stancefoot(1) = u1 - q4*s3*u3 + c3*u4; 
vels.stancefoot(2) = u2 + c3*q4*u3 + s3*u4; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 


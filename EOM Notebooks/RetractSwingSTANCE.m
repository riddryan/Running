function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6); 
u3 = x(7); u4 = x(8); u5 = x(9); u6 = x(10); 

c3m5 = cos(q3 - q5); s3m5 = sin(q3 - q5); 

MM = zeros(4,4); rhs = zeros(4,1);

% Mass Matrix
MM(1,1) = mpelvis*(q4*q4); MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = 0; 
MM(2,1) = 0; MM(2,2) = mpelvis; MM(2,3) = 0; MM(2,4) = 0; 
MM(3,1) = c3m5*q4*q6; MM(3,2) = q6*s3m5; MM(3,3) = q6*q6; MM(3,4) = 0; 
MM(4,1) = -(q4*s3m5); MM(4,2) = c3m5; MM(4,3) = 0; MM(4,4) = 1; 

% righthand side terms
rhs(1) = -(u3*chip) + u5*chip - q3*khip + q5*khip + hipl*khip - ...
q4*(2*u3*u4*mpelvis + g*mpelvis*cos(q3 - gslope)); 
rhs(2) = u4*cstance - kstance*stancel + q4*(kstance + mpelvis*(u3*u3)) - ...
g*mpelvis*sin(q3 - gslope); 
rhs(3) = -2*c3m5*q6*u3*u4 - 2*q6*u5*u6 + (u3 - u5)*chip - (-q3 + q5 + ...
hipl)*khip - q6*g*cos(q5 - gslope) + q4*q6*s3m5*(u3*u3); 
rhs(4) = 2*s3m5*u3*u4 - u6*cswing + kswing*swingl + c3m5*q4*(u3*u3) + ...
q6*(-kswing + u5*u5) - g*sin(q5 - gslope); 

udot = MM\rhs;
xdot = [x(4+1:8); udot];

PelvisJacobian(1,1) = -(q4*sin(q3)); PelvisJacobian(1,2) = cos(q3); ...
PelvisJacobian(1,3) = 0; PelvisJacobian(1,4) = 0; 
PelvisJacobian(2,1) = q4*cos(q3); PelvisJacobian(2,2) = sin(q3); ...
PelvisJacobian(2,3) = 0; PelvisJacobian(2,4) = 0; 


PelvisJacobianDot(1,1) = -(q4*u3*cos(q3)) - u4*sin(q3); ...
PelvisJacobianDot(1,2) = -(u3*sin(q3)); PelvisJacobianDot(1,3) = 0; ...
PelvisJacobianDot(1,4) = 0; 
PelvisJacobianDot(2,1) = u4*cos(q3) - q4*u3*sin(q3); PelvisJacobianDot(2,2) = ...
u3*cos(q3); PelvisJacobianDot(2,3) = 0; PelvisJacobianDot(2,4) = 0; 


points.pelvis(1) = q4*cos(q3); 
points.pelvis(2) = q4*sin(q3); 


points.swingfoot(1) = q4*cos(q3) + q6*cos(q5); 
points.swingfoot(2) = q4*sin(q3) + q6*sin(q5); 


points.stancefoot(1) = 0; 
points.stancefoot(2) = 0; 


PE = (khip*((-q3 + q5 + hipl)*(-q3 + q5 + hipl)) + kstance*((-q4 + ...
stancel)*(-q4 + stancel)) + kswing*((-q6 + swingl)*(-q6 + swingl)) + ...
2*q4*g*mpelvis*sin(q3 - gslope))/2.;

KE = (mpelvis*(q4*q4*(u3*u3) + u4*u4))/2.;

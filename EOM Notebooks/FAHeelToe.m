function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); 
u1 = x(4); u2 = x(5); u3 = x(6); 

c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2); 

MM = zeros(3,3); rhs = zeros(3,1);

% Mass Matrix
MM(1,1) = mpelvis*(lheel*lheel); MM(1,2) = c1m2*q3*lheel*mpelvis; MM(1,3) = ...
-(s1m2*lheel*mpelvis); 
MM(2,1) = MM(1,2); MM(2,2) = mpelvis*(q3*q3); MM(2,3) = 0; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mpelvis; 

% righthand side terms
rhs(1) = -(u1*cfoot) - q1*kfoot + footangle*kfoot - ...
2*c1m2*u2*u3*lheel*mpelvis - g*lheel*mpelvis*cos(q1 - gslope) - ...
q3*s1m2*lheel*mpelvis*(u2*u2); 
rhs(2) = q3*mpelvis*(-2*u2*u3 - g*cos(q2 - gslope) + s1m2*lheel*(u1*u1)); 
rhs(3) = -(u3*cleg) + kleg*lleg + c1m2*lheel*mpelvis*(u1*u1) + q3*(-kleg + ...
mpelvis*(u2*u2)) - g*mpelvis*sin(q2 - gslope); 

udot = MM\rhs;
xdot = [x(3+1:6); udot];

kineticEnergy = (mpelvis*(2*u1*(c1m2*q3*u2 - s1m2*u3)*lheel + q3*q3*(u2*u2) + ...
u3*u3 + u1*u1*(lheel*lheel)))/2.;

potentialEnergy = (kfoot*((-q1 + footangle)*(-q1 + footangle)) + kleg*((-q3 + ...
lleg)*(-q3 + lleg)) - 2*g*mpelvis*(-(lheel*sin(q1 - gslope)) - q3*sin(q2 - ...
gslope)))/2.;

PEgrav = -(mpelvis*(-(q3*g*cos(gslope)*sin(q2)) - g*lheel*sin(q1 - gslope) + ...
q3*g*cos(q2)*sin(gslope)));

PEspring = (kfoot*((q1 - footangle)*(q1 - footangle)))/2. + (kleg*((q3 - ...
lleg)*(q3 - lleg)))/2.;

comvx = u3*cos(q2) - u1*lheel*sin(q1) - q3*u2*sin(q2);

comvy = u1*lheel*cos(q1) + q3*u2*cos(q2) + u3*sin(q2);

points.foot(1) = lheel*cos(q1); 
points.foot(2) = lheel*sin(q1); 


points.pelvis(1) = lheel*cos(q1) + q3*cos(q2); 
points.pelvis(2) = lheel*sin(q1) + q3*sin(q2); 


points.COMpos(1) = lheel*cos(q1) + q3*cos(q2); 
points.COMpos(2) = lheel*sin(q1) + q3*sin(q2); 


velJacobian(1,1) = -(lheel*sin(q1)); velJacobian(1,2) = -(q3*sin(q2)); ...
velJacobian(1,3) = cos(q2); 
velJacobian(2,1) = lheel*cos(q1); velJacobian(2,2) = q3*cos(q2); ...
velJacobian(2,3) = sin(q2); 
velJacobian(3,1) = -(lheel*sin(q1)); velJacobian(3,2) = 0; velJacobian(3,3) = ...
0; 
velJacobian(4,1) = lheel*cos(q1); velJacobian(4,2) = 0; velJacobian(4,3) = 0; 


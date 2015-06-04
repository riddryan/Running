function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); 
u1 = x(3); u2 = x(4); 

c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2); 

MM = zeros(2,2); rhs = zeros(2,1);

% Mass Matrix
MM(1,1) = mpelvis*(lheel*lheel); MM(1,2) = c1m2*lheel*lleg*mpelvis; 
MM(2,1) = MM(1,2); MM(2,2) = mpelvis*(lleg*lleg); 

% righthand side terms
rhs(1) = -(u1*cfoot) - q1*kfoot + footangle*kfoot - g*lheel*mpelvis*cos(q1 - ...
gslope) - s1m2*lheel*lleg*mpelvis*(u2*u2); 
rhs(2) = lleg*mpelvis*(-(g*cos(q2 - gslope)) + s1m2*lheel*(u1*u1)); 

udot = MM\rhs;
xdot = [x(2+1:4); udot];

kineticEnergy = (mpelvis*(2*c1m2*u1*u2*lheel*lleg + u1*u1*(lheel*lheel) + ...
u2*u2*(lleg*lleg)))/2.;

potentialEnergy = (kfoot*((-q1 + footangle)*(-q1 + footangle)))/2. - ...
g*mpelvis*(-(lheel*sin(q1 - gslope)) - lleg*sin(q2 - gslope));

PEgrav = -(mpelvis*(-(g*lleg*cos(gslope)*sin(q2)) - g*lheel*sin(q1 - gslope) ...
+ g*lleg*cos(q2)*sin(gslope)));

PEspring = (kfoot*((q1 - footangle)*(q1 - footangle)))/2.;

comvx = -(u1*lheel*sin(q1)) - u2*lleg*sin(q2);

comvy = u1*lheel*cos(q1) + u2*lleg*cos(q2);

points.foot(1) = lheel*cos(q1); 
points.foot(2) = lheel*sin(q1); 


points.pelvis(1) = lheel*cos(q1) + lleg*cos(q2); 
points.pelvis(2) = lheel*sin(q1) + lleg*sin(q2); 


points.COMpos(1) = lheel*cos(q1) + lleg*cos(q2); 
points.COMpos(2) = lheel*sin(q1) + lleg*sin(q2); 


velJacobian(1,1) = -(lheel*sin(q1)); velJacobian(1,2) = -(lleg*sin(q2)); 
velJacobian(2,1) = lheel*cos(q1); velJacobian(2,2) = lleg*cos(q2); 
velJacobian(3,1) = -(lheel*sin(q1)); velJacobian(3,2) = 0; 
velJacobian(4,1) = lheel*cos(q1); velJacobian(4,2) = 0; 


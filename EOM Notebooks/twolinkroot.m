function xdot = twolinkroot(t, x)

% Define constants
lleg=1;lheel=1;mheel=0;mpelvis=1;gslope=0;g=1;kfoot=1;
footangle=pi/4;cfoot=.1;
% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); 
u1 = x(5); u2 = x(6); u3 = x(7); u4 = x(8); 

c3 = cos(q3); c4 = cos(q4); s3 = sin(q3); s4 = sin(q4); c3m4 = cos(q3 - q4); s3m4 = sin(q3 - q4); 

MM = zeros(4,4); rhs = zeros(4,1);

% Mass Matrix
MM(1,1) = mheel + mpelvis; MM(1,2) = 0; MM(1,3) = -(s3*lleg*mheel); MM(1,4) = ...
-(s4*lheel*mheel); 
MM(2,1) = MM(1,2); MM(2,2) = mheel + mpelvis; MM(2,3) = c3*lleg*mheel; ...
MM(2,4) = c4*lheel*mheel; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mheel*(lleg*lleg); MM(3,4) = ...
c3m4*lheel*lleg*mheel; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = ...
mheel*(lheel*lheel); 

% righthand side terms
rhs(1) = c3*lleg*mheel*(u3*u3) + c4*lheel*mheel*(u4*u4) + ...
g*mpelvis*sin(gslope); 
rhs(2) = -(g*mpelvis*cos(gslope)) + s3*lleg*mheel*(u3*u3) + ...
s4*lheel*mheel*(u4*u4); 
rhs(3) = -(s3m4*lheel*lleg*mheel*(u4*u4)); 
rhs(4) = -(u4*cfoot) - q4*kfoot + footangle*kfoot + ...
s3m4*lheel*lleg*mheel*(u3*u3); 

% udot = MM\rhs;
% xdot = [x(4+1:8); udot];

kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kfoot*((-q4 + footangle)*(-q4 + footangle)))/2. + ...
g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kfoot*((q4 - footangle)*(q4 - footangle)))/2.;

comvx = u1;

comvy = u2;

points.foot(1) = q1 + c3*lleg; 
points.foot(2) = q2 + s3*lleg; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COMpos(1) = q1; 
points.COMpos(2) = q2; 


velJacobian(1,1) = 1; velJacobian(1,2) = 0; velJacobian(1,3) = 0; ...
velJacobian(1,4) = 0; 
velJacobian(2,1) = 0; velJacobian(2,2) = 1; velJacobian(2,3) = 0; ...
velJacobian(2,4) = 0; 
velJacobian(3,1) = 1; velJacobian(3,2) = 0; velJacobian(3,3) = -(s3*lleg); ...
velJacobian(3,4) = 0; 
velJacobian(4,1) = 0; velJacobian(4,2) = 1; velJacobian(4,3) = c3*lleg; ...
velJacobian(4,4) = 0; 
velJacobian(5,1) = 1; velJacobian(5,2) = 0; velJacobian(5,3) = -(s3*lleg); ...
velJacobian(5,4) = -(s4*lheel); 
velJacobian(6,1) = 0; velJacobian(6,2) = 1; velJacobian(6,3) = c3*lleg; ...
velJacobian(6,4) = c4*lheel; 


C(1,1) = 1; C(1,2) = 0; C(1,3) = -(s3*lleg); C(1,4) = -(s4*lheel); 
C(2,1) = 0; C(2,2) = 1; C(2,3) = c3*lleg; C(2,4) = c4*lheel; 


CDot(1,1) = 0; CDot(1,2) = 0; CDot(1,3) = -(c3*u3*lleg); CDot(1,4) = ...
-(c4*u4*lheel); 
CDot(2,1) = 0; CDot(2,2) = 0; CDot(2,3) = -(s3*u3*lleg); CDot(2,4) = ...
-(s4*u4*lheel); 

bigM = [MM -C.';C zeros(size(C,1))];
bigRHS = [rhs;-CDot*u];

bigSol = bigM \ bigRHS;
udot(1:4,1) = bigSol(1:4);
xdot = [x(4+1:8); udot];


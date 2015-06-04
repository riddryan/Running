function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6); 
u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12); 



MM = zeros(6,6); rhs = zeros(6,1);

% Mass Matrix
MM(1,1) = mpelvis; MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = 0; MM(1,5) = 0; ...
MM(1,6) = 0; 
MM(2,1) = MM(1,2); MM(2,2) = mpelvis; MM(2,3) = 0; MM(2,4) = 0; MM(2,5) = 0; ...
MM(2,6) = 0; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = 0; MM(3,4) = 0; MM(3,5) = 0; ...
MM(3,6) = 0; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = 0; MM(4,5) ...
= 0; MM(4,6) = 0; 
MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
MM(5,5) = 0; MM(5,6) = 0; 
MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
MM(6,5) = MM(5,6); MM(6,6) = 0; 

% righthand side terms
rhs(1) = g*mpelvis*sin(gslope); 
rhs(2) = -(g*mpelvis*cos(gslope)); 
rhs(3) = 0; 
rhs(4) = kleg*(-q4 + lleg); 
rhs(5) = (-q5 + q6 + footangle)*kfoot; 
rhs(6) = -((-q5 + q6 + footangle)*kfoot); 

udot = MM\rhs;
xdot = [x(6+1:12); udot];

kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kfoot*((-q5 + q6 + footangle)*(-q5 + q6 + footangle)) + ...
kleg*((-q4 + lleg)*(-q4 + lleg)) + 2*g*mpelvis*(q2*cos(gslope) - ...
q1*sin(gslope)))/2.;

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kfoot*((q5 - q6 - footangle)*(q5 - q6 - footangle)))/2. + ...
(kleg*((q4 - lleg)*(q4 - lleg)))/2.;

points.foot(1) = q1 + q4*cos(q3); 
points.foot(2) = q2 + q4*sin(q3); 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.heel(1) = q1 + q4*cos(q3) + lheel*cos(q6); 
points.heel(2) = q2 + q4*sin(q3) + lheel*sin(q6); 


points.toe(1) = q1 + q4*cos(q3) + ltoe*cos(q5); 
points.toe(2) = q2 + q4*sin(q3) + ltoe*sin(q5); 


posHeelToeconstraints(1) = q1 + q4*cos(q3) + lheel*cos(q6) == 0; 
posHeelToeconstraints(2) = q2 + q4*sin(q3) + lheel*sin(q6) == 0; 
posHeelToeconstraints(3) = q2 + q4*sin(q3) + ltoe*sin(q5) == 0; 


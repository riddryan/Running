function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6); 
u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12); 

c1 = cos(q1); s1 = sin(q1); 

MM = zeros(6,6); rhs = zeros(6,1);

% Mass Matrix
MM(1,1) = 2*q2*q6*mfoot + (mfoot + msoftseries)*(q2*q2) + mfoot*(q6*q6); ...
MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = -(s1*(q6*mfoot + q2*(mfoot + ...
msoftseries))); MM(1,5) = c1*(q6*mfoot + q2*(mfoot + msoftseries)); MM(1,6) = ...
0; 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + msoftseries; MM(2,3) = 0; MM(2,4) = ...
c1*(mfoot + msoftseries); MM(2,5) = s1*(mfoot + msoftseries); MM(2,6) = ...
mfoot; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = msoftstomach; MM(3,4) = 0; ...
MM(3,5) = msoftstomach; MM(3,6) = 0; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot + ...
mpelvis + msoftseries + msoftstomach; MM(4,5) = 0; MM(4,6) = c1*mfoot; 
MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
MM(5,5) = mfoot + mpelvis + msoftseries + msoftstomach; MM(5,6) = s1*mfoot; 
MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
MM(6,5) = MM(5,6); MM(6,6) = mfoot; 

% righthand side terms
rhs(1) = -(q6*mfoot*(2*u1*(u2 + u6) + g*cos(q1 - gslope))) - ...
q2*(2*u1*(u6*mfoot + u2*(mfoot + msoftseries)) + g*(mfoot + ...
msoftseries)*cos(q1 - gslope)); 
rhs(2) = -(u2*cleg) + kleg*(-q2 + lleg) + (q6*mfoot + q2*(mfoot + ...
msoftseries))*(u1*u1) - g*mfoot*sin(q1 - gslope) - g*msoftseries*sin(q1 - ...
gslope); 
rhs(3) = -(u3*csoftstomach) - q3*ksoftstomach + ksoftstomach*lsoftstomach - ...
g*msoftstomach*cos(gslope); 
rhs(4) = 2*s1*u1*u2*mfoot + 2*s1*u1*u6*mfoot + 2*s1*u1*u2*msoftseries + ...
c1*q6*mfoot*(u1*u1) + c1*q2*(mfoot + msoftseries)*(u1*u1) + ...
g*mfoot*sin(gslope) + g*mpelvis*sin(gslope) + g*msoftseries*sin(gslope) + ...
g*msoftstomach*sin(gslope); 
rhs(5) = -2*c1*u1*u2*mfoot - 2*c1*u1*u6*mfoot - 2*c1*u1*u2*msoftseries - ...
g*mfoot*cos(gslope) - g*mpelvis*cos(gslope) - g*msoftseries*cos(gslope) - ...
g*msoftstomach*cos(gslope) + q6*s1*mfoot*(u1*u1) + q2*s1*(mfoot + ...
msoftseries)*(u1*u1); 
rhs(6) = -(u6*csoftseries) + ksoftseries*(-q6 + lsoftseries) + (q2 + ...
q6)*mfoot*(u1*u1) - g*mfoot*sin(q1 - gslope); 

udot = MM\rhs;
xdot = [x(6+1:12); udot];

constraintJacobianStance(1,1) = -((q2 + q6)*s1); ...
constraintJacobianStance(1,2) = c1; constraintJacobianStance(1,3) = 0; ...
constraintJacobianStance(1,4) = 1; constraintJacobianStance(1,5) = 0; ...
constraintJacobianStance(1,6) = c1; 
constraintJacobianStance(2,1) = c1*(q2 + q6); constraintJacobianStance(2,2) = ...
s1; constraintJacobianStance(2,3) = 0; constraintJacobianStance(2,4) = 0; ...
constraintJacobianStance(2,5) = 1; constraintJacobianStance(2,6) = s1; 


constraintJacobianStanceDot(1,1) = -(c1*(q2 + q6)*u1) - s1*(u2 + u6); ...
constraintJacobianStanceDot(1,2) = -(s1*u1); constraintJacobianStanceDot(1,3) ...
= 0; constraintJacobianStanceDot(1,4) = 0; constraintJacobianStanceDot(1,5) = ...
0; constraintJacobianStanceDot(1,6) = -(s1*u1); 
constraintJacobianStanceDot(2,1) = -((q2 + q6)*s1*u1) + c1*(u2 + u6); ...
constraintJacobianStanceDot(2,2) = c1*u1; constraintJacobianStanceDot(2,3) = ...
0; constraintJacobianStanceDot(2,4) = 0; constraintJacobianStanceDot(2,5) = ...
0; constraintJacobianStanceDot(2,6) = c1*u1; 


constraintJacobianAerial(1,1) = 1; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 0; constraintJacobianAerial(1,4) = 0; ...
constraintJacobianAerial(1,5) = 0; constraintJacobianAerial(1,6) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 1; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 0; ...
constraintJacobianAerial(2,5) = 0; constraintJacobianAerial(2,6) = 0; 
constraintJacobianAerial(3,1) = 0; constraintJacobianAerial(3,2) = 0; ...
constraintJacobianAerial(3,3) = 0; constraintJacobianAerial(3,4) = 0; ...
constraintJacobianAerial(3,5) = 0; constraintJacobianAerial(3,6) = 1; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; ...
constraintJacobianAerialDot(1,5) = 0; constraintJacobianAerialDot(1,6) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; ...
constraintJacobianAerialDot(2,5) = 0; constraintJacobianAerialDot(2,6) = 0; 
constraintJacobianAerialDot(3,1) = 0; constraintJacobianAerialDot(3,2) = 0; ...
constraintJacobianAerialDot(3,3) = 0; constraintJacobianAerialDot(3,4) = 0; ...
constraintJacobianAerialDot(3,5) = 0; constraintJacobianAerialDot(3,6) = 0; 


kineticEnergy = (2*c1*u2*u4*mfoot + 2*s1*u2*u5*mfoot - 2*q6*u1*(s1*u4 - ...
c1*u5)*mfoot + 2*u2*u6*mfoot + 2*c1*u4*u6*mfoot + 2*s1*u5*u6*mfoot + ...
2*c1*u2*u4*msoftseries + 2*s1*u2*u5*msoftseries + 2*q2*u1*(q6*u1*mfoot - ...
(s1*u4 - c1*u5)*(mfoot + msoftseries)) + 2*u3*u5*msoftstomach + (mfoot + ...
msoftseries)*(q2*q2)*(u1*u1) + mfoot*(q6*q6)*(u1*u1) + mfoot*(u2*u2) + ...
msoftseries*(u2*u2) + msoftstomach*(u3*u3) + mfoot*(u4*u4) + mpelvis*(u4*u4) ...
+ msoftseries*(u4*u4) + msoftstomach*(u4*u4) + mfoot*(u5*u5) + ...
mpelvis*(u5*u5) + msoftseries*(u5*u5) + msoftstomach*(u5*u5) + ...
mfoot*(u6*u6))/2.;

potentialEnergy = (-2*q6*ksoftseries*lsoftseries + 2*q5*g*mfoot*cos(gslope) + ...
2*q5*g*mpelvis*cos(gslope) + 2*q5*g*msoftseries*cos(gslope) + ...
2*q5*g*msoftstomach*cos(gslope) + q3*(-2*ksoftstomach*lsoftstomach + ...
2*g*msoftstomach*cos(gslope)) + kleg*(q2*q2) + ksoftstomach*(q3*q3) + ...
ksoftseries*(q6*q6) + kleg*(lleg*lleg) + ...
ksoftseries*(lsoftseries*lsoftseries) + ...
ksoftstomach*(lsoftstomach*lsoftstomach) + 2*q6*g*mfoot*sin(q1 - gslope) - ...
2*q2*(kleg*lleg - g*(mfoot + msoftseries)*sin(q1 - gslope)) - ...
2*q4*g*mfoot*sin(gslope) - 2*q4*g*mpelvis*sin(gslope) - ...
2*q4*g*msoftseries*sin(gslope) - 2*q4*g*msoftstomach*sin(gslope))/2.;

PEgrav = -(mpelvis*(-(q5*g*cos(gslope)) + q4*g*sin(gslope))) - ...
msoftstomach*(-((q3 + q5)*g*cos(gslope)) + q4*g*sin(gslope)) - ...
msoftseries*(-(q5*g*cos(gslope)) - q2*g*power(c1*c1 + s1*s1,-0.5)*sin(q1 - ...
gslope) + q4*g*sin(gslope)) - mfoot*(-(q5*g*cos(gslope)) - g*(q2*power(c1*c1 ...
+ s1*s1,-0.5) + q6*power(c1*c1 + s1*s1,-0.5))*sin(q1 - gslope) + ...
q4*g*sin(gslope));

PEspring = (kleg*((q2 - lleg)*(q2 - lleg)))/2. + (ksoftseries*((q6 - ...
lsoftseries)*(q6 - lsoftseries)))/2. + (ksoftstomach*((q3 - lsoftstomach)*(q3 ...
- lsoftstomach)))/2.;

comvx = (u4*mfoot + u4*mpelvis + u4*msoftseries - s1*u1*(q6*mfoot + q2*(mfoot ...
+ msoftseries)) + c1*(u6*mfoot + u2*(mfoot + msoftseries)) + ...
u4*msoftstomach)*power(mfoot + mpelvis + msoftseries + msoftstomach,-1);

comvy = (u5*mfoot + u5*mpelvis + u5*msoftseries + c1*u1*(q6*mfoot + q2*(mfoot ...
+ msoftseries)) + s1*(u6*mfoot + u2*(mfoot + msoftseries)) + (u3 + ...
u5)*msoftstomach)*power(mfoot + mpelvis + msoftseries + msoftstomach,-1);

points.foot(1) = q4 + c1*(q2 + q6); 
points.foot(2) = q5 + (q2 + q6)*s1; 


points.pelvis(1) = q4; 
points.pelvis(2) = q5; 


points.softstomach(1) = q4; 
points.softstomach(2) = q3 + q5; 


points.COMpos(1) = c1*((q2 + q6)*mfoot + q2*msoftseries)*power(mfoot + ...
mpelvis + msoftseries + msoftstomach,-1) + (q4*mfoot + q4*mpelvis + ...
q4*msoftseries + q4*msoftstomach)*power(mfoot + mpelvis + msoftseries + ...
msoftstomach,-1); 
points.COMpos(2) = s1*((q2 + q6)*mfoot + q2*msoftseries)*power(mfoot + ...
mpelvis + msoftseries + msoftstomach,-1) + (q5*mfoot + q5*mpelvis + ...
q5*msoftseries + (q3 + q5)*msoftstomach)*power(mfoot + mpelvis + msoftseries ...
+ msoftstomach,-1); 


points.softseries(1) = c1*q2 + q4; 
points.softseries(2) = q5 + q2*s1; 


velJacobian(1,1) = 0; velJacobian(1,2) = 0; velJacobian(1,3) = 0; ...
velJacobian(1,4) = 1; velJacobian(1,5) = 0; velJacobian(1,6) = 0; 
velJacobian(2,1) = 0; velJacobian(2,2) = 0; velJacobian(2,3) = 0; ...
velJacobian(2,4) = 0; velJacobian(2,5) = 1; velJacobian(2,6) = 0; 
velJacobian(3,1) = 0; velJacobian(3,2) = 0; velJacobian(3,3) = 0; ...
velJacobian(3,4) = 1; velJacobian(3,5) = 0; velJacobian(3,6) = 0; 
velJacobian(4,1) = 0; velJacobian(4,2) = 0; velJacobian(4,3) = 1; ...
velJacobian(4,4) = 0; velJacobian(4,5) = 1; velJacobian(4,6) = 0; 
velJacobian(5,1) = -(q2*s1) - q6*s1; velJacobian(5,2) = c1; velJacobian(5,3) ...
= 0; velJacobian(5,4) = 1; velJacobian(5,5) = 0; velJacobian(5,6) = c1; 
velJacobian(6,1) = c1*q2 + c1*q6; velJacobian(6,2) = s1; velJacobian(6,3) = ...
0; velJacobian(6,4) = 0; velJacobian(6,5) = 1; velJacobian(6,6) = s1; 
velJacobian(7,1) = -(q2*s1); velJacobian(7,2) = c1; velJacobian(7,3) = 0; ...
velJacobian(7,4) = 1; velJacobian(7,5) = 0; velJacobian(7,6) = 0; 
velJacobian(8,1) = c1*q2; velJacobian(8,2) = s1; velJacobian(8,3) = 0; ...
velJacobian(8,4) = 0; velJacobian(8,5) = 1; velJacobian(8,6) = 0; 


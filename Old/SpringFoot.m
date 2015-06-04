function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6); 
u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12); 

c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 

MM = zeros(6,6); rhs = zeros(6,1);

% Mass Matrix
MM(1,1) = mfoot + mheel + mpelvis + mtoe; MM(1,2) = 0; MM(1,3) = ...
-(q4*s3*(mfoot + mheel + mtoe)); MM(1,4) = c3*(mfoot + mheel + mtoe); MM(1,5) ...
= -(s5*ltoe*mtoe); MM(1,6) = -(s6*lheel*mheel); 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + mheel + mpelvis + mtoe; MM(2,3) = ...
c3*q4*(mfoot + mheel + mtoe); MM(2,4) = s3*(mfoot + mheel + mtoe); MM(2,5) = ...
c5*ltoe*mtoe; MM(2,6) = c6*lheel*mheel; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = (mfoot + mheel + ...
mtoe)*(q4*q4); MM(3,4) = 0; MM(3,5) = c3m5*q4*ltoe*mtoe; MM(3,6) = ...
c3m6*q4*lheel*mheel; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot + ...
mheel + mtoe; MM(4,5) = s3m5*ltoe*mtoe; MM(4,6) = s3m6*lheel*mheel; 
MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
MM(5,5) = mtoe*(ltoe*ltoe); MM(5,6) = 0; 
MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
MM(6,5) = MM(5,6); MM(6,6) = mheel*(lheel*lheel); 

% righthand side terms
rhs(1) = 2*u4*(c3*cleg + s3*u3*(mfoot + mheel + mtoe)) + c3*q4*(mfoot + mheel ...
+ mtoe)*(u3*u3) + c5*ltoe*mtoe*(u5*u5) + c6*lheel*mheel*(u6*u6) + ...
g*mfoot*sin(gslope) + g*mheel*sin(gslope) + g*mpelvis*sin(gslope) + ...
g*mtoe*sin(gslope); 
rhs(2) = 2*u4*(s3*cleg - c3*u3*(mfoot + mheel + mtoe)) - g*mfoot*cos(gslope) ...
- g*mheel*cos(gslope) - g*mpelvis*cos(gslope) - g*mtoe*cos(gslope) + ...
q4*s3*(mfoot + mheel + mtoe)*(u3*u3) + s5*ltoe*mtoe*(u5*u5) + ...
s6*lheel*mheel*(u6*u6); 
rhs(3) = -(u3*cachilles) + u6*cachilles - q3*kachilles + q6*kachilles + ...
achillesangle*kachilles - q4*(2*u3*u4*(mfoot + mheel + mtoe) + g*mfoot*cos(q3 ...
- gslope) + g*mheel*cos(q3 - gslope) + g*mtoe*cos(q3 - gslope) + ...
s3m5*ltoe*mtoe*(u5*u5) + s3m6*lheel*mheel*(u6*u6)); 
rhs(4) = u4*cleg + kleg*lleg + q4*(-kleg + (mfoot + mheel + mtoe)*(u3*u3)) + ...
c3m5*ltoe*mtoe*(u5*u5) + c3m6*lheel*mheel*(u6*u6) - g*mfoot*sin(q3 - gslope) ...
- g*mheel*sin(q3 - gslope) - g*mtoe*sin(q3 - gslope); 
rhs(5) = -(u5*cfoot) + u6*cfoot - q5*kfoot + q6*kfoot + footangle*kfoot - ...
2*c3m5*u3*u4*ltoe*mtoe - g*ltoe*mtoe*cos(q5 - gslope) + ...
q4*s3m5*ltoe*mtoe*(u3*u3); 
rhs(6) = (u3 - u6)*cachilles + (u5 - u6)*cfoot - (-q3 + q6 + ...
achillesangle)*kachilles - (-q5 + q6 + footangle)*kfoot - ...
2*c3m6*u3*u4*lheel*mheel - g*lheel*mheel*cos(q6 - gslope) + ...
q4*s3m6*lheel*mheel*(u3*u3); 

udot = MM\rhs;
xdot = [x(6+1:12); udot];

constraintJacobianHeel(1,1) = 1; constraintJacobianHeel(1,2) = 0; ...
constraintJacobianHeel(1,3) = -(q4*s3); constraintJacobianHeel(1,4) = c3; ...
constraintJacobianHeel(1,5) = 0; constraintJacobianHeel(1,6) = -(s6*lheel); 
constraintJacobianHeel(2,1) = 0; constraintJacobianHeel(2,2) = 1; ...
constraintJacobianHeel(2,3) = c3*q4; constraintJacobianHeel(2,4) = s3; ...
constraintJacobianHeel(2,5) = 0; constraintJacobianHeel(2,6) = c6*lheel; 


constraintJacobianHeelDot(1,1) = 0; constraintJacobianHeelDot(1,2) = 0; ...
constraintJacobianHeelDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianHeelDot(1,4) = -(s3*u3); constraintJacobianHeelDot(1,5) = ...
0; constraintJacobianHeelDot(1,6) = -(c6*u6*lheel); 
constraintJacobianHeelDot(2,1) = 0; constraintJacobianHeelDot(2,2) = 0; ...
constraintJacobianHeelDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianHeelDot(2,4) = c3*u3; constraintJacobianHeelDot(2,5) = 0; ...
constraintJacobianHeelDot(2,6) = -(s6*u6*lheel); 


constraintJacobianHeelToe(1,1) = 1; constraintJacobianHeelToe(1,2) = 0; ...
constraintJacobianHeelToe(1,3) = -(q4*s3); constraintJacobianHeelToe(1,4) = ...
c3; constraintJacobianHeelToe(1,5) = 0; constraintJacobianHeelToe(1,6) = ...
-(s6*lheel); 
constraintJacobianHeelToe(2,1) = 0; constraintJacobianHeelToe(2,2) = 1; ...
constraintJacobianHeelToe(2,3) = c3*q4; constraintJacobianHeelToe(2,4) = s3; ...
constraintJacobianHeelToe(2,5) = 0; constraintJacobianHeelToe(2,6) = ...
c6*lheel; 
constraintJacobianHeelToe(3,1) = 0; constraintJacobianHeelToe(3,2) = 1; ...
constraintJacobianHeelToe(3,3) = c3*q4; constraintJacobianHeelToe(3,4) = s3; ...
constraintJacobianHeelToe(3,5) = c5*ltoe; constraintJacobianHeelToe(3,6) = 0; 


constraintJacobianHeelToeDot(1,1) = 0; constraintJacobianHeelToeDot(1,2) = 0; ...
constraintJacobianHeelToeDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianHeelToeDot(1,4) = -(s3*u3); ...
constraintJacobianHeelToeDot(1,5) = 0; constraintJacobianHeelToeDot(1,6) = ...
-(c6*u6*lheel); 
constraintJacobianHeelToeDot(2,1) = 0; constraintJacobianHeelToeDot(2,2) = 0; ...
constraintJacobianHeelToeDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianHeelToeDot(2,4) = c3*u3; constraintJacobianHeelToeDot(2,5) ...
= 0; constraintJacobianHeelToeDot(2,6) = -(s6*u6*lheel); 
constraintJacobianHeelToeDot(3,1) = 0; constraintJacobianHeelToeDot(3,2) = 0; ...
constraintJacobianHeelToeDot(3,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianHeelToeDot(3,4) = c3*u3; constraintJacobianHeelToeDot(3,5) ...
= -(s5*u5*ltoe); constraintJacobianHeelToeDot(3,6) = 0; 


constraintJacobianToe(1,1) = 1; constraintJacobianToe(1,2) = 0; ...
constraintJacobianToe(1,3) = -(q4*s3); constraintJacobianToe(1,4) = c3; ...
constraintJacobianToe(1,5) = -(s5*ltoe); constraintJacobianToe(1,6) = 0; 
constraintJacobianToe(2,1) = 0; constraintJacobianToe(2,2) = 1; ...
constraintJacobianToe(2,3) = c3*q4; constraintJacobianToe(2,4) = s3; ...
constraintJacobianToe(2,5) = c5*ltoe; constraintJacobianToe(2,6) = 0; 


constraintJacobianToeDot(1,1) = 0; constraintJacobianToeDot(1,2) = 0; ...
constraintJacobianToeDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianToeDot(1,4) = -(s3*u3); constraintJacobianToeDot(1,5) = ...
-(c5*u5*ltoe); constraintJacobianToeDot(1,6) = 0; 
constraintJacobianToeDot(2,1) = 0; constraintJacobianToeDot(2,2) = 0; ...
constraintJacobianToeDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianToeDot(2,4) = c3*u3; constraintJacobianToeDot(2,5) = ...
-(s5*u5*ltoe); constraintJacobianToeDot(2,6) = 0; 


constraintJacobianAerial(1,1) = 0; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 1; constraintJacobianAerial(1,4) = 0; ...
constraintJacobianAerial(1,5) = 0; constraintJacobianAerial(1,6) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 0; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 1; ...
constraintJacobianAerial(2,5) = 0; constraintJacobianAerial(2,6) = 0; 
constraintJacobianAerial(3,1) = 0; constraintJacobianAerial(3,2) = 0; ...
constraintJacobianAerial(3,3) = 0; constraintJacobianAerial(3,4) = 0; ...
constraintJacobianAerial(3,5) = 1; constraintJacobianAerial(3,6) = 0; 
constraintJacobianAerial(4,1) = 0; constraintJacobianAerial(4,2) = 0; ...
constraintJacobianAerial(4,3) = 0; constraintJacobianAerial(4,4) = 0; ...
constraintJacobianAerial(4,5) = 0; constraintJacobianAerial(4,6) = 1; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; ...
constraintJacobianAerialDot(1,5) = 0; constraintJacobianAerialDot(1,6) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; ...
constraintJacobianAerialDot(2,5) = 0; constraintJacobianAerialDot(2,6) = 0; 
constraintJacobianAerialDot(3,1) = 0; constraintJacobianAerialDot(3,2) = 0; ...
constraintJacobianAerialDot(3,3) = 0; constraintJacobianAerialDot(3,4) = 0; ...
constraintJacobianAerialDot(3,5) = 0; constraintJacobianAerialDot(3,6) = 0; 
constraintJacobianAerialDot(4,1) = 0; constraintJacobianAerialDot(4,2) = 0; ...
constraintJacobianAerialDot(4,3) = 0; constraintJacobianAerialDot(4,4) = 0; ...
constraintJacobianAerialDot(4,5) = 0; constraintJacobianAerialDot(4,6) = 0; 


kineticEnergy = (2*c3m6*q4*u3*u6*lheel*mheel + 2*s3m6*u4*u6*lheel*mheel + ...
2*c3m5*q4*u3*u5*ltoe*mtoe + 2*s3m5*u4*u5*ltoe*mtoe - 2*u1*(s6*u6*lheel*mheel ...
+ s5*u5*ltoe*mtoe + q4*s3*u3*(mfoot + mheel + mtoe) - c3*u4*(mfoot + mheel + ...
mtoe)) + 2*u2*(c6*u6*lheel*mheel + c5*u5*ltoe*mtoe + c3*q4*u3*(mfoot + mheel ...
+ mtoe) + s3*u4*(mfoot + mheel + mtoe)) + (mfoot + mheel + mpelvis + ...
mtoe)*(u1*u1) + (mfoot + mheel + mpelvis + mtoe)*(u2*u2) + ...
mfoot*(q4*q4)*(u3*u3) + mheel*(q4*q4)*(u3*u3) + mtoe*(q4*q4)*(u3*u3) + ...
mfoot*(u4*u4) + mheel*(u4*u4) + mtoe*(u4*u4) + mheel*(u6*u6)*(lheel*lheel) + ...
mtoe*(u5*u5)*(ltoe*ltoe))/2.;

potentialEnergy = (-2*q3*q6*kachilles - 2*q3*achillesangle*kachilles + ...
2*q6*achillesangle*kachilles - 2*q5*q6*kfoot - 2*q5*footangle*kfoot + ...
2*q6*footangle*kfoot - 2*q4*kleg*lleg + 2*q2*g*(mfoot + mheel + mpelvis + ...
mtoe)*cos(gslope) + kachilles*(q3*q3) + kleg*(q4*q4) + kfoot*(q5*q5) + ...
kachilles*(q6*q6) + kfoot*(q6*q6) + kachilles*(achillesangle*achillesangle) + ...
kfoot*(footangle*footangle) + kleg*(lleg*lleg) + 2*q4*g*mfoot*sin(q3 - ...
gslope) + 2*q4*g*mheel*sin(q3 - gslope) + 2*q4*g*mtoe*sin(q3 - gslope) + ...
2*g*ltoe*mtoe*sin(q5 - gslope) + 2*g*lheel*mheel*sin(q6 - gslope) - ...
2*q1*g*(mfoot + mheel + mpelvis + mtoe)*sin(gslope))/2.;

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope))) - ...
mfoot*(-(q2*g*cos(gslope)) - q4*g*sin(q3 - gslope) + q1*g*sin(gslope)) - ...
mtoe*(-(q2*g*cos(gslope)) - q4*s3*g*cos(gslope) - g*ltoe*sin(q5 - gslope) + ...
q1*g*sin(gslope) + c3*q4*g*sin(gslope)) - mheel*(-(q2*g*cos(gslope)) - ...
q4*s3*g*cos(gslope) - g*lheel*sin(q6 - gslope) + q1*g*sin(gslope) + ...
c3*q4*g*sin(gslope));

PEspring = (kachilles*((q3 - q6 - achillesangle)*(q3 - q6 - ...
achillesangle)))/2. + (kfoot*((q5 - q6 - footangle)*(q5 - q6 - ...
footangle)))/2. + (kleg*((q4 - lleg)*(q4 - lleg)))/2.;

comvx = (c3*u4*mfoot + c3*u4*mheel - s6*u6*lheel*mheel + c3*u4*mtoe - ...
s5*u5*ltoe*mtoe - q4*s3*u3*(mfoot + mheel + mtoe) + u1*(mfoot + mheel + ...
mpelvis + mtoe))*power(mfoot + mheel + mpelvis + mtoe,-1);

comvy = (s3*u4*mfoot + s3*u4*mheel + c6*u6*lheel*mheel + s3*u4*mtoe + ...
c5*u5*ltoe*mtoe + c3*q4*u3*(mfoot + mheel + mtoe) + u2*(mfoot + mheel + ...
mpelvis + mtoe))*power(mfoot + mheel + mpelvis + mtoe,-1);

points.foot(1) = q1 + c3*q4; 
points.foot(2) = q2 + q4*s3; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COMpos(1) = c6*lheel*mheel*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
c5*ltoe*mtoe*power(mfoot + mheel + mpelvis + mtoe,-1) + (q1*mfoot + q1*mheel ...
+ q1*mpelvis + q1*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
c3*(q4*mfoot + q4*mheel + q4*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1); 
points.COMpos(2) = s6*lheel*mheel*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
s5*ltoe*mtoe*power(mfoot + mheel + mpelvis + mtoe,-1) + (q2*mfoot + q2*mheel ...
+ q2*mpelvis + q2*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
s3*(q4*mfoot + q4*mheel + q4*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1); 


points.heel(1) = q1 + c3*q4 + c6*lheel; 
points.heel(2) = q2 + q4*s3 + s6*lheel; 


points.toe(1) = q1 + c3*q4 + c5*ltoe; 
points.toe(2) = q2 + q4*s3 + s5*ltoe; 


velJacobian(1,1) = 1; velJacobian(1,2) = 0; velJacobian(1,3) = 0; ...
velJacobian(1,4) = 0; velJacobian(1,5) = 0; velJacobian(1,6) = 0; 
velJacobian(2,1) = 0; velJacobian(2,2) = 1; velJacobian(2,3) = 0; ...
velJacobian(2,4) = 0; velJacobian(2,5) = 0; velJacobian(2,6) = 0; 
velJacobian(3,1) = 1; velJacobian(3,2) = 0; velJacobian(3,3) = -(q4*s3); ...
velJacobian(3,4) = c3; velJacobian(3,5) = 0; velJacobian(3,6) = 0; 
velJacobian(4,1) = 0; velJacobian(4,2) = 1; velJacobian(4,3) = c3*q4; ...
velJacobian(4,4) = s3; velJacobian(4,5) = 0; velJacobian(4,6) = 0; 
velJacobian(5,1) = 1; velJacobian(5,2) = 0; velJacobian(5,3) = -(q4*s3); ...
velJacobian(5,4) = c3; velJacobian(5,5) = 0; velJacobian(5,6) = -(s6*lheel); 
velJacobian(6,1) = 0; velJacobian(6,2) = 1; velJacobian(6,3) = c3*q4; ...
velJacobian(6,4) = s3; velJacobian(6,5) = 0; velJacobian(6,6) = c6*lheel; 
velJacobian(7,1) = 1; velJacobian(7,2) = 0; velJacobian(7,3) = -(q4*s3); ...
velJacobian(7,4) = c3; velJacobian(7,5) = -(s5*ltoe); velJacobian(7,6) = 0; 
velJacobian(8,1) = 0; velJacobian(8,2) = 1; velJacobian(8,3) = c3*q4; ...
velJacobian(8,4) = s3; velJacobian(8,5) = c5*ltoe; velJacobian(8,6) = 0; 


vpelvisx = u1;

vpelvisy = u2;

vheelx = u1 - q4*s3*u3 + c3*u4 - s6*u6*lheel;

vheely = u2 + c3*q4*u3 + s3*u4 + c6*u6*lheel;

function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6); 
u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12); 

c5 = cos(q5); c6 = cos(q6); s5 = sin(q5); s6 = sin(q6); c5m6 = cos(q5 - q6); s5m6 = sin(q5 - q6); 

MM = zeros(6,6); rhs = zeros(6,1);

% Mass Matrix
MM(1,1) = mpelvis; MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = 0; MM(1,5) = 0; ...
MM(1,6) = 0; 
MM(2,1) = 0; MM(2,2) = mpelvis; MM(2,3) = 0; MM(2,4) = 0; MM(2,5) = 0; ...
MM(2,6) = 0; 
MM(3,1) = 0; MM(3,2) = 0; MM(3,3) = 0; MM(3,4) = 0; MM(3,5) = 0; MM(3,6) = 0; 
MM(4,1) = 0; MM(4,2) = 0; MM(4,3) = 0; MM(4,4) = 0; MM(4,5) = 0; MM(4,6) = 0; 
MM(5,1) = -(s5*lthigh*(1 + mfoot)); MM(5,2) = c5*lthigh*(1 + mfoot); MM(5,3) ...
= 0; MM(5,4) = 0; MM(5,5) = (1 + mfoot)*(lthigh*lthigh); MM(5,6) = ...
c5m6*lshank*lthigh*mfoot; 
MM(6,1) = -(s6*lshank*mfoot); MM(6,2) = c6*lshank*mfoot; MM(6,3) = 0; MM(6,4) ...
= 0; MM(6,5) = c5m6*lshank*lthigh*mfoot; MM(6,6) = mfoot*(lshank*lshank); 

% righthand side terms
rhs(1) = g*mpelvis*sin(gslope); 
rhs(2) = -(g*mpelvis*cos(gslope)); 
rhs(3) = 0; 
rhs(4) = -(u4*cstance) - q4*kstance + kstance*stancel; 
rhs(5) = -(u5*chip) - u5*cknee + u6*cknee + hipl*khip + q6*kknee - q5*(khip + ...
kknee) + kknee*kneel - 2*g*lthigh*mfoot*cos(q5 - gslope) - ...
s5m6*lshank*lthigh*mfoot*(u6*u6); 
rhs(6) = (u5 - u6)*cknee - kknee*(-q5 + q6 + kneel) - g*lshank*mfoot*cos(q6 - ...
gslope) + s5m6*lshank*lthigh*mfoot*(u5*u5); 

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


constraintJacobianAerial(1,1) = 0; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 1; constraintJacobianAerial(1,4) = 0; ...
constraintJacobianAerial(1,5) = 0; constraintJacobianAerial(1,6) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 0; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 1; ...
constraintJacobianAerial(2,5) = 0; constraintJacobianAerial(2,6) = 0; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; ...
constraintJacobianAerialDot(1,5) = 0; constraintJacobianAerialDot(1,6) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; ...
constraintJacobianAerialDot(2,5) = 0; constraintJacobianAerialDot(2,6) = 0; 


kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kstance*((-q4 + stancel)*(-q4 + stancel)))/2. + ...
g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

kineticEnergy2 = (mknee*(-2*s5*u1*u5*lthigh + 2*c5*u2*u5*lthigh + u1*u1 + ...
u2*u2 + u5*u5*(lthigh*lthigh)) + mfoot*(-2*s6*u1*u6*lshank + ...
2*c6*u2*u6*lshank - 2*s5*u1*u5*lthigh + 2*c5*u2*u5*lthigh + ...
2*c5m6*u5*u6*lshank*lthigh + u1*u1 + u2*u2 + u6*u6*(lshank*lshank) + ...
u5*u5*(lthigh*lthigh)))/2.;

potentialEnergy2 = 2*q2*g*cos(gslope) + s6*g*lshank*cos(gslope) + (khip*((q5 ...
- hipl)*(q5 - hipl)))/2. + (kknee*((q5 - q6 - kneel)*(q5 - q6 - kneel)))/2. + ...
2*g*lthigh*sin(q5 - gslope) - 2*q1*g*sin(gslope) - c6*g*lshank*sin(gslope);

PEgrav2 = 2*q2*g*cos(gslope) + s6*g*lshank*cos(gslope) + 2*g*lthigh*sin(q5 - ...
gslope) - 2*q1*g*sin(gslope) - c6*g*lshank*sin(gslope);

PEspring2 = (khip*((q5 - hipl)*(q5 - hipl)))/2. + (kknee*((q5 - q6 - ...
kneel)*(q5 - q6 - kneel)))/2.;

stanceE = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

kneeE = (kknee*((q5 - q6 - kneel)*(q5 - q6 - kneel)))/2.;

hipE = (khip*((q5 - hipl)*(q5 - hipl)))/2.;

points.stancefoot(1) = q1 + q4*cos(q3); 
points.stancefoot(2) = q2 + q4*sin(q3); 


points.swingfoot(1) = q1 + c6*lshank + c5*lthigh; 
points.swingfoot(2) = q2 + s6*lshank + s5*lthigh; 


points.knee(1) = q1 + c5*lthigh; 
points.knee(2) = q2 + s5*lthigh; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 


vels.stancefoot(1) = u1 + u4*cos(q3) - q4*u3*sin(q3); 
vels.stancefoot(2) = u2 + q4*u3*cos(q3) + u4*sin(q3); 


vels.swingfoot(1) = u1 - s6*u6*lshank - s5*u5*lthigh; 
vels.swingfoot(2) = u2 + c6*u6*lshank + c5*u5*lthigh; 


vels.knee(1) = u1 - s5*u5*lthigh; 
vels.knee(2) = u2 + c5*u5*lthigh; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 


Jswingfoot(1,1) = 1; Jswingfoot(1,2) = 0; Jswingfoot(1,3) = 0; ...
Jswingfoot(1,4) = 0; Jswingfoot(1,5) = -(s5*lthigh); Jswingfoot(1,6) = ...
-(s6*lshank); 
Jswingfoot(2,1) = 0; Jswingfoot(2,2) = 1; Jswingfoot(2,3) = 0; ...
Jswingfoot(2,4) = 0; Jswingfoot(2,5) = c5*lthigh; Jswingfoot(2,6) = ...
c6*lshank; 


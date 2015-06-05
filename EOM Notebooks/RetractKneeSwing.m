function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); 
u1 = x(5); u2 = x(6); u3 = x(7); u4 = x(8); 

c3 = cos(q3); c4 = cos(q4); s3 = sin(q3); s4 = sin(q4); c3m4 = cos(q3 - q4); s3m4 = sin(q3 - q4); 

MM = zeros(4,4); rhs = zeros(4,1);

% Mass Matrix
MM(1,1) = mfoot + mknee + mpelvis; MM(1,2) = 0; MM(1,3) = -(s3*lthigh*(mfoot ...
+ mknee)); MM(1,4) = -(s4*lshank*mfoot); 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + mknee + mpelvis; MM(2,3) = ...
c3*lthigh*(mfoot + mknee); MM(2,4) = c4*lshank*mfoot; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = (mfoot + ...
mknee)*(lthigh*lthigh); MM(3,4) = c3m4*lshank*lthigh*mfoot; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = ...
mfoot*(lshank*lshank); 

% righthand side terms
rhs(1) = c3*lthigh*(mfoot + mknee)*(u3*u3) + c4*lshank*mfoot*(u4*u4) + ...
g*(2*mfoot + mpelvis)*sin(gslope); 
rhs(2) = -(g*(2*mfoot + mpelvis)*cos(gslope)) + s3*lthigh*(mfoot + ...
mknee)*(u3*u3) + s4*lshank*mfoot*(u4*u4); 
rhs(3) = hipl*khip + q4*kknee - q3*(khip + kknee) + kknee*kneel - ...
2*g*lthigh*mfoot*cos(q3 - gslope) - s3m4*lshank*lthigh*mfoot*(u4*u4); 
rhs(4) = -(kknee*(-q3 + q4 + kneel)) - g*lshank*mfoot*cos(q4 - gslope) + ...
s3m4*lshank*lthigh*mfoot*(u3*u3); 

udot = MM\rhs;
xdot = [x(4+1:8); udot];

kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (khip*((-q3 + hipl)*(-q3 + hipl)) + kknee*((-q3 + q4 + ...
kneel)*(-q3 + q4 + kneel)) + 2*g*mpelvis*(q2*cos(gslope) - ...
q1*sin(gslope)))/2.;

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (khip*((q3 - hipl)*(q3 - hipl)))/2. + (kknee*((q3 - q4 - ...
kneel)*(q3 - q4 - kneel)))/2.;

points.foot(1) = q1 + c4*lshank + c3*lthigh; 
points.foot(2) = q2 + s4*lshank + s3*lthigh; 


points.knee(1) = q1 + c3*lthigh; 
points.knee(2) = q2 + s3*lthigh; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 


vels.foot(1) = u1 - s4*u4*lshank - s3*u3*lthigh; 
vels.foot(2) = u2 + c4*u4*lshank + c3*u3*lthigh; 


vels.knee(1) = u1 - s3*u3*lthigh; 
vels.knee(2) = u2 + c3*u3*lthigh; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 


accs(1,1) = (-(c3m4*lthigh*(kknee*(-q3 + q4 + kneel) + c4*g*lshank - ...
s4*lshank*xpacc + c4*lshank*ypacc - s3m4*lshank*lthigh*(u3*u3))) + ...
lshank*(-(hipl*khip) - q4*kknee + q3*(khip + kknee) - kknee*kneel + ...
2*c3*g*lthigh - 2*s3*lthigh*xpacc + 2*c3*lthigh*ypacc + ...
s3m4*lshank*lthigh*(u4*u4)))*power(lshank,-1)*power(lthigh,-2)*power(-2 + ...
c3m4*c3m4,-1); accs(1,2) = power(lshank,-2)*power(lthigh,-1)*power(-2 + ...
c3m4*c3m4,-1)*(c3m4*hipl*khip*lshank + c3m4*kknee*kneel*lshank + ...
2*kknee*kneel*lthigh + c4*g*lshank*lthigh + q4*kknee*(c3m4*lshank + 2*lthigh) ...
- q3*(c3m4*(khip + kknee)*lshank + 2*kknee*lthigh) - s4*lshank*lthigh*xpacc + ...
c4*lshank*lthigh*ypacc - g*lshank*lthigh*cos(2*q3 - q4) - ...
lshank*lthigh*ypacc*cos(2*q3 - q4) - 2*s3m4*lshank*(u3*u3)*(lthigh*lthigh) - ...
(lthigh*(u4*u4)*(lshank*lshank)*sin(2*(q3 - q4)))/2. + ...
lshank*lthigh*xpacc*sin(2*q3 - q4)); 


MM(1,1) = -2*s3*lthigh; MM(1,2) = 2*c3*lthigh; MM(1,3) = 2*(lthigh*lthigh); ...
MM(1,4) = c3m4*lshank*lthigh; 
MM(2,1) = -(s4*lshank); MM(2,2) = c4*lshank; MM(2,3) = c3m4*lshank*lthigh; ...
MM(2,4) = lshank*lshank; 


rhs(1) = hipl*khip + q4*kknee - q3*(khip + kknee) + kknee*kneel - ...
2*g*lthigh*cos(q3 - gslope) - s3m4*lshank*lthigh*(u4*u4); 
rhs(2) = -(kknee*(-q3 + q4 + kneel)) - g*lshank*cos(q4 - gslope) + ...
s3m4*lshank*lthigh*(u3*u3); 

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
rhs(3) = -(hipl*khip) + q4*kknee - q3*(khip + kknee) + kknee*kneel + ...
khip*stanceangle - 2*g*lthigh*mfoot*cos(q3 - gslope) - ...
s3m4*lshank*lthigh*mfoot*(u4*u4); 
rhs(4) = -(kknee*(-q3 + q4 + kneel)) - g*lshank*mfoot*cos(q4 - gslope) + ...
s3m4*lshank*lthigh*mfoot*(u3*u3); 

udot = MM\rhs;
xdot = [x(4+1:8); udot];

kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kknee*((-q3 + q4 + kneel)*(-q3 + q4 + kneel)) + khip*((q3 ...
+ hipl - stanceangle)*(q3 + hipl - stanceangle)) + ...
2*g*mpelvis*(q2*cos(gslope) - q1*sin(gslope)))/2.;

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kknee*((q3 - q4 - kneel)*(q3 - q4 - kneel)))/2. + (khip*((-q3 - ...
hipl + stanceangle)*(-q3 - hipl + stanceangle)))/2.;

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


accs(1,1) = -((-(c3m4*lthigh*(kknee*(-q3 + q4 + kneel) + c4*g*lshank*mfoot - ...
s4*lshank*mfoot*xpacc + c4*lshank*mfoot*ypacc - ...
s3m4*lshank*lthigh*mfoot*(u3*u3))) + lshank*(hipl*khip - q4*kknee + q3*(khip ...
+ kknee) - kknee*kneel + 2*c3*g*lthigh*mfoot - khip*stanceangle - ...
s3*lthigh*(1 + mfoot)*xpacc + c3*lthigh*(1 + mfoot)*ypacc + ...
s3m4*lshank*lthigh*mfoot*(u4*u4)))*power(lshank,-1)*power(lthigh,-2)*power(1 ...
+ mfoot - mfoot*(c3m4*c3m4),-1)); accs(1,2) = ...
power(lshank,-2)*(-((-(q3*kknee) + q4*kknee + kknee*kneel + c4*g*lshank*mfoot ...
- s4*lshank*mfoot*xpacc + c4*lshank*mfoot*ypacc - ...
s3m4*lshank*lthigh*mfoot*(u3*u3))*power(mfoot,-1)) + ...
c3m4*(-(c3m4*lthigh*(kknee*(-q3 + q4 + kneel) + c4*g*lshank*mfoot - ...
s4*lshank*mfoot*xpacc + c4*lshank*mfoot*ypacc - ...
s3m4*lshank*lthigh*mfoot*(u3*u3))) + lshank*(hipl*khip - q4*kknee + q3*(khip ...
+ kknee) - kknee*kneel + 2*c3*g*lthigh*mfoot - khip*stanceangle - ...
s3*lthigh*(1 + mfoot)*xpacc + c3*lthigh*(1 + mfoot)*ypacc + ...
s3m4*lshank*lthigh*mfoot*(u4*u4)))*power(lthigh,-1)*power(1 + mfoot - ...
mfoot*(c3m4*c3m4),-1)); 


MM(1,1) = -(s3*lthigh*(1 + mfoot)); MM(1,2) = c3*lthigh*(1 + mfoot); MM(1,3) ...
= (1 + mfoot)*(lthigh*lthigh); MM(1,4) = c3m4*lshank*lthigh*mfoot; 
MM(2,1) = -(s4*lshank*mfoot); MM(2,2) = c4*lshank*mfoot; MM(2,3) = ...
c3m4*lshank*lthigh*mfoot; MM(2,4) = mfoot*(lshank*lshank); 


rhs(1) = -(hipl*khip) + q4*kknee - q3*(khip + kknee) + kknee*kneel + ...
khip*stanceangle - 2*g*lthigh*mfoot*cos(q3 - gslope) - ...
s3m4*lshank*lthigh*mfoot*(u4*u4); 
rhs(2) = -(kknee*(-q3 + q4 + kneel)) - g*lshank*mfoot*cos(q4 - gslope) + ...
s3m4*lshank*lthigh*mfoot*(u3*u3); 


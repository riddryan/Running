function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); 
u1 = x(5); u2 = x(6); u3 = x(7); u4 = x(8); 

c3 = cos(q3); s3 = sin(q3); 

MM = zeros(4,4); rhs = zeros(4,1);

% Mass Matrix
MM(1,1) = mfoot + mpelvis; MM(1,2) = 0; MM(1,3) = -(q4*s3*mfoot); MM(1,4) = ...
c3*mfoot; 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + mpelvis; MM(2,3) = c3*q4*mfoot; MM(2,4) ...
= s3*mfoot; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mfoot*(q4*q4); MM(3,4) = 0; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot; 

% righthand side terms
rhs(1) = 2*s3*u3*u4*mfoot + c3*q4*mfoot*(u3*u3) + g*(mfoot + ...
mpelvis)*sin(gslope); 
rhs(2) = -2*c3*u3*u4*mfoot - g*(mfoot + mpelvis)*cos(gslope) + ...
q4*s3*mfoot*(u3*u3); 
rhs(3) = -(q3*khip) + khip*(-hipl + stanceangle) - q4*mfoot*(2*u3*u4 + ...
g*cos(q3 - gslope)); 
rhs(4) = kswing*swingl + q4*(-kswing + mfoot*(u3*u3)) - g*mfoot*sin(q3 - ...
gslope); 

udot = MM\rhs;
xdot = [x(4+1:8); udot];

constraintJacobianStance(1,1) = 0; constraintJacobianStance(1,2) = 1; ...
constraintJacobianStance(1,3) = c3*q4; constraintJacobianStance(1,4) = s3; 


constraintJacobianStanceDot(1,1) = 0; constraintJacobianStanceDot(1,2) = 0; ...
constraintJacobianStanceDot(1,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianStanceDot(1,4) = c3*u3; 


kineticEnergy = (u1*(-2*q4*s3*u3 + 2*c3*u4) + 2*u2*(c3*q4*u3 + s3*u4) + u1*u1 ...
+ u2*u2 + q4*q4*(u3*u3) + u4*u4)/2.;

potentialEnergy = (2*q2*g*cos(gslope) + khip*((q3 + hipl - stanceangle)*(q3 + ...
hipl - stanceangle)) + kswing*((-q4 + swingl)*(-q4 + swingl)) + 2*q4*g*sin(q3 ...
- gslope) - 2*q1*g*sin(gslope))/2.;

PEgrav = q2*g*cos(gslope) + q4*g*sin(q3 - gslope) - q1*g*sin(gslope);

PEspring = (khip*((-q3 - hipl + stanceangle)*(-q3 - hipl + stanceangle)))/2. ...
+ (kswing*((q4 - swingl)*(q4 - swingl)))/2.;

points.swingfoot(1) = q1 + c3*q4; 
points.swingfoot(2) = q2 + q4*s3; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 


vels.swingfoot(1) = u1 - q4*s3*u3 + c3*u4; 
vels.swingfoot(2) = u2 + c3*q4*u3 + s3*u4; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 


accs(1,1) = (-(q3*khip) + khip*(-hipl + stanceangle) - q4*(2*u3*u4 - s3*xpacc ...
+ c3*(g + ypacc)))*power(q4,-2); accs(1,2) = -(s3*g) + kswing*swingl - ...
c3*xpacc - s3*ypacc + q4*(-kswing + u3*u3); 


MM(1,1) = -(q4*s3); MM(1,2) = c3*q4; MM(1,3) = q4*q4; MM(1,4) = 0; 
MM(2,1) = c3; MM(2,2) = s3; MM(2,3) = 0; MM(2,4) = 1; 


rhs(1) = -(q3*khip) + khip*(-hipl + stanceangle) - q4*(2*u3*u4 + g*cos(q3 - ...
gslope)); 
rhs(2) = kswing*swingl + q4*(-kswing + u3*u3) - g*sin(q3 - gslope); 


C(1,1) = 0; C(1,2) = 1; C(1,3) = c3*q4; C(1,4) = s3; 


CDot(1,1) = 0; CDot(1,2) = 0; CDot(1,3) = -(q4*s3*u3) + c3*u4; CDot(1,4) = ...
c3*u3; 


accsandconstraints(1,1) = (power(q4,-2)*(-2*q3*khip*(s3*s3) + 2*khip*(-hipl + ...
stanceangle)*(s3*s3) + kswing*(q4*q4)*sin(2*q3) - q4*(4*u3*u4 - 2*s3*xpacc + ...
kswing*swingl*sin(2*q3))))/2.; accsandconstraints(1,2) = ...
(power(q4,-1)*(-2*c3*q4*(-(c3*kswing*swingl) + xpacc) + ...
q4*q4*(-2*kswing*(c3*c3) + 2*(u3*u3)) + q3*khip*sin(2*q3) + khip*(hipl - ...
stanceangle)*sin(2*q3)))/2.; accsandconstraints(1,3) = (c3*q3*khip + ...
c3*khip*(hipl - stanceangle) + q4*(g - s3*kswing*swingl + ypacc) + ...
s3*kswing*(q4*q4))*power(q4,-1); 


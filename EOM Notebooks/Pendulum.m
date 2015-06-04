function xdot = fstatederivative(t, x)

% Define constants

% Define forces: 

% State assignments
q1 = x(1); 
u1 = x(2); 



MM = zeros(1,1); rhs = zeros(1,1);

% Mass Matrix
MM(1,1) = mfoot*(lheel*lheel); 

% righthand side terms
rhs(1) = -(q1*kfoot) + footangle*kfoot - g*lheel*mfoot*cos(q1 - gslope); 

udot = MM\rhs;
xdot = [x(1+1:2); udot];

kineticEnergy = (mfoot*(u1*u1)*(lheel*lheel))/2.;

potentialEnergy = (kfoot*((-q1 + footangle)*(-q1 + footangle)))/2. + ...
g*lheel*mfoot*sin(q1 - gslope);

PEgrav = g*lheel*mfoot*sin(q1 - gslope);

PEspring = (kfoot*((q1 - footangle)*(q1 - footangle)))/2.;



syms t q1(t) u1(t)

lheel=1;
mfoot=1;
kfoot = 1;
footangle = 1;
g = 1;
gslope = 0;

MM = sym(zeros(1,1)); rhs = sym(zeros(1,1));

% Mass Matrix
MM(1,1) = mfoot*(lheel*lheel); 

% righthand side terms
rhs(1) = -(q1*kfoot) + footangle*kfoot - g*lheel*mfoot*cos(q1 - gslope); 

eqs = [1 0; 0 MM]*[diff(q1,t);diff(u1,t)] == [u1;rhs];
vars = [q1;u1];

[DAEs,DAEvars,index] = reduceDAEIndex(eqs,vars);
[DAEs,DAEvars] = reduceRedundancies(DAEs,DAEvars);

f = daeFunction(DAEs,DAEvars);
y0est = [pi/4;0]; yp0est = zeros(2,1);
opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
            [y0, yp0] = decic(f, 0, y0est, [], yp0est, [], opt);
            [tt,xx]=ode15i(f,[0 5], y0, yp0, opt);
            
            for i = 1:length(tt)
                q1=xx(i,1);u1=xx(i,2);
                kineticEnergy(i) = (mfoot*(u1*u1)*(lheel*lheel))/2.;
                
                potentialEnergy(i) = (kfoot*((-q1 + footangle)*(-q1 + footangle)))/2. + ...
                    g*lheel*mfoot*sin(q1 - gslope);
                
                PEgrav(i) = g*lheel*mfoot*sin(q1 - gslope);
                
                PEspring(i) = (kfoot*((q1 - footangle)*(q1 - footangle)))/2.;
            end
            
            TotE = potentialEnergy+kineticEnergy;


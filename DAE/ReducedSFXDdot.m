function xdot = ReducedSFXDdot( t , x )
        mpelvis = 1;
        gslope = 0;
        g = 1;
        lleg=1; lheel = .2; ltoe = .2;
        footangle = pi/4;
        kleg=0; kachilles=1; kfoot = 0.1;
        cleg=0.5; cfoot=0.5;
        
        q1=x(1);q2=x(2);q3=x(3);u1=x(4);u2=x(5);u3=x(6);
        c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2);
        
        MM = (zeros(3,3)); rhs = (zeros(3,1));
        

% Mass Matrix
MM(1,1) = mpelvis*(lheel*lheel); MM(1,2) = c1m2*q3*lheel*mpelvis; MM(1,3) = ...
-(s1m2*lheel*mpelvis); 
MM(2,1) = MM(1,2); MM(2,2) = mpelvis*(q3*q3); MM(2,3) = 0; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mpelvis; 

% righthand side terms
rhs(1) = -(u1*cfoot) - q1*kfoot + footangle*kfoot - ...
2*c1m2*u2*u3*lheel*mpelvis - g*lheel*mpelvis*cos(q1 - gslope) - ...
q3*s1m2*lheel*mpelvis*(u2*u2); 
rhs(2) = q3*mpelvis*(-2*u2*u3 - g*cos(q2 - gslope) + s1m2*lheel*(u1*u1)); 
rhs(3) = -(u3*cleg) + kleg*lleg + c1m2*lheel*mpelvis*(u1*u1) + q3*(-kleg + ...
mpelvis*(u2*u2)) - g*mpelvis*sin(q2 - gslope); 
        

accs = MM \ rhs;
xdot = [x(1:3);accs];
end


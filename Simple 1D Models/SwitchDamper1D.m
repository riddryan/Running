function SwitchDamper1D

m=1;
c1=-.5;
c2=.5;
g=1;
restlength=1;

x0 = [restlength;-1];

[t,x] = ode45(@StateDeriv,[0 10],x0);

for i = 1:length(t)
    pos(i) = x(i,1);
    vel(i) = x(i,2);
    
    if vel(i)<0
        force(i) = c1*vel(i);
    else
        force(i) = c2*vel(i);
    end
end

figure
subplot(211)
plot(pos,force)
subplot(212)
plot(vel,force)




function xddot = StateDeriv(t,x)
    xddot = zeros(2,1);
    
    xddot(1) = x(2);
    
    if x(2)<0
        xddot(2) = c1/m*x(2)-g;
    else
        xddot(2) = c2/m*x(2)-g;
    end
end

end
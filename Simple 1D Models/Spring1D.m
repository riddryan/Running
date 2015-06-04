function Spring1D

m=1;
k=5;
g=1;
restlength=1;

x0 = [restlength;-1];

[t,x] = ode45(@StateDeriv,[0 5],x0);

for i = 1:length(t)
    pos(i) = x(i,1);
    vel(i) = x(i,2);
   force(i) = -k*(pos(i)-restlength);
end
figure
subplot(211)
plot(pos,force)
subplot(212)
plot(vel,force)




function xddot = StateDeriv(t,x)
    xddot = zeros(2,1);
    
    xddot(1) = x(2);
    xddot(2) = -k/m*(x(1)-restlength)-g;
end

end
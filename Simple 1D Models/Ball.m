function xdot = Ball(t,x)
g=9.81; %gravity

x1 = x(1); %x1 is position
x2 = x(2); %x2 is velocity

x1dot = x2; % x1dot = x2
x2dot = -g;   % x2dot = -g (acceleration of the ball)

xdot = [x1dot ; x2dot];  %Put the velocity and accleration into a vector


end


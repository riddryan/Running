
tspan = [0  3]; %Let the ball drop for 5 seconds

y0 = 1;  %start the ball at a height of 1
v0 = 0;  %start the ball at zero velocity

x0 = [y0;v0];  %Put the starting height (y0) and starting velocity (v0) into a vector

%Simulate the ball dropping for the span of time in tspan, with initial
%conditions x0

[t,x] = ode45('Ball',tspan,x0);

%Extract positions and velocities from the solution matrix x
positions = x(:,1);  % position of the ball at each instant in time
velocities = x(:,2); % velocity of the ball at each instant in time

%plot the results
figure
plot(t,positions)  %Plot position of the ball versus time
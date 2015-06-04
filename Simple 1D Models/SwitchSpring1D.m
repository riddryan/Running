function [t,pos,vel,force] = SwitchSpring1D(varargin)

ForOneCycle = 1;
SwitchTime = 0.5;
m=1;
k1=5;
k2=3;
g=1;
restlength=1;
x0 = [restlength;-1];
plots = 0;

for i = 1 : 2 : length(varargin)
    option = varargin{i};
    value = varargin{i + 1};
    switch option
        case 'x0'
            x0=value;
        case 'k1'
            k1=value;
        case 'k2'
            k2=value;
        case 'SwitchTime'
            SwitchTime=value;
        case 'restlength'
            restlength=value;
        case 'plots'
            plots=value;
    end
end
      

options=[];
if ForOneCycle
    options=odeset('Events',@OneCycle,'OutputFcn',@odeplot);
end

[t,x] = ode45(@StateDeriv,[0 10],x0,options);

for i = 1:length(t)
    pos(i) = x(i,1);
    vel(i) = x(i,2);
    [~,force(i)] = StateDeriv(t(i),x(i,:));
end

if plots
    figure
    subplot(221)
    plot(t,pos)
    ylabel('Position')
    subplot(223)
    plot(t,vel)
    xlabel('Time')
    ylabel('Velocity')
    
    subplot(222)
    plot(pos,force)
    xlabel('Position')
    ylabel('Force')
    subplot(224)
    plot(vel,force)
    xlabel('Velocity')
    ylabel('Force')
end


function [xddot,springforce] = StateDeriv(t,x)
    xddot = zeros(2,1);
    
    xddot(1) = x(2);
    
    if  t<SwitchTime
        springforce = -k1*(x(1)-restlength);
    else
        springforce = -k2*(x(1)-restlength);
    end
    
    xddot(2) = springforce/m-g;
    

end

    function [value,isterminal,direction]=OneCycle(t,x)
       value = x(1)-x0(1);
       direction=1;
       isterminal=1;
    end

end
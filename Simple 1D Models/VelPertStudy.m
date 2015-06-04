restlength = 1;
vel0 = -1;

pertsize = .1;

[t,pos,vel,force] = SwitchSpring1D('x0',[restlength vel0]);

tbar = t(end); posbar = pos(end); velbar = vel(end); forcebar = force(end);

%Positive velocity perturbation
[t1,pos1,vel1,force1] = SwitchSpring1D('x0',[restlength vel0 + pertsize]);

tf = t1(end); posf = pos1(end); velf = vel1(end); forcef = force1(end);

    figure
    subplot(221)
    plot(t,pos)
    hold on
    plot(t1,pos1,'r')
    legend('Nominal','Positive Velocity Pert')
    ylabel('Position')
    subplot(223)
    plot(t,vel)
    hold on
    plot(t1,vel1,'r')
    xlabel('Time')
    ylabel('Velocity')
    
    subplot(222)
    plot(pos,force)
    hold on
    plot(pos1,force1,'r')
    xlabel('Position')
    ylabel('Force')
    subplot(224)
    plot(vel,force)
    hold on
    plot(vel1,force1,'r')
    xlabel('Velocity')
    ylabel('Force')

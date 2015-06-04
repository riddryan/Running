clearvars
        mpelvis = 1;
        gslope = 0;
        g = 1;
        lleg=1; lheel = .2; ltoe = .2;
        footangle = pi/4;
        kleg=0; kachilles=1; kfoot = 0.1;
        cleg=0.5; cfoot=0.5;
        
x0 = [pi/4 pi/2 1 0 0 0]';
tspan = [0 1];


[tt,xx] = ode15s(@(t,x) ReducedSFXDdot(t,x),tspan,x0);

figure
        plotter = RunnerPlotter;
        
        animstep = ceil(length(tt)/20);
        count = 0;
        for i = 1:animstep:length(tt)
            count = count+1;
            cla;
            x = xx(i,:);
            q1 = x(1); q2 = x(2); q3 = x(3);
            u1 = x(4); u2 = x(5); u3 = x(6);
            
            c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2);
            foot = [cos(q1)*lheel;sin(q1)*lheel];
            pelvis = foot + q3*[cos(q2);sin(q2)];
            
            plot([-5 5],[0 0],'k','LineWidth',2)
            hold on
            
            plotter.plotLine([0;0],foot);
            numcoils=3;
            springwidth=.03;
            plotter.plotSpring(foot(1),foot(2),...
                pelvis(1),pelvis(2),...
                numcoils,1,springwidth,'Color',[0 1 1])
            
            plotter.plotSpring(foot(1),0,foot(1),foot(2),3,.3,.02,'Color',[0 1 1]);
            plotter.plotMass(pelvis);
            xLims = pelvis(1) + [-1 1];
            xlim(xLims);
            
            yLims = pelvis(2) + [-1.5 .5];
            ylim(yLims);
            pause(0.1);
            
            kineticEnergy(count) = (mpelvis*(2*u1*(c1m2*q3*u2 - s1m2*u3)*lheel + q3*q3*(u2*u2) + ...
                u3*u3 + u1*u1*(lheel*lheel)))/2.;
            
            potentialEnergy(count) = (kfoot*((-q1 + footangle)*(-q1 + footangle)) + kleg*((-q3 + ...
                lleg)*(-q3 + lleg)) - 2*g*mpelvis*(-(lheel*sin(q1 - gslope)) - q3*sin(q2 - ...
                gslope)))/2.;
            
        end
        TotE = kineticEnergy + potentialEnergy;
        figure
        plot(TotE)
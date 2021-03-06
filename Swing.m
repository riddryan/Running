classdef Swing < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
    
    properties
        gslope = 0;
        g = 1;
        N = 4;
        %rest lengths
        swingl = .45;
        %rest angles
        hipl = 0.05;

        statestovary = [];
        statestomeasure = [];
        
        impulsecoeff = 2;
        
        sephips = 0;
        
        %Springs
        kswing = 15; khip = 0.92;
        
        SLIPdata = [];
        SLIPx0 = [];
        SLIPxf = [];
        
        rigidlegimpulse = 0;
        usefloorconstraint = 1;
        
        useHSevent = 0;
        
        phases = {'Toe' 'Aerial'};
    end
    
    
    methods (Static)
        
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            savepath = [dir '\Animations\'];
            aviname = [savepath 'Swing1.avi'];
            onephasesim = 0;
            manystep = 0;
            test = 'experiment';
            
            LineWidth=3;
            LineSize=3;
            TextSize=14;
            fontstyle='bold';
            fonttype='Times New Roman';
            
            runner = Swing;
            
            load('./SavedGaits/SLIP/SLIP_NoAerial_unmatchedSL.mat','SLIPdata')
            runner.SLIPx0 = [SLIPdata(end,4);SLIPdata(end,5)];
            runner.SLIPxf = [SLIPdata(1,4);SLIPdata(1,5)];
            %add another step to the cycle
%             finaltime = SLIPdata(end,1);
%             finalx = SLIPdata(end,6)-SLIPdata(1,6);
%             steps = size(SLIPdata,1);
%             SLIPdata = [SLIPdata;SLIPdata(2:end,:);SLIPdata(2:end,:)];
%             SLIPdata(steps+1:end,1) = SLIPdata(steps+1:end,1) + finaltime;
%             SLIPdata(steps+1:end,6) = SLIPdata(steps+1:end,6) + finalx;
%             SLIPdata(2*steps:end,1) = SLIPdata(2*steps:end,1) + finaltime;
%             SLIPdata(2*steps:end,6) = SLIPdata(2*steps:end,6) + finalx;
runner.SLIPdata = SLIPdata;

IC = SwingState;
switch test
    
        case 'experiment'
        
            runner.useHSevent = 0;
%             runner.SLIPdata(:,2:3) = 0;
            runner.sephips = 0;
            
        runner.rigidlegimpulse = 0;
        
                    runner.kswing = 1; %0.01
                    runner.khip = 5; %0.01

                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = -1.6;
        
        IC.swingfoot.Angle = runner.SLIPx0(1);
        IC.swingfoot.Length = runner.SLIPx0(2);
        
        IC.swingfoot.AngleDot = 0;
        IC.swingfoot.LengthDot = 0;
        
        x0 = IC.getVector();
    
    case 'leg yank'
        
        runner.rigidlegimpulse = 1;
        runner.kswing = 0.5; %0.01
        runner.khip = 1.2; %0.01
        
        runner.gslope = 0;
        runner.swingl = 1;
        runner.hipl = -1.3;
        
        IC.swingfoot.Angle = runner.SLIPx0(1);
        IC.swingfoot.Length = runner.SLIPx0(2);
        
        IC.swingfoot.AngleDot = 0;
        IC.swingfoot.LengthDot = 0;
        
        x0 = IC.getVector();
        
    case 'noleg'
        
        runner.sephips = 1;
        runner.kswing = 0; %0.01
        runner.khip = 0; %0.01
        
        runner.gslope = 0;
        runner.swingl = 0.95;
        runner.hipl = -.8;
        
        IC.swingfoot.Angle = runner.SLIPx0(1);
        IC.swingfoot.Length = runner.SLIPx0(2);
        
        IC.swingfoot.AngleDot = 0;
        IC.swingfoot.LengthDot = -0.85;
        
        x0 = IC.getVector();
        
    case 'nohip'
        
        
        runner.kswing = 7; %0.01
        runner.khip = 0; %0.01
        
        runner.gslope = 0;
        runner.swingl = 0.95;
        runner.hipl = -.65;
        
        IC.swingfoot.Angle = runner.SLIPx0(1);
        IC.swingfoot.Length = runner.SLIPx0(2);
        
        IC.swingfoot.AngleDot = 1.05;
        IC.swingfoot.LengthDot = -1.5;
        
        x0 = IC.getVector();
        
    case 'step'
                    
                
                    runner.kswing = 35; %0.01
                    runner.khip = 2; %0.01

                    runner.gslope = 0;
                    runner.swingl = 0.95;
                    runner.hipl = -.65;
                    
                    IC.swingfoot.Angle = runner.SLIPx0(1);
                    IC.swingfoot.Length = runner.SLIPx0(2);
                    
                    IC.swingfoot.AngleDot = 0;
                    IC.swingfoot.LengthDot = 0;
                    
                    x0 = IC.getVector();
                    
                case 'nosprings'
                                   runner.kswing = 0;
                    runner.khip = 0;

                    runner.gslope = 0;
                    runner.swingl = 0.95;
                    runner.hipl = -.65;
                    
                    IC.swingfoot.Angle = runner.SLIPx0(1);
                    IC.swingfoot.Length = runner.SLIPx0(2);
                    
                    IC.swingfoot.AngleDot = 1.05;
                    IC.swingfoot.LengthDot = -1;
                    
                    x0 = IC.getVector();
                otherwise
                    error('Undefined Test Case')
            end
            
            [x0,runner] = runner.GoodInitialConditions(x0);

            %Make sure foot starts on ground
            
%             runner.kstance = 0.3;
%             runner.footangle = x0(5) - x0(6)+.3;
            


%% Simulate a single phase
if onephasesim
    phase = 'Stance';
    tspan = [0 1];
    figure
    options = odeset('AbsTol',[],'RelTol',[],'OutputFcn',@odeplot);
    [allt,allx] = ode45(@(t,x) runner.XDoubleDot(t,x,phase),tspan,x0);
    
    
    figure
    runner.anim(allx);
    
    for i = 1:length(allt)
        energies = runner.getEnergies(allx(i,:));
        TOTE(i) = energies.Total;
        KE(i) = energies.KE;
        PE(i) = energies.PE;
        PEgrav(i) = energies.PEgrav;
        PEspring(i) = energies.PEspring;
    end
    
    figure
    plot(allt,TOTE)
    hold on
    plot(allt,KE)
    plot(allt,PE)
    plot(allt,PEgrav)
    plot(allt,PEspring)
    legend('Tot','KE','PE','PEgrav','PEspring')
    keyboard;
end
%% Take a Step
figure
tic
if ~manystep
    [xf,tf,allx,allt,tair,runner,phasevec] = runner.onestep(x0,'interleaveAnimation',1);
    runner.printStepCharacteristics(x0,xf,tf,tair);
else
    [xf,tf,allx,allt,phasevec]= runner.manystep(x0);
    runner.anim(allx);
end
toc
%             runner.printStepCharacteristics(x0,xf,tf,tair);

%             keyboard;
%% Check Energy and constraints

for i = 1:size(allx,1)
    phase = runner.phases{phasevec(i)};
    energies = runner.getEnergies(allt(i),allx(i,:));
    TOTE(i) = energies.Total;
    KE(i) = energies.KE;
    PE(i) = energies.PE;
    PEgrav(i) = energies.PEgrav;
    pts = runner.getPoints(allt(i),allx(i,:));
    stancefootx(i) = pts.stancefoot(1);
    stancefooty(i) = pts.stancefoot(2);
        swingfootx(i) = pts.swingfoot(1);
    swingfooty(i) = pts.swingfoot(2);
    pelvx(i) = pts.pelvis(1);
    pelvy(i) = pts.pelvis(2);
    
    vels = runner.getVels(allt(i),allx(i,:));
    stancefootvelx(i) = vels.stancefoot(1);
    stancefootvely(i) = vels.stancefoot(2);
    GRF(i,:) = runner.getGRF(allt(i),allx(i,:),phase);
end


figure
plot(allt,TOTE)
hold on
plot(allt,KE)
plot(allt,PE)
plot(allt,PEgrav)
legend('Tot','KE','PE','PEgrav')


figure
plot(allt,GRF)
title('GRF')




%% Animation
keyboard;
figure
if saveAnimation
    obj = VideoWriter(aviname);
    open(obj);
end

for i = 1 : 4 : length(allx)
    cla;
    if saveAnimation
        runner.plot(allt(i),allx(i,:),'aviWriter',obj);
    else
        runner.plot(allt(i),allx(i, :));
    end
    pause(0.01);
    
end

if saveAnimation
    close(obj);
end


        end
        
    end
    
    methods
        %% Class Constructor and Properties-related Functions
        function [this] = Swing(input)
            %%
            this = this@Runner();
            
            if (nargin == 1 && ~isempty(input))
                parmnames = fieldnames('this');
                for i = 1:length(parmnames)
                    pstring = sprintf(['this.' parmnames{i} ' = input.%g'],this.(parmnames{i}));
                    eval(pstring);
                end
            end
            
        end
        
        %% Simulation Functions

        function [xf,tf,allx,allt,tair,this,phasevec]= onestep(this, x0,varargin)
            %%
            RelTol = 1e-6; %10; %
            AbsTol = 1e-6; %10; %
            tmax = 2; %2; %6;
            dt = 1e-2;
            interleaveAnimation = 0; %1; %
            interleaveAnimationFrameskip = 2;
            aviname = [];
            keeppelvisstates = 0;
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'interleaveAnimation'
                        interleaveAnimation = value;
                    case 'interleaveAnimationFrameskip'
                        interleaveAnimationFrameskip = value;
                    case 'tmax'
                        tmax = value;
                    case 'aviname'
                        aviname=value;
                    case 'keeppelvisstates'
                        keeppelvisstates = value;
                end
            end
            
            
            %%
            this.getQandUdefs(x0);
            if iscolumn(x0)
                x0 = x0';
            end
            [x0,this] = this.GoodInitialConditions(x0);
            sim = 1;
            phasenum= 1;
            tstart = 0;
            allt = [];
            allx = [];
            phasevec = [];
              phaseevents = {@(t,x) this.ToeEvents(t,x) @(t,x) this.AerialEvents(t,x)};  

            while sim
                %% Phase transition & Integration
                phase =  this.phases{phasenum}; %Get name of phase corresponding to phasenum
                
                %Check to see if foot goes through floor
                
                
                if this.rigidlegimpulse && strcmp('Toe',phase)
                    
%                     vpelvx = this.SLIPdata(1,8);
%                     vpelvy = this.SLIPdata(1,9);
%                     thetadot = this.SLIPdata(1,10);
%                     rdot = this.SLIPdata(1,11);
%                     theta = x0(1);
%                     r = x0(2);
%                     
%                     fv = [vpelvx;vpelvy] + r*thetadot*[-sin(theta);cos(theta)];
%                     x0(3) = (fv(2)*cos(theta)-fv(1)*sin(theta)+sin(theta)*vpelvx-cos(theta)*vpelvy/r);
%                     x0(4) = fv(1)*cos(theta)+fv(2)*sin(theta)-cos(theta)*vpelvx-sin(theta)*vpelvy;
%                     
% %                     x0(3) = (-sin(theta)*vpelvx + cos(theta)*vpelvy)/r/1;
% %                     x0(4) = (cos(theta)*vpelvx + sin(theta)*vpelvy)/1;
%                     
%                     x0([3 4]) = x0([3 4])*2;
%                     
% %                     x0([3 4]) = [-.5 -.5];
%                     vels = this.getVels(tstart,x0);

                      x0 = getYankImpulse(this,x0,tstart);
%                     vels = this.getVels(tstart,x0);
                    
                    phasenum = phasenum+1;
                    phase = this.phases{phasenum};
                elseif strcmp('Toe',phase)
                    
                    phasenum = phasenum+1;
                    phase = this.phases{phasenum};
%                     xdot = this.XDoubleDot(tstart,x0',phase);
%                     xsmallstep = x0' + xdot*1e-6;
%                     pts = this.getPoints(tstart+1e-6,xsmallstep);
%                     [GRF] = this.getGRF(tstart,x0,'Toe');
%                     vels = this.getVels(tstart,x0);
%                     if (GRF<0 && x0(3)==0 && x0(4) == 0) || vels.swingfoot(2)>0
%                         phasenum = phasenum+1;
%                         phase = this.phases{phasenum};
%                     else %if you dont want any sliding
%                         allx=[x0;x0]; allt = [0 0]; phasevec=[2;2];
%                         break;
%                     end

                end
                
                odex0 = x0;
                opts = odeset('Events', phaseevents{phasenum},'RelTol',RelTol','AbsTol',AbsTol); %Set integration options
                [t,x,~,~,ie] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tstart:dt:tstart+tmax,odex0,opts); %Integrate dynamics
                
                if ~isempty(ie)
                    ie = ie(end);
                end
                
                %%  Recording & Concatenating Integration Results
                dexes=1:length(t);

                allt = [allt;t(dexes)]; allx = [allx;x(dexes,:)]; phasevec = [phasevec; phasenum*ones(length(dexes),1)];
                tstart = allt(end);
                x0 = allx(end,:);
                
                %% Decide which Phase to Move To
                [eventpossibilities] = phaseevents{phasenum}(tstart,x0);
                fallevent = length(eventpossibilities); %Event in which model falls for that phase
                
                if isempty(ie)
                    sim = 0;
                    break;
                end
                if ie == fallevent || phasenum == length(this.phases) %Fallen or reached last phase
                    sim = 0;
                elseif ie ==1 %First event is reserved for expected behavior
                    phasenum = phasenum+1; %Move forward one phase
                else
                    sim = 0;
                end
                
            end
            xf = allx(end,:); tf = allt(end); tair=allt(find(phasevec==2,1));
            
            if (interleaveAnimation)
                
                if ~isempty(aviname)
                    obj = VideoWriter(aviname);
                    open(obj);
                end
                
                for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
                    cla;
                    if ~isempty(aviname)
                        this.plot(allt(i),allx(i,:),'aviWriter',obj);
                    else
                        this.plot(allt(i),allx(i, :));
                    end
                    pause(0.01);
                    
                end
                
                if ~isempty(aviname)
                    close(obj);
                end
                
            end
            
            
            
        end
        
        function [xf,tf,allx,allt,phasevec,this]= manystep(this, x0,varargin)
            numsteps = 2;
            onestepargin = [];
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'numsteps'
                        numsteps = value;
                    otherwise
                        onestepargin = varargin{i:end};
                end
            end
            
            allx = []; allt = []; phasevec=[];
            tcount = 0;
            for i = 1:numsteps
                if ~isempty(onestepargin)
                    [xf,tf,x,t,~,this,phses]= onestep(this, x0,...
                                                      onestepargin);
                else
                    [xf,tf,x,t,~,this,phses]= onestep(this, x0);
                end
                allt = [allt;t+tcount];
                allx = [allx;x];
                phasevec = [phasevec; phses];
                x0 = xf;
                tcount = tf;
            end
            
        end
        
        function [value, isTerminal, direction]  = ToeEvents(this,t,state)
            [GRF] = this.getGRF(t,state,'Toe');
            value = GRF(end);
            direction = 0;
            isTerminal = 1;
        end

        function [value, isTerminal, direction]  = AerialEvents(this,t,state)
%             pts = this.getPoints(t,state);
%             value = pts.swingfoot(2);       
%             direction = 0;
%             if t>.03
%             isTerminal = 1;
%             else
%                isTerminal = 0; 
%             end
            
if ~this.useHSevent
    value(1) = this.SLIPdata(end,1) - t;
    direction(1) = 0;
    isTerminal(1) = 1;
else
    pts = this.getPoints(t,state);
    value(1) = pts.swingfoot(2);
    direction(1) = 0;
    if t >= this.SLIPdata(end,1) || state(1)>-pi/2
        isTerminal(1) = 1;
    else
        isTerminal(1) = 0;
    end
end
        end
        
        function [newstate,this] = GoodInitialConditions(this,x0,varargin)
            
            this.getQandUdefs(x0);
            
            newstate = x0;
            
        end
        

        %% Graphical Functions
        function plot(this, time,state, varargin)
            %%
            
            aviWriter = [];
            shouldAntiAlias = 0;
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'aviWriter'
                        aviWriter = value;
                    case 'shouldAntiAlias'
                        shouldAntiAlias = value;
                        
                end
            end
            
            %%
            
            
            points = this.getPoints(time,state);
            
            plotter = RunnerPlotter;
            
            %Draw Ground
            plot([-5 5],[0 0],'k','LineWidth',2)
            
            hold on
            
            
            %Draw Wall Lines
            %       linesStart = floor(xLims(1) * 5) / 5;
            %       linesEnd = floor(xLims(2) * 5) / 5;
            %       linesToDraw = linesStart : 0.2 : linesEnd;
            %
            %       for x = linesToDraw
            %           line([x x], [0 yLims(2)], 'color', ones(3,1) * 0.3)
            %       end
            
            
            stancel = 1;
            %Draw Springs
            numcoils=3;
            springwidth=.07;
            plotter.plotSpring(points.stancefoot(1),points.stancefoot(2),...
                points.pelvis(1),points.pelvis(2),...
                numcoils,1,springwidth) %stance spring
            
            if this.kswing>0
            plotter.plotSpring(points.swingfoot(1),points.swingfoot(2),...
                points.pelvis(1),points.pelvis(2),...
                2,this.swingl,0.04,'Color',[101 156 255]/255) %swing spring
            else
                plotter.plotLine(points.pelvis,points.swingfoot,'LineStyle','--')
            end
            
            if this.khip>0
            stancedir = (points.stancefoot - points.pelvis)/norm(points.stancefoot - points.pelvis);
            swingdir = (points.swingfoot - points.pelvis)/norm(points.swingfoot - points.pelvis);
            stancepoint = points.pelvis + .2 * stancel * stancedir;
            swingpoint = points.pelvis + .2 * stancel * swingdir;
            
            plotter.plotAngSpring(stancepoint,swingpoint,points.pelvis,2,.05,...
                'Color',[232 40 76]/255) %achilles spring
            end
            
            
            %Draw Masses
            plotter.plotMass(points.pelvis);
            plotter.plotMass(points.stancefoot,'scaling',0);
            plotter.plotMass(points.swingfoot,'scaling',0);
            
            axis equal;
            
            %Set Axis Limits
            xLims = [points.pelvis(1)]+ [-1 1];
            
            try xlim(xLims);
            catch e
                e;
            end
            
            yLims = [points.pelvis(2)] + [-1.5 .5];
            ylim(yLims);
            
            
            
            
            set(gcf, 'color', 'w');
            
            axis off;
            box off;
            
            set(gca, 'ActivePositionProperty', 'OuterPosition');
            set(gca, 'Position', [0, 0, 1, 1]);
            
            
            if (~isempty(aviWriter))
                if (shouldAntiAlias)
                    antiAliasedFig = myaa();
                    drawnow;
                    pause(0.01);
                    thisFrame = getframe(antiAliasedFig);
                    close(antiAliasedFig);
                else
                    thisFrame = getframe(gcf);
                end
                writeVideo(aviWriter, thisFrame);
            end
            
            
        end
        
       
        
        function [speed,stepfreq,steplength,airfrac] = anim(this,x0,varargin)
            tmax=5;
            interleaveAnimationFrameskip=2;
            aviname=[];
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'interleaveAnimationFrameskip'
                        interleaveAnimationFrameskip = value;
                    case 'tmax'
                        tmax = value;
                    case 'aviname'
                        aviname = value;
                end
            end
            [h,w]=size(x0);
            if h==1 || w==1
                [xf, tf, allx, allt, tair]= onestep(this, x0,'aviname',aviname,'interleaveAnimation',1,'interLeaveAnimationFrameSkip',interleaveAnimationFrameskip,'tmax',tmax);
                [speed,steplength,stepfreq] = this.getGaitChar(x0,tf,xf,tair);
            else %Already given states
                allx = x0;
                for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
                    cla;
                    this.plot(allt(i),allx(i, :));
                    pause(0.01);
                end
                speed=[]; steplength=[]; stepfreq=[]; airfrac=[];
            end
            
            
        end
        
        
        %% Mathematica Output
        
        
        function [] = getQandUdefs(this,x)
            %%
            % Add the values for x1,x2,...,xN,u1,u2,...,uN to the work space of the
            % function that calls this function.
            ws = 'caller';
            assignin(ws,'q3',x(1));
            assignin(ws,'q4',x(2));
            assignin(ws,'u3',x(3));
            assignin(ws,'u4',x(4));
            
            
        end
        
        function [xpacc,ypacc,stanceangle,stancelength,xp,yp,xvel,yvel,angvel,lengthvel] = getSLIPstates(this,targettimes)
            SLIPdata = this.SLIPdata;
            SLIPtf = SLIPdata(end,1);
            if max(targettimes)>SLIPtf 
                % If useHSevent = 1, and the foot has not hit the ground by
                % the time the SLIP model simulation has ended, simulate
                % the free fall of the SLIP model at those future times.
                
                if isrow(targettimes)
                    targettimes = targettimes';
                end
                
                x0 = SLIPdata(end,6); y0 = SLIPdata(end,7);
                vx0 = SLIPdata(end,8); vy0 = SLIPdata(end,9);
                times = targettimes(targettimes>SLIPtf)-SLIPtf;
                xs = x0 + vx0*times; ys = y0 + vy0*times - 1/2*this.g*times.^2;
                vxs = vx0*ones(size(times)); vys = vy0*ones(size(times)) - this.g*times;
                xaccs = zeros(size(times)); yaccs = -this.g*ones(size(times));
                angs = SLIPdata(end,4)*ones(size(times)); lengths = SLIPdata(end,5)*ones(size(times));
                angvels = zeros(size(times)); lengthvels = zeros(size(times));
                
                SLIPdata = [SLIPdata; times+SLIPtf xaccs yaccs angs lengths xs ys vxs vys angvels lengthvels];
            end
            
            SLIPtimes = SLIPdata(:,1);
            SLIPstates = SLIPdata(:,2:end);
            interpstates = interp1(SLIPtimes,SLIPstates,targettimes,'spline');
            
            xpacc = interpstates(:,1);
            ypacc = interpstates(:,2);
            stanceangle = interpstates(:,3);
            stancelength = interpstates(:,4);
            xp = interpstates(:,5);
            yp = interpstates(:,6);
            xvel = interpstates(:,7);
            yvel = interpstates(:,8);
            angvel = interpstates(:,9);
            lengthvel = interpstates(:,10);
        end
        
        function [xddot, constraintForces] = XDoubleDot(this,time,x,phase)
            %%
            %Phase specifies what equations to use, EG 'aerial' or 'stance'
            this.getParams();
            this.getQandUdefs(x);
            c3 = cos(q3); s3 = sin(q3);
            u = x(this.N/2+1:end); %velocity states
            
            [xpacc,ypacc,stanceangle] = this.getSLIPstates(time);
            
            if strcmp(phase,'Aerial')
                
                if ~this.sephips
                accs(1,1) = (-(q3*khip) + khip*(-hipl + stanceangle) - q4*(2*u3*u4 - s3*xpacc ...
                    + c3*(g + ypacc)))*power(q4,-2); accs(1,2) = -(s3*g) + kswing*swingl - ...
                    c3*xpacc - s3*ypacc + q4*(-kswing + u3*u3);
                
                else
accs(1,1) = (-(q3*khip) + hipl*khip - q4*(2*u3*u4 - s3*xpacc + c3*(g + ...
ypacc)))*power(q4,-2); accs(1,2) = -(s3*g) + kswing*swingl - c3*xpacc - ...
s3*ypacc + q4*(-kswing + u3*u3); 

                end
                
                xddot = [u;accs'];
                constraintForces = [];
            elseif strcmp(phase,'Toe')
                
                if ~this.sephips
accsandconstraints(1,1) = (power(q4,-2)*(-2*q3*khip*(s3*s3) + 2*khip*(-hipl + ...
stanceangle)*(s3*s3) + kswing*(q4*q4)*sin(2*q3) - q4*(4*u3*u4 - 2*s3*xpacc + ...
kswing*swingl*sin(2*q3))))/2.; accsandconstraints(1,2) = ...
(power(q4,-1)*(-2*c3*q4*(-(c3*kswing*swingl) + xpacc) + ...
q4*q4*(-2*kswing*(c3*c3) + 2*(u3*u3)) + q3*khip*sin(2*q3) + khip*(hipl - ...
stanceangle)*sin(2*q3)))/2.; accsandconstraints(1,3) = (c3*q3*khip + ...
c3*khip*(hipl - stanceangle) + q4*(g - s3*kswing*swingl + ypacc) + ...
s3*kswing*(q4*q4))*power(q4,-1); 
                else
accsandconstraints(1,1) = (power(q4,-2)*(-2*q3*khip*(s3*s3) + ...
2*hipl*khip*(s3*s3) + kswing*(q4*q4)*sin(2*q3) - q4*(4*u3*u4 - 2*s3*xpacc + ...
kswing*swingl*sin(2*q3))))/2.; accsandconstraints(1,2) = ...
(power(q4,-1)*(-2*c3*q4*(-(c3*kswing*swingl) + xpacc) + ...
q4*q4*(-2*kswing*(c3*c3) + 2*(u3*u3)) + q3*khip*sin(2*q3) - ...
hipl*khip*sin(2*q3)))/2.; accsandconstraints(1,3) = g + q4*s3*kswing - ...
s3*kswing*swingl + ypacc + c3*(q3 - hipl)*khip*power(q4,-1); 
                end

                
         

            xddot = [u;accsandconstraints(1:2)'];
            constraintForces = accsandconstraints(3);
                
            end
            
            
            
        end
        
    
        function [E] = getEnergies(this,time,state)
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); s3 = sin(q3);
            
            [~,~,stanceangle,stancelength,q1,q2,u1,u2,angvel,lengthvel] = this.getSLIPstates(time);

            
            kineticEnergy = (u1*(-2*q4*s3*u3 + 2*c3*u4) + 2*u2*(c3*q4*u3 + s3*u4) + u1*u1 ...
                + u2*u2 + q4*q4*(u3*u3) + u4*u4)/2.;
            
            potentialEnergy = (2*q2*g*cos(gslope) + khip*((q3 + hipl - stanceangle)*(q3 + ...
                hipl - stanceangle)) + kswing*((-q4 + swingl)*(-q4 + swingl)) + 2*q4*g*sin(q3 ...
                - gslope) - 2*q1*g*sin(gslope))/2.;
            
            PEgrav = q2*g*cos(gslope) + q4*g*sin(q3 - gslope) - q1*g*sin(gslope);
            
            PEspring = (khip*((-q3 - hipl + stanceangle)*(-q3 - hipl + stanceangle)))/2. ...
                + (kswing*((q4 - swingl)*(q4 - swingl)))/2.;
            
            
            E.KE = kineticEnergy;
            E.PE = PEgrav + PEspring;
            E.PEgrav = PEgrav;
            E.PEspring = PEspring;
            E.Total = E.KE + E.PE;
            
        end
        
        function [points] = getPoints(this, time, state)
            
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); s3 = sin(q3);
            
            [~,~,stanceangle,stancelength,xp,yp] = this.getSLIPstates(time);
            
            points.swingfoot(1) = xp + c3*q4;
            points.swingfoot(2) = yp + q4*s3;
            
            points.stancefoot(1) = xp + cos(stanceangle)*stancelength;
            points.stancefoot(2) = yp + sin(stanceangle)*stancelength;
            
            points.pelvis(1) = xp;
            points.pelvis(2) = yp;
            
            
            points.COM(1) = xp;
            points.COM(2) = yp;
            
        end
        
        function [vels] = getVels(this,time,state)
            
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); s3 = sin(q3);
            
            [~,~,stanceangle,stancelength,~,~,xvel,yvel,angvel,lengthvel] = this.getSLIPstates(time);
            
            
vels.stancefoot(1) = xvel - stancelength*sin(stanceangle)*angvel + cos(stanceangle)*lengthvel; 
vels.stancefoot(2) = yvel + cos(stanceangle)*stancelength*angvel + sin(stanceangle)*lengthvel; 


% vels.swingfoot(1) = xvel - q4*s3*u3 + c3*u4; 
% vels.swingfoot(2) = yvel + c3*q4*u3 + s3*u4; 
 vels.swingfoot(1) = - q4*s3*u3 + c3*u4; 
vels.swingfoot(2) = c3*q4*u3 + s3*u4; 


vels.pelvis(1) = xvel; 
vels.pelvis(2) = yvel; 


vels.COM(1) = xvel; 
vels.COM(2) = yvel; 

        end
        
        
        function [comWR] = getcomWR(this,state,phase)
            comWR = 0;
        end
        
                function [C,Ceq] = MassesAddToOne(this,varargin)
            r = this;
            C=[];
            Ceq = 1 - r.mpelvis - 2*r.mfoot;
        end
        
        %% Additional Dynamics Calculations
        
        function [GRF] = getGRF(this,t,x,phase)
            if size(x,1)==1
                x=x';
            end
            
            if ~strcmp('Aerial',phase)
                [~,GRF] = this.XDoubleDot(t,x,phase);
            else
                GRF=[0;0];
            end
        end
        
  
        function swingPower = getSwingPower(this,x)
            force = this.getSwingForce(x);
            velocity = x(4);
            swingPower = force.*velocity;
        end
        
        function swingforce = getSwingForce(this,x)
            swingforce = -this.kswing*(x(2)-this.swingl);
        end
        
        function hippower= getHipPower(this,time,x)
            sz = SwingState(x);
[~,~,stanceangle,~,~,~,~,~,angvel,~] = this.getSLIPstates(time);
            force = this.getHipForce(x,stanceangle);
            velocity = angvel - sz.swingfoot.AngleDot;
            hippower = force.*velocity;
        end
        
        function hipforce = getHipForce(this,x,stanceangle)
            sz = SwingState(x);
            hipforce = -this.khip*(stanceangle - sz.swingfoot.Angle - this.hipl);
        end
        
        %% Other Gait Information
        
        function x0 = getYankImpulse(this,x0,tstart)
            [xpacc,ypacc,stanceangle,stancelength,xp,yp,xvel,yvel,angvel,lengthvel] = this.getSLIPstates(tstart);
            x0(4) = this.impulsecoeff*lengthvel;
        end
        
        function [c,ceq] = floorconstraint(this,~,~,~,allx,allt,varargin)
            
            fty = zeros(size(allx,1),1);
            for i = 1:size(allx,1)
               pts = this.getPoints(allt(i),allx(i,:));
               fty(i) = pts.swingfoot(2);
            end
            c= [];
            ceq = sum(fty(fty<0));
            if isempty(ceq)
                ceq = 0;
            end
        end
        
        function [c, ceq, limitCycleError, cExtra, ceqExtra, cost] = fixedPointConstraint(this,x,runcharic,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost)
        %%
        % x contains the model states available for the optimizer to change
        % as well as parameter values for the parameters specified in
        % parametersToAlter. runcharic is a structure with fields speed,
        % steplength, and airfrac, you can set these fields to empty if you
        % don't care about them.  
        
        %Call the initial guess for the fixed point
        x0 = initialConditionGuess;
        %Change the states available to the optimizer (this.statestovary) to what they
        %are during the current iteration for the simulation
        x0(this.statestovary) = x(1:length(this.statestovary));
        

        
        if ~isempty(parametersToAlter)
            parameterValues = x(length(this.statestovary)+1:end);
            this = this.setParametersFromList(parametersToAlter, parameterValues);
        end
        
        if isempty(additionalCost)
            [xf, tf, allx, allt, tair,this] = this.onestep(x0);
        elseif nargin(additionalCost)==7
            [xf, tf, allx, allt, tair,this] = this.onestep(x0);
        else
            [xf, tf, allx, allt, tair,this,phasevec] = this.onestep(x0);
        end
        x0 = allx(1,:);
        if isrow(xf)
            xf = xf';
        end
        
        limitCycleError = xf(1:2) - this.SLIPxf;
        
        c=[];
        ceq=limitCycleError;

        if ~isempty(runcharic.speed)
            [speed] = getSpeed(this, x0, xf, tf);
            ceq = [ceq; runcharic.speed - speed];
        end
        if ~isempty(runcharic.steplength)
            [steplength] = getStepLength(this, x0, xf,tf);
            ceq = [ceq; runcharic.steplength - steplength];
        end
%         if ~isempty(runcharic.airfrac)
%             [airfrac] = getAerialFraction(this, x0, tf, tair);
%             ceq = [ceq; runcharic.airfrac - airfrac];
%         end
        if this.usefloorconstraint
        [~,flooreq] = this.floorconstraint(x0,xf,tf,allx,allt);
        ceq = [ceq;flooreq];
        end
        
        if isempty(additionalConstraintFunction)
            cExtra = [];
            ceqExtra = [];
        else
            if ~isa(additionalConstraintFunction,'cell')
            [cExtra, ceqExtra] = additionalConstraintFunction(this,x0,xf,tf,allx,allt,tair);
            else
                cExtra=[];ceqExtra=[];
                for i = 1:length(additionalConstraintFunction)
                   [ctemp, ceqtemp] = additionalConstraintFunction{i}(this,x0,xf,tf,allx,allt,tair);
                   cExtra=[cExtra;ctemp];ceqExtra=[ceqExtra;ceqtemp];
                end
            end
            c = [c; cExtra];
            ceq = [ceq; ceqExtra];
        end
        
        if isempty(additionalCost)
            cost = 0;
        else
            if nargin(additionalCost)==6
                cost = additionalCost(this,x0,xf,tf,allx,allt,tair);
            else
                cost = additionalCost(this,x0,xf,tf,allx,allt,tair,phasevec);
            end
            
        end
        
        
        
        end
    
            function [] = printStepCharacteristics(this, x0, varargin)
        %%
        if nargin>2
           xf=varargin{1};
           tf=varargin{2};
           tair=varargin{3};
        else
            [xf, tf, allx, ~, tair] = this.onestep(x0);
            x0 = allx(1,:);
        end
        
        limitCycleError = xf(1:2) - this.SLIPxf';
        
            [speed] = getSpeed(this, x0, xf, tf);
            [steplength] = getStepLength(this, x0, xf, tf);
        
        fprintf('step parameters: speed: %g, step length = %g, \n dblSprtTime = %g, limitCycleError = %g\n', speed, steplength, tair/tf, norm(limitCycleError));
            end
    
            function [] = print(this,x0,varargin)
               this.printStepCharacteristics(x0,varargin{:}); 
            end
        
        function [speed] = getSpeed(this, x0, xf, tf)
            if isempty(xf)
                [~,tf] = this.onestep(x0);
            end
            
            [~,~,~,~,xp0,~] = this.getSLIPstates(0);
            [~,~,~,~,xpf,~] = this.getSLIPstates(tf);
            
            speed = (xpf-xp0) / tf;
            if isnan(speed)
                speed = 0;
            end
        end
        
        function [steplength] = getStepLength(this, x0, xf, tf)
            if isempty(xf)
                [~,tf] = this.onestep(x0);
            end
            
            [~,~,~,~,xp0,~] = this.getSLIPstates(0);
            [~,~,~,~,xpf,~] = this.getSLIPstates(tf);
            
            steplength = (xpf-xp0);
            if isnan(steplength)
                steplength = 0;
            end
        end
        
        
%         function [airfrac] = getAerialFraction(this, x0, tf, tair)
%             if isempty(tair) || isempty(tair)
%                 [xf,tf,allx,allt,tair] = this.onestep(x0);
%             end
%             airfrac = (tf-tair)/tf;
%         end
%         
        function [speed,steplength,stepfreq] = getGaitChar(this,x0,tf,xf,tair)
            [speed] = this.getSpeed(x0, xf, tf);
            [steplength] = this.getStepLength(x0, xf, tf);
            stepfreq = speed/steplength;
%             [airfrac] = this.getAerialFraction(x0, tf, tair);
        end
       
        
        function [A, B] = linearizeOneStepReturnMap(this, x0, parameters, varargin)
            %%
            perturbationAmount = 1e-4;
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'perturbationAmount'
                        perturbationAmount = value;
                end
            end
            
            if (isempty(varargin))
                varargin = {'', {}};
            end
            
            %%
            for i = 1 : length(parameters)
                B(i, :) = this.getSensitivityToParameters(x0, parameters{i}, ...
                    'perturbationAmount', perturbationAmount, varargin{:})';
            end
            
            A = this.getSensitivityToStates(x0, 'perturbationAmount', perturbationAmount, varargin{:});
        end
        
        function [AMatrix] = getSensitivityToStates(this, x0, varargin)
            %%
            perturbationAmount = 1e-4;
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'perturbationAmount'
                        perturbationAmount = value;
                end
            end
            
            %%
            if (isempty(varargin))
                varargin = {'', {}};
            end
            
            AMatrix = ones(length(x0)) * NaN;
            xNextUnperturbed = this.onestep(x0,varargin{:});
%             xNextUnperturbed(1:2) = this.SLIPxf';
            for i = 1 : length(x0)
                x0Perturbed = x0;
                x0Perturbed(i) = x0Perturbed(i) + perturbationAmount;
                [xNextPerturbed,tf] = this.onestep(x0Perturbed, varargin{:});
                vels = this.getVels(tf,xNextPerturbed);
                SLIPpert = [xNextPerturbed(1:2) vels.pelvis];
                SLIPunpert = [xNextUnperturbed(1:2) this.SLIPdata(1,[8 9])];
                AMatrix(:, i) = (SLIPpert - SLIPunpert) / perturbationAmount;
            end
        end
        
        function [BMatrix] = getSensitivityToParameters(this, x0, parameterName, varargin)
            %%
            perturbationAmount = 1e-4;
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'perturbationAmount'
                        perturbationAmount = value;
                end
            end
            
            if (isempty(varargin))
                varargin = {'', {}};
            end
            
            xNextUnperturbed = this.onestep(x0,varargin{:});
%             xNextUnperturbed(1:2) = this.SLIPxf';
            this.(parameterName) = this.(parameterName) + perturbationAmount;
            [xNextPerturbed,tf] = this.onestep(x0, varargin{:});
            
            vels = this.getVels(tf,xNextPerturbed);
            SLIPpert = [xNextPerturbed(1:2) vels.pelvis];
            SLIPunpert = [xNextUnperturbed(1:2) this.SLIPdata(1,[8 9])];
            
            BMatrix = (SLIPpert - SLIPunpert) / perturbationAmount;
        end
        
       
        
        
    end
    
    
end

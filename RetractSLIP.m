classdef RetractSLIP < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
    
    properties
        mpelvis = 1;
        gslope = 0;
        g = 1;
        N = 12;
        statestovary = [3 5 6 7:8 11:12];
        statestomeasure = [3 4 5 6 7:8 11:12];
        %rest lengths
        stancel=1; swingl = .8;
        %rest angles
        hipl = 0;
        impulsecoeff = 2;
        rigidlegimpulse = 0;
        tanimpulsecoeff = 0;
        lockable = 0;
        lockstate = 0;
        
        %Springs
        kstance = 12; kswing = 0.01; khip = 0.01;
        %Dampers
        cstance = 0; cswing = 0; chip = 0;
        
        runcharic = struct('speed',0.9745,'steplength',1.1905,'airfrac',0.2703);
        
        useHSevent = 0;
        sephips = 0;
        
        phases = {'Stance' 'Aerial'};
    end
    
    
    methods (Static)
        
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            savepath = [dir '\Animations\'];
            aviname = [savepath 'RetractSLIP2.avi'];
            onephasesim = 0;
            manystep = 0;
            test = 'locksephips';
            
            LineWidth=3;
            LineSize=3;
            TextSize=14;
            fontstyle='bold';
            fonttype='Times New Roman';
            
            runner = RetractSLIP;
            

            
            IC = RetractSLIPState;
            switch test
                                                case 'locksephips'
                    runner.lockable = 1;
                    runner.tanimpulsecoeff = 0;
                    runner.sephips=1;
                    runner.rigidlegimpulse = 1;
                    runner.impulsecoeff = 3;
                    runner.useHSevent = 1;
                    runner.kstance = 12.8734; %12
                    runner.kswing = 0; %0.01
                    runner.khip = 5; %0.01
                    runner.cstance = 0;
                    runner.cswing = 0;
                    runner.chip = 0;
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = 1.3;
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    runner.statestomeasure = [3 4 5 6 7:8 11:12];
                    
                    
                    %
                    IC.pelvis.x = -0.4278;
                    IC.pelvis.y = 0.9039;
                    IC.pelvis.xDot = 1.0138;
                    IC.pelvis.yDot = -0.1651;
                   
                    IC.stancefoot.AngleDot = -0.8457;
                    IC.stancefoot.LengthDot = -0.5830;
                    
                    IC.swingfoot.Angle = -2.0;
                    IC.swingfoot.Length = 0.75;
                    IC.swingfoot.AngleDot = 0.35;
                    IC.swingfoot.LengthDot = -1.2;
                    x0 = IC.getVector();
                    
                                case 'tanimpulse_hipstogether'
                    runner.tanimpulsecoeff = 1;
                    runner.sephips=0;
                    runner.rigidlegimpulse = 1;
                    runner.impulsecoeff = 3;
                    runner.useHSevent = 1;
                    runner.kstance = 12.8734; %12
                    runner.kswing = 0; %0.01
                    runner.khip = 2.5; %0.01
                    runner.cstance = 0;
                    runner.cswing = 0;
                    runner.chip = 0;
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = 0.1;
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    runner.statestomeasure = [3 4 5 6 7:8 11:12];
                    
                    
                    %
                    IC.pelvis.x = -0.4279;
                    IC.pelvis.y = 0.9038;
                    IC.pelvis.xDot = 1;
                    IC.pelvis.yDot = -0.3;
                   
                    IC.stancefoot.AngleDot = -0.7;
                    IC.stancefoot.LengthDot = -0.5830;
                    
                    IC.swingfoot.Angle = -2.0;
                    IC.swingfoot.Length = 0.93;
                    IC.swingfoot.AngleDot = 0.2;
                    IC.swingfoot.LengthDot = -1.34;
                    x0 = IC.getVector();
                    
                case 'tanimpulse'
                    runner.tanimpulsecoeff = 1;
                                        runner.sephips=1;
                    runner.rigidlegimpulse = 1;
                    runner.impulsecoeff = 3;
                    runner.useHSevent = 1;
                    runner.kstance = 15; %12
                    runner.kswing = 0; %0.01
                    runner.khip = 3; %0.01
                    runner.cstance = 0;
                    runner.cswing = 0;
                    runner.chip = 0;
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = pi/2;
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    runner.statestomeasure = [3 4 5 6 7:8 11:12];
                    
                    
                    %
                    IC.pelvis.xDot = 1;
                    IC.pelvis.yDot = -0.3;
                    
                    IC.stancefoot.AngleDot = -0.7;
                    IC.stancefoot.LengthDot = -0.5830;
                    
                    IC.swingfoot.Angle = -2.15;
                    IC.swingfoot.Length = 0.88;
                    IC.swingfoot.AngleDot = -0.85;
                    IC.swingfoot.LengthDot = -1.2;
                    x0 = IC.getVector();
                    
                case 'sephipsimpulse'
                    runner.sephips=1;
                    runner.rigidlegimpulse = 1;
                    runner.impulsecoeff = 3;
                    runner.useHSevent = 1;
                    runner.kstance = 15; %12
                    runner.kswing = 0; %0.01
                    runner.khip = 3; %0.01
                    runner.cstance = 0;
                    runner.cswing = 0;
                    runner.chip = 0;
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = pi/2;
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    runner.statestomeasure = [3 5 6 7:8 11:12];
                    
                    
                    %
                    IC.pelvis.xDot = 1;
                    IC.pelvis.yDot = -0.3;
                    
                    IC.stancefoot.AngleDot = -0.7;
                    IC.stancefoot.LengthDot = -0.5830;
                    
                    IC.swingfoot.Angle = -2.15;
                    IC.swingfoot.Length = 0.88;
                    IC.swingfoot.AngleDot = -0.85;
                    IC.swingfoot.LengthDot = -1.2;
                    x0 = IC.getVector();
                case 'no impulse'
                    runner.rigidlegimpulse = 0;
                    runner.impulsecoeff = 2.5;
                    runner.useHSevent = 1;
                    runner.kstance = 10; %12
                    runner.kswing = 10; %0.01
                    runner.khip = 2.5; %0.01
                    runner.cstance = 0;
                    runner.cswing = 0;
                    runner.chip = 0;
                    runner.gslope = 0;
                    runner.swingl = 0.65;
                    runner.hipl = -0.6;
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    runner.statestomeasure = [3 5 6 7:8 11:12];
                    
                    
                    %
                    IC.pelvis.xDot = 1.1;
                    IC.pelvis.yDot = -0.2;
                    
                    IC.stancefoot.AngleDot = -0.8457;
                    IC.stancefoot.LengthDot = -0.5830;
                    
                    IC.swingfoot.Angle = -1.8;
                    IC.swingfoot.Length = 0.95;
                    IC.swingfoot.AngleDot = -0.85;
                    IC.swingfoot.LengthDot = -.5;
                    x0 = IC.getVector();
                    
                                    case 'step'
                    runner.rigidlegimpulse = 1;
                    runner.impulsecoeff = 2.5;
                    runner.useHSevent = 1;
                    runner.kstance = 13.5; %12
                    runner.kswing = 0; %0.01
                    runner.khip = 2; %0.01
                    runner.cstance = 0;
                    runner.cswing = 0;
                    runner.chip = 0;
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = -0.4;
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    runner.statestomeasure = [3 5 6 7:8 11:12];
                    
                    
                    %
                    IC.pelvis.xDot = 1.1;
                    IC.pelvis.yDot = -0.11;
                    
                    IC.stancefoot.AngleDot = -0.8457;
                    IC.stancefoot.LengthDot = -0.5830;
                    
                    IC.swingfoot.Angle = -2.15;
                    IC.swingfoot.Length = 0.88;
                    IC.swingfoot.AngleDot = -0.85;
                    IC.swingfoot.LengthDot = -0.85;
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
                    energies = runner.getEnergies(allx(i,:),phase);
                    TOTE(i) = energies.Total;
                    KE(i) = energies.KE;
                    PE(i) = energies.PE;
                    PEgrav(i) = energies.PEgrav;
                    PE(i) = energies.PE;
                    PE2(i) = energies.PE2
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
                energies = runner.getEnergies(allx(i,:),phase);
                TOTE(i) = energies.Total;
                TOTE2(i) = energies.Total2;
                KE(i) = energies.KE;
                PE(i) = energies.PE;
                PEgrav(i) = energies.PEgrav;
                PEspring(i) = energies.PEspring;
                PEspring2(i) = energies.PEspring2;
                PE2(i) = energies.PE2;
                PEgrav2(i) = energies.PEgrav2;
                PE2(i) = energies.PE2;
                KE2(i) = energies.KE2;
                pts = runner.getPoints(allx(i,:));
                stancefootx(i) = pts.stancefoot(1);
                stancefooty(i) = pts.stancefoot(2);
                swingfootx(i) = pts.swingfoot(1);
                swingfooty(i) = pts.swingfoot(2);
                vels = runner.getVels(allx(i,:));
                stancefootvelx(i) = vels.stancefoot(1);
                stancefootvely(i) = vels.stancefoot(2);
                swingfootvelx(i) = vels.swingfoot(1);
                swingfootvely(i) = vels.swingfoot(2);
                GRF(i,:) = runner.getGRF(allt(i),allx(i,:),phase);
            end
            
            figure
            subplot(211)
            plot(allt,TOTE)
            hold on
            plot(allt,KE)
            plot(allt,PE)
            plot(allt,PEgrav)
            plot(allt,PEspring)
            title('Inertial World')
            legend('Tot','KE','PE','PEgrav','PEspring')
            
            subplot(212)
            plot(allt,PEspring2)
            hold on
            plot(allt,PEgrav2)
            plot(allt,KE2)
            plot(allt,PE2)
            plot(allt,TOTE2)
            title('Massless World')
            legend('springs','grav','KE','PE','Tot')
            
           figure
           subplot(211)
           plot(allt,GRF)
           title('GRF')
           subplot(212)
           plot(allt,allx(:,2))

      
                
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
                    runner.plot(allx(i,:),'aviWriter',obj);
                else
                    runner.plot(allx(i, :));
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
        function [this] = RetractSLIP(input)
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
            tmax = 1.5; %2; %6;
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
            [x0,this] = this.GoodInitialConditions(x0,'keeppelvisstates',keeppelvisstates);
            sim = 1;
            phasenum= 1;
            tstart = 0;
            allt = [];
            allx = [];
            phasevec = [];
            phaseevents = { @(t,x) this.StanceEvents(t,x), @(t,x) this.AerialEvents(t,x)};
            %
            
            while sim
                %% Phase transition & Integration
                phase =  this.phases{phasenum}; %Get name of phase corresponding to phasenum
                if strcmp(phase,'Aerial') && this.rigidlegimpulse && phasevec(end) == 1
                    x0 = getYankImpulse(this,x0,tstart);
                    x0 = getTanImpulse(this,x0,tstart);
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
                if isempty(ie)
                    sim = 0;
                    break;
                end
                if ie == 1 && phasenum == 1
                   phasenum = phasenum+1; 
                elseif ie == 1 && phasenum ==2
                    sim = 0;
                elseif ie == 2 %Lock the leg
                    phasenum = phasenum;
                    this.lockstate = 1;
                    phaseevents = { @(t,x) this.StanceEvents(t,x), @(t,x) this.AerialEvents(t,x)};
                    x0 = this.phaseTransition(allt(end),allx(end,:),phase);
                end
                
            end
            this.lockstate = 0;
            xf = allx(end,:); tf = allt(end);
            tair = allt(find(phasevec==2,1));
            if isempty(tair)
                tair = tf;
            end
            
            %Switch legs
            xf([3:6 9:12]) = xf([5 6 3 4 11 12 9 10]); 
           [xf,~] = this.phaseTransition(tf,xf,'Stance');
            
            if (interleaveAnimation)
                
                if ~isempty(aviname)
                    obj = VideoWriter(aviname);
                    open(obj);
                end
                
                for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
                    cla;
                    if ~isempty(aviname)
                        this.plot(allx(i,:),'aviWriter',obj);
                    else
                        this.plot(allx(i, :));
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
                if i == 1
                   keeppelvisstates = 0; 
                elseif i==2
                    keeppelvisstates = 1;
                end
                if ~isempty(onestepargin)
                    [xf,tf,x,t,~,this,phses]= onestep(this, x0,'keeppelvisstates',keeppelvisstates,...
                                                      onestepargin);
                else
                    [xf,tf,x,t,~,this,phses]= onestep(this, x0,'keeppelvisstates',keeppelvisstates);
                end
                allt = [allt;t+tcount];
                allx = [allx;x];
                phasevec = [phasevec; phses];
                x0 = xf;
                tcount = tf;
            end
            
            
            
        end
        
        
        function [value, isTerminal, direction]  = StanceEvents(this,t,state)
            %Leg reaches full extention
            ss = RetractSLIPState(state);
            value = ss.stancefoot.Length - this.stancel;
            direction = 1;
            if t>0.05
                isTerminal = 1;
            else
                isTerminal = 0;
            end
            
            %Event at full extension if not already locked, and locking is enabled
            if this.lockable && ~this.lockstate
                value(2) = ss.swingfoot.Length - this.stancel;
                isTerminal(2) = 1;
                direction(2) = 0;
            end
        end

        function [value, isTerminal, direction]  = AerialEvents(this,t,state)
            if ~this.useHSevent
                value(1) = this.runcharic.steplength/this.runcharic.speed - t;
                direction(1) = 0;
                isTerminal(1) = 1;
            else
                pts = this.getPoints(state);
                value(1) = pts.swingfoot(2);
                direction(1) = 0;
                    isTerminal(1) = 1;
            end
            
            %Event at full extension if not already locked, and locking is enabled
            if this.lockable && ~this.lockstate
                ss = RetractSLIPState(state);
                value(2) = ss.swingfoot.Length - this.stancel;
                isTerminal(2) = 1;
                direction(2) = 0;
            end
        end
        
        function [newstate,this] = GoodInitialConditions(this,x0,varargin)
            keeppelvisstates = 0; %Set to 1 to solve consistent IC w/o changing pelvis states
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'keeppelvisstates'
                        keeppelvisstates = value;
                end
            end
            
            this.getQandUdefs(x0);
            
            newstate = x0;
            
            
            if ~keeppelvisstates
                %Shift body (without changing configuration) so that heel is on ground
                points = this.getPoints(newstate);
                pelvx = points.pelvis(1) - points.stancefoot(1);
                pelvy = points.pelvis(2) - points.stancefoot(2);
                newstate([1 2])=[pelvx pelvy];
                
                %Get consistent velocities w/ constraints
%                 Jc = this.getConstraints(newstate,'Stance');
%                 nullJc = null(Jc);
%                 unew = nullJc * (nullJc \ newstate(7:12)');
                unew = x0(7:12);
                unew(3) = (sin(q3)*u1-cos(q3)*u2)/q4;
                unew(4) = -cos(q3)*(u1+tan(q3)*u2);
                newstate = [newstate(1:6) unew];
            else
                
                unew = x0(7:12);
                unew(3) = (sin(q3)*u1-cos(q3)*u2)/q4;
                unew(4) = -cos(q3)*(u1+tan(q3)*u2);
                newstate = [newstate(1:6) unew];
            end
        end
        

        %% Graphical Functions
        function plot(this, state, varargin)
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
            
            
            points = this.getPoints(state);
            
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
            
            
            
            %Draw Springs
            numcoils=3;
            springwidth=.07;
            plotter.plotSpring(points.stancefoot(1),points.stancefoot(2),...
                points.pelvis(1),points.pelvis(2),...
                numcoils,this.stancel,springwidth) %stance spring
            
            if this.kswing>0
            plotter.plotSpring(points.swingfoot(1),points.swingfoot(2),...
                points.pelvis(1),points.pelvis(2),...
                2,this.swingl,0.04,'Color',[101 156 255]/255) %swing spring
            else
                plotter.plotLine(points.pelvis,points.swingfoot,'LineStyle','--');
            end
            
            if this.sephips
                lpelvis = 0.3;
                torso = points.pelvis + [0 lpelvis];
                plotter.plotLine(points.pelvis,torso)
            end
            
            if this.khip>0
                if this.sephips
                    torsodir = [0 1];
                    torsopoint = points.pelvis + 0.7 * lpelvis * torsodir;
                    stancedir = (points.stancefoot - points.pelvis)/norm(points.stancefoot - points.pelvis);
                    swingdir = (points.swingfoot - points.pelvis)/norm(points.swingfoot - points.pelvis);
                    stancepoint = points.pelvis + .2 * this.stancel * stancedir;
                    swingpoint = points.pelvis + .2 * this.stancel * swingdir;
                    
%                     if points.swingfoot(2)>1e-4
                    plotter.plotCircSpring(torsopoint,swingpoint,.05,1,2,.05,...
                        (state(5)-this.hipl),'Color',[101 156 255]/255)
%                     end
                    
%                     if points.stancefoot(2)>1e-4
                    plotter.plotCircSpring(torsopoint,stancepoint,0.05,1,2,.05,...
                        (state(3)-this.hipl),'Color',[0 0 0]/255)
%                     end
                    
                else
                    stancedir = (points.stancefoot - points.pelvis)/norm(points.stancefoot - points.pelvis);
                    swingdir = (points.swingfoot - points.pelvis)/norm(points.swingfoot - points.pelvis);
                    stancepoint = points.pelvis + .2 * this.stancel * stancedir;
                    swingpoint = points.pelvis + .2 * this.stancel * swingdir;
                    
                    plotter.plotAngSpring(stancepoint,swingpoint,points.pelvis,2,.05,...
                        'Color',[232 40 76]/255) %achilles spring
                end
            end
            
            %Draw Masses
            plotter.plotMass(points.pelvis);
            plotter.plotMass(points.stancefoot,'scaling',0);
            plotter.plotMass(points.swingfoot,'scaling',0);
            
            axis equal;
            
            %Set Axis Limits
            xLims = [points.pelvis(1)]+ [-1 1];
            xlim(xLims);
            
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
                [speed,steplength,stepfreq,airfrac] = this.getGaitChar(x0,tf,xf,tair);
            else %Already given states
                allx = x0;
                for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
                    cla;
                    this.plot(allx(i, :));
                    pause(0.01);
                end
                speed=[]; steplength=[]; stepfreq=[]; airfrac=[];
            end
            
            
        end
        
        
        %% Mathematica Output
        
            function [xNew,Impulse] = phaseTransition(this,time,x,phaseToSwitchTo)
        %%  
        % Transition between two phases.  That is, switch between two sets
        % of equation of motion.  This in general will be associated with
        % instantaneous collisions that cause a net energy loss of the
        % system.  However, if other parameters such as springs are changed
        % between phases, it is also possible to insert energy into the
        % system.
        
        %make x is a column vector
        if size(x,1)==1
            x = x';
        end
        this.getQandUdefs(x);
        
        if strcmp(phaseToSwitchTo,'Stance') && ~this.lockstate
        unew = x(7:12);
        unew(3) = (sin(q3)*u1-cos(q3)*u2)/q4;
        unew(4) = -cos(q3)*(u1+tan(q3)*u2);
        
        xNew = x;
        xNew(7:12) = unew;
        Impulse = [];
        else
            phase = phaseToSwitchTo;
            [MM,rhs] = this.getMMandRHS(time,x,phase);
            [Jc,Jcdot] = this.getConstraints(x,phase);
            
            [d1,d2] = size(Jc);
            u = x(this.N/2+1:end);
            
            
            
            MMbig = [MM Jc'; Jc zeros(d1)];
            RHSbig = [MM*u;zeros(d1,1)];
            
            VelsAndImpulses = MMbig \ RHSbig;
            xNew = [x(1:this.N/2);VelsAndImpulses(1:this.N/2)];
            
            xNew = this.positionSwitches(xNew);
            
            Impulse = -VelsAndImpulses(this.N/2+1:end);
        end
        
        if (sum(isnan(xNew)))
            xNew
            phaseToSwitchTo
            error('xNew cannot have any NaNs!')
        end
        
            end
    
            function [MM,rhs] = getMMandRHS(this,time,x,phase)
                %%
                %Phase specifies what equations to use, EG 'aerial' or 'stance'
                state = x;
                this.getParams();
                this.getQandUdefs(state);
                u = x(this.N/2+1:end); %velocity states
                
                c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
                
                MM = zeros(6,6); rhs = zeros(6,1);
                
                % righthand side terms
                if strcmp(phase,'Aerial')
                    % Mass Matrix
                    MM(1,1) = mpelvis; MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = 0; MM(1,5) = 0; ...
                        MM(1,6) = 0;
                    MM(2,1) = 0; MM(2,2) = mpelvis; MM(2,3) = 0; MM(2,4) = 0; MM(2,5) = 0; ...
                        MM(2,6) = 0;
                    MM(3,1) = -(q4*s3); MM(3,2) = c3*q4; MM(3,3) = q4*q4; MM(3,4) = 0; MM(3,5) = ...
                        0; MM(3,6) = 0;
                    MM(4,1) = c3; MM(4,2) = s3; MM(4,3) = 0; MM(4,4) = 1; MM(4,5) = 0; MM(4,6) = ...
                        0;
                    MM(5,1) = -(q6*s5); MM(5,2) = c5*q6; MM(5,3) = 0; MM(5,4) = 0; MM(5,5) = ...
                        q6*q6; MM(5,6) = 0;
                    MM(6,1) = c5; MM(6,2) = s5; MM(6,3) = 0; MM(6,4) = 0; MM(6,5) = 0; MM(6,6) = ...
                        1;
                    
                    % righthand side terms
                    if ~this.sephips
                    rhs(1) = g*mpelvis*sin(gslope);
                    rhs(2) = -(g*mpelvis*cos(gslope));
                    rhs(3) = -(u3*chip) + u5*chip - q3*khip + q5*khip + hipl*khip - q4*(2*u3*u4 + ...
                        g*cos(q3 - gslope));
                    rhs(4) = -(u4*cswing) + kswing*swingl + q4*(-kswing + u3*u3) - g*sin(q3 - ...
                        gslope);
                    rhs(5) = -2*q6*u5*u6 + (u3 - u5)*chip - (-q3 + q5 + hipl)*khip - q6*g*cos(q5 ...
                        - gslope);
                    rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + u5*u5) - g*sin(q5 - ...
                        gslope);
                    else
                        rhs(1) = g*mpelvis*sin(gslope); 
rhs(2) = -(g*mpelvis*cos(gslope)); 
rhs(3) = -(u3*chip) - q3*khip - hipl*khip - q4*(2*u3*u4 + g*cos(q3 - ...
gslope)); 
rhs(4) = -(u4*cswing) + kswing*swingl + q4*(-kswing + u3*u3) - g*sin(q3 - ...
gslope); 
rhs(5) = -(u5*chip) - q5*khip - hipl*khip - q6*(2*u5*u6 + g*cos(q5 - ...
gslope)); 
rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + u5*u5) - g*sin(q5 - ...
gslope); 
                    end
                else
                    % Mass Matrix
                    MM(1,1) = mpelvis; MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = 0; MM(1,5) = 0; ...
                        MM(1,6) = 0;
                    MM(2,1) = 0; MM(2,2) = mpelvis; MM(2,3) = 0; MM(2,4) = 0; MM(2,5) = 0; ...
                        MM(2,6) = 0;
                    MM(3,1) = 0; MM(3,2) = 0; MM(3,3) = 0; MM(3,4) = 0; MM(3,5) = 0; MM(3,6) = 0;
                    MM(4,1) = 0; MM(4,2) = 0; MM(4,3) = 0; MM(4,4) = 0; MM(4,5) = 0; MM(4,6) = 0;
                    MM(5,1) = -(q6*s5); MM(5,2) = c5*q6; MM(5,3) = 0; MM(5,4) = 0; MM(5,5) = ...
                        q6*q6; MM(5,6) = 0;
                    MM(6,1) = c5; MM(6,2) = s5; MM(6,3) = 0; MM(6,4) = 0; MM(6,5) = 0; MM(6,6) = ...
                        1;
                    
                    % righthand side terms
                    if ~this.sephips
                    rhs(1) = g*mpelvis*sin(gslope);
                    rhs(2) = -(g*mpelvis*cos(gslope));
                    rhs(3) = 0;
                    rhs(4) = -(u4*cstance) - q4*kstance + kstance*stancel;
                    rhs(5) = -2*q6*u5*u6 + (u3 - u5)*chip - (-q3 + q5 + hipl)*khip - q6*g*cos(q5 ...
                        - gslope);
                    rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + u5*u5) - g*sin(q5 - ...
                        gslope);
                    else
                        rhs(1) = g*mpelvis*sin(gslope); 
rhs(2) = -(g*mpelvis*cos(gslope)); 
rhs(3) = 0; 
rhs(4) = -(u4*cstance) - q4*kstance + kstance*stancel; 
rhs(5) = -(u5*chip) - q5*khip - hipl*khip - q6*(2*u5*u6 + g*cos(q5 - ...
gslope)); 
rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + u5*u5) - g*sin(q5 - ...
gslope); 
                    end
                end
                
                
                
                
            end
            
                function [xddot, constraintForces] = XDoubleDot(this,time,x,phase)
        %% 
        %Phase specifies what equations to use, EG 'aerial' or 'stance'
        
        [MM,rhs] = this.getMMandRHS(time,x,phase);
        [Jc,Jcdot] = this.getConstraints(x,phase);
        
        [d1,~] = size(Jc);
        u = x(this.N/2+1:end); %velocity states
        
        MMbig = [MM Jc'; Jc zeros(d1)];
        
        if numel(Jcdot)>0
            RHSbig = [rhs;-Jcdot*u];
        else
            RHSbig = rhs;
        end
        
%         if cond(MMbig)<1e-10
%            blah = 1; 
%         end

        AccsAndConstraints = MMbig \ RHSbig;
        
        accs = AccsAndConstraints(1:this.N/2);
        
        xddot = [u;accs];
        
       constraintForces = -AccsAndConstraints(this.N/2+1:end);
%        if strcmp(phase,'stance')
%        constraintForces = constraintForces - [0;rhs(7)]; 
%        end
        %Check Constraints, these should be equal to zero if constraints
        %are working.  For debugging.
        
%         constraintacc = Jc*accs +Jcdot*u;
%         constraintvel = Jc*u;
        
    end
    
    
    function [C, CDot] = getConstraints(this,state,phase)
        %%
        this.getQandUdefs(state);
        
        c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
        
        switch phase
            case {'Stance'}
                constraintJacobianStance(1,1) = 1; constraintJacobianStance(1,2) = 0; ...
                    constraintJacobianStance(1,3) = -(q4*s3); constraintJacobianStance(1,4) = c3; ...
                    constraintJacobianStance(1,5) = 0; constraintJacobianStance(1,6) = 0;
                constraintJacobianStance(2,1) = 0; constraintJacobianStance(2,2) = 1; ...
                    constraintJacobianStance(2,3) = c3*q4; constraintJacobianStance(2,4) = s3; ...
                    constraintJacobianStance(2,5) = 0; constraintJacobianStance(2,6) = 0;
                
                
                constraintJacobianStanceDot(1,1) = 0; constraintJacobianStanceDot(1,2) = 0; ...
                    constraintJacobianStanceDot(1,3) = -(c3*q4*u3) - s3*u4; ...
                    constraintJacobianStanceDot(1,4) = -(s3*u3); constraintJacobianStanceDot(1,5) ...
                    = 0; constraintJacobianStanceDot(1,6) = 0;
                constraintJacobianStanceDot(2,1) = 0; constraintJacobianStanceDot(2,2) = 0; ...
                    constraintJacobianStanceDot(2,3) = -(q4*s3*u3) + c3*u4; ...
                    constraintJacobianStanceDot(2,4) = c3*u3; constraintJacobianStanceDot(2,5) = ...
                    0; constraintJacobianStanceDot(2,6) = 0;
                
                
                
                C = constraintJacobianStance;
                CDot = constraintJacobianStanceDot;
                
            case 'Aerial'
                C = [];
                CDot = [];
                
            otherwise
                error('Unknown phase for running model: %s', phase);
                
                
        end
        
        if this.lockstate
           C = [C;[0 0 0 0 0 1]];
           CDot = [CDot;[0 0 0 0 0 0]];
        end
        
    end
    
    
    function [E] = getEnergies(this,state,phase)
        this.getParams();
        this.getQandUdefs(state);
        c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
        mfoot = 1;
        if strcmp(phase,'Stance')
            if ~this.sephips
kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kstance*((-q4 + stancel)*(-q4 + stancel)))/2. + ...
g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

kineticEnergy2 = (mfoot*(-2*q6*s5*u1*u5 + 2*s5*u2*u6 + c5*(2*q6*u2*u5 + ...
2*u1*u6) + u1*u1 + u2*u2 + q6*q6*(u5*u5) + u6*u6) + mfoot*((2*q4*u2*u3 + ...
2*u1*u4)*cos(q3) + u1*u1 + u2*u2 + q4*q4*(u3*u3) + u4*u4 - 2*q4*u1*u3*sin(q3) ...
+ 2*u2*u4*sin(q3)))/2.;

potentialEnergy2 = 2*q2*g*cos(gslope) + (khip*((q3 - q5 - hipl)*(q3 - q5 - ...
hipl)))/2. + (kswing*((q6 - swingl)*(q6 - swingl)))/2. + q4*g*sin(q3 - ...
gslope) + q6*g*sin(q5 - gslope) - 2*q1*g*sin(gslope);

PEgrav2 = 2*q2*g*cos(gslope) + q4*g*sin(q3 - gslope) + q6*g*sin(q5 - gslope) ...
- 2*q1*g*sin(gslope);

PEspring2 = (khip*((q3 - q5 - hipl)*(q3 - q5 - hipl)))/2. + (kswing*((q6 - ...
swingl)*(q6 - swingl)))/2.;

stanceE = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

swingE = (kswing*((q6 - swingl)*(q6 - swingl)))/2.;

hipE = (khip*((q3 - q5 - hipl)*(q3 - q5 - hipl)))/2.;
            else
                kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kstance*((-q4 + stancel)*(-q4 + stancel)))/2. + ...
g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

kineticEnergy2 = (mfoot*(-2*q6*s5*u1*u5 + 2*s5*u2*u6 + c5*(2*q6*u2*u5 + ...
2*u1*u6) + u1*u1 + u2*u2 + q6*q6*(u5*u5) + u6*u6) + mfoot*((2*q4*u2*u3 + ...
2*u1*u4)*cos(q3) + u1*u1 + u2*u2 + q4*q4*(u3*u3) + u4*u4 - 2*q4*u1*u3*sin(q3) ...
+ 2*u2*u4*sin(q3)))/2.;

potentialEnergy2 = 2*q2*g*cos(gslope) + (khip*((-q5 - hipl)*(-q5 - hipl)))/2. ...
+ (kswing*((q6 - swingl)*(q6 - swingl)))/2. + q4*g*sin(q3 - gslope) + ...
q6*g*sin(q5 - gslope) - 2*q1*g*sin(gslope);

PEgrav2 = 2*q2*g*cos(gslope) + q4*g*sin(q3 - gslope) + q6*g*sin(q5 - gslope) ...
- 2*q1*g*sin(gslope);

PEspring2 = (khip*((-q5 - hipl)*(-q5 - hipl)))/2. + (kswing*((q6 - ...
swingl)*(q6 - swingl)))/2.;

stanceE = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

swingE = (kswing*((q6 - swingl)*(q6 - swingl)))/2.;

hipE = (khip*((-q5 - hipl)*(-q5 - hipl)))/2.;
            end
        else

            if ~this.sephips
kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = 0;

kineticEnergy2 = (mfoot*(-2*q4*s3*u1*u3 + 2*s3*u2*u4 + c3*(2*q4*u2*u3 + ...
2*u1*u4) + u1*u1 + u2*u2 + q4*q4*(u3*u3) + u4*u4) + mfoot*(-2*q6*s5*u1*u5 + ...
2*s5*u2*u6 + c5*(2*q6*u2*u5 + 2*u1*u6) + u1*u1 + u2*u2 + q6*q6*(u5*u5) + ...
u6*u6))/2.;

potentialEnergy2 = 2*q2*g*cos(gslope) + (khip*((q3 - q5 - hipl)*(q3 - q5 - ...
hipl)))/2. + (kswing*((q4 - swingl)*(q4 - swingl)))/2. + (kswing*((q6 - ...
swingl)*(q6 - swingl)))/2. + q4*g*sin(q3 - gslope) + q6*g*sin(q5 - gslope) - ...
2*q1*g*sin(gslope);

PEgrav2 = 2*q2*g*cos(gslope) + q4*g*sin(q3 - gslope) + q6*g*sin(q5 - gslope) ...
- 2*q1*g*sin(gslope);

PEspring2 = (khip*((q3 - q5 - hipl)*(q3 - q5 - hipl)))/2. + (kswing*((q4 - ...
swingl)*(q4 - swingl)))/2. + (kswing*((q6 - swingl)*(q6 - swingl)))/2.;

stanceE = (kswing*((q4 - swingl)*(q4 - swingl)))/2.;

swingE = (kswing*((q6 - swingl)*(q6 - swingl)))/2.;

hipE = (khip*((q3 - q5 - hipl)*(q3 - q5 - hipl)))/2.;
            else
               kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = 0;

kineticEnergy2 = (mfoot*(-2*q4*s3*u1*u3 + 2*s3*u2*u4 + c3*(2*q4*u2*u3 + ...
2*u1*u4) + u1*u1 + u2*u2 + q4*q4*(u3*u3) + u4*u4) + mfoot*(-2*q6*s5*u1*u5 + ...
2*s5*u2*u6 + c5*(2*q6*u2*u5 + 2*u1*u6) + u1*u1 + u2*u2 + q6*q6*(u5*u5) + ...
u6*u6))/2.;

potentialEnergy2 = 2*q2*g*cos(gslope) + (khip*((-q3 - hipl)*(-q3 - hipl)))/2. ...
+ (khip*((-q5 - hipl)*(-q5 - hipl)))/2. + (kswing*((q4 - swingl)*(q4 - ...
swingl)))/2. + (kswing*((q6 - swingl)*(q6 - swingl)))/2. + q4*g*sin(q3 - ...
gslope) + q6*g*sin(q5 - gslope) - 2*q1*g*sin(gslope);

PEgrav2 = 2*q2*g*cos(gslope) + q4*g*sin(q3 - gslope) + q6*g*sin(q5 - gslope) ...
- 2*q1*g*sin(gslope);

PEspring2 = (khip*((-q3 - hipl)*(-q3 - hipl)))/2. + (khip*((-q5 - hipl)*(-q5 ...
- hipl)))/2. + (kswing*((q4 - swingl)*(q4 - swingl)))/2. + (kswing*((q6 - ...
swingl)*(q6 - swingl)))/2.;

stanceE = (kswing*((q4 - swingl)*(q4 - swingl)))/2.;

swingE = (kswing*((q6 - swingl)*(q6 - swingl)))/2.;

hipE = (khip*((-q3 - hipl)*(-q3 - hipl)))/2. + (khip*((-q5 - hipl)*(-q5 - ...
hipl)))/2.; 
            end
        end
        
        E.stanceE = stanceE;
        E.swingE = swingE;
        E.hipE = hipE;
        
        E.KE = kineticEnergy;
        E.PE = potentialEnergy;
        E.PEgrav = PEgrav;
        E.PEspring = PEspring;
        E.Total = E.KE + E.PE;
  
        E.KE2 = kineticEnergy2;
        E.PE2 = potentialEnergy2;
        E.PEgrav2 = PEgrav2;
        E.PEspring2 = PEspring2;
        E.Total2 = E.KE2 + E.PE2;      
        
    end
    
        function [points] = getPoints(this, state)
            
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
            
points.stancefoot(1) = q1 + c3*q4; 
points.stancefoot(2) = q2 + q4*s3; 


points.swingfoot(1) = q1 + c5*q6; 
points.swingfoot(2) = q2 + q6*s5; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 
            
        end
        
        function [vels] = getVels(this, state)
            
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
            
            vels.stancefoot(1) = u1 - q4*s3*u3 + c3*u4; 
vels.stancefoot(2) = u2 + c3*q4*u3 + s3*u4; 


vels.swingfoot(1) = u1 - q6*s5*u5 + c5*u6; 
vels.swingfoot(2) = u2 + c5*q6*u5 + s5*u6; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 

        end
        
        
        function [comWR] = getcomWR(this,state,phase)
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
            
            GRF = this.getGRF(0,state,phase);
            vels = this.getVels(state);
            comWR = dot(GRF,vels.COM);
        end
        
                function [C,Ceq] = MassesAddToOne(this,varargin)
            r = this;
            C=[];
            Ceq = 1 - r.mpelvis - 2*r.mfoot;
                end
        
                function [C,Ceq] = SetPointConstraint(this,varargin)
                   C=[];
                   Ceq = 1 - this.swingl;
                end
                
                function [C,Ceq] = PositiveImpulseAndSwingl(this,varargin)
                    C=-this.impulsecoeff;
                    Ceq = 1 - this.swingl;
                end
                
                function [C,Ceq] = VertImpulse(this,x0,xf,tf,allx,allt,tair,phasevec)
                   Ceq = [];
                   TOdex = find(phasevec==3,1); 
                   vels = this.getVels(allx(TOdex,:));
                   C = -vels.stancefoot(2);
                end
                
                function [C,Ceq] = PositiveImpulse(this,varargin)
                    C=-this.impulsecoeff;
                    Ceq = [];
                end
                
                function [C,Ceq] = NoTanImpulse(this,varargin)
                   C = -this.impulsecoeff;
                   Ceq(1,1) = 1 - this.swingl;
                   Ceq(2,1) = this.tanimpulsecoeff;
                end
                
                function [c,ceq] = floorconstraint(this,~,~,~,allx,allt,varargin)
                    
                    fty = zeros(size(allx,1),1);
                    for i = 1:size(allx,1)
                        pts = this.getPoints(allx(i,:));
                        fty(i) = pts.swingfoot(2);
                    end
                    c= [];
                    ceq = sum(fty(fty<0));
                    if isempty(ceq)
                        ceq = 0;
                    end
                end
                
                function cost = impulsecost(this,x0,xf,tf,allx,allt,tair,phasevec)
                   TOdex = find(phasevec==3,1); 
                   vels = this.getVels(allx(TOdex,:));
                   cost = dot(vels.stancefoot,vels.stancefoot);
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
        
        function stancePower = getStancePower(this,x,phase)
            force = this.getStanceForce(x,phase);
            velocity = x(10);
            stancePower = force.*velocity;
        end
        
        function stanceforce = getStanceForce(this,x,phase)
            if strcmp(phase,'Aerial') || strcmp(phase,'Toe')
                kstance = this.kswing;
                cstance = this.cswing;
                stancel = this.swingl;
            else
                kstance = this.kstance;
                cstance = this.cstance;
                stancel = this.stancel;
            end
            stanceforce = -kstance*(x(4)-this.stancel)-cstance*(x(10));
        end
  
        function swingPower = getSwingPower(this,x)
            force = this.getSwingForce(x);
            velocity = x(10);
            swingPower = force.*velocity;
        end
        
        function swingforce = getSwingForce(this,x)
            swingforce = -this.kswing*(x(6)-this.swingl)-this.cswing*(x(12));
        end
        
        function hippower= getHipPower(this,x)
            sz = RetractSLIPState(x);
            force = this.getHipForce(x);
            velocity = sz.stancefoot.AngleDot - sz.swingfoot.AngleDot;
            hippower = force.*velocity;
        end
        
        function hipforce = getHipForce(this,x)
            sz = RetractSLIPState(x);
            hipforce = -this.khip*(sz.stancefoot.Angle - sz.swingfoot.Angle - this.hipl) ...
                       -this.chip*(sz.stancefoot.AngleDot - sz.swingfoot.AngleDot);
        end
        
        %% Other Gait Information
        
                    function [] = print(this,x0,varargin)
               this.printStepCharacteristics(x0,varargin{:}); 
            end
        
        function x0 = getYankImpulse(this,x0,tstart)
            x0(10) = x0(10) - this.impulsecoeff*x0(10);
        end
        
        function x0 = getTanImpulse(this,x0,tstart)
            x0(9) = x0(9) - this.tanimpulsecoeff*x0(9);
        end
        
        function [speed] = getSpeed(this, x0, xf, tf)
            if isempty(xf)
                [xf,tf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = RetractSLIPState(x0);
            xfstruc = RetractSLIPState(xf);
            
            speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
        end
        
        function [steplength] = getStepLength(this, x0, xf)
            if isempty(xf)
                [xf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = RetractSLIPState(x0);
            xfstruc = RetractSLIPState(xf);
            
            steplength = (xfstruc.pelvis.x - x0struc.pelvis.x);
        end
        
        function [airfrac] = getAerialFraction(this, x0, tf, tair)
            if isempty(tair) || isempty(tair)
                [xf,tf,allx,allt,tair] = this.onestep(x0);
            end
            airfrac = (tf-tair)/tf;
        end
        
        function [speed,steplength,stepfreq,airfrac] = getGaitChar(this,x0,tf,xf,tair)
            [speed] = this.getSpeed(x0, xf, tf);
            [steplength] = this.getStepLength(x0, xf);
            stepfreq = speed/steplength;
            [airfrac] = this.getAerialFraction(x0, tf, tair);
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
            
            A = this.getSensitivtyToStates(x0, 'perturbationAmount', perturbationAmount, varargin{:});
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
            xNextUnperturbed = this.oneStep(x0, varargin{:});
            for i = 1 : length(x0)
                x0Perturbed = x0;
                x0Perturbed(i) = x0Perturbed(i) + perturbationAmount;
                xNextPerturbed = this.oneStep(x0Perturbed, varargin{:});
                AMatrix(:, i) = (xNextPerturbed - xNextUnperturbed) / perturbationAmount;
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
            
            xNextUnperturbed = this.oneStep(x0, varargin{:});
            this.(parameterName) = this.(parameterName) + perturbationAmount;
            xNextPerturbed = this.oneStep(x0, varargin{:});
            
            BMatrix = (xNextPerturbed - xNextUnperturbed) / perturbationAmount;
        end
        
       
        
        
    end
    
    
end

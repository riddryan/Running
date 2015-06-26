classdef SwingSLIP < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
    
    properties
        mpelvis = 1;
        gslope = 0;
        g = 1;
        N = 12;
        statestovary = [3 5 7:8]; 
        statestomeasure = [3 4 7:8];   
        %rest lengths
        stancel=1; swingl = .8;
        %rest angles
        hipl = 0;
        
        mfoot = 1;
        
        impulsecoeff = 0;
        tanimpulsecoeff = 0;
        lockable = 0;
        lockstate = 0;
        
        %Springs
        kstance = 12; kswing = 0.01; khip = 0.01;
        %Dampers
        cstance = 0; cswing = 0; chip = 0;
        
        runcharic = struct('speed',0.9745,'steplength',1.1905,'airfrac',0.2703);
        
        useHSevent = 0;
        
        phases = {'Aerial' 'Stance' 'Aerial'};
    end
    
    
    methods (Static)
        
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            savepath = [dir '\Animations\'];
            aviname = [savepath 'SwingSLIP2.avi'];
            onephasesim = 0;
            manystep = 0;
            test = 'NoSprings';
            
            LineWidth=3;
            LineSize=3;
            TextSize=14;
            fontstyle='bold';
            fonttype='Times New Roman';
            
            runner = SwingSLIP;
            

            
            IC = SwingSLIPState;
            switch test
                
                case 'NoSprings'
                    
                    runner.lockable = 0;
                    runner.tanimpulsecoeff = 1.3;
                    runner.impulsecoeff = 2.5;
                    %                     runner.useHSevent = 1;
                    runner.kstance = 13.786169998104491; %12
                    runner.kswing = 0; %0.01
                    runner.khip = 0; %0.01
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = 0.7;
                    
                    IC.pelvis.xDot = 1.003510012689240;
                    IC.pelvis.yDot = 0.137917306814717;
                    
                    IC.stancefoot.Angle = -1.146316858313909;
                    IC.stancefoot.Length = runner.stancel;
                    
                    IC.swingfoot.Angle = -1.977468566646678;
                    IC.swingfoot.Length = runner.stancel;
                    
                    x0 = IC.getVector();
                
                case 'NoSwingSpring'
                    
                    runner.lockable = 0;
                    runner.tanimpulsecoeff = 0;
                    runner.impulsecoeff = 4;
                    %                     runner.useHSevent = 1;
                    runner.kstance = 13.786169998104491; %12
                    runner.kswing = 7.5; %0.01
                    runner.khip = 18; %0.01
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = 0.7;
                    
                    IC.pelvis.xDot = 1.003510012689240;
                    IC.pelvis.yDot = 0.137917306814717;
                    
                    IC.stancefoot.Angle = -1.146316858313909;
                    IC.stancefoot.Length = runner.stancel;
                    
                    IC.swingfoot.Angle = -1.977468566646678;
                    IC.swingfoot.Length = runner.stancel;
                    
                    x0 = IC.getVector();
                    
                case 'NoSwingSpringLock'
                    
                    runner.lockable = 1;
                    runner.tanimpulsecoeff = 0;
                    runner.impulsecoeff = 2.5;
                    %                     runner.useHSevent = 1;
                    runner.kstance = 13.786169998104491; %12
                    runner.kswing = 0; %0.01
                    runner.khip = 6; %0.01
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = 1.3;
                    
                    IC.pelvis.xDot = 1.003510012689240;
                    IC.pelvis.yDot = 0.137917306814717;
                    
                    IC.stancefoot.Angle = -1.146316858313909;
                    IC.stancefoot.Length = runner.stancel;
                    
                    IC.swingfoot.Angle = -1.977468566646678;
                    IC.swingfoot.Length = runner.stancel;
                    
                    x0 = IC.getVector();
                
                case 'LockNoImpulse'

                    runner.lockable = 1;
                    runner.tanimpulsecoeff = 0;
                    runner.impulsecoeff = 0;
%                     runner.useHSevent = 1;
                    runner.kstance = 12.8734; %12
                    runner.kswing = 15; %0.01
                    runner.khip = 4.5; %0.01
                    runner.gslope = 0;
                    runner.swingl = 0.7;
                    runner.hipl = 1.5;
                    
                    IC.pelvis.xDot = 1.0138;
                    IC.pelvis.yDot = 0.1651;
                    
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    
                    IC.swingfoot.Angle = -2;
                    IC.swingfoot.Length = runner.stancel;

                    x0 = IC.getVector();
                    
                               
                otherwise
                    error('Undefined Test Case')
            end
            
            [x0,runner] = runner.GoodInitialConditions(x0);
            
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
                [xf,tf,allx,allt,tair,runner,phasevec,tstance] = runner.onestep(x0,'interleaveAnimation',1);
                runner.printStepCharacteristics(x0,xf,tf,tair,tstance,allt,allx);
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
        function [this] = SwingSLIP(input)
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

        function [xf,tf,allx,allt,tair,this,phasevec,tstance]= onestep(this, x0,varargin)
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
            [x0,this] = this.GoodInitialConditions(x0);
            sim = 1;
            phasenum= 1;
            tstart = 0;
            allt = [];
            allx = [];
            phasevec = [];
            phaseevents = {@(t,x) this.AerialEvents1(t,x), @(t,x) this.StanceEvents(t,x), @(t,x) this.AerialEvents2(t,x)};;
            %
            
            while sim
                %% Phase transition & Integration
                phase =  this.phases{phasenum}; %Get name of phase corresponding to phasenum
                x0 = this.phaseTransition(tstart,x0,phase);
                if phasenum == 1
                    x0 = this.getTOImpulse(x0);
                end
                odex0 = x0;
                opts = odeset('Events', phaseevents{phasenum},'RelTol',RelTol','AbsTol',AbsTol); %Set integration options
                [t,x,~,~,ie] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tstart:dt:tstart+tmax,odex0,opts); %Integrate dynamics
                
                if ~isempty(ie) || t(end) == tmax;
                    ie = ie(end);
                end
                
                if sum(sum(isnan(x)))
                   blah = 1; 
                end
                
                %%  Recording & Concatenating Integration Results
                dexes=1:length(t);

                allt = [allt;t(dexes)]; allx = [allx;x(dexes,:)]; phasevec = [phasevec; phasenum*ones(length(dexes),1)];
                tstart = allt(end);
                x0 = allx(end,:);
                
                %% Decide which Phase to Move To
                if isempty(ie) || ie == 3
                    sim = 0;
                    break;
                end
                if ie == 1
                    if phasenum == 3
                        sim = 0;
                    else
                        phasenum = phasenum+1;
                    end
                end
                
                if ie == 2
                    phasenum = phasenum;
                    this.lockstate = 1;
                    phaseevents = {@(t,x) this.AerialEvents1(t,x), @(t,x) this.StanceEvents(t,x), @(t,x) this.AerialEvents2(t,x)};
                end
                
            end
            this.lockstate = 0;
            xf = allx(end,:); tf = allt(end);
            tair = allt(find(phasevec==3,1));
            tstance = allt(find(phasevec==2,1));
            if isempty(tair)
                tair = tf;
            end
            
            %Switch legs
            xf([3:6 9:12]) = xf([5 6 3 4 11 12 9 10]); 
           [xf,~] = this.phaseTransition(tf,xf,'Stance');
           
           %Remap swing leg to stance leg during Aerial Phase 1
%            airdex1 = 1:find(phasevec==2,1);
%            airdex2 = find(phasevec==2,1,'last'):length(phasevec);
%            allx(airdex1,[3 4 9 10]) = allx(airdex2,[5 6 11 2]);
            
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
        
        function [value, isTerminal, direction]  = AerialEvents1(this,t,state)

                pts = this.getPoints(state);
                value(1) = pts.stancefoot(2);
                direction(1) = 0;
                isTerminal(1) = 1;

            %Event at full extension if not already locked, and locking is enabled
            if this.lockable && ~this.lockstate
                ss = SwingSLIPState(state);
                value(2) = ss.swingfoot.Length - this.stancel;
                if t>0.05
                isTerminal(2) = 1;
                else
                   isTerminal(2) = 0; 
                end
                direction(2) = 1;
            end
        end
        
        function [value, isTerminal, direction]  = StanceEvents(this,t,state)
            %Leg reaches full extention
            ss = SwingSLIPState(state);
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
                direction(2) = 1;
            end
            
            if ~this.useHSevent
                value(3) = (this.runcharic.steplength/this.runcharic.speed*(1+this.runcharic.airfrac) - t);
                direction(3) = 0;
                isTerminal(3) = 1;
            end
        end

        function [value, isTerminal, direction]  = AerialEvents2(this,t,state)
            if ~this.useHSevent
                value(1) = (this.runcharic.steplength/this.runcharic.speed*(1+this.runcharic.airfrac) - t);
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
                ss = SwingSLIPState(state);
                value(2) = ss.swingfoot.Length - this.stancel;
                isTerminal(2) = 1;
                direction(2) = 1;
            end
        end
        
        function [newstate,this] = GoodInitialConditions(this,x0,varargin)
            this.getQandUdefs(x0);
            

                %Shift body (without changing configuration) so that heel is on ground
                ss = SwingSLIPState(x0);
                points = this.getPoints(x0);
                ss.pelvis.x = points.pelvis(1) - points.swingfoot(1);
                ss.pelvis.y = points.pelvis(2) - points.swingfoot(2);
                
                ang = ss.swingfoot.Angle;
                length = ss.swingfoot.Length;
                vpelx = ss.pelvis.xDot;
                vpely = ss.pelvis.yDot;
                
                %Conserve Velocity of Pelvis; determine required leg
                %velocities
                ss.swingfoot.AngleDot = (sin(ang)*vpelx - cos(ang)*vpely)/length;
                ss.swingfoot.LengthDot = -cos(ang)*(vpelx + tan(ang)*vpely);
%                 
                newstate = ss.getVector;
                
                if sum(isnan(newstate))
                   blah = 1; 
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
            

            
            if this.khip>0
                                lpelvis = 0.3;
                torso = points.pelvis + [0 lpelvis];
                plotter.plotLine(points.pelvis,torso)
                
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
                    
            end
            
            %Draw Masses
            plotter.plotMass(points.pelvis);
            plotter.plotMass(points.stancefoot,'scaling',0);
            plotter.plotMass(points.swingfoot,'scaling',0);
            
            axis equal;
            
            %Set Axis Limits
            xLims = [points.pelvis(1)]+ [-1 2.5];
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
                [xf, tf, allx, allt, tair,~,~,tstance]= onestep(this, x0,'aviname',aviname,'interleaveAnimation',1,'interLeaveAnimationFrameSkip',interleaveAnimationFrameskip,'tmax',tmax);
                [speed,steplength,stepfreq,airfrac] = this.getGaitChar(x0,tf,xf,tair,tstance,allt, allx);
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
            function [MM,rhs] = getMMandRHS(this,time,x)
                %%
                %Phase specifies what equations to use, EG 'aerial' or 'stance'
                state = x;
                this.getParams();
                this.getQandUdefs(state);
                c5 = cos(q5); s5 = sin(q5); 
                
                MM = zeros(6,6); rhs = zeros(6,1);
                
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
                rhs(1) = g*mpelvis*sin(gslope);
                rhs(2) = -(g*mpelvis*cos(gslope));
                rhs(3) = 0;
                rhs(4) = -(u4*cstance) - q4*kstance + kstance*stancel;
                rhs(5) = -(u5*chip) - q5*khip - hipl*khip - q6*(2*u5*u6 + g*cos(q5 - ...
                    gslope));
                rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + u5*u5) - g*sin(q5 - ...
                    gslope);
                
                
                
            end
    
    
    function [C, CDot] = getConstraints(this,state,phase)
        %%
        this.getQandUdefs(state);
        
        switch phase
            case {'Stance'}
constraintJacobianStance(1,1) = 1; constraintJacobianStance(1,2) = 0; ...
constraintJacobianStance(1,3) = -(q4*sin(q3)); constraintJacobianStance(1,4) ...
= cos(q3); constraintJacobianStance(1,5) = 0; constraintJacobianStance(1,6) = ...
0; 
constraintJacobianStance(2,1) = 0; constraintJacobianStance(2,2) = 1; ...
constraintJacobianStance(2,3) = q4*cos(q3); constraintJacobianStance(2,4) = ...
sin(q3); constraintJacobianStance(2,5) = 0; constraintJacobianStance(2,6) = ...
0; 


constraintJacobianStanceDot(1,1) = 0; constraintJacobianStanceDot(1,2) = 0; ...
constraintJacobianStanceDot(1,3) = -(q4*u3*cos(q3)) - u4*sin(q3); ...
constraintJacobianStanceDot(1,4) = -(u3*sin(q3)); ...
constraintJacobianStanceDot(1,5) = 0; constraintJacobianStanceDot(1,6) = 0; 
constraintJacobianStanceDot(2,1) = 0; constraintJacobianStanceDot(2,2) = 0; ...
constraintJacobianStanceDot(2,3) = u4*cos(q3) - q4*u3*sin(q3); ...
constraintJacobianStanceDot(2,4) = u3*cos(q3); ...
constraintJacobianStanceDot(2,5) = 0; constraintJacobianStanceDot(2,6) = 0; 
                
                
                
                C = constraintJacobianStance;
                CDot = constraintJacobianStanceDot;
                
            case 'Aerial'
constraintJacobianAerial(1,1) = 0; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 1; constraintJacobianAerial(1,4) = 0; ...
constraintJacobianAerial(1,5) = 0; constraintJacobianAerial(1,6) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 0; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 1; ...
constraintJacobianAerial(2,5) = 0; constraintJacobianAerial(2,6) = 0; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; ...
constraintJacobianAerialDot(1,5) = 0; constraintJacobianAerialDot(1,6) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; ...
constraintJacobianAerialDot(2,5) = 0; constraintJacobianAerialDot(2,6) = 0; 

                C = constraintJacobianAerial;
                CDot = constraintJacobianAerialDot;
                
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
         c5 = cos(q5); s5 = sin(q5);
         
        
        kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kstance*((-q4 + stancel)*(-q4 + stancel)))/2. + ...
g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

kineticEnergy2 = (mfoot*(-2*q6*s5*u1*u5 + 2*s5*u2*u6 + c5*(2*q6*u2*u5 + ...
2*u1*u6) + u1*u1 + u2*u2 + q6*q6*(u5*u5) + u6*u6))/2.;

potentialEnergy2 = q2*g*cos(gslope) + (khip*((-q5 - hipl)*(-q5 - hipl)))/2. + ...
(kswing*((q6 - swingl)*(q6 - swingl)))/2. + q6*g*sin(q5 - gslope) - ...
q1*g*sin(gslope);

PEgrav2 = q2*g*cos(gslope) + q6*g*sin(q5 - gslope) - q1*g*sin(gslope);

PEspring2 = (khip*((-q5 - hipl)*(-q5 - hipl)))/2. + (kswing*((q6 - ...
swingl)*(q6 - swingl)))/2.;

stanceE = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

swingE = (kswing*((q6 - swingl)*(q6 - swingl)))/2.;

hipE = (khip*((-q5 - hipl)*(-q5 - hipl)))/2.;
        
        
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
            c5 = cos(q5);s5 = sin(q5);
            
points.stancefoot(1) = q1 + q4*cos(q3); 
points.stancefoot(2) = q2 + q4*sin(q3); 


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
            c5 = cos(q5);s5 = sin(q5);
            
vels.stancefoot(1) = u1 + u4*cos(q3) - q4*u3*sin(q3); 
vels.stancefoot(2) = u2 + q4*u3*cos(q3) + u4*sin(q3); 


vels.swingfoot(1) = u1 - q6*s5*u5 + c5*u6; 
vels.swingfoot(2) = u2 + c5*q6*u5 + s5*u6; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 

        end
        
        
        function [comWR] = getcomWR(this,state,phase)
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
                    ceq = [];
                    c  = -sum(fty(fty<0));
                    if isempty(c)
                        c = -1e-10;
                    end
                end
                
                function [c,ceq] = floorandswinglconstraint(this,~,~,~,allx,allt,varargin)
                    
                    fty = zeros(size(allx,1),1);
                 fty = zeros(size(allx,1),1);
                    for i = 1:size(allx,1)
                        pts = this.getPoints(allx(i,:));
                        fty(i) = pts.swingfoot(2);
                    end
                    ceq = 1 - this.swingl;
                    c  = -sum(fty(fty<0));
                    if isempty(c)
                        c = -1e-10;
                    end
                end
                
                function cost = impulsecost(this,x0,xf,tf,allx,allt,tair,phasevec)
                   TOdex = find(phasevec==3,1); 
                   vels = this.getVels(allx(TOdex,:));
                   cost = dot(vels.stancefoot,vels.stancefoot);
                end
        
        %% Additional Dynamics Calculations
        
         function [finalStates, finalParameters, limitCycleError, ...
                c, ceq, exitflag, optimoutput, lambda] = ...
                findLimitCycle(this, initialConditionGuess, varargin)
            %%
            %Find a limit cycle for the model.  That is find parameter values of
            %parametersToAlter and initial conditions of the model such that the
            %state of the model at the next step is the same as at the start of
            %the simulation.  Must give an initial guess initialConditionGuess for the initial
            %conditions for the model.  Must specify if you want to let the
            %optimizer change parameters in parametersToAlter.  Can specify a
            %function to minimize while finding a limit cycle, as well as
            %additional constraints
            
            
            %Default options
            additionalConstraintFunction = []; %Needs to be of the form [c,ceq] = additionalConstraintFunction(x), where c is a column vector of
            %the violation of the additional inequality constraints, and ceq is a
            %column vector of the violation of additiona equality constraints
            parametersToAlter = {}; %Should be a cell list of strings
            TolCon=[];             %Constraint tolerance for fmincon
            addedCostFcn = [];
            plotiter = 1;
            minmass = 1e-3;
            dbrun = 0;
            TolX = [];
            MaxEvals = [];
            algorithm = 'sqp';
            LB = [];
            FinDiffType = [];
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'additionalConstraintFunction'
                        additionalConstraintFunction = value;
                    case 'parametersToAlter'
                        parametersToAlter = value;
                    case 'TolCon'
                        TolCon = value;
                    case 'Objective'
                        addedCostFcn = value;
                    case 'plotiter'
                        plotiter = value;
                    case 'TolX'
                        TolX = value;
                    case 'MaxEvals'
                        MaxEvals = value;
                    case 'Algo'
                        algorithm = value;
                    case 'LB'
                        LB = value;
                    case 'FinDiffType'
                        FinDiffType= value;
                end
            end
            if isempty(algorithm)
                algorithm = 'sqp';
            end
            
            constraintFunction = @(x) this.ConstFcn(x,parametersToAlter,additionalConstraintFunction,initialConditionGuess,addedCostFcn);
            objectiveFunction = @(x) this.CostFcn(x,parametersToAlter,additionalConstraintFunction,initialConditionGuess,addedCostFcn);
            
            
            optimizerGuess = initialConditionGuess(this.statestovary); %Give optimizer only the states of the model that are allowed to be varied
            if (~isempty(parametersToAlter))
                optimizerGuess = [optimizerGuess this.getParametersFromList(parametersToAlter)'];
            end
            
            if isempty(LB)
                LB = -Inf*ones(size(optimizerGuess));
            end
            UB = Inf*ones(size(optimizerGuess));
            %If parameter is a mass, length, spring, or damping, don't let it be
            %negative.
                for i = 1:length(parametersToAlter)
                    if strcmp(parametersToAlter{i}(1),'c') ||  strcmp(parametersToAlter{i}(1),'l') || strcmp(parametersToAlter{i}(1),'k')
                        LB(length(this.statestovary)+i) = -1e-10;
                        
                    elseif  strcmp(parametersToAlter{i}(1),'m')
                        LB(length(this.statestovary)+i) = minmass;
                        
                    elseif regexpi(parametersToAlter{i},'transition')
                        LB(length(this.statestovary)+i) = .8;
                        UB(length(this.statestovary)+i) = 1;
                        %          elseif regexpi(parametersToAlter{i},'impulsecoeff')
                        %              LB(length(this.statestovary)+i) = 0;
                    end
                end
            
            if plotiter
                plotfcns = {@optimplotx,@optimplotfval,@optimplotconstrviolation,@optimplotstepsize};
            else
                plotfcns =[];
            end
            
            opt= optimset;
            opt.Algorithm = algorithm;
            %       opt.Algorithm = 'active-set';
            opt.Display = 'iter-detailed';
            %       opt.DiffMinChange = 1e-4;
            opt.TolCon = TolCon;
            opt.TolConSQP = TolCon;
            opt.PlotFcns = plotfcns;
            opt.TolX = TolX;
            opt.MaxFunEvals = MaxEvals;
            opt.FinDiffType = FinDiffType;
            if strcmp(opt.Algorithm,'active-set') && isempty(addedCostFcn)
                opt.TolFun = 0;
            end
            
            if ~dbrun
                [finalOptimizerState,~,exitflag,optimoutput,lambda] = fmincon(objectiveFunction, optimizerGuess, ...
                    [], [], [], [], LB, UB, constraintFunction, opt);
            else
                finalOptimizerState =  optimizerGuess;
                exitflag = [];
                optimoutput = [];
                lambda = [];
            end
            
            
            [finalVariedStates, finalParameters] = this.separateStatesAndParameters(finalOptimizerState, parametersToAlter);
            this = this.setParametersFromList(parametersToAlter,finalParameters);
            
            constraintFunction = @(x) this.ConstFcn(x,parametersToAlter,additionalConstraintFunction,initialConditionGuess,addedCostFcn);
            [c, ceq, limitCycleError] = constraintFunction(finalOptimizerState);
            
            
            finalStates = initialConditionGuess;
            finalStates(this.statestovary) = finalVariedStates;
            finalStates = this.GoodInitialConditions(finalStates);
            
            newrunner = this;
            newrunner.printStepCharacteristics(finalStates);
        end
        
        function [c, ceq, limitCycleError, cExtra, ceqExtra, cost] = fixedPointConstraint(this,x,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost)
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
        
            [xf, tf, allx, allt, tair,this,phasevec,tstance] = this.onestep(x0);
        x0 = allx(1,:);
        if isrow(xf)
            xf = xf';
        end
        
        limitCycleError = xf(this.statestomeasure) - allx(find(allt==tstance,1),this.statestomeasure)';
        
        c=[];
        ceq=limitCycleError;
        
        runcharic = this.runcharic;
        if ~isempty(runcharic.speed)
            [speed] = getSpeed(this, x0, xf, tf);
            ceq = [ceq; runcharic.speed - speed];
        end
        if ~isempty(runcharic.steplength)
            [steplength] = getStepLength(this, x0, xf, tstance, allt, allx);
            ceq = [ceq; runcharic.steplength - steplength];
        end
        if ~isempty(runcharic.airfrac)
            [airfrac] = getAerialFraction(this, x0, tf, tair,tstance);
            ceq = [ceq; runcharic.airfrac - airfrac];
        end
        
        if isempty(additionalConstraintFunction)
            cExtra = [];
            ceqExtra = [];
        else
            if ~isa(additionalConstraintFunction,'cell')
                if nargin(additionalConstraintFunction)<8
            [cExtra, ceqExtra] = additionalConstraintFunction(this,x0,xf,tf,allx,allt,tair);
                else
                 [cExtra, ceqExtra] = additionalConstraintFunction(this,x0,xf,tf,allx,allt,tair,phasevec);   
                end
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
    
            function [cost] = CostFcn(this,x,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost)
       global xLast myf myc myceq myError
       
       if ~isequal(x,xLast)
           [myc, myceq, myError, cExtra, ceqExtra, myf] = fixedPointConstraint(this,x,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost);
           xLast = x;
       end
       
       cost = myf;
    end
    
    function [c, ceq, limitCycleError, cExtra, ceqExtra, cost] = ConstFcn(this,x,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost)
        global xLast myf myc myceq myError
        
        if ~isequal(x,xLast)
            [myc, myceq, myError, cExtra, ceqExtra, myf] = fixedPointConstraint(this,x,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost);
            xLast = x;
        end
        
        c = myc;
        ceq = myceq;
        limitCycleError = myError;
        
    end
    
    %%
        
        function [GRF] = getGRF(this,t,x,phase)
            if size(x,1)==1
                x=x';
            end
            
            if ~strcmp('Aerial',phase)
                [~,GRF] = this.XDoubleDot(t,x,phase);
                GRF = GRF(1:2);
            else
                GRF=[0;0];
            end
        end
        
        function stancePower = getStancePower(this,x,phase)
            force = this.getStanceForce(x,phase);
            velocity = x(10);
            stancePower = force.*velocity;
        end
        
        function stanceforce = getStanceForce(this,x)
            this.getParams();
            stanceforce = -kstance*(x(4)-this.stancel)-cstance*(x(10));
        end
  
        function swingPower = getSwingPower(this,x)
            force = this.getSwingForce(x);
            velocity = x(12);
            swingPower = force.*velocity;
        end
        
        function swingforce = getSwingForce(this,x)
            swingforce = -this.kswing*(x(6)-this.swingl)-this.cswing*(x(12));
        end
        
        function hippower= getHipPower(this,x)
            sz = SwingSLIPState(x);
            force = this.getHipForce(x);
            velocity = sz.swingfoot.AngleDot;
            hippower = force.*velocity;
        end
        
        function hipforce = getHipForce(this,x)
            sz = SwingSLIPState(x);
            hipforce = -this.khip*(sz.swingfoot.Angle - this.hipl) ...
                       -this.chip*(sz.swingfoot.AngleDot);
        end
        
        %% Other Gait Information
        
        function [] = print(this,x0,varargin)
            this.printStepCharacteristics(x0,varargin{:});
        end
        
        function [] = printStepCharacteristics(this, x0, varargin)
            %%
            if nargin>2
                xf=varargin{1};
                tf=varargin{2};
                tair=varargin{3};
                tstance=varargin{4};
                allt = varargin{5};
                allx = varargin{6};
            else
                [xf, tf, allx, allt, tair,this,phasevec,tstance] = this.onestep(x0);
                x0 = allx(1,:);
            end
            
            
            limitCycleError = xf(this.statestomeasure) - allx(find(allt==tstance,1),this.statestomeasure)';
            
            [speed,steplength,stepfreq,airfrac] = this.getGaitChar(x0,tf,xf,tair,tstance,allt, allx);
            
            fprintf('step parameters: speed: %g, step length = %g, \n airfrac = %g, limitCycleError = %g\n', speed, steplength, airfrac, norm(limitCycleError));
        end
        
        function x0 = getTOImpulse(this,x0)
            ss = SwingSLIPState(x0);
            ss.swingfoot.AngleDot = ss.swingfoot.AngleDot*(1 - this.tanimpulsecoeff);
            ss.swingfoot.LengthDot = ss.swingfoot.LengthDot*(1 - this.impulsecoeff);
            x0 = ss.getVector;
        end
        
        
        function [speed] = getSpeed(this, x0, xf, tf)
            if isempty(xf)
                [xf,tf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = SwingSLIPState(x0);
            xfstruc = SwingSLIPState(xf);
            
            speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
        end
        
        function [steplength] = getStepLength(this, x0, xf, tstance, allt, allx)
            if isempty(xf)
                [xf,tf,allx,allt,tair,this,phasevec,tstance] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = SwingSLIPState(allx(find(allt==tstance,1),:));
            xfstruc = SwingSLIPState(xf);
            
            steplength = (xfstruc.pelvis.x - x0struc.pelvis.x);
        end
        
        function [airfrac] = getAerialFraction(this, x0, tf, tair,tstance)
            if isempty(tair) || isempty(tair)
                [xf,tf,allx,allt,tair,phasevec,tstance] = this.onestep(x0);
            end
            airfrac = (tstance)/tair;
        end
        
        function [speed,steplength,stepfreq,airfrac] = getGaitChar(this,x0,tf,xf,tair,tstance,allt, allx)
            [speed] = this.getSpeed(x0, xf, tf);
            [steplength] = this.getStepLength(x0, xf, tstance, allt, allx);
            stepfreq = speed/steplength;
            [airfrac] = this.getAerialFraction(x0, tf, tair,tstance);
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

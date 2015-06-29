classdef SwingKneeSLIP < Runner
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
        stancel=1; kneel = -0.5;
        %rest angles
        hipl = 0;
        
        lthigh = 0.6;
        lshank = 0.45;
        
        mfoot = 1;
        
        impulsecoeff = 0;
        tanimpulsecoeff = 0;
        lockable = 1;
        lockstate = 0;
        
        %Springs
        kstance = 13; kknee = 1; khip = 1;
        %Dampers
        cstance = 0; cknee = 0; chip = 0;
        
        speed = 0.9745;
        steplength = 1.1905;
        airfrac = 0.2703;
        
        useHSevent = 0;
        
        phases = {'Aerial' 'Stance' 'Aerial'};
    end
    
    properties (Dependent = true)
        runcharic;
    end
    
    methods (Static)
        
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            savepath = [dir '\Animations\'];
            aviname = [savepath 'SwingKneeSLIP2.avi'];
            onephasesim = 0;
            manystep = 0;
            test = 'LockNoImpulse';
            
            LineWidth=3;
            LineSize=3;
            TextSize=14;
            fontstyle='bold';
            fonttype='Times New Roman';
            
            runner = SwingKneeSLIP;
            

            
            IC = SwingKneeSLIPState;
            switch test
                
                case 'LockNoImpulse'

                    runner.lockable = 1;
                    runner.tanimpulsecoeff = 0;
                    runner.impulsecoeff = 0;
                    runner.useHSevent = 0;
                    runner.kstance = 12.8734; %12
                    runner.kknee = 2; %0.01
                    runner.khip = 7; %0.01
                    runner.gslope = 0;
                    runner.kneel = 0.8;
                    runner.hipl = -0.8;
                    
                    runner.lthigh = 0.56;
                    runner.lshank = 0.45;
                    
                    IC.pelvis.xDot = 1.0138;
                    IC.pelvis.yDot = 0.1651;
                    
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    
                    IC.knee.Angle = -1.88;

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
                pelvx(i) = pts.pelvis(1);
                pelvy(i) = pts.pelvis(2);
                stancefootx(i) = pts.stancefoot(1);
                stancefooty(i) = pts.stancefoot(2);
                swingfootx(i) = pts.swingfoot(1);
                swingfooty(i) = pts.swingfoot(2);
                kneex(i) = pts.knee(1);
                kneey(i) = pts.knee(2);
                vels = runner.getVels(allx(i,:));
                stancefootvelx(i) = vels.stancefoot(1);
                stancefootvely(i) = vels.stancefoot(2);
                swingfootvelx(i) = vels.swingfoot(1);
                swingfootvely(i) = vels.swingfoot(2);
                kneevelx(i) = vels.knee(1);
                kneevely(i) = vels.knee(2);
                
                leglength(i) = norm(pts.pelvis-pts.swingfoot);
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
        function [this] = SwingKneeSLIP(input)
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
                   keyboard; 
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
            
            if isempty(tstance)
               tstance = 0; 
            end
            
            %Switch legs
            xf = this.switchLegs(xf);
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
        
        function [value, isTerminal, direction]  = AerialEvents1(this,t,state)

                pts = this.getPoints(state);
                value(1) = pts.stancefoot(2);
                direction(1) = -1;
                isTerminal(1) = 1;

            %Event at full extension if not already locked, and locking is enabled
            if this.lockable && ~this.lockstate
                pts = this.getPoints(state);
                value(2) = norm(pts.pelvis - pts.swingfoot) - this.stancel;
                if t>0.05 && state(5)>-pi/2 && state(12)>0
                isTerminal(2) = 1;
                else
                   isTerminal(2) = 0; 
                end
                direction(2) = 0;
            end
        end
        
        function [value, isTerminal, direction]  = StanceEvents(this,t,state)
            %Leg reaches full extention
            ss = SwingKneeSLIPState(state);
            value = ss.stancefoot.Length - this.stancel;
            direction = 1;
            if t>0.05
                isTerminal = 1;
            else
                isTerminal = 0;
            end
            
            %Event at full extension if not already locked, and locking is enabled
            if this.lockable && ~this.lockstate
                pts = this.getPoints(state);
                value(2) = norm(pts.pelvis - pts.swingfoot) - this.stancel;
                if state(5)>-pi/2 && state(12)>0
                    isTerminal(2) = 1;
                else
                    isTerminal(2) = 0;
                end
                direction(2) = 0;
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
                pts = this.getPoints(state);
                value(2) = norm(pts.pelvis - pts.swingfoot) - this.stancel;
                if state(5)>-pi/2 && state(12)>0
                    isTerminal(2) = 1;
                else
                    isTerminal(2) = 0;
                end
                direction(2) = 0;
            end
        end
        
        function xswitched = switchLegs(this,x)
            
            %This does not give the correct swing leg states after
            %switching since that information is not used in finding a
            %limit cycle.  The correct swing leg states before the second
            %heel strike simply equals the swing leg states at the previous
            %heel strike in simulation
            
            %Convert the swing leg states into the equivalent stance leg
            %states for collision
            pts = this.getPoints(x);
            ssNEW = SwingKneeSLIPState(x);
            
            ssNEW.stancefoot.Angle = atan2(pts.swingfoot(2)-pts.pelvis(2),pts.swingfoot(1)-pts.pelvis(1));
            ssNEW.stancefoot.Length = dot([cos(ssNEW.stancefoot.Angle);sin(ssNEW.stancefoot.Angle)],...
                                          pts.swingfoot - pts.pelvis);
            
   
            xswitched = ssNEW.getVector;
        end
        
        function [newstate,this] = GoodInitialConditions(this,x0,varargin)
            this.getQandUdefs(x0);
            this.getParams();
            ss = SwingKneeSLIPState(x0);
            points = this.getPoints(x0);
            
            
            %Set Shank angle so that distance between swingfoot and
            %pelvis is equal to this.stancel.  If lshank and lthigh are not
            %long enough, this may have zero solutions.  If lshank + lthigh
            %= stancel, there is one solution (at singularity).  In general
            %there should be two solutions.  We pick the one such that the
            %shank is more clockwise than the thigh, such that the knee is
            %flexed slightly instead of hyperextended.
            
%             refangle = -pi/2 - (pi/2 - ss.stancefoot.Angle);
%             ghostfoot = this.stancel*[cos(refangle);sin(refangle)];
%             
%             x = fsolve(@(x) norm(ghostfoot - 
            
            relangle = acos((-lshank^2 - lthigh^2 + stancel^2)/(2*lshank*lthigh));
            
            if relangle>0
                ss.swingfoot.Angle = -relangle + ss.knee.Angle;
            else
                ss.swingfoot.Angle = relangle + ss.knee.Angle;
            end
            
            %Shift body (without changing configuration) so that heel is on ground
            points = this.getPoints(ss.getVector);
            
            ss.pelvis.x = points.pelvis(1) - points.swingfoot(1);
            ss.pelvis.y = points.pelvis(2) - points.swingfoot(2);
            
            shankang = ss.swingfoot.Angle;
            thighang = ss.knee.Angle;
            vpelx = ss.pelvis.xDot;
            vpely = ss.pelvis.yDot;
            
            %Conserve Velocity of Pelvis; determine required leg
            %velocities
            ss.knee.AngleDot = (csc(thighang-shankang)*(cos(shankang)*vpelx + sin(shankang)*vpely))/lthigh;
            ss.swingfoot.AngleDot = -(csc(thighang-shankang)*(cos(thighang)*vpelx + sin(thighang)*vpely))/lshank;
            %
            newstate = ss.getVector;
            
            if sum(isnan(newstate))
                keyboard;
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
            this.getParams();
            ss = SwingKneeSLIPState(state);
            plotter = RunnerPlotter;
            
            %Draw Ground
            plot([-5 5],[0 0],'k','LineWidth',2)
            
            hold on
            
            
            %Draw Lines
            plotter.plotLine(points.pelvis,points.knee,'Color',[101 156 255]/255);
            plotter.plotLine(points.knee,points.swingfoot,'Color',[101 156 255]/255);
            
            %Draw Springs
            if state(10)~=0 %state(3)>=-pi/2 || state(10)~=0
            numcoils=3;
            springwidth=.07;
            plotter.plotSpring(points.stancefoot(1),points.stancefoot(2),...
                points.pelvis(1),points.pelvis(2),...
                numcoils,this.stancel,springwidth) %stance spring
            end
            
            %Knee Spring
            if this.kknee > 0
                pelvdir = -[cos(ss.knee.Angle) sin(ss.knee.Angle)];
                footdir = [cos(ss.swingfoot.Angle) sin(ss.swingfoot.Angle)];
                kneestretch = -(ss.knee.Angle - ss.swingfoot.Angle - this.kneel)/abs(this.kneel);
                pelvpoint = points.knee + .2 * lthigh * pelvdir;
                footpoint = points.knee + .2 * lshank * footdir;
                
                plotter.plotCircSpring(pelvpoint,footpoint,0.05,1,2,0.05,kneestretch,'Color',[150 150 76]/255)

            end
            

            
            if this.khip>0
                lpelvis = 0.3;
                torso = points.pelvis + [0 lpelvis];
                plotter.plotLine(points.pelvis,torso)
                
                    torsodir = [0 1];
                    torsopoint = points.pelvis + 0.7 * lpelvis * torsodir;
                    swingdir = [cos(ss.knee.Angle) sin(ss.knee.Angle)];
                    swingpoint = points.pelvis + .2 * this.lthigh * swingdir;
                    
%                     if points.swingfoot(2)>1e-4
                    plotter.plotCircSpring(torsopoint,swingpoint,.05,1,2,.05,...
                        (state(5)-this.hipl),'Color',[101 156 255]/255)
%                     end
                    
            end
            
            %Draw Masses
            plotter.plotMass(points.pelvis);
            if state(10)~=0
            plotter.plotMass(points.stancefoot,'scaling',0);
            end
            plotter.plotMass(points.swingfoot,'scaling',0);
            plotter.plotMass(points.knee,'scaling',0);
            
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
                c5 = cos(q5); c6 = cos(q6); s5 = sin(q5); s6 = sin(q6); c5m6 = cos(q5 - q6); s5m6 = sin(q5 - q6); 
                
                MM = zeros(6,6); rhs = zeros(6,1);
                
% Mass Matrix
MM(1,1) = mpelvis; MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = 0; MM(1,5) = 0; ...
MM(1,6) = 0; 
MM(2,1) = 0; MM(2,2) = mpelvis; MM(2,3) = 0; MM(2,4) = 0; MM(2,5) = 0; ...
MM(2,6) = 0; 
MM(3,1) = 0; MM(3,2) = 0; MM(3,3) = 0; MM(3,4) = 0; MM(3,5) = 0; MM(3,6) = 0; 
MM(4,1) = 0; MM(4,2) = 0; MM(4,3) = 0; MM(4,4) = 0; MM(4,5) = 0; MM(4,6) = 0; 
MM(5,1) = -(s5*lthigh*(1 + mfoot)); MM(5,2) = c5*lthigh*(1 + mfoot); MM(5,3) ...
= 0; MM(5,4) = 0; MM(5,5) = (1 + mfoot)*(lthigh*lthigh); MM(5,6) = ...
c5m6*lshank*lthigh*mfoot; 
MM(6,1) = -(s6*lshank*mfoot); MM(6,2) = c6*lshank*mfoot; MM(6,3) = 0; MM(6,4) ...
= 0; MM(6,5) = c5m6*lshank*lthigh*mfoot; MM(6,6) = mfoot*(lshank*lshank); 

% righthand side terms
rhs(1) = g*mpelvis*sin(gslope); 
rhs(2) = -(g*mpelvis*cos(gslope)); 
rhs(3) = 0; 
rhs(4) = -(u4*cstance) - q4*kstance + kstance*stancel; 
rhs(5) = -(u5*chip) - u5*cknee + u6*cknee + hipl*khip + q6*kknee - q5*(khip + ...
kknee) + kknee*kneel - 2*g*lthigh*mfoot*cos(q5 - gslope) - ...
s5m6*lshank*lthigh*mfoot*(u6*u6); 
rhs(6) = (u5 - u6)*cknee - kknee*(-q5 + q6 + kneel) - g*lshank*mfoot*cos(q6 - ...
gslope) + s5m6*lshank*lthigh*mfoot*(u5*u5); 
                
                
                
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
           C = [C;[0 0 0 0 -1 1]];
           CDot = [CDot;[0 0 0 0 0 0]];
        end
        
    end
    
    
    function [E] = getEnergies(this,state,phase)
        this.getParams();
        this.getQandUdefs(state);
         c5 = cos(q5); c6 = cos(q6); s5 = sin(q5); s6 = sin(q6); c5m6 = cos(q5 - q6); s5m6 = sin(q5 - q6); 
         
         mknee = 1;
         
kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;

potentialEnergy = (kstance*((-q4 + stancel)*(-q4 + stancel)))/2. + ...
g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));

PEspring = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

kineticEnergy2 = (mknee*(-2*s5*u1*u5*lthigh + 2*c5*u2*u5*lthigh + u1*u1 + ...
u2*u2 + u5*u5*(lthigh*lthigh)) + mfoot*(-2*s6*u1*u6*lshank + ...
2*c6*u2*u6*lshank - 2*s5*u1*u5*lthigh + 2*c5*u2*u5*lthigh + ...
2*c5m6*u5*u6*lshank*lthigh + u1*u1 + u2*u2 + u6*u6*(lshank*lshank) + ...
u5*u5*(lthigh*lthigh)))/2.;

potentialEnergy2 = 2*q2*g*cos(gslope) + s6*g*lshank*cos(gslope) + (khip*((q5 ...
- hipl)*(q5 - hipl)))/2. + (kknee*((q5 - q6 - kneel)*(q5 - q6 - kneel)))/2. + ...
2*g*lthigh*sin(q5 - gslope) - 2*q1*g*sin(gslope) - c6*g*lshank*sin(gslope);

PEgrav2 = 2*q2*g*cos(gslope) + s6*g*lshank*cos(gslope) + 2*g*lthigh*sin(q5 - ...
gslope) - 2*q1*g*sin(gslope) - c6*g*lshank*sin(gslope);

PEspring2 = (khip*((q5 - hipl)*(q5 - hipl)))/2. + (kknee*((q5 - q6 - ...
kneel)*(q5 - q6 - kneel)))/2.;

stanceE = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;

kneeE = (kknee*((q5 - q6 - kneel)*(q5 - q6 - kneel)))/2.;

hipE = (khip*((q5 - hipl)*(q5 - hipl)))/2.;
        
        
        E.stanceE = stanceE;
        E.kneeE = kneeE;
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
            c5 = cos(q5); c6 = cos(q6); s5 = sin(q5); s6 = sin(q6); c5m6 = cos(q5 - q6); s5m6 = sin(q5 - q6); 
            
points.stancefoot(1) = q1 + q4*cos(q3); 
points.stancefoot(2) = q2 + q4*sin(q3); 


points.swingfoot(1) = q1 + c6*lshank + c5*lthigh; 
points.swingfoot(2) = q2 + s6*lshank + s5*lthigh; 


points.knee(1) = q1 + c5*lthigh; 
points.knee(2) = q2 + s5*lthigh; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 

        end
        
        function [vels] = getVels(this, state)
            
            this.getParams();
            this.getQandUdefs(state);
            c5 = cos(q5); c6 = cos(q6); s5 = sin(q5); s6 = sin(q6); c5m6 = cos(q5 - q6); s5m6 = sin(q5 - q6); 
            
vels.stancefoot(1) = u1 + u4*cos(q3) - q4*u3*sin(q3); 
vels.stancefoot(2) = u2 + q4*u3*cos(q3) + u4*sin(q3); 


vels.swingfoot(1) = u1 - s6*u6*lshank - s5*u5*lthigh; 
vels.swingfoot(2) = u2 + c6*u6*lshank + c5*u5*lthigh; 


vels.knee(1) = u1 - s5*u5*lthigh; 
vels.knee(2) = u2 + c5*u5*lthigh; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 

        end
        
        function Jswingfoot = getJswingfoot(this,state)
                        this.getParams();
            this.getQandUdefs(state);
            c5 = cos(q5); c6 = cos(q6); s5 = sin(q5); s6 = sin(q6); c5m6 = cos(q5 - q6); s5m6 = sin(q5 - q6); 
            
            Jswingfoot(1,1) = 1; Jswingfoot(1,2) = 0; Jswingfoot(1,3) = 0; ...
Jswingfoot(1,4) = 0; Jswingfoot(1,5) = -(s5*lthigh); Jswingfoot(1,6) = ...
-(s6*lshank); 
Jswingfoot(2,1) = 0; Jswingfoot(2,2) = 1; Jswingfoot(2,3) = 0; ...
Jswingfoot(2,4) = 0; Jswingfoot(2,5) = c5*lthigh; Jswingfoot(2,6) = ...
c6*lshank; 
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
            [speed] = getSpeed(this, x0, xf, tf, tstance, allt, allx);
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
            ss = SwingKneeSLIP(x);
            stanceforce = -this.kstance*(ss.stancefoot.Length - this.stancel)+ ...
                          -this.cstance*(ss.stancefoot.LengthDot);
        end
  
        function swingPower = getKneePower(this,x)
            force = this.getKneeForce(x);
            ss = SwingKneeSLIP(x);
            velocity = ss.knee.AngleDot - ss.swingfoot.AngleDot;
            swingPower = force.*velocity;
        end
        
        function swingforce = getKneeForce(this,x)
            ss = SwingKneeSLIP(x);
            swingforce = -this.kknee*(ss.knee.Angle - ss.swingfoot.Angle - this.kneel) + ...
                         -this.cswing*(ss.knee.AngleDot - ss.swingfoot.AngleDot);
        end
        
        function hippower= getHipPower(this,x)
            ss = SwingKneeSLIPState(x);
            force = this.getHipForce(x);
            velocity = ss.knee.AngleDot;
            hippower = force.*velocity;
        end
        
        function hipforce = getHipForce(this,x)
            ss = SwingKneeSLIPState(x);
            hipforce = -this.khip*(ss.knee.Angle - this.hipl) ...
                       -this.chip*(ss.knee.AngleDot);
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
            this.getParams();
            ss = SwingKneeSLIPState(x0);
            pts = this.getPoints(x0);
            Jswingfoot = this.getJswingfoot(x0);
            
            legangle = atan2(pts.swingfoot(2)-pts.pelvis(2),pts.swingfoot(1)-pts.pelvis(1));
            legdir = [cos(legangle);sin(legangle)];
            tandir = [0 -1;1 0]*legdir;

            vsep = dot([ss.pelvis.xDot;ss.pelvis.yDot],legdir);
            vtan = dot([ss.pelvis.xDot;ss.pelvis.yDot],tandir);
            
            vfoot = this.impulsecoeff*vsep*legdir;
            vfoot = vfoot + this.tanimpulsecoeff*vtan*tandir;
            
            shankang = ss.swingfoot.Angle;
            thighang = ss.knee.Angle;
            vpelx = ss.pelvis.xDot;
            vpely = ss.pelvis.yDot;
            
            ss.knee.AngleDot = csc(thighang - shankang)*(-vfoot(1)*cos(shankang) - vfoot(2)*sin(shankang) + cos(shankang)*vpelx + sin(shankang)*vpely)/lthigh;
            ss.swingfoot.AngleDot = csc(thighang - shankang)*(vfoot(1)*cos(thighang) + vfoot(2)*sin(thighang) - cos(thighang)*vpelx - sin(thighang)*vpely)/lshank;
            
%                         ss.knee.AngleDot = (csc(thighang-shankang)*(cos(shankang)*vpelx + sin(shankang)*vpely))/lthigh;
%             ss.swingfoot.AngleDot = -(csc(thighang-shankang)*(cos(thighang)*vpelx + sin(thighang)*vpely))/lshank;
            
            x0 = ss.getVector;
        end
        
        
        function [speed] = getSpeed(this, x0, xf, tf, tstance, allt, allx)
            if isempty(xf)
                [xf,tf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = SwingKneeSLIPState(allx(find(allt==tstance,1),:));
            xfstruc = SwingKneeSLIPState(xf);
            
            speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / (tf - tstance);
        end
        
        function [steplength] = getStepLength(this, x0, xf, tstance, allt, allx)
            if isempty(xf)
                [xf,tf,allx,allt,tair,this,phasevec,tstance] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = SwingKneeSLIPState(allx(find(allt==tstance,1),:));
            xfstruc = SwingKneeSLIPState(xf);
            
            steplength = (xfstruc.pelvis.x - x0struc.pelvis.x);
        end
        
        function [airfrac] = getAerialFraction(this, x0, tf, tair,tstance)
            if isempty(tair) || isempty(tair)
                [xf,tf,allx,allt,tair,phasevec,tstance] = this.onestep(x0);
            end
            airfrac = (tstance)/tair;
        end
        
        function [speed,steplength,stepfreq,airfrac] = getGaitChar(this,x0,tf,xf,tair,tstance,allt, allx)
            [speed] = this.getSpeed(x0, xf, tf, tstance, allt, allx);
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
        
       
        function runcharic = get.runcharic(this)
           runcharic.speed = this.speed;
           runcharic.steplength = this.steplength;
           runcharic.airfrac = this.airfrac;
        end
        
        function this = set.runcharic(this,rc)
           this.speed = rc.speed;
           this.steplength = rc.steplength;
           this.airfrac = rc.airfrac;
        end
        
    end
    
    
end

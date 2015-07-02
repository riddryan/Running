classdef SLIP < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
    
    properties
        mpelvis = 1;
        mfoot = 0;
        gslope = 0;
        g = 1;
        N = 8;
        statestovary = [3 4 7 8];
        statestomeasure = [3 4 7 8];
        %rest lengths
        stancel=1;
        %rest angles
        
        %Springs
        kstance = 12.873128756709848;
        x0 =   [-0.427857226694680;...
            0.903837668950403;...
            -1.128671904307318;...
            0.999992169093619;...
            1.013809789981797;...
            -0.165114140645987;...
            -0.845681433083618;...
            -0.582991283170142];
        
        phases = {'Stance' 'Aerial'};
        
        
    end
    
    
    methods (Static)
        
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            savepath = [dir '\Animations\'];
            aviname = [savepath 'SLIP1.avi'];
            onephasesim = 0;
            manystep = 0;
            test = 'step';
            
            LineWidth=3;
            LineSize=3;
            TextSize=14;
            fontstyle='bold';
            fonttype='Times New Roman';
            
            runner = SLIP;
            

            
            IC = SLIPState;
            switch test
                case 'step'
                    
                    runner.kstance = 12.8734; %12
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    %
                    IC.pelvis.xDot = 1.0138;
                    IC.pelvis.yDot = -0.1651;
                    
                    IC.stancefoot.AngleDot = -0.8457;
                    IC.stancefoot.LengthDot = -0.5830;
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
                energies = runner.getEnergies(allx(i,:));
                TOTE(i) = energies.Total;
                KE(i) = energies.KE;
                PE(i) = energies.PE;
                PEgrav(i) = energies.PEgrav;
                pts = runner.getPoints(allx(i,:));
                stancefootx(i) = pts.stancefoot(1);
                stancefooty(i) = pts.stancefoot(2);
                vels = runner.getVels(allx(i,:));
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
        function [this] = SLIP(input)
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
            xpre = x0;
            ypelvstart = xpre(2);
            phaseevents = { @(t,x) this.StanceEvents(t,x), @(t,x) this.AerialEvents(t,x,ypelvstart)}; 
            
            while sim
                %% Phase transition & Integration
                phase =  this.phases{phasenum}; %Get name of phase corresponding to phasenum
                if strcmp(phase,'Aerial')
                    if x0(6)<=0 %if pelvis velocity is 0 or negative
                        break;
                    end
                    x0([3 4]) = xpre([3 4]);
                    x0([7 8]) = [0 0];
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
            xf = allx(end,:); tf = allt(end);
            xf([3 4]) = xpre([3 4]);
            tair = allt(find(phasevec==2,1));
            
            if isempty(tair)
                tair = tf;
            end
            
            %Switch legs
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
        
        function  [value, isTerminal, direction] = Fall(this,t,state)
            ss = SLIPState(state);
            fell = ss.pelvis.y;
            value=fell;
            isTerminal=1;
            direction=0;
        end
        
        function [value, isTerminal, direction]  = StanceEvents(this,t,state)
            %Leg reaches full extention
            ss = SLIPState(state);
            value = ss.stancefoot.Length - this.stancel;
            direction = 1;
            if t>0.05
                isTerminal = 1;
            else
                isTerminal = 0;
            end
  
            [value(2,1),isTerminal(2,1),direction(2,1)] = this.Fall(t,state);
        end

        function [value, isTerminal, direction]  = AerialEvents(this,t,state,ystop)
            ss = SLIPState(state);
            value = ss.pelvis.y - ystop;     
            direction = -1;
            isTerminal = 1;
            
            [value(2,1),isTerminal(2,1),direction(2,1)] = this.Fall(t,state);
        end
        
        function [newstate,this] = GoodInitialConditions(this,x0,varargin)

            this.getQandUdefs(x0);
            
            newstate = x0;
            
                %Shift body (without changing configuration) so that heel is on ground
                points = this.getPoints(newstate);
                pelvx = points.pelvis(1) - points.stancefoot(1);
                pelvy = points.pelvis(2) - points.stancefoot(2);
                newstate([1 2])=[pelvx pelvy];
                
                ss = SLIPState(newstate);
                ang = ss.stancefoot.Angle;
                length = ss.stancefoot.Length;
                vpelx = ss.pelvis.xDot;
                vpely = ss.pelvis.yDot;
                
                %Conserve Velocity of Pelvis; determine required leg
                %velocities
                ss.stancefoot.AngleDot = (sin(ang)*vpelx - cos(ang)*vpely)/length;
                ss.stancefoot.LengthDot = -cos(ang)*(vpelx + tan(ang)*vpely);
                
                %Get consistent velocities w/ constraints
%                 Jc = this.getConstraints(newstate,'Stance');
%                 nullJc = null(Jc);
%                 unew = nullJc * (nullJc \ newstate(this.N/2+1:end)');
%                 newstate = [newstate(1:this.N/2) unew'];

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
            
            %Draw Masses
            plotter.plotMass(points.pelvis);
            plotter.plotMass(points.stancefoot,'scaling',0);
            
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
        
        function [MM,rhs] = getMMandRHS(this,time,state)
            
            this.getParams();
            this.getQandUdefs(state);
            
            c3 = cos(q3); s3 = sin(q3);
            
            MM = zeros(4,4); rhs = zeros(4,1);
            
            % Mass Matrix
            MM(1,1) = mfoot + mpelvis; MM(1,2) = 0; MM(1,3) = -(q4*s3*mfoot); MM(1,4) = ...
                c3*mfoot;
            MM(2,1) = MM(1,2); MM(2,2) = mfoot + mpelvis; MM(2,3) = c3*q4*mfoot; MM(2,4) ...
                = s3*mfoot;
            MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mfoot*(q4*q4); MM(3,4) = 0;
            MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot;
            
            % righthand side terms
            rhs(1) = 2*s3*u3*u4*mfoot + c3*q4*mfoot*(u3*u3) + g*(mfoot + ...
                mpelvis)*sin(gslope);
            rhs(2) = -2*c3*u3*u4*mfoot - g*(mfoot + mpelvis)*cos(gslope) + ...
                q4*s3*mfoot*(u3*u3);
            rhs(3) = -(q4*mfoot*(2*u3*u4 + g*cos(q3 - gslope)));
            rhs(4) = kstance*stancel + q4*(-kstance + mfoot*(u3*u3)) - g*mfoot*sin(q3 - ...
                gslope);
            
        end
    
    
    function [C, CDot] = getConstraints(this,state,phase)
        %%
        this.getQandUdefs(state);
        
       c3 = cos(q3); s3 = sin(q3); 
        
        switch phase
            case {'Stance'}
                constraintJacobianStance(1,1) = 1; constraintJacobianStance(1,2) = 0; ...
constraintJacobianStance(1,3) = -(q4*s3); constraintJacobianStance(1,4) = c3; 
constraintJacobianStance(2,1) = 0; constraintJacobianStance(2,2) = 1; ...
constraintJacobianStance(2,3) = c3*q4; constraintJacobianStance(2,4) = s3; 


constraintJacobianStanceDot(1,1) = 0; constraintJacobianStanceDot(1,2) = 0; ...
constraintJacobianStanceDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianStanceDot(1,4) = -(s3*u3); 
constraintJacobianStanceDot(2,1) = 0; constraintJacobianStanceDot(2,2) = 0; ...
constraintJacobianStanceDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianStanceDot(2,4) = c3*u3; 
                
                
                
                C = constraintJacobianStance;
                CDot = constraintJacobianStanceDot;
                
            case 'Aerial'
                C = [0 0 1 0;0 0 0 1];
                CDot = [0 0 0 0;0 0 0 0];
                
            otherwise
                error('Unknown phase for running model: %s', phase);
                
                
        end
        
    end
    
    
    function [E] = getEnergies(this,state)
        this.getParams();
        this.getQandUdefs(state);
        
        kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;
        
        potentialEnergy = (kstance*((-q4 + stancel)*(-q4 + stancel)))/2. + ...
            g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));
        
        PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));
        
        PEspring = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;
        
        E.KE = kineticEnergy;
        E.PE = potentialEnergy;
        E.PEgrav = PEgrav;
        E.PEspring = PEspring;
        E.Total = E.KE + E.PE;
        
    end
    
        function [points] = getPoints(this, state)
            
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); s3 = sin(q3); 
            
points.stancefoot(1) = q1 + c3*q4; 
points.stancefoot(2) = q2 + q4*s3; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 

            
        end
        
        function [vels] = getVels(this, state)
            
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); s3 = sin(q3); 
            
           vels.stancefoot(1) = u1 - q4*s3*u3 + c3*u4; 
vels.stancefoot(2) = u2 + c3*q4*u3 + s3*u4; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 
        end
        
        
        function [comWR] = getcomWR(this,state,phase)
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); s3 = sin(q3);
            
            GRF = this.getGRF(0,state,phase);
            vels = this.getVels(state);
            comWR = dot(GRF,vels.COM);
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
        
        function stancePower = getStancePower(this,x)
            force = this.getStanceForce(x);
            velocity = x(8);
            stancePower = force.*velocity;
        end
        
        function stanceforce = getStanceForce(this,x)
                kstance = this.kstance;
                stancel = this.stancel;
                
            stanceforce = -kstance*(x(4)-this.stancel);
        end
  
        %% Other Gait Information
        
        function [] = print(this,varargin)
           this.printStepCharacteristics(varargin{:}); 
        end
        
        function error = limError(this,x0,runcharic)
            [xf,tf,allx,allt,tair,this,phasevec] = this.onestep(x0);
            error = xf(this.statestomeasure) - x0(this.statestomeasure)';
            
            if ~isempty(runcharic.speed)
                [speed] = getSpeed(this, x0, xf, tf);
                error = [error; runcharic.speed - speed];
            end
            if ~isempty(runcharic.steplength)
                [steplength] = getStepLength(this, x0, xf);
                error = [error; runcharic.steplength - steplength];
            end
            if ~isempty(runcharic.airfrac)
                [airfrac] = getAerialFraction(this, x0, tf, tair);
                error = [error; runcharic.airfrac - airfrac];
            end
        end
        
        function [speed] = getSpeed(this, x0, xf, tf)
            if isempty(xf)
                [xf,tf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = SLIPState(x0);
            xfstruc = SLIPState(xf);
            
            speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
        end
        
        function [steplength] = getStepLength(this, x0, xf)
            if isempty(xf)
                [xf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = SLIPState(x0);
            xfstruc = SLIPState(xf);
            
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

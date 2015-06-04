classdef RetractSLIP < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
    
    properties
        mpelvis = 1;
        gslope = 0;
        g = 1;
        N = 12;
        statestovary = [3 5 6 9:12];
        statestomeasure = [3:6 9:12];
        %rest lengths
        stancel=1; swingl = .8;
        %rest angles
        hipl = 0;
        toeoffimpulse = 0;
        
        %Springs
        kstance = 12; kswing = 0.01; khip = 0.01;
        %Dampers
        cstance = 0; cswing = 0; chip = 0;
        
        phases = {'Stance' 'Toe' 'Aerial'};
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
            test = 'toeimpulse';
            
            LineWidth=3;
            LineSize=3;
            TextSize=14;
            fontstyle='bold';
            fonttype='Times New Roman';
            
            runner = RetractSLIP;
            

            
            IC = RetractSLIPState;
            switch test
                case 'goodstep'
                    load('./SavedGaits/RetractSLIP1.mat','r','xstar')
                    runner = r;
                    x0 = xstar;
                    
                case 'toeimpulse'
                    load('./SavedGaits/RetractSLIP1.mat','r','xstar')
                    runner = r;
                    x0 = xstar;
                    
                    runner.toeoffimpulse = 1;
                    runner.swingl = 1;
                    runner.kswing = 4;
                    runner.khip = 12;
                    runner.hipl = -1.6;
                    
                    IC = RetractSLIPState(x0);
                    IC.swingfoot.Angle = -2.0;
                    IC.swingfoot.Length = 0.85;
                    IC.swingfoot.AngleDot = 0.5;
                    IC.swingfoot.LengthDot = -1;
                    x0 = IC.getVector(); 
                    
                case 'step'
                    
                    runner.kstance = 12.8734; %12
                    runner.kswing = 60; %0.01
                    runner.khip = 10; %0.01
                    runner.cstance = 0;
                    runner.cswing = 0;
                    runner.chip = 0;
                    runner.gslope = 0;
                    runner.swingl = 1;
                    runner.hipl = -1.9;
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    
                    
                    %
                    IC.pelvis.xDot = 1.0138;
                    IC.pelvis.yDot = -0.1651;
                    
                    IC.stancefoot.AngleDot = -0.8457;
                    IC.stancefoot.LengthDot = -0.5830;
                    
                    IC.swingfoot.Angle = -2.0;
                    IC.swingfoot.Length = 0.8062;
                    IC.swingfoot.AngleDot = 1;
                    IC.swingfoot.LengthDot = -3;
                    x0 = IC.getVector();
                case 'Vertical Fall'
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    IC.swingfoot.Angle = -0.8457;
                    IC.swingfoot.Length = -0.5830;
                    
                    IC.stancefoot.AngleDot = 0;
                    IC.stancefoot.LengthDot = 0;
                    IC.swingfoot.AngleDot = 0;
                    IC.swingfoot.LengthDot = -0;
                    x0 = IC.getVector();
                    
                case 'Switch Set Point'
                    IC.stancefoot.Angle = -1.1287;
                    IC.stancefoot.Length = runner.stancel;
                    IC.swingfoot.Angle = -2.1955;
                    IC.swingfoot.Length = 0.8874;
                    
                    IC.stancefoot.AngleDot = -0.8457;
                    IC.stancefoot.LengthDot = -.5830;
                    IC.swingfoot.AngleDot = -.3376;
                    IC.swingfoot.LengthDot = -1.2144;
                    
                    IC.pelvis.x = -0.4279;
                    IC.pelvis.y = 0.9038;
                    IC.pelvis.xDot = 1.0138;
                    IC.pelvis.yDot = -0.1651;
                    
                    runner.phases = {'Stance' 'Aerial'};
                    runner.swingl = 0.5527;
                    runner.kstance = 12.8;
                    runner.kswing = 13.0636;
                    runner.khip = 1.5983;
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
                    PEspringinertial(i) = energies.PEspringinertial;
                    PEspringmassless(i) = energies.PEspringmassless
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
                KE(i) = energies.KE;
                PE(i) = energies.PE;
                PEgrav(i) = energies.PEgrav;
                PEspringinertial(i) = energies.PEspringinertial;
                PEswing(i) = energies.PEswing;
                PEswing2(i) = energies.PEswing2;
                PEhip(i) = energies.PEhip;
                PEspringmassless(i) = energies.PEspringmassless;
                PEgravmassless(i) = energies.PEgravmassless;
                PEmassless(i) = energies.PEmassless;
                KEmassless(i) = energies.KEmassless;
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
            plot(allt,PEhip)
            hold on
            plot(allt,PEswing)
            plot(allt,PEswing2)
            plot(allt,PEgravmassless)
            plot(allt,KEmassless)
            legend('hip','swing','swing2','grav','KE')
            
            figure
            subplot(211)
            plot(allt,TOTE)
            hold on
            plot(allt,KE)
            plot(allt,PE)
            plot(allt,PEgrav)
            plot(allt,PEspringinertial)
            title('Inertial World')
            legend('Tot','KE','PE','PEgrav','PEspring')
            
            subplot(212)
            plot(allt,PEswing)
            hold on
            plot(allt,PEhip)
            title('Massless World')
            legend('Swing Leg Spring','Hip Spring')
            
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
            [x0,this] = this.GoodInitialConditions(x0,'keeppelvisstates',keeppelvisstates);
            sim = 1;
            phasenum= 1;
            tstart = 0;
            allt = [];
            allx = [];
            phasevec = [];
            %             phaseevents = { @(t,x) this.HeelToeEvents(t,x), @(t,x) this.BottomedEvents(t,x),...
            %                 @(t,x) this.ToeEvents(t,x)  , @(t,x) this.AerialEvents(t,x) };
            if length(this.phases)==3
            phaseevents = { @(t,x) this.StanceEvents(t,x), @(t,x) this.ToeEvents(t,x), @(t,x) this.AerialEvents(t,x)};
            else
              phaseevents = { @(t,x) this.StanceEvents(t,x), @(t,x) this.AerialEvents(t,x)};  
            end
            %
            
            while sim
                %% Phase transition & Integration
                phase =  this.phases{phasenum}; %Get name of phase corresponding to phasenum
                if strcmp(phase,'Toe')
                    if this.toeoffimpulse
%                        x0(9) = (-sin(x0(3))*x0(7) + cos(x0(3))*x0(8))/x0(4)/1;
%                        x0(10) = (cos(x0(3))*x0(7) + sin(x0(3))*x0(8))/1;
                         fv = [x0(7);x0(8)] + x0(4)*x0(9)*[-sin(x0(3));cos(x0(3))];
                       x0(9) = (fv(2)*cos(x0(3))-fv(1)*sin(x0(3))+sin(x0(3))*x0(7)-cos(x0(3))*x0(8)/x0(4));
                       x0(10) = fv(1)*cos(x0(3))+fv(2)*sin(x0(3))-cos(x0(3))*x0(7)-sin(x0(3))*x0(8);
                       vels = this.getVels(x0);
                    end
                    [GRF] = this.getGRF(t,x0,'Toe');
                    if GRF(end)>0 || this.toeoffimpulse
                        phasenum = phasenum+1;
                        continue;
                    end
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
            if length(this.phases)==3
                tair = allt(find(phasevec==3,1));
            else
                tair = allt(find(phasevec==2,1));
            end
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
        
        function  [value, isTerminal, direction] = Fall(this,t,state)
            ss = RetractSLIPState(state);
            fell = ss.pelvis.y;
            value=fell;
            isTerminal=1;
            direction=0;
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
            %Swing Leg Trips
            pts = this.getPoints(state);
            value(2,1) = pts.swingfoot(2);
            direction(2,1) = 0;
            isTerminal(2,1) = 1;
            
            
            [value(3,1),isTerminal(3,1),direction(3,1)] = this.Fall(t,state);
        end
        
        function [value, isTerminal, direction]  = ToeEvents(this,t,state)
            %Leg reaches full extention
            [GRF] = this.getGRF(t,state,'Toe');
%             value = GRF(2);
            value = GRF(end);
            direction = 0;
            isTerminal = 1;
        
%             legforce = this.getStanceForce(state,'Toe');
%             hipforce = this.getHipForce(state);
%             legforcevec = legforce*sin(state(3));
%             hipforcevec = hipforce*cos(state(3))/state(4);
%             resforce = legforcevec + hipforcevec - this.g;
%             value = resforce;
%             direction = 0;
%             isTerminal = 1;
            
            %Swing Leg Trips
            pts = this.getPoints(state);
            value(2,1) = pts.swingfoot(2);
            direction(2,1) = 0;
            isTerminal(2,1) = 1;
            
            
            [value(3,1),isTerminal(3,1),direction(3,1)] = this.Fall(t,state);
        end

        function [value, isTerminal, direction]  = AerialEvents(this,t,state)
            pts = this.getPoints(state);
            value = pts.swingfoot(2);       
            direction = 0;
            isTerminal = 1;
            
                        value(2) = pts.stancefoot(2);
            direction(2) = -1;
            isTerminal(2) = 1;
            
            
            [value(3,1),isTerminal(3,1),direction(3,1)] = this.Fall(t,state);
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
                Jc = this.getConstraints(newstate,'Stance');
                nullJc = null(Jc);
                unew = nullJc * (nullJc \ newstate(7:12)');
                newstate = [newstate(1:6) unew'];
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
            
            plotter.plotSpring(points.swingfoot(1),points.swingfoot(2),...
                points.pelvis(1),points.pelvis(2),...
                numcoils,this.swingl,springwidth) %swing spring
            
            

            stancedir = (points.stancefoot - points.pelvis)/norm(points.stancefoot - points.pelvis);
            swingdir = (points.swingfoot - points.pelvis)/norm(points.swingfoot - points.pelvis);
            stancepoint = points.pelvis + .2 * this.stancel * stancedir;
            swingpoint = points.pelvis + .2 * this.stancel * swingdir;
            
            plotter.plotAngSpring(stancepoint,swingpoint,points.pelvis,2,.05,...
                'Color',[232 40 76]/255) %achilles spring
            
            
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
        
        unew = x(7:12);
        unew(3) = (sin(q3)*u1-cos(q3)*u2)/q4;
        unew(4) = -cos(q3)*(u1+tan(q3)*u2);
        
        xNew = x;
        xNew(7:12) = unew;
        Impulse = [];
        
        if (sum(isnan(xNew)))
            xNew
            phaseToSwitchTo
            error('xNew cannot have any NaNs!')
        end
        
            end
    
    function [xddot, constraintForces] = XDoubleDot(this,time,x,phase)
        %%
        %Phase specifies what equations to use, EG 'aerial' or 'stance'
        state = x;
        this.getParams();
        this.getQandUdefs(state);
        u = x(this.N/2+1:end); %velocity states
        
                    c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
        if strcmp(phase,'Aerial')
            
            kstance = this.kswing;
            cstance = this.cswing;
            stancel = this.swingl;
            
            MM = zeros(6,6); rhs = zeros(6,1);
            % Mass Matrix
            mfoot = 0;
            MM(1,1) = 2*mfoot + mpelvis; MM(1,2) = 0; MM(1,3) = -(q4*s3*mfoot); MM(1,4) = ...
                c3*mfoot; MM(1,5) = -(q6*s5*mfoot); MM(1,6) = c5*mfoot;
            MM(2,1) = MM(1,2); MM(2,2) = 2*mfoot + mpelvis; MM(2,3) = c3*q4*mfoot; ...
                MM(2,4) = s3*mfoot; MM(2,5) = c5*q6*mfoot; MM(2,6) = s5*mfoot;
            mfoot = 1;
            MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mfoot*(q4*q4); MM(3,4) = 0; ...
                MM(3,5) = 0; MM(3,6) = 0;
            MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot; ...
                MM(4,5) = 0; MM(4,6) = 0;
            MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
                MM(5,5) = mfoot*(q6*q6); MM(5,6) = 0;
            MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
                MM(6,5) = MM(5,6); MM(6,6) = mfoot;
            
            % righthand side terms
            mfoot = 0;
            rhs(1) = 2*s3*u3*u4*mfoot + 2*s5*u5*u6*mfoot + c3*q4*mfoot*(u3*u3) + ...
                c5*q6*mfoot*(u5*u5) + 2*g*mfoot*sin(gslope) + g*mpelvis*sin(gslope);
            rhs(2) = -2*c3*u3*u4*mfoot - 2*c5*u5*u6*mfoot - 2*g*mfoot*cos(gslope) - ...
                g*mpelvis*cos(gslope) + q4*s3*mfoot*(u3*u3) + q6*s5*mfoot*(u5*u5);
            mfoot = 1;
            rhs(3) = -(u3*chip) + u5*chip - q3*khip + q5*khip + hipl*khip - ...
                q4*mfoot*(2*u3*u4 + g*cos(q3 - gslope));
            rhs(4) = -(u4*cstance) + kstance*stancel + q4*(-kstance + mfoot*(u3*u3)) - ...
                g*mfoot*sin(q3 - gslope);
            rhs(5) = (u3 - u5)*chip - (-q3 + q5 + hipl)*khip - 2*q6*u5*u6*mfoot - ...
                q6*g*mfoot*cos(q5 - gslope);
            rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + mfoot*(u5*u5)) - ...
                g*mfoot*sin(q5 - gslope);
            
                xddot = [u;MM\rhs];
                constraintForces = [];

            
        elseif strcmp(phase,'Toe')
            kstance = this.kswing;
            cstance = this.cswing;
            stancel = this.swingl;
            
            MM = zeros(6,6); rhs = zeros(6,1);
            % Mass Matrix
            mfoot = 0;
            MM(1,1) = 2*mfoot + mpelvis; MM(1,2) = 0; MM(1,3) = -(q4*s3*mfoot); MM(1,4) = ...
                c3*mfoot; MM(1,5) = -(q6*s5*mfoot); MM(1,6) = c5*mfoot;
            MM(2,1) = MM(1,2); MM(2,2) = 2*mfoot + mpelvis; MM(2,3) = c3*q4*mfoot; ...
                MM(2,4) = s3*mfoot; MM(2,5) = c5*q6*mfoot; MM(2,6) = s5*mfoot;
            mfoot = 1;
            MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mfoot*(q4*q4); MM(3,4) = 0; ...
                MM(3,5) = 0; MM(3,6) = 0;
            MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot; ...
                MM(4,5) = 0; MM(4,6) = 0;
            MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
                MM(5,5) = mfoot*(q6*q6); MM(5,6) = 0;
            MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
                MM(6,5) = MM(5,6); MM(6,6) = mfoot;
            
            % righthand side terms
            mfoot = 0;
            rhs(1) = 2*s3*u3*u4*mfoot + 2*s5*u5*u6*mfoot + c3*q4*mfoot*(u3*u3) + ...
                c5*q6*mfoot*(u5*u5) + 2*g*mfoot*sin(gslope) + g*mpelvis*sin(gslope);
            rhs(2) = -2*c3*u3*u4*mfoot - 2*c5*u5*u6*mfoot - 2*g*mfoot*cos(gslope) - ...
                g*mpelvis*cos(gslope) + q4*s3*mfoot*(u3*u3) + q6*s5*mfoot*(u5*u5);
            mfoot = 1;
            rhs(3) = -(u3*chip) + u5*chip - q3*khip + q5*khip + hipl*khip - ...
                q4*mfoot*(2*u3*u4 + g*cos(q3 - gslope));
            rhs(4) = -(u4*cstance) + kstance*stancel + q4*(-kstance + mfoot*(u3*u3)) - ...
                g*mfoot*sin(q3 - gslope);
            rhs(5) = (u3 - u5)*chip - (-q3 + q5 + hipl)*khip - 2*q6*u5*u6*mfoot - ...
                q6*g*mfoot*cos(q5 - gslope);
            rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + mfoot*(u5*u5)) - ...
                g*mfoot*sin(q5 - gslope);
            
            pelvaccs = [0;-this.g]; %Don't let pelvis be affected by leg springs
            
            [Jc,Jcdot] = this.getConstraints(x,'Toe');
%             Jc = Jc(2,:);
%             Jcdot = Jcdot(2,:);
            [d1,~] = size(Jc);
            MMbig = [MM Jc'; Jc zeros(d1)];
            RHSbig = [rhs;-Jcdot*u];
            
            
            MMbig(1:2,7:8) = 0;
%             MMbig(7:8,1:2) = 0;
            
            AccsAndConstraints = MMbig \ RHSbig;
            xddot = [u;AccsAndConstraints(1:6)];
            constraintForces = AccsAndConstraints(7:8);
%             C2 = MMbig(3:end,1:2);
%             M2 = MMbig(3:end,3:end);
%             AccsAndConstraints = M2 \ (RHSbig(3:end) - C2*pelvaccs);
%             
%             legaccs(1:4,1) = AccsAndConstraints(1:4);
%             xddot = [u;pelvaccs;legaccs];
%             constraintForces = -AccsAndConstraints(5:6);
            
            
        elseif strcmp(phase,'Stance')
%             q3 = q3 + pi;
            
             MM = zeros(6,6); rhs = zeros(6,1);
            % Mass Matrix
            mfoot = 0;
            MM(1,1) = 2*mfoot + mpelvis; MM(1,2) = 0; MM(1,3) = -(q4*s3*mfoot); MM(1,4) = ...
                c3*mfoot; MM(1,5) = -(q6*s5*mfoot); MM(1,6) = c5*mfoot;
            MM(2,1) = MM(1,2); MM(2,2) = 2*mfoot + mpelvis; MM(2,3) = c3*q4*mfoot; ...
                MM(2,4) = s3*mfoot; MM(2,5) = c5*q6*mfoot; MM(2,6) = s5*mfoot;
            MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mfoot*(q4*q4); MM(3,4) = 0; ...
                MM(3,5) = 0; MM(3,6) = 0;
            MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot; ...
                MM(4,5) = 0; MM(4,6) = 0;
            mfoot = 1;
            MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
                MM(5,5) = mfoot*(q6*q6); MM(5,6) = 0;
            MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
                MM(6,5) = MM(5,6); MM(6,6) = mfoot;
            
            % righthand side terms
            mfoot = 0;
            khip = 0;
            rhs(1) = 2*s3*u3*u4*mfoot + 2*s5*u5*u6*mfoot + c3*q4*mfoot*(u3*u3) + ...
                c5*q6*mfoot*(u5*u5) + 2*g*mfoot*sin(gslope) + g*mpelvis*sin(gslope);
            rhs(2) = -2*c3*u3*u4*mfoot - 2*c5*u5*u6*mfoot - 2*g*mfoot*cos(gslope) - ...
                g*mpelvis*cos(gslope) + q4*s3*mfoot*(u3*u3) + q6*s5*mfoot*(u5*u5);
            rhs(3) = -(u3*chip) + u5*chip - q3*khip + q5*khip + hipl*khip - ...
                q4*mfoot*(2*u3*u4 + g*cos(q3 - gslope));
            rhs(4) = -(u4*cstance) + kstance*stancel + q4*(-kstance + mfoot*(u3*u3)) - ...
                g*mfoot*sin(q3 - gslope);
            mfoot = 1;
            khip = this.khip;
            rhs(5) = (u3 - u5)*chip - (-q3 + q5 + hipl)*khip - 2*q6*u5*u6*mfoot - ...
                q6*g*mfoot*cos(q5 - gslope);
            rhs(6) = -(u6*cswing) + kswing*swingl + q6*(-kswing + mfoot*(u5*u5)) - ...
                g*mfoot*sin(q5 - gslope);
            
            [Jc,Jcdot] = this.getConstraints(x,'Stance');
            [d1,~] = size(Jc);
             MMbig = [MM Jc'; Jc zeros(d1)];
             RHSbig = [rhs;-Jcdot*u];
            AccsAndConstraints = MMbig \ RHSbig;
        
        accs = AccsAndConstraints(1:this.N/2);
        
        xddot = [u;accs];
        
       constraintForces = -AccsAndConstraints(this.N/2+1:end);
        end
        
        
    end
    
    
    function [C, CDot] = getConstraints(this,state,phase)
        %%
        this.getQandUdefs(state);
        
        c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
        
        switch phase
            case {'Stance', 'Toe'}
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
        
    end
    
    
    function [E] = getEnergies(this,state,phase)
        this.getParams();
        this.getQandUdefs(state);
        if strcmp(phase,'Aerial')
            kstance = this.kswing;
            stancel = this.swingl;
        end
        c3 = cos(q3); c5 = cos(q5); s3 = sin(q3); s5 = sin(q5);
        
        kineticEnergy = (mpelvis*(u1*u1 + u2*u2))/2.;
        PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope)));
        PEstance = (kstance*((q4 - stancel)*(q4 - stancel)))/2.;
        if strcmp(phase,'Aerial') || strcmp(phase,'Toe')
            PEspringinertial = 0;
        else
            PEspringinertial = PEstance;
        end
        
        
        [vels] = this.getVels(state);
        [pts] = this.getPoints(state);
        
        KEmassless = 1/2*norm(vels.stancefoot)^2 + 1/2*norm(vels.swingfoot)^2;
        PEgravmassless = this.g*pts.stancefoot(2)*cos(gslope) + this.g*pts.swingfoot(2)*cos(gslope) + ...
            g*pts.stancefoot(1)*sin(gslope) + g*pts.swingfoot(1)*sin(gslope);
        
        
        PEswing = (kswing*((q6 - swingl)*(q6 - swingl)))/2.;
        PEswing2 = (kswing*((q4 - swingl)*(q4 - swingl)))/2.;
        PEhip = (khip*((q3 - q5 - hipl)*(q3 - q5 - hipl)))/2.;
        PEspringmassless = PEswing+PEhip+PEswing2;
        PEmassless = PEspringmassless + PEgravmassless;
        
        E.KE = kineticEnergy;
        E.KEmassless = KEmassless;
        E.PEmassless = PEmassless;
        E.PE = PEgrav + PEspringinertial;
        E.PEgrav = PEgrav;
        E.PEspringinertial = PEspringinertial;
        E.PEspring = E.PEspringinertial;
        E.PEspringmassless = PEspringmassless;
        E.PEgravmassless = PEgravmassless;
        E.PEswing = PEswing;
        E.PEswing2 = PEswing2;
        E.PEhip = PEhip;
        E.PEstance = PEstance;
        E.Total = E.KE + E.PE;
        
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

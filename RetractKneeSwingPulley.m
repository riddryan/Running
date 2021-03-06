classdef RetractKneeSwingPulley < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
    
    properties
        gslope = 0;
        g = 1;
        N = 6;
        statestovary = [];
        statestomeasure = [];
        
        mpelvis = 1;
        mfoot = 1; %as a ratio of mass relative to knee mass of 1
        mpulley=0.01;
        
        %Segment Lengths
        lthigh = 0.6;
        lshank = 0.4;
        lpulley = 0.15;
        
        %Springs
        kknee = 15; khip = 0.92;
        
        %rest lengths
        kneel = .45;
        hipl = 0.05;
        
        
        SLIPdata = [];
        SLIPx0 = [];
        SLIPxf = [];
        
        usefloorconstraint = 0;
        sephips = 1;
        kneelock = 1;
        unlockpulley = 0;
        
        phases = {'PulleyLock' 'KneeLock'};
    end
    
    
    methods (Static)
        
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            savepath = [dir '\Animations\'];
            onephasesim = 0;
            manystep = 0;
            test = 'kneelock';
            aviname = [test '.avi'];
            
            LineWidth=3;
            LineSize=3;
            TextSize=14;
            fontstyle='bold';
            fonttype='Times New Roman';
            
            runner = RetractKneeSwingPulley;
            
%             SLIPfname = './SavedGaits/RetractKneeSwingPulley/SLIPNominal.mat';
            SLIPfname = './SavedGaits/SLIP/SLIP_NoAerial_unmatchedSL.mat';
            [ runner.SLIPdata, runner.SLIPx0, runner.SLIPxf ] = getSLIPdata( SLIPfname );
            
            
            IC = RetractKneeSwingPulleyState;
            switch test
                               
                case 'kneelock'
                    runner.mfoot = 0.1;
                    runner.mpulley = 2;
                    
                    runner.kknee = 4; %0.01
                    runner.khip = 10; %0.01
                    
                    runner.gslope = 0;
                    runner.kneel = 0.7;
                    runner.hipl = -1.1;
                    
                    IC.foot.Angle = runner.SLIPx0(1);
                    IC.knee.Angle = runner.SLIPx0(1);
                    
                    IC.foot.AngleDot = 0;
                    IC.knee.AngleDot = 0;
                    
                    IC.pulley.Angle = 0;
                    
                    x0 = IC.getVector();
                    
                    
                case 'first'
                    
                            runner.kknee = 1; %0.01
        runner.khip = 8; %0.01
        
        runner.gslope = 0;
        runner.kneel = 0.25;
        runner.hipl = -1.05;
        
        IC.foot.Angle = runner.SLIPx0(1);
        IC.knee.Angle = runner.SLIPx0(1);
        
        IC.foot.AngleDot = 0;
        IC.knee.AngleDot = 0;
                otherwise
                    error('Undefined Test Case')
            end
            
            [x0,runner] = runner.GoodInitialConditions(x0);
            
            %Make sure foot starts on ground
            
            %             runner.kstance = 0.3;
            %             runner.footangle = x0(5) - x0(6)+.3;
            
            
            
            %% Simulate a single phase
            if onephasesim
                phase = 'Aerial';
                tspan = [0 1];
                figure
                options = odeset('AbsTol',[],'RelTol',[],'OutputFcn',@odeplot);
                [allt,allx] = ode45(@(t,x) runner.XDoubleDot(t,x,phase),tspan,x0);
                
                
                figure
                runner.anim(allx,'allt',allt,'interleaveAnimationFrameskip',1);
                
                for i = 1:length(allt)
                    energies = runner.getEnergies(allt(i),allx(i,:));
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
                pulleyx(i) = pts.pulley(1);
                pulleyy(i) = pts.pulley(2);
                stancefootx(i) = pts.stancefoot(1);
                stancefooty(i) = pts.stancefoot(2);
                swingfootx(i) = pts.foot(1);
                swingfooty(i) = pts.foot(2);
                vels = runner.getVels(allt(i),allx(i,:));
                footvelx(i) = vels.foot(1);
                footvely(i) = vels.foot(2);
                GRF(i,:) = runner.getGRF(allt(i),allx(i,:),phase);
                knee(i) = allx(i,1)-allx(i,2);
                
                kneedist(i) = runner.getKneeDist(allt(i),allx(i,:));
                
                hippower(i) = runner.getHipPower(allt(i),allx(i,:));
                kneepower(i) = runner.getKneePower(allt(i),allx(i,:));
            end
            
            
            figure
            plot(allt,TOTE)
            hold on
            plot(allt,KE)
            plot(allt,PE)
            plot(allt,PEgrav)
            legend('Tot','KE','PE','PEgrav')
            
            figure
            plot(allt,hippower)
            hold on
            plot(allt,kneepower)
            legend('hip','knee')
            ylabel('power')
      
            

            
            
            
            
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
        function [this] = RetractKneeSwingPulley(input)
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
            if this.unlockpulley
            phaseevents = {@(t,x) this.PulleyLockEvents(t,x) @(t,x) this.PulleyFreeEvents(t,x) @(t,x)this.KneeLockEvents(t,x)};  
            else
               phaseevents = {@(t,x) this.PulleyFreeEvents(t,x) @(t,x)this.KneeLockEvents(t,x)};   
            end
            while sim
                %% Phase transition & Integration
                phase =  this.phases{phasenum}; %Get name of phase corresponding to phasenum

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
                if isempty(ie) || ie == 1
                    sim = 0;
                    break;
                else
                    phasenum = phasenum+1;
                end

            end
            xf = allx(end,:); tf = allt(end); tair=allt(1);
            
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

        function [value, isTerminal, direction]  = PulleyLockEvents(this,t,state)
            %End when the alotted time for the step is complete
            value(1) = this.SLIPdata(end,1) - t;
            direction(1) = 0;
            isTerminal(1) = 1;
            
            value(2) = this.getKneeDist(t,state) - this.kneel;
            direction(2) = 0;
            isTerminal(2) = 1;
            

        end
        
        function [value, isTerminal, direction]  = PulleyFreeEvents(this,t,state)
            %End when the alotted time for the step is complete
            value(1) = this.SLIPdata(end,1) - t;
            direction(1) = 0;
            isTerminal(1) = 1;
            
            if this.kneelock
                value(2)=(state(2))-(state(1));
                value(3) = state(2) + 2*pi - state(1);
                direction(2) = 0;
                direction(3) = 0;
                if state(1)>-pi/2
                    isTerminal(2) = 1;
                    isTerminal(3) = 1;
                else
                    isTerminal(2) = 0;
                    isTerminal(3) = 1;
                end
                
            end
        end
        
        function [value, isTerminal, direction]  = KneeLockEvents(this,t,state)
            %End when the alotted time for the step is complete
            value(1) = this.SLIPdata(end,1) - t;
            direction(1) = 0;
            isTerminal(1) = 1;
            
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
            
            this.getParams();
            points = this.getPoints(time,state);
            
            plotter = RunnerPlotter;
            
            %Draw Ground
            plot([-5 5],[0 0],'k','LineWidth',2)
            
            hold on
            
            %Draw Torso
            if this.sephips
            lpelvis = 0.3;
            torso = points.pelvis + [0 lpelvis];
            plotter.plotLine(points.pelvis,torso)
            end
            
            %Draw Shank
            plotter.plotLine(points.pelvis,points.knee)
            
            %Draw foot
            plotter.plotLine(points.knee,points.foot)
            
            %Draw pulley
            plotter.plotLine(points.pelvis,points.pulley,'Color',[101 158 255]/255)
            
            
            %Draw Springs
            
            %SLIP Spring
            numcoils=3;
            springwidth=.07;
            plotter.plotSpring(points.stancefoot(1),points.stancefoot(2),...
                points.pelvis(1),points.pelvis(2),...
                numcoils,1,springwidth)
            
            %Hip Spring
            if this.khip > 0
                
                if this.sephips
                     hipstretch = (state(1) - this.hipl)/abs(this.hipl);
                kneedir = [cos(state(1)) sin(state(1))];
                torsodir = [0 1];
                swingpoint = points.pelvis + .3 * lthigh * kneedir;
                torsopoint = points.pelvis + 0.7 * lpelvis * torsodir;
                %
                %                 springdir = (torsopoint - swingpoint)/norm(torsopoint - swingpoint);
                %                 perpdir = ([0 1;0 -1]*springdir')';
                %
                %                 plotter.plotAngSpring(torsopoint,swingpoint,points.pelvis,2,.05,...
                %                     'Color',[232 40 76]/255)
                
                plotter.plotCircSpring(torsopoint,swingpoint,0.1,0,2,0.05,hipstretch,'Color',[232 40 76]/255)
                
                else
                kneedir = [cos(state(1)) sin(state(1))];
                [~,~,stanceangle] = getSLIPstates(this.SLIPdata,time);
                hipstretch = (stanceangle - state(1) - this.hipl)/abs(this.hipl);
                stancedir = [cos(stanceangle) sin(stanceangle)];
                swingpoint = points.pelvis + .3 * lthigh * kneedir;
                stancepoint = points.pelvis + 0.3 * lthigh * stancedir;
                
                plotter.plotCircSpring(stancepoint,swingpoint,0.2,0,2,0.05,hipstretch,'Color',[232 40 76]/255)
                end
                
            end
            
            %Knee Spring
            if this.kknee > 0
%                 footdir = [cos(state(2)) sin(state(2))];
%                 pulleypoint = points.pulley;
%                 footpoint = points.knee + .2 * lshank * footdir;
%                 kneestretch = (norm(points.foot - points.pulley) - this.kneel)/abs(this.kneel);
%                 plotter.plotCircSpring(pulleypoint,footpoint,0.05,1,2,0.1,kneestretch,'Color',[150 150 76]/255)
            plotter.plotSpring(points.pulley(1),points.pulley(2),...
                               points.foot(1),points.foot(2),...
                               3,this.kneel,0.07,'Color',[150 150 76]/255)
            end
            
            
            %Draw Masses
            plotter.plotMass(points.pelvis);
            if this.sephips
            plotter.plotMass(torso,'scaling',0)
            end
            plotter.plotMass(points.foot,'scaling',0);
            plotter.plotMass(points.knee,'scaling',0);
            
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
            allt = [];
            
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
                    case 'allt'
                        allt = value;
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
            assignin(ws,'q5',x(3));
            assignin(ws,'u3',x(4));
            assignin(ws,'u4',x(5));
            assignin(ws,'u5',x(6));
            
            
        end
        
        function [xddot, constraintForces] = XDoubleDot(this,time,x,phase)
            %%
            %Phase specifies what equations to use, EG 'aerial' or 'stance'
            this.getParams();
            this.getQandUdefs(x);
            c3 = cos(q3); c4 = cos(q4); c5 = cos(q5); s3 = sin(q3); s4 = sin(q4); s5 = sin(q5); c3m4 = cos(q3 - q4); c3m5 = cos(q3 - q5); c4m5 = cos(q4 - q5); s3m4 = sin(q3 - q4); s3m5 = sin(q3 - q5); s4m5 = sin(q4 - q5); 
            u = x(this.N/2+1:end); %velocity states
            
            [xpacc,ypacc,stanceangle] = getSLIPstates(this.SLIPdata,time);
            
                MM = zeros(3,3); rhs = zeros(3,1);
                if ~this.sephips
                    


                else
                    
MM(1,1) = (1 + mfoot)*(lthigh*lthigh); MM(1,2) = c3m4*lshank*lthigh*mfoot; ...
MM(1,3) = 0; 
MM(2,1) = MM(1,2); MM(2,2) = mfoot*(lshank*lshank); MM(2,3) = 0; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mpulley*(lpulley*lpulley); 


rhs(1) = -(q3*khip) + hipl*khip - s3m5*kknee*lpulley*lthigh + ...
s3m4*kknee*lshank*lthigh - c3m4*lshank*lthigh*mfoot*ypacc - ...
2*g*lthigh*mfoot*cos(q3 - gslope) - s3m4*lshank*lthigh*mfoot*(u4*u4) - (1 + ...
mfoot)*xpacc*(lthigh*lthigh) + ...
s3m5*kknee*kneel*lpulley*lthigh*power(-2*c4m5*lpulley*lshank - ...
2*c3m5*lpulley*lthigh + 2*c3m4*lshank*lthigh + lpulley*lpulley + ...
lshank*lshank + lthigh*lthigh,-0.5) - ...
s3m4*kknee*kneel*lshank*lthigh*power(-2*c4m5*lpulley*lshank - ...
2*c3m5*lpulley*lthigh + 2*c3m4*lshank*lthigh + lpulley*lpulley + ...
lshank*lshank + lthigh*lthigh,-0.5); 
rhs(2) = -(c3m4*lshank*lthigh*mfoot*xpacc) - mfoot*ypacc*(lshank*lshank) + ...
lshank*(-(g*mfoot*cos(q4 - gslope)) + s3m4*lthigh*mfoot*(u3*u3) - ...
kknee*(s4m5*lpulley + s3m4*lthigh)*power(-2*c4m5*lpulley*lshank - ...
2*c3m5*lpulley*lthigh + 2*c3m4*lshank*lthigh + lpulley*lpulley + ...
lshank*lshank + lthigh*lthigh,-0.5)*(-kneel + power(-2*c4m5*lpulley*lshank - ...
2*c3m5*lpulley*lthigh + 2*c3m4*lshank*lthigh + lpulley*lpulley + ...
lshank*lshank + lthigh*lthigh,0.5))); 
rhs(3) = lpulley*(-(g*mpulley*cos(q5 - gslope)) + kknee*(s4m5*lshank + ...
s3m5*lthigh)*power(-2*c4m5*lpulley*lshank - 2*c3m5*lpulley*lthigh + ...
2*c3m4*lshank*lthigh + lpulley*lpulley + lshank*lshank + ...
lthigh*lthigh,-0.5)*(-kneel + power(-2*c4m5*lpulley*lshank - ...
2*c3m5*lpulley*lthigh + 2*c3m4*lshank*lthigh + lpulley*lpulley + ...
lshank*lshank + lthigh*lthigh,0.5))); 
                    
                end
                
                [Jc,Jcdot] = this.getConstraints(x,phase);
                
                [d1,~] = size(Jc);
                u = x(this.N/2+1:end); %velocity states
                
                MMbig = [MM Jc'; Jc zeros(d1)];
                
                if numel(Jcdot)>0
                    RHSbig = [rhs;-Jcdot*u];
                else
                    RHSbig = rhs;
                end
                
                
                AccsAndConstraints = MMbig \ RHSbig;
                
                accs = AccsAndConstraints(1:this.N/2);
                
                xddot = [u;accs];
                
                constraintForces = -AccsAndConstraints(this.N/2+1:end);
                
                if strcmp(phase,'KneeLock')
                    %To lock the knee, need a constraint force equal and
                    %opposite to the unconstrained acceleration of the knee
                    %angle + an acceleration equal to the hip acceleration
                    constraintForces = -xddot(4)+xddot(3)*this.mfoot;
                    %Enforce velocity & acceleration constraint
                    xddot([2 4]) = xddot([1 3]);
                end
                
            
            
        end
        
        function [C,CDot] = getConstraints(this,x,phase)
            if strcmp(phase,'PulleyLock')
                CPulleyLock(1,1) = 0; CPulleyLock(1,2) = 0; CPulleyLock(1,3) = 1;
                CPulleyLockDot(1,1) = 0; CPulleyLockDot(1,2) = 0; CPulleyLockDot(1,3) = 0;
                
                C = CPulleyLock; CDot = CPulleyLockDot;
            elseif strcmp(phase,'PulleyFree')
%                 CPulleyFree(1,1) = 0; CPulleyFree(1,2) = -1; CPulleyFree(1,3) = 1;
%                 CPulleyFreeDot(1,1) = 0; CPulleyFreeDot(1,2) = 0; CPulleyFreeDot(1,3) = 0;
%                 
%                 C = CPulleyFree; CDot = CPulleyFreeDot;
                C=[];
                CDot=[];
            elseif strcmp(phase,'KneeLock')
%                 CKneeLock(1,1) = 0; CKneeLock(1,2) = -1; CKneeLock(1,3) = 1;
%                 CKneeLock(2,1) = -1; CKneeLock(2,2) = 1; CKneeLock(2,3) = 0;
%                 
%                 CKneeLockDot(1,1) = 0; CKneeLockDot(1,2) = 0; CKneeLockDot(1,3) = 0;
%                 CKneeLockDot(2,1) = 0; CKneeLockDot(2,2) = 0; CKneeLockDot(2,3) = 0;

%                 C = CKneeLock; CDot = CKneeLockDot;

 C=[-1 1 0]; CDot =[0 0 0];
if ~this.unlockpulley
   C = [C;0 0 1]; CDot = [CDot;0 0 0]; 
end
    
            end
                
                
        end
        
    
        function [E] = getEnergies(this,time,state)
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); c4 = cos(q4); c5 = cos(q5); s3 = sin(q3); s4 = sin(q4); s5 = sin(q5); c3m4 = cos(q3 - q4); c3m5 = cos(q3 - q5); c4m5 = cos(q4 - q5); s3m4 = sin(q3 - q4); s3m5 = sin(q3 - q5); s4m5 = sin(q4 - q5); 
            
            [~,~,stanceangle,stancelength,q1,q2,u1,u2,angvel,lengthvel] = getSLIPstates(this.SLIPdata,time);

if this.sephips            
kineticEnergy = (2*c3m4*u3*u4*lshank*lthigh*mfoot + 2*u2*(c4*u4*lshank*mfoot ...
+ c3*u3*lthigh*(1 + mfoot)) - 2*u1*(s4*u4*lshank*mfoot + s3*u3*lthigh*(1 + ...
mfoot)) + (1 + mfoot)*(u1*u1) + (1 + mfoot)*(u2*u2) + ...
mfoot*(u4*u4)*(lshank*lshank) + u3*u3*(lthigh*lthigh) + ...
mfoot*(u3*u3)*(lthigh*lthigh))/2.;

potentialEnergy = (2*q2*g*cos(gslope) + khip*((-q3 + hipl)*(-q3 + hipl)) + ...
kknee*((kneel - power(power(-(c5*lpulley) + c4*lshank + c3*lthigh,2) + ...
power(-(s5*lpulley) + s4*lshank + s3*lthigh,2),0.5))*(kneel - ...
power(power(-(c5*lpulley) + c4*lshank + c3*lthigh,2) + power(-(s5*lpulley) + ...
s4*lshank + s3*lthigh,2),0.5))) + 2*g*lthigh*sin(q3 - gslope) - ...
2*q1*g*sin(gslope) - 2*g*mfoot*(-(q2*cos(gslope)) - lthigh*sin(q3 - gslope) - ...
lshank*sin(q4 - gslope) + q1*sin(gslope)))/2.;

PEgrav = q2*g*cos(gslope) + g*lthigh*sin(q3 - gslope) - q1*g*sin(gslope) - ...
mfoot*(-(q2*g*cos(gslope)) - s4*g*lshank*cos(gslope) - g*lthigh*sin(q3 - ...
gslope) + q1*g*sin(gslope) + c4*g*lshank*sin(gslope));

PEspring = (khip*((q3 - hipl)*(q3 - hipl)))/2. + (kknee*((-kneel + ...
power(power(-(c5*lpulley) + c4*lshank + c3*lthigh,2) + power(-(s5*lpulley) + ...
s4*lshank + s3*lthigh,2),0.5))*(-kneel + power(power(-(c5*lpulley) + ...
c4*lshank + c3*lthigh,2) + power(-(s5*lpulley) + s4*lshank + ...
s3*lthigh,2),0.5))))/2.;
else

end
            
            
            E.KE = kineticEnergy;
            E.PE = PEgrav + PEspring;
            E.PEgrav = PEgrav;
            E.PEspring = PEspring;
            E.Total = E.KE + E.PE;
            
        end
        
        function [points] = getPoints(this, time, state)
            
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); c4 = cos(q4); c5 = cos(q5); s3 = sin(q3); s4 = sin(q4); s5 = sin(q5); c3m4 = cos(q3 - q4); c3m5 = cos(q3 - q5); c4m5 = cos(q4 - q5); s3m4 = sin(q3 - q4); s3m5 = sin(q3 - q5); s4m5 = sin(q4 - q5); 
            
            [~,~,stanceangle,stancelength,q1,q2,u1,u2,angvel,lengthvel] = getSLIPstates(this.SLIPdata,time);
            
points.foot(1) = q1 + c4*lshank + c3*lthigh; 
points.foot(2) = q2 + s4*lshank + s3*lthigh; 


points.knee(1) = q1 + c3*lthigh; 
points.knee(2) = q2 + s3*lthigh; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COM(1) = q1; 
points.COM(2) = q2; 


points.pulley(1) = q1 + c5*lpulley; 
points.pulley(2) = q2 + s5*lpulley; 

            points.stancefoot(1) = q1 + cos(stanceangle)*stancelength;
            points.stancefoot(2) = q2 + sin(stanceangle)*stancelength;
        end
        
        function [vels] = getVels(this,time,state)
            
            this.getParams();
            this.getQandUdefs(state);
            c3 = cos(q3); c4 = cos(q4); c5 = cos(q5); s3 = sin(q3); s4 = sin(q4); s5 = sin(q5); c3m4 = cos(q3 - q4); c3m5 = cos(q3 - q5); c4m5 = cos(q4 - q5); s3m4 = sin(q3 - q4); s3m5 = sin(q3 - q5); s4m5 = sin(q4 - q5); 
            
            [~,~,stanceangle,stancelength,~,~,u1,u2,angvel,lengthvel] = getSLIPstates(this.SLIPdata,time);
            
            
vels.foot(1) = u1 - s4*u4*lshank - s3*u3*lthigh; 
vels.foot(2) = u2 + c4*u4*lshank + c3*u3*lthigh; 


vels.knee(1) = u1 - s3*u3*lthigh; 
vels.knee(2) = u2 + c3*u3*lthigh; 


vels.pelvis(1) = u1; 
vels.pelvis(2) = u2; 


vels.pulley(1) = u1 - s5*u5*lpulley; 
vels.pulley(2) = u2 + c5*u5*lpulley; 


vels.COM(1) = u1; 
vels.COM(2) = u2; 
            
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
                GRF=[0;0];
        end
        
  
        function kneepower = getKneePower(this,time,x)
            force = this.getKneeForce(time,x);
            vels = this.getVels(time,x);
            velocity = vels.foot - vels.pulley;
            kneepower = dot(force,velocity);
        end
        
        function kneeforce = getKneeForce(this,time,x)
            pts = this.getPoints(time,x);
            kneeforce = -this.kknee*(pts.foot - pts.pulley -this.kneel);
        end
        
        function kneedist = getKneeDist(this,time,x)
            pts = this.getPoints(time,x);
            kneedist = norm(pts.foot - pts.pulley);
        end
        
        function hippower= getHipPower(this,time,x)
            sz = RetractKneeSwingPulleyState(x);
[~,~,stanceangle,~,~,~,~,~,angvel,~] = getSLIPstates(this.SLIPdata,time);
            force = this.getHipForce(x,stanceangle);
            if this.sephips
                velocity = sz.knee.AngleDot;
            else
                velocity = angvel - sz.knee.AngleDot;
            end
            hippower = force.*velocity;
        end
        
        function hipforce = getHipForce(this,x,stanceangle)
            if this.sephips
                sz = RetractKneeSwingPulleyState(x);
                hipforce = -this.khip*(sz.knee.Angle - this.hipl);
            else
                sz = RetractKneeSwingPulleyState(x);
                hipforce = -this.khip*(stanceangle - sz.knee.Angle - this.hipl);
            end
        end
        
        %% Other Gait Information
        
        function [c,ceq] = floorconstraint(this,~,~,~,allx,allt,varargin)
            
            fty = zeros(size(allx,1),1);
            for i = 1:size(allx,1)
               pts = this.getPoints(allt(i),allx(i,:));
               fty(i) = pts.foot(2);
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
        
        pts = this.getPoints(tf,xf);
        limitCycleError = pts.foot - pts.pelvis - this.SLIPxf(2)*[cos(this.SLIPxf(1)) sin(this.SLIPxf(1))];
        
        c=[];
        ceq=limitCycleError';

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
        
        pts = this.getPoints(tf,xf);
        limitCycleError = pts.foot - pts.pelvis - this.SLIPxf(2)*[cos(this.SLIPxf(1)) sin(this.SLIPxf(1))];
        
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
            
            [~,~,~,~,xp0,~] = getSLIPstates(this.SLIPdata,0);
            [~,~,~,~,xpf,~] = getSLIPstates(this.SLIPdata,tf);
            
            speed = (xpf-xp0) / tf;
            if isnan(speed)
                speed = 0;
            end
        end
        
        function [steplength] = getStepLength(this, x0, xf, tf)
            if isempty(xf)
                [~,tf] = this.onestep(x0);
            end
            
            [~,~,~,~,xp0,~] = getSLIPstates(this.SLIPdata,0);
            [~,~,~,~,xpf,~] = getSLIPstates(this.SLIPdata,tf);
            
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

classdef SeriesBottomRunner < Runner
    %Generic running model with basic functions to take derivatives,
    %integrate dynamics, implement collisions, and more.  You need
    %subclasses with more specific information about a particular model to
    %use this class.
    
    properties
        %Mass
        mpelvis = .83; msoftstomach = .01;  mfoot=0; msoftseries=.01; msoftparallel=.15;
        
        %Leg and soft stomach stuff
        
        cleg=0.3; csoftstomach=0.3; csoftseries=0.3; csoftparallel=2;
        lleg=1; lsoftstomach=.2; lsoftseries=.4; lsoftparallel=.4;
        kleg=35; ksoftstomach=15; ksoftseries = 39; ksoftparallel=18;
        
        bottomthresh = 1.25;
        unbottomthresh = 0.15;
        
        phases={'unbottomed', 'bottomed','unlocked','aerial'};
        
        gslope = 0;
        g = 1;
        
        %State Stuff
        N=14;
        statestovary = [1 3 ...
            8 9 10 13 14];
        statestomeasure = [3 ...
            8 9 10 13 14];
        
    end
    
    methods (Static)
        
        function [] = test()
            %%
            saveAnimation = 0;
            aviname = 'C:\Users\HBCL Student\Documents\Ryan\Model and Structure\dynamicWalking\Branches\riddryan\NewStruct\testanim.avi';
            
            
            runner = SeriesBottomRunner;
            
            IC = SeriesBottomRunnerState;
            
            IC.stanceLeg.Angle = -1.32;
            IC.stanceLeg.AngleDot = -.5;
            
            IC.stanceLeg.Length = runner.lleg;
            IC.stanceLeg.LengthDot = 0;
            
            IC.softstomach.stretch = runner.lsoftstomach;
            IC.softstomach.stretchDot = 0;
            
            IC.softseries.stretch = runner.lsoftseries;
            IC.softseries.stretchDot = -.73;
            
            IC.softparallel.stretch = runner.lsoftparallel;
            IC.softparallel.stretchDot = -1;
            
            x0 = IC.getVector();
            
            runner.ksoftseries = 20;
            runner.ksoftparallel = 10;
            runner.csoftparallel = .6;
            runner.csoftseries = 0;
            runner.cleg = 0;
            runner.kleg = 15;
%             runner.csoftstomach = 0;
%             runner.cleg = 0;
%             
            %Make sure foot starts on ground
            x0 = runner.GoodInitialConditions(x0);
            
            
            %Take a Step
            [xf,tf,allx,allt,tair,~,phasevec] = runner.onestep(x0);
            
            %% Check Energy and constraints
            energies0 = runner.getEnergies(x0);
            energy0 = energies0.Total;
            r=runner;
            for i = 1:length(allx)
                phase = runner.phases{phasevec(i)};
                
                points=runner.getPoints(allx(i,:));
                footx(i)=points.foot(1);
                footy(i)=points.foot(2);
                Jvel = runner.getVelJacob(allx(i,:));
                u = allx(i,runner.N/2+1:runner.N)';
                vels = Jvel*u;
                footvel(i,1:2) = vels(5:6);
                
                energies = runner.getEnergies(allx(i,:));
                tote(i) = energies.Total;
                
                
                
                Jc = runner.getConstraints(allx(i,:),phase);
                
                
                ConstraintError(i,1) = norm(Jc*u);
                
                x = allx(i,:);
                GRF(i,:) = r.getGRF(x,phase);
                comWR(i,1) = r.getcomWR(x,phase);
                stomachPower(i,1) = r.getStomachPower(x);
                seriesPower(i,1) = r.getSeriesPower(x);
                parallelPower(i,1) = r.getParallelPower(x);
                legPower(i,1) = r.getLegPower(x);
                
                
            end
            
            figure
            subplot(211)
            plot(allx(:,2))
            legend('leg displacement')
            subplot(212)
            plot(allx(:,6))
            legend('soft series displacement')
           
            figure
            subplot(311)
            plot(allt,footx)
            hold on
            plot(allt,footy,'r')
            plot(allt,footvel(:,1),'m')
            plot(allt,footvel(:,2),'k')
            title('Foot')
            legend('xpos','ypos','xvel','yvel')
            
            subplot(312)
            plot(allt,tote)
            legend('total energy')
            
            subplot(313)
            plot(allt,ConstraintError)
            title('Constraints Error')
            
            figure
            subplot(211)
            plot(allt,GRF)
            legend('grfX','grfY')
            subplot(212)
            hold on
            plot(allt,comWR,'k')
            plot(allt,stomachPower,'r')
            plot(allt,seriesPower,'b')
            plot(allt,parallelPower,'g')
            plot(allt,stomachPower+parallelPower+seriesPower,'m')
            plot(allt,legPower,'c');
            legend('COM WR','Stomach','Soft Series','Soft Parallel','All Soft','Leg Spring')
            
            [speed] = runner.getSpeed(x0, xf, tf)
            [steplength] = runner.getStepLength(x0, xf)
            [airfrac] = runner.getAerialFraction(x0, tf, tair)
            
            figure
            plot(allt,-runner.ksoftseries*(allx(:,6)-runner.lsoftseries),'r')
            hold on
            plot(allt,-runner.kleg*(allx(:,2)-runner.lleg),'g')
            plot(allt,seriesPower,'b')
            legend('Series Force','Leg Force', 'Series Power')
            
%             figure
%             plot(allt,allx(:,7));
%             hold on
%             plot(allt,allx(:,14),'r')
%             legend('Parallel Pos','Parallel Vel');
            
            
            
            %% Animation
            figure
            if saveAnimation
                obj = VideoWriter(aviname);
                open(obj);
            end
            
            for i = 1 : 2: length(allx)
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
        
        
        function [this] = SeriesBottomRunner(input)
            %%
            this = this@Runner();
            
            if (nargin == 1 && ~isempty(input))
                parmnames = fieldnames('this');
                for i = 1:length(pnames)
                    pstring = sprintf(['this.' parmnames{i} ' = input.%g'],this.(parmnames{i}));
                    eval(pstring);
                end
            end
            
        end
        
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
            this.getQandUdefs(state);
            statestruc = SoftStomachSeriesRunnerState(state);
            
            plotter = RunnerPlotter;
            
            
            
            %Draw Ground
            plot([-5 5],[0 0],'k','LineWidth',2)
            
            hold on
            %Draw Lines
            %       plotter.plotLine(points.foot,points.pelvis);
            %       plotter.plotLine(points.pelvis,points.soft);
            
            %% Draw Springs
            
            %Calc points to draw parallel soft tissue
            springstart=.2;
            dsep = .1;
            legdir = (points.softseries' - points.foot')/norm(points.softseries - points.foot);
            RotNeg90 = [0 1;-1 0];
            perpdir = RotNeg90*legdir;
            
            legoffset = springstart*legdir';
            paroffset = dsep*perpdir' + legoffset;
            
            seriesfoot = points.foot + legoffset;
            seriessoft = points.softseries + legoffset;
            seriespelvis = points.pelvis + legoffset;
            seriesstomach = points.softstomach + legoffset;
            
            parfoot = points.foot + paroffset;
            parpar = points.softparallel + paroffset;
            
            numcoils=3;
            springwidth=.07;
            plotter.plotSpring(seriessoft(1),seriessoft(2),...
                seriespelvis(1),seriespelvis(2),...
                numcoils,this.lleg,springwidth) %leg spring
            
            numcoils=2;
            springwidth=.05;
            softcolor=[66 128 204]/255;
            
            plotter.plotSpring(seriespelvis(1),seriespelvis(2),...
                seriesstomach(1),seriesstomach(2),...
                numcoils,this.lsoftstomach,springwidth,...
                'Color',softcolor) %soft stomach spring
            
            numcoils=2;
            springwidth=.05;
            softcolor=[66 128 204]/255;
            
            plotter.plotSpring(seriesfoot(1),seriesfoot(2),...
                seriessoft(1),seriessoft(2),...
                numcoils,this.lsoftseries,springwidth,...
                'Color',softcolor) %soft series spring
            
            
            numcoils=2;
            springwidth=.05;
            softcolor=[66 128 204]/255;
            
            plotter.plotSpring(parfoot(1),parfoot(2),...
                parpar(1),parpar(2),...
                numcoils,this.lsoftparallel,springwidth,...
                'Color',softcolor) %soft parallel spring
            
            plotter.plotLine(points.foot,seriesfoot);
            plotter.plotLine(seriesfoot,parfoot);
            %%
            minmass = min([this.mpelvis this.msoftstomach this.msoftseries this.msoftparallel]);
            maxmass = max([this.mpelvis this.msoftstomach this.msoftseries this.msoftparallel]);
            massrange = maxmass-minmass;
            
            %Draw Masses
            plotter.plotMass(seriespelvis,'scaling',this.mpelvis-minmass/massrange);
            plotter.plotMass(seriesstomach,'scaling',this.msoftstomach-minmass/massrange);
            plotter.plotMass(seriessoft,'scaling',this.msoftseries-minmass/massrange);
            plotter.plotMass(parpar,'scaling',this.msoftparallel-minmass/massrange);
            %Draw Foot
            plotter.plotFoot(points.foot, statestruc.stanceLeg.Angle);
            axis equal;
            
            
            %Set Axis Limits
            xLims = [points.pelvis(1)]+ [-1 1];
            xlim(xLims);
            
            yLims = [points.pelvis(2)] + [-1.5 .5];
            ylim(yLims);
            
            
            %Draw Wall Lines
            %       linesStart = floor(xLims(1) * 5) / 5;
            %       linesEnd = floor(xLims(2) * 5) / 5;
            %       linesToDraw = linesStart : 0.2 : linesEnd;
            %
            %       for x = linesToDraw
            %           line([x x], [0 yLims(2)], 'color', ones(3,1) * 0.3)
            %       end
            
            
            
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
        
        function [xf,tf,allx,allt,tair,this,phasevec]= onestep(this, x0,varargin)
            %%
            RelTol = 1e-6; %10; %
            AbsTol = 1e-6; %10; %
            tmax = 5; %2; %6;
            dt = 1e-2;
            interleaveAnimation = 0; %1; %
            interleaveAnimationFrameskip = 2;
            
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
                    case 'anim'
                        interleaveAnimation = value;
                end
            end
            
            phases = this.phases;
            
            %%
            this.getQandUdefs(x0);
            
            %Figure out where pelvis is using stance leg kinematics (this takes
            %care of initial conditions that aren't consistenent with stance
            %constraints).  This should ideally already have been done before
            %calling onestep.
            
            x0 = this.GoodInitialConditions(x0);
            
            %% Unbottomed Phase (collision)
            phase = phases{1};
            collisionoptions = odeset('Events', @(t,x) this.BottomOrFall(t,x),'RelTol',RelTol','AbsTol',AbsTol);
            tstart=0;
            [collisionx0,~] = this.phaseTransition(tstart,x0,phase);
            [stancet,stancestates,~,bottomedeventstates] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tstart:dt:tmax,collisionx0,collisionoptions);
            
            bottomedlegdisplacement = bottomedeventstates(1,2);
            phasevec = ones(length(stancet),1);
            allt=stancet; allx=stancestates;
            
            tbottom = stancet(end);
            %% Bottoming out phase
            phase = phases{2};
            bottomingoptions = odeset('Events', @(t,x) this.UnbottomOrFall(t,x,bottomedlegdisplacement),'RelTol',RelTol','AbsTol',AbsTol);
            [bottomingx0,~] = this.phaseTransition(allt(end),allx(end,:),phase);
            [bottomt,bottomstates] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tbottom:dt:tbottom+tmax,bottomingx0,bottomingoptions);
            
            phasevec = [phasevec; 2*ones(length(bottomt),1)];
            tunbottom = bottomt(end);
            
            allt = [allt; bottomt]; allx = [allx;bottomstates];
            
            %% Unbottomed phase (toe off)
            phase = phases{3};
            unbotoptions = odeset('Events', @(t,x) this.LiftOrFall(t,x),'RelTol',RelTol','AbsTol',AbsTol);
            [unbotx0,~] = this.phaseTransition(allt(end),allx(end,:),phase);
            [unbott,unbotstates] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tunbottom:dt:tunbottom+tmax,unbotx0,unbotoptions);
            
            phasevec = [phasevec; 3*ones(length(unbott),1)];
            allt = [allt; unbott]; allx = [allx;unbotstates];
            tair = unbott(end);
            %% Aerial Phase
            phase = phases{4};
            [airx0,~] = this.phaseTransition(allt(end),allx(end,:),phase);
            
            %Set Swing leg angle and length
            landangle = x0(1);
            airx0(1)=landangle;
            airx0(2)=this.lleg;
            airx0(6)=this.lsoftseries;
            airx0(7)=this.lsoftparallel;
            airx0(8)=0;
            airx0(9)=0;
            airx0(13)=0;
            airx0(14)=0;
            
            airoptions = odeset('Events', @(t,x) this.HeelStrikeOrFall(t,x,landangle),'RelTol',RelTol','AbsTol',AbsTol);
            
            [airt,airstates] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tair:dt:tair+tmax,airx0,airoptions);
            
            tf = airt(end);
            allt = [allt; airt]; allx = [allx;airstates];
            phasevec = [phasevec; 4*ones(length(airt),1)];
            %% Switch States for Next Step & Plotting
            
            %Make sure states are consistent with heel strike
            xf = switchLegs(this,allx(end,:),landangle);
            
            %Calculate states after collision
            if sum(isnan(xf))
               bad=1; 
            end
            [xf,impulses] = this.phaseTransition(tf,xf,'unbottomed');
            
            if (interleaveAnimation)
                for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
                    cla;
                    thisState = allx(i, :);
                    %               points = this.getPoints(thisState);
                    this.plot(thisState);
                    pause(0.01);
                end
            end
            
        end
        
        function [value, isTerminal, direction] = BottomOrFall(this,t,state)
            points = this.getPoints(state);
            
                        %Series spring bottoms out 
%             value1 = state(13); %soft series spring velocity



%             bottomtol = 5e-2;
%             currseriespower = this.getSeriesPower(state);
%             dt = 1e-1;
%             accs = this.XDoubleDot(t+dt,state,'unbottomed');
%             nextstate = [state(1:7) + state(8:14)*dt + 1/2*accs(8:14)*dt^2 ; state(8:14) + accs(8:14)*dt];
%             nextseriespower = this.getSeriesPower(nextstate);
%             
%             value1 = nextseriespower - currseriespower + bottomtol;

            forcethresh = this.bottomthresh;
            value1 = -this.ksoftseries*(state(6) - this.lsoftseries) - this.csoftseries*(state(13)) - forcethresh; %bottom out when reaching a force threshold

            isTerminal1 = 1;
            direction1=1;
            
            
            %Falling
            fell = points.COMpos(2);
            
            value2=fell;
            isTerminal2=1;
            direction2=0;
            
            value = [value1;value2]; isTerminal = [isTerminal1; isTerminal2]; direction = [direction1; direction2];
            
            
        end
        
        
        function [value, isTerminal, direction] = UnbottomOrFall(this,t,state,bottomedlegdisplacement)
            points = this.getPoints(state);
            
            %When leg displacement reaches same level it was at when the
            %series spring bottomed out
%             value1 = bottomedlegdisplacement - state(2);
             forcethresh = this.unbottomthresh;
             value1 = -this.kleg*(state(2) - this.lleg) - this.cleg*(state(9)) - forcethresh;

            %When leg reaches neutral, unbottom series spring
%             value1 = state(2) - this.lleg;
            isTerminal1 = 1;
            direction1 = -1;
            
            
            %Falling
            fell = points.COMpos(2);
            
            value2=fell;
            isTerminal2=1;
            direction2=0;
            
            value = [value1;value2]; isTerminal = [isTerminal1; isTerminal2]; direction = [direction1; direction2];
            
            
        end
        
        function [value, isTerminal, direction] = LiftOrFall(this,t,state)
            points = this.getPoints(state);
            
            %Lifting Off the Ground
            phase = 'unbottomed';
            [~,lambdas]=this.XDoubleDot(t,state,phase);
            GRFvert = lambdas(2);
            
            value1 = GRFvert;
            isTerminal1 = 1;
            direction1=-1;
            
            
            %Falling
            fell = points.COMpos(2);
            
            value2=fell;
            isTerminal2=1;
            direction2=0;
            
            value = [value1;value2]; isTerminal = [isTerminal1; isTerminal2]; direction = [direction1; direction2];
            
            
        end
        
        function [value, isTerminal, direction] = HeelStrikeOrFall(this,t,state,landangle)
            points = this.getPoints(state);
            
            %Lifting Off the Ground
            swingfoot = points.foot(2);
            
            value1 = swingfoot;
            isTerminal1 = 1;
            direction1=-1;
            
            
            %Falling
            fell = points.COMpos(2);
            
            value2=fell;
            isTerminal2=1;
            direction2=0;
            
            value = [value1;value2]; isTerminal = [isTerminal1; isTerminal2]; direction = [direction1; direction2];
            
            
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
        
        function [xnew] = switchLegs(this,x,landangle)
            %Switch the leg angle at collision to be the touchdown angle, and
            %the length of the leg spring to be at rest.
            
            newstatestruc = SeriesBottomRunnerState(x);
            newstatestruc.stanceLeg.Angle = landangle;
            newstatestruc.stanceLeg.Length = this.lleg;
            newstatestruc.softseries.stretch = this.lsoftseries;
            newstatestruc.softparallel.stretch = this.lsoftparallel;
            
            xnew = getVector(newstatestruc);
            
        end
        
        function [speed] = getSpeed(this, x0, xf, tf)
            if isempty(xf)
                [xf,tf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = SeriesBottomRunnerState(x0);
            xfstruc = SeriesBottomRunnerState(xf);
            
            speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
        end
        
        function [steplength] = getStepLength(this, x0, xf)
            if isempty(xf)
                [xf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = SeriesBottomRunnerState(x0);
            xfstruc = SeriesBottomRunnerState(xf);
            
            steplength = (xfstruc.pelvis.x - x0struc.pelvis.x);
        end
        
        function [airfrac] = getAerialFraction(this, x0, tf, tair)
            if isempty(tair) || isempty(tair)
                [xf,tf,allx,allt,tair] = this.onestep(x0);
            end
            airfrac = (tf-tair)/tf;
        end
        
        function [MM,rhs] = getMMandRHS(this,time,state)
            %%
            this.getParams();
            this.getQandUdefs(state);
            
            c1 = cos(q1); s1 = sin(q1);
            
            MM = zeros(7,7); rhs = zeros(7,1);

% Mass Matrix
MM(1,1) = -2*q6*q7*msoftparallel + 2*q2*(-(q7*msoftparallel) + q6*(mfoot + ...
msoftparallel)) + (mfoot + msoftparallel + msoftseries)*(q2*q2) + (mfoot + ...
msoftparallel)*(q6*q6) + msoftparallel*(q7*q7); MM(1,2) = 0; MM(1,3) = 0; ...
MM(1,4) = -(s1*(-(q7*msoftparallel) + q6*(mfoot + msoftparallel) + q2*(mfoot ...
+ msoftparallel + msoftseries))); MM(1,5) = c1*(-(q7*msoftparallel) + ...
q6*(mfoot + msoftparallel) + q2*(mfoot + msoftparallel + msoftseries)); ...
MM(1,6) = 0; MM(1,7) = 0; 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + msoftparallel + msoftseries; MM(2,3) = ...
0; MM(2,4) = c1*(mfoot + msoftparallel + msoftseries); MM(2,5) = s1*(mfoot + ...
msoftparallel + msoftseries); MM(2,6) = mfoot + msoftparallel; MM(2,7) = ...
-msoftparallel; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = msoftstomach; MM(3,4) = 0; ...
MM(3,5) = msoftstomach; MM(3,6) = 0; MM(3,7) = 0; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot + ...
mpelvis + msoftparallel + msoftseries + msoftstomach; MM(4,5) = 0; MM(4,6) = ...
c1*(mfoot + msoftparallel); MM(4,7) = -(c1*msoftparallel); 
MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
MM(5,5) = mfoot + mpelvis + msoftparallel + msoftseries + msoftstomach; ...
MM(5,6) = s1*(mfoot + msoftparallel); MM(5,7) = -(s1*msoftparallel); 
MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
MM(6,5) = MM(5,6); MM(6,6) = mfoot + msoftparallel; MM(6,7) = -msoftparallel; 
MM(7,1) = MM(1,7); MM(7,2) = MM(2,7); MM(7,3) = MM(3,7); MM(7,4) = MM(4,7); ...
MM(7,5) = MM(5,7); MM(7,6) = MM(6,7); MM(7,7) = msoftparallel; 

% righthand side terms
rhs(1) = q7*msoftparallel*(2*u1*(u2 + u6 - u7) + g*cos(q1 - gslope)) - ...
q6*(2*u1*(-(u7*msoftparallel) + u2*(mfoot + msoftparallel) + u6*(mfoot + ...
msoftparallel)) + g*(mfoot + msoftparallel)*cos(q1 - gslope)) - ...
q2*(2*u1*(-(u7*msoftparallel) + u6*(mfoot + msoftparallel) + u2*(mfoot + ...
msoftparallel + msoftseries)) + g*(mfoot + msoftparallel + ...
msoftseries)*cos(q1 - gslope)); 
rhs(2) = -(u2*cleg) + kleg*(-q2 + lleg) + (-(q7*msoftparallel) + q6*(mfoot + ...
msoftparallel) + q2*(mfoot + msoftparallel + msoftseries))*(u1*u1) - ...
g*mfoot*sin(q1 - gslope) - g*msoftparallel*sin(q1 - gslope) - ...
g*msoftseries*sin(q1 - gslope); 
rhs(3) = -(u3*csoftstomach) - q3*ksoftstomach + ksoftstomach*lsoftstomach - ...
g*msoftstomach*cos(gslope); 
rhs(4) = 2*s1*u1*u2*mfoot + 2*s1*u1*u6*mfoot + 2*s1*u1*u2*msoftparallel + ...
2*s1*u1*u6*msoftparallel - 2*s1*u1*u7*msoftparallel + 2*s1*u1*u2*msoftseries ...
- c1*q7*msoftparallel*(u1*u1) + c1*q6*(mfoot + msoftparallel)*(u1*u1) + ...
c1*q2*(mfoot + msoftparallel + msoftseries)*(u1*u1) + g*mfoot*sin(gslope) + ...
g*mpelvis*sin(gslope) + g*msoftparallel*sin(gslope) + ...
g*msoftseries*sin(gslope) + g*msoftstomach*sin(gslope); 
rhs(5) = -2*c1*u1*u2*mfoot - 2*c1*u1*u6*mfoot - 2*c1*u1*u2*msoftparallel - ...
2*c1*u1*u6*msoftparallel + 2*c1*u1*u7*msoftparallel - 2*c1*u1*u2*msoftseries ...
- g*mfoot*cos(gslope) - g*mpelvis*cos(gslope) - g*msoftparallel*cos(gslope) - ...
g*msoftseries*cos(gslope) - g*msoftstomach*cos(gslope) - ...
q7*s1*msoftparallel*(u1*u1) + q6*s1*(mfoot + msoftparallel)*(u1*u1) + ...
q2*s1*(mfoot + msoftparallel + msoftseries)*(u1*u1); 
rhs(6) = -(u6*csoftseries) + ksoftseries*(-q6 + lsoftseries) + ...
(-(q7*msoftparallel) + q2*(mfoot + msoftparallel) + q6*(mfoot + ...
msoftparallel))*(u1*u1) - g*mfoot*sin(q1 - gslope) - g*msoftparallel*sin(q1 - ...
gslope); 
rhs(7) = -(u7*csoftparallel) + ksoftparallel*(-q7 + lsoftparallel) - (q2 + q6 ...
- q7)*msoftparallel*(u1*u1) + g*msoftparallel*sin(q1 - gslope); 
            
        end
        
        function [C, CDot] = getConstraints(this,state,phase)
            %%
            this.getParams();
            this.getQandUdefs(state);
            
            c1 = cos(q1); s1 = sin(q1);
            
            switch phase
                case 'unbottomed'
constraintJacobianUnbottomed(1,1) = -((q2 + q6)*s1); ...
constraintJacobianUnbottomed(1,2) = c1; constraintJacobianUnbottomed(1,3) = ...
0; constraintJacobianUnbottomed(1,4) = 1; constraintJacobianUnbottomed(1,5) = ...
0; constraintJacobianUnbottomed(1,6) = c1; constraintJacobianUnbottomed(1,7) ...
= 0; 
constraintJacobianUnbottomed(2,1) = c1*(q2 + q6); ...
constraintJacobianUnbottomed(2,2) = s1; constraintJacobianUnbottomed(2,3) = ...
0; constraintJacobianUnbottomed(2,4) = 0; constraintJacobianUnbottomed(2,5) = ...
1; constraintJacobianUnbottomed(2,6) = s1; constraintJacobianUnbottomed(2,7) ...
= 0; 
constraintJacobianUnbottomed(3,1) = 0; constraintJacobianUnbottomed(3,2) = 1; ...
constraintJacobianUnbottomed(3,3) = 0; constraintJacobianUnbottomed(3,4) = 0; ...
constraintJacobianUnbottomed(3,5) = 0; constraintJacobianUnbottomed(3,6) = 0; ...
constraintJacobianUnbottomed(3,7) = 0; 


constraintJacobianUnbottomedDot(1,1) = -(c1*(q2 + q6)*u1) - s1*(u2 + u6); ...
constraintJacobianUnbottomedDot(1,2) = -(s1*u1); ...
constraintJacobianUnbottomedDot(1,3) = 0; ...
constraintJacobianUnbottomedDot(1,4) = 0; ...
constraintJacobianUnbottomedDot(1,5) = 0; ...
constraintJacobianUnbottomedDot(1,6) = -(s1*u1); ...
constraintJacobianUnbottomedDot(1,7) = 0; 
constraintJacobianUnbottomedDot(2,1) = -((q2 + q6)*s1*u1) + c1*(u2 + u6); ...
constraintJacobianUnbottomedDot(2,2) = c1*u1; ...
constraintJacobianUnbottomedDot(2,3) = 0; ...
constraintJacobianUnbottomedDot(2,4) = 0; ...
constraintJacobianUnbottomedDot(2,5) = 0; ...
constraintJacobianUnbottomedDot(2,6) = c1*u1; ...
constraintJacobianUnbottomedDot(2,7) = 0; 
constraintJacobianUnbottomedDot(3,1) = 0; ...
constraintJacobianUnbottomedDot(3,2) = 0; ...
constraintJacobianUnbottomedDot(3,3) = 0; ...
constraintJacobianUnbottomedDot(3,4) = 0; ...
constraintJacobianUnbottomedDot(3,5) = 0; ...
constraintJacobianUnbottomedDot(3,6) = 0; ...
constraintJacobianUnbottomedDot(3,7) = 0; 

                    C = constraintJacobianUnbottomed;
                    CDot = constraintJacobianUnbottomedDot;
                    
                case 'bottomed'
                    constraintJacobianBottomed(1,1) = -((q2 + q6)*s1); ...
constraintJacobianBottomed(1,2) = c1; constraintJacobianBottomed(1,3) = 0; ...
constraintJacobianBottomed(1,4) = 1; constraintJacobianBottomed(1,5) = 0; ...
constraintJacobianBottomed(1,6) = c1; constraintJacobianBottomed(1,7) = 0; 
constraintJacobianBottomed(2,1) = c1*(q2 + q6); ...
constraintJacobianBottomed(2,2) = s1; constraintJacobianBottomed(2,3) = 0; ...
constraintJacobianBottomed(2,4) = 0; constraintJacobianBottomed(2,5) = 1; ...
constraintJacobianBottomed(2,6) = s1; constraintJacobianBottomed(2,7) = 0; 
constraintJacobianBottomed(3,1) = 0; constraintJacobianBottomed(3,2) = 0; ...
constraintJacobianBottomed(3,3) = 0; constraintJacobianBottomed(3,4) = 0; ...
constraintJacobianBottomed(3,5) = 0; constraintJacobianBottomed(3,6) = 1; ...
constraintJacobianBottomed(3,7) = 0; 


constraintJacobianBottomedDot(1,1) = -(c1*(q2 + q6)*u1) - s1*(u2 + u6); ...
constraintJacobianBottomedDot(1,2) = -(s1*u1); ...
constraintJacobianBottomedDot(1,3) = 0; constraintJacobianBottomedDot(1,4) = ...
0; constraintJacobianBottomedDot(1,5) = 0; constraintJacobianBottomedDot(1,6) ...
= -(s1*u1); constraintJacobianBottomedDot(1,7) = 0; 
constraintJacobianBottomedDot(2,1) = -((q2 + q6)*s1*u1) + c1*(u2 + u6); ...
constraintJacobianBottomedDot(2,2) = c1*u1; ...
constraintJacobianBottomedDot(2,3) = 0; constraintJacobianBottomedDot(2,4) = ...
0; constraintJacobianBottomedDot(2,5) = 0; constraintJacobianBottomedDot(2,6) ...
= c1*u1; constraintJacobianBottomedDot(2,7) = 0; 
constraintJacobianBottomedDot(3,1) = 0; constraintJacobianBottomedDot(3,2) = ...
0; constraintJacobianBottomedDot(3,3) = 0; constraintJacobianBottomedDot(3,4) ...
= 0; constraintJacobianBottomedDot(3,5) = 0; ...
constraintJacobianBottomedDot(3,6) = 0; constraintJacobianBottomedDot(3,7) = ...
0; 

C = constraintJacobianBottomed; CDot = constraintJacobianBottomedDot;
                case 'unlocked'
                    
                    constraintJacobianUnlocked(1,1) = -((q2 + q6)*s1); ...
constraintJacobianUnlocked(1,2) = c1; constraintJacobianUnlocked(1,3) = 0; ...
constraintJacobianUnlocked(1,4) = 1; constraintJacobianUnlocked(1,5) = 0; ...
constraintJacobianUnlocked(1,6) = c1; constraintJacobianUnlocked(1,7) = 0; 
constraintJacobianUnlocked(2,1) = c1*(q2 + q6); ...
constraintJacobianUnlocked(2,2) = s1; constraintJacobianUnlocked(2,3) = 0; ...
constraintJacobianUnlocked(2,4) = 0; constraintJacobianUnlocked(2,5) = 1; ...
constraintJacobianUnlocked(2,6) = s1; constraintJacobianUnlocked(2,7) = 0; 


constraintJacobianUnlockedDot(1,1) = -(c1*(q2 + q6)*u1) - s1*(u2 + u6); ...
constraintJacobianUnlockedDot(1,2) = -(s1*u1); ...
constraintJacobianUnlockedDot(1,3) = 0; constraintJacobianUnlockedDot(1,4) = ...
0; constraintJacobianUnlockedDot(1,5) = 0; constraintJacobianUnlockedDot(1,6) ...
= -(s1*u1); constraintJacobianUnlockedDot(1,7) = 0; 
constraintJacobianUnlockedDot(2,1) = -((q2 + q6)*s1*u1) + c1*(u2 + u6); ...
constraintJacobianUnlockedDot(2,2) = c1*u1; ...
constraintJacobianUnlockedDot(2,3) = 0; constraintJacobianUnlockedDot(2,4) = ...
0; constraintJacobianUnlockedDot(2,5) = 0; constraintJacobianUnlockedDot(2,6) ...
= c1*u1; constraintJacobianUnlockedDot(2,7) = 0; 

C = constraintJacobianUnlocked; CDot = constraintJacobianUnlockedDot;
                case 'aerial'
constraintJacobianAerial(1,1) = 1; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 0; constraintJacobianAerial(1,4) = 0; ...
constraintJacobianAerial(1,5) = 0; constraintJacobianAerial(1,6) = 0; ...
constraintJacobianAerial(1,7) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 1; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 0; ...
constraintJacobianAerial(2,5) = 0; constraintJacobianAerial(2,6) = 0; ...
constraintJacobianAerial(2,7) = 0; 
constraintJacobianAerial(3,1) = 0; constraintJacobianAerial(3,2) = 0; ...
constraintJacobianAerial(3,3) = 0; constraintJacobianAerial(3,4) = 0; ...
constraintJacobianAerial(3,5) = 0; constraintJacobianAerial(3,6) = 1; ...
constraintJacobianAerial(3,7) = 0; 
constraintJacobianAerial(4,1) = 0; constraintJacobianAerial(4,2) = 0; ...
constraintJacobianAerial(4,3) = 0; constraintJacobianAerial(4,4) = 0; ...
constraintJacobianAerial(4,5) = 0; constraintJacobianAerial(4,6) = 0; ...
constraintJacobianAerial(4,7) = 1; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; ...
constraintJacobianAerialDot(1,5) = 0; constraintJacobianAerialDot(1,6) = 0; ...
constraintJacobianAerialDot(1,7) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; ...
constraintJacobianAerialDot(2,5) = 0; constraintJacobianAerialDot(2,6) = 0; ...
constraintJacobianAerialDot(2,7) = 0; 
constraintJacobianAerialDot(3,1) = 0; constraintJacobianAerialDot(3,2) = 0; ...
constraintJacobianAerialDot(3,3) = 0; constraintJacobianAerialDot(3,4) = 0; ...
constraintJacobianAerialDot(3,5) = 0; constraintJacobianAerialDot(3,6) = 0; ...
constraintJacobianAerialDot(3,7) = 0; 
constraintJacobianAerialDot(4,1) = 0; constraintJacobianAerialDot(4,2) = 0; ...
constraintJacobianAerialDot(4,3) = 0; constraintJacobianAerialDot(4,4) = 0; ...
constraintJacobianAerialDot(4,5) = 0; constraintJacobianAerialDot(4,6) = 0; ...
constraintJacobianAerialDot(4,7) = 0; 
                    
                    C = constraintJacobianAerial;
                    CDot = constraintJacobianAerialDot;
                    
                otherwise
                    error('Unknown phase for running model: %s', phase);
                    
                    
            end
            
        end
        
        function [J] = getVelJacob(this,state)
            this.getQandUdefs(state);
            c1 = cos(q1); s1 = sin(q1);
            
            
velJacobian(1,1) = 0; velJacobian(1,2) = 0; velJacobian(1,3) = 0; ...
velJacobian(1,4) = 1; velJacobian(1,5) = 0; velJacobian(1,6) = 0; ...
velJacobian(1,7) = 0; 
velJacobian(2,1) = 0; velJacobian(2,2) = 0; velJacobian(2,3) = 0; ...
velJacobian(2,4) = 0; velJacobian(2,5) = 1; velJacobian(2,6) = 0; ...
velJacobian(2,7) = 0; 
velJacobian(3,1) = 0; velJacobian(3,2) = 0; velJacobian(3,3) = 0; ...
velJacobian(3,4) = 1; velJacobian(3,5) = 0; velJacobian(3,6) = 0; ...
velJacobian(3,7) = 0; 
velJacobian(4,1) = 0; velJacobian(4,2) = 0; velJacobian(4,3) = 1; ...
velJacobian(4,4) = 0; velJacobian(4,5) = 1; velJacobian(4,6) = 0; ...
velJacobian(4,7) = 0; 
velJacobian(5,1) = -(q2*s1) - q6*s1; velJacobian(5,2) = c1; velJacobian(5,3) ...
= 0; velJacobian(5,4) = 1; velJacobian(5,5) = 0; velJacobian(5,6) = c1; ...
velJacobian(5,7) = 0; 
velJacobian(6,1) = c1*q2 + c1*q6; velJacobian(6,2) = s1; velJacobian(6,3) = ...
0; velJacobian(6,4) = 0; velJacobian(6,5) = 1; velJacobian(6,6) = s1; ...
velJacobian(6,7) = 0; 
velJacobian(7,1) = -(q2*s1); velJacobian(7,2) = c1; velJacobian(7,3) = 0; ...
velJacobian(7,4) = 1; velJacobian(7,5) = 0; velJacobian(7,6) = 0; ...
velJacobian(7,7) = 0; 
velJacobian(8,1) = c1*q2; velJacobian(8,2) = s1; velJacobian(8,3) = 0; ...
velJacobian(8,4) = 0; velJacobian(8,5) = 1; velJacobian(8,6) = 0; ...
velJacobian(8,7) = 0; 
velJacobian(9,1) = -(q2*s1) - q6*s1 + q7*s1; velJacobian(9,2) = c1; ...
velJacobian(9,3) = 0; velJacobian(9,4) = 1; velJacobian(9,5) = 0; ...
velJacobian(9,6) = c1; velJacobian(9,7) = -c1; 
velJacobian(10,1) = c1*q2 + c1*q6 - c1*q7; velJacobian(10,2) = s1; ...
velJacobian(10,3) = 0; velJacobian(10,4) = 0; velJacobian(10,5) = 1; ...
velJacobian(10,6) = s1; velJacobian(10,7) = -s1; 

            
            
            
            J=velJacobian;
        end
        
        function [E] = getEnergies(this,state)
            this.getParams();
            this.getQandUdefs(state);
            c1 = cos(q1); s1 = sin(q1);
            
            kineticEnergy = (2*c1*u2*u4*mfoot + 2*s1*u2*u5*mfoot + 2*u2*u6*mfoot + ...
2*c1*u4*u6*mfoot + 2*s1*u5*u6*mfoot + 2*q7*s1*u1*u4*msoftparallel + ...
2*c1*u2*u4*msoftparallel - 2*c1*q7*u1*u5*msoftparallel + ...
2*s1*u2*u5*msoftparallel + 2*u2*u6*msoftparallel + 2*c1*u4*u6*msoftparallel + ...
2*s1*u5*u6*msoftparallel - 2*u2*u7*msoftparallel - 2*c1*u4*u7*msoftparallel - ...
2*s1*u5*u7*msoftparallel - 2*u6*u7*msoftparallel - ...
2*q6*u1*(q7*u1*msoftparallel + (s1*u4 - c1*u5)*(mfoot + msoftparallel)) + ...
2*c1*u2*u4*msoftseries + 2*s1*u2*u5*msoftseries + ...
2*q2*u1*(-(q7*u1*msoftparallel) + q6*u1*(mfoot + msoftparallel) - (s1*u4 - ...
c1*u5)*(mfoot + msoftparallel + msoftseries)) + 2*u3*u5*msoftstomach + (mfoot ...
+ msoftparallel + msoftseries)*(q2*q2)*(u1*u1) + (mfoot + ...
msoftparallel)*(q6*q6)*(u1*u1) + msoftparallel*(q7*q7)*(u1*u1) + ...
mfoot*(u2*u2) + msoftparallel*(u2*u2) + msoftseries*(u2*u2) + ...
msoftstomach*(u3*u3) + mfoot*(u4*u4) + mpelvis*(u4*u4) + ...
msoftparallel*(u4*u4) + msoftseries*(u4*u4) + msoftstomach*(u4*u4) + ...
mfoot*(u5*u5) + mpelvis*(u5*u5) + msoftparallel*(u5*u5) + msoftseries*(u5*u5) ...
+ msoftstomach*(u5*u5) + mfoot*(u6*u6) + msoftparallel*(u6*u6) + ...
msoftparallel*(u7*u7))/2.;

potentialEnergy = (-2*q7*ksoftparallel*lsoftparallel - ...
2*q6*ksoftseries*lsoftseries + 2*q5*g*mfoot*cos(gslope) + ...
2*q5*g*mpelvis*cos(gslope) + 2*q5*g*msoftparallel*cos(gslope) + ...
2*q5*g*msoftseries*cos(gslope) + 2*q5*g*msoftstomach*cos(gslope) + ...
q3*(-2*ksoftstomach*lsoftstomach + 2*g*msoftstomach*cos(gslope)) + ...
kleg*(q2*q2) + ksoftstomach*(q3*q3) + ksoftseries*(q6*q6) + ...
ksoftparallel*(q7*q7) + kleg*(lleg*lleg) + ...
ksoftparallel*(lsoftparallel*lsoftparallel) + ...
ksoftseries*(lsoftseries*lsoftseries) + ...
ksoftstomach*(lsoftstomach*lsoftstomach) + 2*q6*g*mfoot*sin(q1 - gslope) + ...
2*q6*g*msoftparallel*sin(q1 - gslope) - 2*q7*g*msoftparallel*sin(q1 - gslope) ...
- 2*q2*(kleg*lleg - g*(mfoot + msoftparallel + msoftseries)*sin(q1 - gslope)) ...
- 2*q4*g*mfoot*sin(gslope) - 2*q4*g*mpelvis*sin(gslope) - ...
2*q4*g*msoftparallel*sin(gslope) - 2*q4*g*msoftseries*sin(gslope) - ...
2*q4*g*msoftstomach*sin(gslope))/2.;

PEgrav = -(mpelvis*(-(q5*g*cos(gslope)) + q4*g*sin(gslope))) - ...
msoftstomach*(-((q3 + q5)*g*cos(gslope)) + q4*g*sin(gslope)) - ...
msoftseries*(-(q5*g*cos(gslope)) - q2*g*power(c1*c1 + s1*s1,-0.5)*sin(q1 - ...
gslope) + q4*g*sin(gslope)) - mfoot*(-(q5*g*cos(gslope)) - g*(q2*power(c1*c1 ...
+ s1*s1,-0.5) + q6*power(c1*c1 + s1*s1,-0.5))*sin(q1 - gslope) + ...
q4*g*sin(gslope)) - msoftparallel*(-(q5*g*cos(gslope)) - g*(q2*power(c1*c1 + ...
s1*s1,-0.5) + q6*power(c1*c1 + s1*s1,-0.5) - q7*power(c1*c1 + ...
s1*s1,-0.5))*sin(q1 - gslope) + q4*g*sin(gslope));

PEspring = (kleg*((q2 - lleg)*(q2 - lleg)))/2. + (ksoftparallel*((q7 - ...
lsoftparallel)*(q7 - lsoftparallel)))/2. + (ksoftseries*((q6 - ...
lsoftseries)*(q6 - lsoftseries)))/2. + (ksoftstomach*((q3 - lsoftstomach)*(q3 ...
- lsoftstomach)))/2.;
            
            E.KE = kineticEnergy;
            E.PE = potentialEnergy;
            E.PEgrav = PEgrav;
            E.PEspring = PEspring;
            E.Total = E.KE + E.PE;
            
        end
        
        function [points] = getPoints(this, state)
            
            this.getParams();
            this.getQandUdefs(state);
            
            c1 = cos(q1); s1 = sin(q1);
           
            points.foot(1) = q4 + c1*(q2 + q6); 
points.foot(2) = q5 + (q2 + q6)*s1; 


points.pelvis(1) = q4; 
points.pelvis(2) = q5; 


points.softstomach(1) = q4; 
points.softstomach(2) = q3 + q5; 


points.COMpos(1) = c1*((q2 + q6)*mfoot + (q2 + q6 - q7)*msoftparallel + ...
q2*msoftseries)*power(mfoot + mpelvis + msoftparallel + msoftseries + ...
msoftstomach,-1) + (q4*mfoot + q4*mpelvis + q4*msoftparallel + q4*msoftseries ...
+ q4*msoftstomach)*power(mfoot + mpelvis + msoftparallel + msoftseries + ...
msoftstomach,-1); 
points.COMpos(2) = s1*((q2 + q6)*mfoot + (q2 + q6 - q7)*msoftparallel + ...
q2*msoftseries)*power(mfoot + mpelvis + msoftparallel + msoftseries + ...
msoftstomach,-1) + (q5*mfoot + q5*mpelvis + q5*msoftparallel + q5*msoftseries ...
+ (q3 + q5)*msoftstomach)*power(mfoot + mpelvis + msoftparallel + msoftseries ...
+ msoftstomach,-1); 


points.softseries(1) = c1*q2 + q4; 
points.softseries(2) = q5 + q2*s1; 


points.softparallel(1) = q4 + c1*(q2 + q6 - q7); 
points.softparallel(2) = q5 + (q2 + q6 - q7)*s1; 
        end
        
        function [newstate] = GoodInitialConditions(this,x0)
            this.getQandUdefs(x0);
            %Shift body (without changing configuration) so that foot is on ground
            stanceang = q1+pi;
            pelvx = (q2+q6)*cos(stanceang);
            pelvy = (q2+q6)*sin(stanceang);
            
            Jc = this.getConstraints(x0,'unbottomed');
            
            upelvx = -(Jc(1,1)*u1 + Jc(1,2)*u2 + Jc(1,6)*u6 + Jc(1,7)*u7);
            upelvy = -(Jc(2,1)*u1 + Jc(2,2)*u2 + Jc(2,6)*u6 + Jc(2,7)*u7);
            
            newstate=x0;
            newstate([4 5 11 12]) = [pelvx pelvy upelvx upelvy];
            
            u = newstate(this.N/2+1:this.N)';
%             Jc*u;
            
            %Make sure leg springs start unstretched
            newstate(2) = this.lleg;
            newstate(6) = this.lsoftseries;
            newstate(7) = this.lsoftparallel;
        end
        
        function [c,ceq] = TakeoffConstraints(this,x0,xf,tf,allx,allt,tair)
            %
            xair = allx(find(allt==tair,1),:);
            statesair = SeriesBottomRunnerState(xair);
            
            c=[];
            % We want the parallel spring not to be storing energy at
            % takeoff
            ceq = statesair.softparallel.stretch - this.lsoftparallel;
            
        end
        
        function Cost = TakeoffCost(this,x0,xf,tf,allx,allt,tair)
            %            
            xair = allx(find(allt==tair,1),:);
            statesair = SeriesBottomRunnerState(xair);
            
            Cost = abs(statesair.softparallel.stretch - this.lsoftparallel) + abs(statesair.softparallel.stretchDot);
            
        end
        
        function [GRF] = getGRF(this,x,phase)
            
            if size(x,1)==1
                x=x';
            end
            
            if ~strcmp(phase,'aerial')
                [xddot, constraintForces] = this.XDoubleDot(0,x,phase);
                GRF = [constraintForces(1); constraintForces(2)];
            else
                GRF=[0;0];
            end
        end
        
        function [comWR] = getcomWR(this,x,phase)
            this.getParams();
            this.getQandUdefs(x);
            
            c1 = cos(q1); s1 = sin(q1);
            
            
comvx = (u4*mfoot + u4*mpelvis + u4*msoftparallel + u4*msoftseries - ...
s1*u1*(-(q7*msoftparallel) + q6*(mfoot + msoftparallel) + q2*(mfoot + ...
msoftparallel + msoftseries)) + c1*(-(u7*msoftparallel) + u6*(mfoot + ...
msoftparallel) + u2*(mfoot + msoftparallel + msoftseries)) + ...
u4*msoftstomach)*power(mfoot + mpelvis + msoftparallel + msoftseries + ...
msoftstomach,-1);

comvy = (u5*mfoot + u5*mpelvis + u5*msoftparallel + u5*msoftseries + ...
c1*u1*(-(q7*msoftparallel) + q6*(mfoot + msoftparallel) + q2*(mfoot + ...
msoftparallel + msoftseries)) + s1*(-(u7*msoftparallel) + u6*(mfoot + ...
msoftparallel) + u2*(mfoot + msoftparallel + msoftseries)) + (u3 + ...
u5)*msoftstomach)*power(mfoot + mpelvis + msoftparallel + msoftseries + ...
msoftstomach,-1);
            
            GRF = this.getGRF(x,phase);
            
            comWR = dot(GRF,[comvx;comvy]);
        end
        
        function stomachPower = getStomachPower(this,x)
            
            if size(x,1)>1 && size(x,2) >1  %If we have more than one state vec
                stomachPower = zeros(size(x,1),1);
                for i = 1:size(x,1)
                    sz = SeriesBottomRunnerState(x(i,:));
                    force = -this.ksoftstomach*(sz.softstomach.stretch - this.lsoftstomach)...
                        -this.csoftstomach*sz.softstomach.stretchDot;
                    velocity = sz.softstomach.stretchDot;
                    stomachPower(i) = force*velocity;
                end
            else         %For one state vec
                sz = SeriesBottomRunnerState(x);
                force = -this.ksoftstomach*(sz.softstomach.stretch - this.lsoftstomach)...
                    -this.csoftstomach*sz.softstomach.stretchDot;
                velocity = sz.softstomach.stretchDot;
                stomachPower = force*velocity;
            end
        end
        
        function LegPower = getLegPower(this,x)
            
            if size(x,1)>1 && size(x,2) >1  %If we have more than one state vec
                LegPower = zeros(size(x,1),1);
                for i = 1:size(x,1)
                    sz = SeriesBottomRunnerState(x(i,:));
                    force = -this.kleg*(sz.stanceLeg.Length - this.lleg)...
                        -this.cleg*sz.stanceLeg.LengthDot;
                    velocity = sz.stanceLeg.LengthDot;
                    LegPower(i) = force*velocity;
                end
            else         %For one state vec
                    sz = SeriesBottomRunnerState(x);
                    force = -this.kleg*(sz.stanceLeg.Length - this.lleg)...
                        -this.cleg*sz.stanceLeg.LengthDot;
                    velocity = sz.stanceLeg.LengthDot;
                    LegPower = force*velocity;
            end
        end
        
        function seriesPower = getSeriesPower(this,x)
            if size(x,1)>1 && size(x,2) >1  %If we have more than one state vec
                seriesPower = zeros(size(x,1),1);
                for i = 1:size(x,1)
                    sz = SeriesBottomRunnerState(x(i,:));
                    force = -this.ksoftseries*(sz.softseries.stretch - this.lsoftseries) ...
                        -this.csoftseries*sz.softseries.stretchDot;
                    velocity = sz.softseries.stretchDot;
                    seriesPower(i) = force*velocity;
                end
            else
                sz = SeriesBottomRunnerState(x);
                force = -this.ksoftseries*(sz.softseries.stretch - this.lsoftseries) ...
                    -this.csoftseries*sz.softseries.stretchDot;
                velocity = sz.softseries.stretchDot;
                seriesPower = force*velocity;
            end
        end
        
        function parallelPower = getParallelPower(this,x)
            if size(x,1)>1 && size(x,2) >1  %If we have more than one state vec
                parallelPower = zeros(size(x,1),1);
                for i = 1:size(x,1)
                    sz = SeriesBottomRunnerState(x(i,:));
                    force = -this.ksoftparallel*(sz.softparallel.stretch - this.lsoftparallel)...
                        -this.csoftparallel*sz.softparallel.stretchDot;
                    velocity = sz.softparallel.stretchDot;
                    parallelPower(i) = force*velocity;
                end
            else
                sz = SeriesBottomRunnerState(x);
                force = -this.ksoftparallel*(sz.softparallel.stretch - this.lsoftparallel)...
                    -this.csoftparallel*sz.softparallel.stretchDot;
                velocity = sz.softparallel.stretchDot;
                parallelPower = force*velocity;
            end
        end
        
        function softPower = getSoftPower(this,x)
            stomachPower = this.getStomachPower(x);
            seriesPower = this.getSeriesPower(x);
            parallelPower = this.getParallelPower(x);
            softPower = stomachPower+seriesPower+parallelPower;
        end
        
        
    end
    
    
end
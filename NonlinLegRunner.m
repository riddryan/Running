classdef NonlinLegRunner < Runner
    %Generic running model with basic functions to take derivatives,
    %integrate dynamics, implement collisions, and more.  You need
    %subclasses with more specific information about a particular model to
    %use this class.
    
    properties
        %Mass
        mpelvis = .9; mfoot=0; msoftparallel=.1;
        
        %Leg and soft stomach stuff
        
        cleg=0.3; csoftparallel=2;
        lleg=1; lsoftparallel=.4;
        ksoftparallel=18;
        
        gslope = 0;
        g = 1;
        
        kimpact = 40;  %stiffness of leg spring at impact
        kstance = 18;  %stiffness of leg after impact
        
        %How much force leg spring must be exerting before transitioning
        %into a more compliant spring (kimpact -> kstance).  This builds in a nonlinearity into
        %the spring which can hopefully account for the impact peak in the
        %ground reaction force data of running humans.  The leg transitions
        %between kimpact and kstance over the range set by the position thresholds transitionstart
        %and transitionend.
        transitionstart = .97;
        transitionend = .95;
        
        
        phases = {'stance', 'aerial'};
        
        %State Stuff
        N=10;
        statestovary = [1 ...
            6 7 8];
        statestomeasure = [...
            6 7 8];
        
    end
    
    methods (Static)
        
        function [] = test()
            %%
            saveAnimation = 0;
            aviname = 'C:\Users\HBCL Student\Documents\Ryan\Model and Structure\dynamicWalking\Branches\riddryan\NewStruct\testanim.avi';
            
            
            runner = NonlinLegRunner;
            
             IC = NonlinLegRunnerState;
            
            IC.stanceLeg.Angle = -1.1105; 
            IC.stanceLeg.AngleDot = -1;
            
            IC.stanceLeg.Length = runner.lleg;
            IC.stanceLeg.LengthDot = -.7;
            
            IC.softparallel.stretch = runner.lsoftparallel;
            IC.softparallel.stretchDot = -.47;
            
            x0 = IC.getVector();
            
            runner.mpelvis=.9;
            runner.msoftparallel=.1;
            runner.csoftparallel = 0.5;
            runner.ksoftparallel = 18;
            runner.cleg = 0.1;
            runner.gslope=0;
%             
            %Make sure foot starts on ground
            x0 = runner.GoodInitialConditions(x0);
            
            
            %Take a Step
            [xf,tf,allx,allt,tair] = runner.onestep(x0);
            
            %% Check Energy and constraints
            energies0 = runner.getEnergies(x0);
            energy0 = energies0.Total;
            r=runner;
            for i = 1:length(allx)
                if allt(i)<tair
                    phase = 'stance';
                else
                    phase = 'aerial';
                end
                
                points=runner.getPoints(allx(i,:));
                footx(i)=points.foot(1);
                footy(i)=points.foot(2);
                Jvel = runner.getVelJacob(allx(i,:));
                u = allx(i,runner.N/2+1:runner.N)';
                vels = Jvel*u;
                footvel(i,1:2) = vels(3:4);
                
                energies = runner.getEnergies(allx(i,:));
                tote(i) = energies.Total;
                
                Jc = runner.getConstraints(allx(i,:),phase);
                ConstraintError(i,1) = norm(Jc*u);
                
                x = allx(i,:);
                GRF(i,:) = r.getGRF(x,phase);
                comWR(i,1) = r.getcomWR(x,phase);
                parallelPower(i,1) = r.getParallelPower(x);
                legPower(i,1) = r.getLegPower(x);
                
                ParForce(i,1) = r.ksoftparallel*(x(3) - r.lsoftparallel) + r.csoftparallel*x(8);
                
                kleghistory(i,1) = r.getkleg(x);
                legforce(i,1) = kleghistory(i,1)*(x(2) - r.lleg) + r.cleg*x(7);
                
            end
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
            hold on
            pt1 = find(kleghistory~=r.kimpact,1);
            pt2 = find(kleghistory==r.kstance,1);
            plot(allt(pt1),GRF(pt1,2),'rx')
            plot(allt(pt2),GRF(pt2,2),'rx')
            legend('grfX','grfY')
            subplot(212)
            hold on
            plot(allt,comWR,'k')
            plot(allt,parallelPower,'g')
            plot(allt,legPower,'r')
            legend('COM WR','Soft Parallel','Leg Spring')
            
            [speed] = runner.getSpeed(x0, xf, tf)
            [steplength] = runner.getStepLength(x0, xf)
            [airfrac] = runner.getAerialFraction(x0, tf, tair)
            
            figure
            plot(allt,allx(:,3));
            hold on
            plot(allt,allx(:,8),'r')
            legend('Parallel Pos','Parallel Vel');
            
            figure
            plot(ParForce)
            hold on
            plot(legforce,'r')
            legend('Parallel Soft Force','Leg force')
            
            
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
        
        
        function [this] = NonlinLegRunner(input)
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
            statestruc = NonlinLegRunnerState(state);
            
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
            legdir = (points.pelvis' - points.foot')/norm(points.pelvis - points.foot);
            RotNeg90 = [0 1;-1 0];
            perpdir = RotNeg90*legdir;
            
            legoffset = springstart*legdir';
            paroffset = dsep*perpdir' + legoffset;
            
            offsetfoot = points.foot + legoffset;
            offsetpelvis = points.pelvis + legoffset;
            
            parfoot = points.foot + paroffset;
            parpar = points.softparallel + paroffset;
            
            numcoils=3;
            springwidth=.07;
            plotter.plotSpring(offsetfoot(1),offsetfoot(2),...
                offsetpelvis(1),offsetpelvis(2),...
                numcoils,this.lleg,springwidth) %leg spring
            
            numcoils=2;
            springwidth=.05;
            softcolor=[66 128 204]/255;
            
            plotter.plotSpring(parfoot(1),parfoot(2),...
                parpar(1),parpar(2),...
                numcoils,this.lsoftparallel,springwidth,...
                'Color',softcolor) %soft parallel spring
            
            plotter.plotLine(points.foot,offsetfoot);
            plotter.plotLine(offsetfoot,parfoot);
            %%
            minmass = min([this.mpelvis this.msoftparallel]);
            maxmass = max([this.mpelvis this.msoftparallel]);
            massrange = maxmass-minmass;
            
            %Draw Masses
            plotter.plotMass(offsetpelvis,'scaling',this.mpelvis-minmass/massrange);
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
        
        function kleg = getkleg(this,state)
            pt1 = this.transitionstart;
            pt2 = this.transitionend;
            StateStruc = NonlinLegRunnerState(state);
            legpos = StateStruc.stanceLeg.Length;
            legvel = StateStruc.stanceLeg.LengthDot;
            
            if legpos/this.lleg >= pt1 && legvel<=0  %Impact phase
                kleg = this.kimpact;
            elseif legpos/this.lleg <= pt1 && legpos/this.lleg >= pt2 && legvel<=0 % Transition Region
                kleg = interp1([pt1 pt2],[this.kimpact this.kstance],legpos); %linear interpolation
            else %We are past the transition region
                kleg = this.kstance;
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
            
            
            %%
            this.getQandUdefs(x0);
            
            %Figure out where pelvis is using stance leg kinematics (this takes
            %care of initial conditions that aren't consistenent with stance
            %constraints).  This should ideally already have been done before
            %calling onestep.
            
            x0 = this.GoodInitialConditions(x0);
            
            %% Stance Phase
            phase = this.phases{1};
            stanceoptions = odeset('Events', @(t,x) this.LiftOrFall(t,x),'RelTol',RelTol','AbsTol',AbsTol);
            tstart=0;
            
            stancex0 = x0;
            [stancet,stancestates] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tstart:dt:tmax,stancex0,stanceoptions);
            
            tair = stancet(end); phasevec = ones(length(stancet),1);
            
            %% Aerial Phase
            phase = this.phases{2};
            airx0 = stancestates(end,:);
            
            %Set Swing leg angle and length
            landangle = x0(1);
            airx0(1)=landangle;
            airx0(2)=this.lleg;
            airx0(3)=this.lsoftparallel;
            airx0(6)=0;
            airx0(7)=0;
            airx0(8)=0;
            
            airoptions = odeset('Events', @(t,x) this.HeelStrikeOrFall(t,x,landangle),'RelTol',RelTol','AbsTol',AbsTol);
            
            [airt,airstates] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tair:dt:tair+tmax,airx0,airoptions);
            
            tf = airt(end);
            allt = [stancet; airt];
            allx = [stancestates;airstates];
            phasevec = [phasevec; 2*ones(length(airt),1)];
            %% Switch States for Next Step & Plotting
            
            %Make sure states are consistent with heel strike
            xf = switchLegs(this,allx(end,:),landangle);
            
            %Calculate states after collision
            [xf,impulses] = this.phaseTransition(tf,xf,'stance');
            
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
        
        function [value, isTerminal, direction] = LiftOrFall(this,t,state)
            points = this.getPoints(state);
            
            %Lifting Off the Ground
            phase = 'stance';
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
            
            newstatestruc = NonlinLegRunnerState(x);
            newstatestruc.stanceLeg.Angle = landangle;
            newstatestruc.stanceLeg.Length = this.lleg;
            newstatestruc.softparallel.stretch = this.lsoftparallel;
            
            xnew = getVector(newstatestruc);
            
        end
        
        function [speed] = getSpeed(this, x0, xf, tf)
            if isempty(xf)
                [xf,tf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = NonlinLegRunnerState(x0);
            xfstruc = NonlinLegRunnerState(xf);
            
            speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
        end
        
        function [steplength] = getStepLength(this, x0, xf)
            if isempty(xf)
                [xf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = NonlinLegRunnerState(x0);
            xfstruc = NonlinLegRunnerState(xf);
            
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
            kleg = this.getkleg(state);
            
            c1 = cos(q1); s1 = sin(q1);
            
MM = zeros(5,5); rhs = zeros(5,1);

% Mass Matrix
MM(1,1) = -2*q2*q3*msoftparallel + (mfoot + msoftparallel)*(q2*q2) + ...
msoftparallel*(q3*q3); MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = ...
-(s1*(-(q3*msoftparallel) + q2*(mfoot + msoftparallel))); MM(1,5) = ...
c1*(-(q3*msoftparallel) + q2*(mfoot + msoftparallel)); 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + msoftparallel; MM(2,3) = -msoftparallel; ...
MM(2,4) = c1*(mfoot + msoftparallel); MM(2,5) = s1*(mfoot + msoftparallel); 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = msoftparallel; MM(3,4) = ...
-(c1*msoftparallel); MM(3,5) = -(s1*msoftparallel); 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot + ...
mpelvis + msoftparallel; MM(4,5) = 0; 
MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
MM(5,5) = mfoot + mpelvis + msoftparallel; 

% righthand side terms
rhs(1) = q3*msoftparallel*(2*u1*(u2 - u3) + g*cos(q1 - gslope)) - ...
q2*(2*u1*(-(u3*msoftparallel) + u2*(mfoot + msoftparallel)) + g*(mfoot + ...
msoftparallel)*cos(q1 - gslope)); 
rhs(2) = -(u2*cleg) + kleg*(-q2 + lleg) + (-(q3*msoftparallel) + q2*(mfoot + ...
msoftparallel))*(u1*u1) - g*mfoot*sin(q1 - gslope) - g*msoftparallel*sin(q1 - ...
gslope); 
rhs(3) = -(u3*csoftparallel) + ksoftparallel*(-q3 + lsoftparallel) + (-q2 + ...
q3)*msoftparallel*(u1*u1) + g*msoftparallel*sin(q1 - gslope); 
rhs(4) = 2*s1*u1*u2*mfoot + 2*s1*u1*u2*msoftparallel - ...
2*s1*u1*u3*msoftparallel - c1*q3*msoftparallel*(u1*u1) + c1*q2*(mfoot + ...
msoftparallel)*(u1*u1) + g*mfoot*sin(gslope) + g*mpelvis*sin(gslope) + ...
g*msoftparallel*sin(gslope); 
rhs(5) = -2*c1*u1*u2*mfoot - 2*c1*u1*u2*msoftparallel + ...
2*c1*u1*u3*msoftparallel - g*mfoot*cos(gslope) - g*mpelvis*cos(gslope) - ...
g*msoftparallel*cos(gslope) - q3*s1*msoftparallel*(u1*u1) + q2*s1*(mfoot + ...
msoftparallel)*(u1*u1); 
        end
        
        function [C, CDot] = getConstraints(this,state,phase)
            %%
            this.getParams();
            this.getQandUdefs(state);
            
            c1 = cos(q1); s1 = sin(q1);
            
            switch phase
                case 'stance'
constraintJacobianStance(1,1) = -(q2*s1); constraintJacobianStance(1,2) = c1; ...
constraintJacobianStance(1,3) = 0; constraintJacobianStance(1,4) = 1; ...
constraintJacobianStance(1,5) = 0; 
constraintJacobianStance(2,1) = c1*q2; constraintJacobianStance(2,2) = s1; ...
constraintJacobianStance(2,3) = 0; constraintJacobianStance(2,4) = 0; ...
constraintJacobianStance(2,5) = 1; 


constraintJacobianStanceDot(1,1) = -(c1*q2*u1) - s1*u2; ...
constraintJacobianStanceDot(1,2) = -(s1*u1); constraintJacobianStanceDot(1,3) ...
= 0; constraintJacobianStanceDot(1,4) = 0; constraintJacobianStanceDot(1,5) = ...
0; 
constraintJacobianStanceDot(2,1) = -(q2*s1*u1) + c1*u2; ...
constraintJacobianStanceDot(2,2) = c1*u1; constraintJacobianStanceDot(2,3) = ...
0; constraintJacobianStanceDot(2,4) = 0; constraintJacobianStanceDot(2,5) = ...
0; 


                    C = constraintJacobianStance;
                    CDot = constraintJacobianStanceDot;
                    
                case 'aerial'
constraintJacobianAerial(1,1) = 1; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 0; constraintJacobianAerial(1,4) = 0; ...
constraintJacobianAerial(1,5) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 1; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 0; ...
constraintJacobianAerial(2,5) = 0; 
constraintJacobianAerial(3,1) = 0; constraintJacobianAerial(3,2) = 0; ...
constraintJacobianAerial(3,3) = 1; constraintJacobianAerial(3,4) = 0; ...
constraintJacobianAerial(3,5) = 0; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; ...
constraintJacobianAerialDot(1,5) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; ...
constraintJacobianAerialDot(2,5) = 0; 
constraintJacobianAerialDot(3,1) = 0; constraintJacobianAerialDot(3,2) = 0; ...
constraintJacobianAerialDot(3,3) = 0; constraintJacobianAerialDot(3,4) = 0; ...
constraintJacobianAerialDot(3,5) = 0; 
                    
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
velJacobian(1,4) = 1; velJacobian(1,5) = 0; 
velJacobian(2,1) = 0; velJacobian(2,2) = 0; velJacobian(2,3) = 0; ...
velJacobian(2,4) = 0; velJacobian(2,5) = 1; 
velJacobian(3,1) = -(q2*s1); velJacobian(3,2) = c1; velJacobian(3,3) = 0; ...
velJacobian(3,4) = 1; velJacobian(3,5) = 0; 
velJacobian(4,1) = c1*q2; velJacobian(4,2) = s1; velJacobian(4,3) = 0; ...
velJacobian(4,4) = 0; velJacobian(4,5) = 1; 
velJacobian(5,1) = -(q2*s1) + q3*s1; velJacobian(5,2) = c1; velJacobian(5,3) ...
= -c1; velJacobian(5,4) = 1; velJacobian(5,5) = 0; 
velJacobian(6,1) = c1*q2 - c1*q3; velJacobian(6,2) = s1; velJacobian(6,3) = ...
-s1; velJacobian(6,4) = 0; velJacobian(6,5) = 1; 

            
            
            
            J=velJacobian;
        end
        
        function [E] = getEnergies(this,state)
            this.getParams();
            this.getQandUdefs(state);
            kleg = this.getkleg(state);
            c1 = cos(q1); s1 = sin(q1);
            
kineticEnergy = (mpelvis*(u4*u4 + u5*u5) + mfoot*(-2*q2*u1*(s1*u4 - c1*u5) + ...
2*u2*(c1*u4 + s1*u5) + q2*q2*(u1*u1) + u2*u2 + u4*u4 + u5*u5) + ...
msoftparallel*(-2*(q2 - q3)*s1*u1*u4 + 2*s1*(u2 - u3)*u5 + c1*(2*(u2 - u3)*u4 ...
+ 2*(q2 - q3)*u1*u5) + (q2 - q3)*(q2 - q3)*(u1*u1) + (u2 - u3)*(u2 - u3) + ...
u4*u4 + u5*u5))/2.;

potentialEnergy = (kleg*((-q2 + lleg)*(-q2 + lleg)) + ksoftparallel*((-q3 + ...
lsoftparallel)*(-q3 + lsoftparallel)) + 2*g*mpelvis*(q5*cos(gslope) - ...
q4*sin(gslope)) - 2*g*mfoot*(-(q5*cos(gslope)) - q2*sin(q1 - gslope) + ...
q4*sin(gslope)) - 2*g*msoftparallel*(-(q5*cos(gslope)) - (q2 - q3)*sin(q1 - ...
gslope) + q4*sin(gslope)))/2.;

PEgrav = -(mpelvis*(-(q5*g*cos(gslope)) + q4*g*sin(gslope))) - ...
mfoot*(-(q5*g*cos(gslope)) - q2*g*sin(q1 - gslope) + q4*g*sin(gslope)) - ...
msoftparallel*(-(q5*g*cos(gslope)) - (q2 - q3)*g*sin(q1 - gslope) + ...
q4*g*sin(gslope));

PEspring = (kleg*((q2 - lleg)*(q2 - lleg)))/2. + (ksoftparallel*((q3 - ...
lsoftparallel)*(q3 - lsoftparallel)))/2.;
            
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
           
points.foot(1) = c1*q2 + q4; 
points.foot(2) = q5 + q2*s1; 


points.pelvis(1) = q4; 
points.pelvis(2) = q5; 


points.COMpos(1) = c1*(q2*mfoot + (q2 - q3)*msoftparallel)*power(mfoot + ...
mpelvis + msoftparallel,-1) + (q4*mfoot + q4*mpelvis + ...
q4*msoftparallel)*power(mfoot + mpelvis + msoftparallel,-1); 
points.COMpos(2) = s1*(q2*mfoot + (q2 - q3)*msoftparallel)*power(mfoot + ...
mpelvis + msoftparallel,-1) + (q5*mfoot + q5*mpelvis + ...
q5*msoftparallel)*power(mfoot + mpelvis + msoftparallel,-1); 


points.softparallel(1) = c1*(q2 - q3) + q4; 
points.softparallel(2) = q5 + (q2 - q3)*s1; 
        end
        
        function [newstate] = GoodInitialConditions(this,x0)
            this.getQandUdefs(x0);
            %Set state velocities to be consistent with constraints while being
            %least squares close to inputted velocities
            Jc = this.getConstraints(x0,'stance');
%             nullJc = null(Jc);
%             unew = nullJc * (nullJc \ x0(6:10)');
%             newstate = [x0(1:5) unew'];
            upelvx = -(Jc(1,1)*u1 + Jc(1,2)*u2 + Jc(1,3)*u3);
            upelvy = -(Jc(2,1)*u1 + Jc(2,2)*u2 + Jc(2,3)*u3);
            
            newstate = x0;
            newstate([9 10]) = [upelvx upelvy];
            
            %Shift body so that heel is on ground
            points = this.getPoints(newstate);
            pelvx = points.pelvis(1) - points.foot(1);
            pelvy = points.pelvis(2) - points.foot(2);
            newstate([4 5]) = [pelvx pelvy];
            
            %       u = newstate(6:10)';
            %       Jc*u; %should be zero if constraints are working
        end
        
        function [c,ceq] = TakeoffConstraints(this,x0,xf,tf,allx,allt,tair)
            %
            xair = allx(find(allt==tair,1),:);
            statesair = NonlinLegRunnerState(xair);
            
            c=[];
            % We want the parallel spring not to be storing energy at
            % takeoff
            ceq = statesair.softparallel.stretch - this.lsoftparallel;
            
        end
        
        function Cost = TakeoffCost(this,x0,xf,tf,allx,allt,tair)
            %            
            xair = allx(find(allt==tair,1),:);
            statesair = NonlinLegRunnerState(xair);
            
            Cost = abs(statesair.softparallel.stretch - this.lsoftparallel) + abs(statesair.softparallel.stretchDot);
            
        end
        
        function [GRF] = getGRF(this,x,phase)
            
            if size(x,1)==1
                x=x';
            end
            
            if strcmp(phase,'stance')
                [xddot, constraintForces] = this.XDoubleDot(0,x,phase);
                GRF = constraintForces;
            else
                GRF=[0;0];
            end
        end
        
        function [comWR] = getcomWR(this,x,phase)
            this.getParams();
            this.getQandUdefs(x);
            
            c1 = cos(q1); s1 = sin(q1);
            
comvx = (c1*u2*mfoot + u4*mfoot + u4*mpelvis + q3*s1*u1*msoftparallel + ...
c1*u2*msoftparallel - c1*u3*msoftparallel + u4*msoftparallel - ...
q2*s1*u1*(mfoot + msoftparallel))*power(mfoot + mpelvis + msoftparallel,-1);

comvy = (u5*mfoot + u5*mpelvis + u5*msoftparallel + ...
c1*u1*(-(q3*msoftparallel) + q2*(mfoot + msoftparallel)) + ...
s1*(-(u3*msoftparallel) + u2*(mfoot + msoftparallel)))*power(mfoot + mpelvis ...
+ msoftparallel,-1);
            
            GRF = this.getGRF(x,phase);
            
            comWR = dot(GRF,[comvx;comvy]);
        end
        
        
        function parallelPower = getParallelPower(this,x)
            sz = NonlinLegRunnerState(x);
            force = -this.ksoftparallel*(sz.softparallel.stretch - this.lsoftparallel)...
                    -this.csoftparallel*sz.softparallel.stretchDot;
            velocity = sz.softparallel.stretchDot;
            parallelPower = force*velocity;
        end
        
        function legPower = getLegPower(this,x)
           kleg = this.getkleg(x);
           sz = NonlinLegRunnerState(x);
           force = -kleg*(sz.stanceLeg.Length-this.lleg)...
                   -this.cleg*sz.stanceLeg.LengthDot;
           velocity = sz.stanceLeg.LengthDot;
           legPower = force*velocity;
        end
        
        function softPower = getSoftPower(this,x)
            parallelPower = this.getParallelPower(x);
            softPower = parallelPower;
        end
        
        
        
        
    end
    
    
end
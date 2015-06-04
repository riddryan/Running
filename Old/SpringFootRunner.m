classdef SpringFootRunner < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
  
    properties
        mpelvis = .8; mfoot=0; mheel=1e-2; mtoe=1e-2;
        gslope = 0;
        g = 1;
        N=12;
        statestovary = [3 5 6 9 10 11 12];
        statestomeasure = [7 8 9 10 11 12];
        lleg=1; lheel = .2; ltoe = .2;
        footangle = pi - 80/360*2*pi; 
        achillesangle = 138/360*2*pi;
        kleg=12.9801; kfoot=0; kachilles=0;
        cleg=0; cfoot=0; cachilles=0;
        phases = {'HeelToe' 'Toe' 'Aerial'};
    end
   
    methods (Static)
       
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            runfirstphase=0;
            savepath = [dir '\Animations\'];
            aviname = [savepath 'SF_StrongerFoot.avi'];
                
            runner = SpringFootRunner;
            
            IC = SpringFootRunnerState;
            
%             x0 = [-.8 1 .2 0 0 -.895 -.56 -.07 0 0];
            IC.stanceLeg.Angle = -1.1129;
            IC.stanceLeg.AngleDot = -1;
            
            IC.stanceLeg.Length = 1;
            IC.stanceLeg.LengthDot = -1;
            
            IC.toe.Angle = -30 /360*2*pi;
            IC.toe.AngleDot = 0;
            
            IC.heel.AngleDot = 0;
            
            IC.heel.Angle = -130 /360*2*pi; 
            
            
            x0 = IC.getVector();
            
            %Make sure foot starts on ground
            x0 = runner.GoodInitialConditions(x0);
            
            runner.kleg = 15;
            runner.kfoot = .4;
            runner.cfoot =0.01;
            runner.kachilles = .4;
            runner.mfoot = 0;
            runner.mheel = 0.01;
            runner.mtoe = .01;
            runner.footangle = x0(5) - x0(6);
            runner.achillesangle = x0(3) - x0(6);
            
            
            
            if runfirstphase
                opts = odeset('Events', @(t,x) runner.ToeOnOrFall(t,x));
                [t,x] = ode45(@(t,x) runner.XDoubleDot(t,x,'Heel'),[0 5],x0,opts);
                
                figure
                for i = 1 : length(x)
                    cla;
                    runner.plot(x(i, :));
                    pause(0.05);
                end
                
                for i = 1:length(x)
                    energies = runner.getEnergies(x(i,:));
                    tote(i) = energies.Total;
                    
                end
                
                figure
                plot(t,tote)
            end
            
%             points=runner.getPoints(x0);
%             runner.achilleslength = norm(points.pelvis - points.heel);\
            
            %Take a Step
            [xf,tf,allx,allt,tair,~,phasevec] = runner.onestep(x0);
            
            
            %% Check Energy and constraints
            energies0 = runner.getEnergies(x0);
            energy0 = energies0.Total;
            
            N = runner.N;
            
            for i = 1:length(allx)
                
                phase = runner.phases{phasevec(i)};
                points=runner.getPoints(allx(i,:));
                footx(i)=points.foot(1);
                footy(i)=points.foot(2);
                heelx(i)=points.heel(1); heely(i) = points.heel(2);
                toex(i)=points.toe(1); toey(i) = points.toe(2);
                Jvel = runner.getVelJacob(allx(i,:));
                u = allx(i,N/2+1:end)';
                vels = Jvel*u;
                footvel(i,1:2) = vels(5:6);
                
                energies = runner.getEnergies(allx(i,:));
                tote(i) = energies.Total;
                
                footpower(i) = runner.getFootPower(allx(i,:));
                achillespower(i) = runner.getAchillesPower(allx(i,:));
                comWR(i) = runner.getcomWR(allx(i,:),phase);
                legPower(i) = runner.getLegPower(allx(i,:));
                
                GRF(i,:) = runner.getGRF(allx(i,:),phase);
                if strcmp(phase,'HeelToe')
                    theeltoe(i) = allt(i);
                    [~,lambdas(i,:)]=runner.XDoubleDot(allt(i),allx(i,:)',phase);
                end
                
                Jc = runner.getConstraints(allx(i,:),phase);
                
%                 ConstraintError(i,:) = Jc*u;
                
                %                 if abs(energies.Total - energy0) > 1e-3
                %                     error('energy conservation failed');
                %                 end
                
            end
            figure
            plot(theeltoe,lambdas);
            hold on
            plot(allt,GRF(:,1),'m')
            plot(allt,GRF(:,2),'c')
            legend('HeelX','HeelY','ToeY','GrfX','GrfY')
            
            figure
%             subplot(311)
%             plot(allt,footx)
%             hold on
%             plot(allt,footy,'r')
%             plot(allt,footvel(:,1),'m')
%             plot(allt,footvel(:,2),'k')
%             title('Foot')
%             legend('xpos','ypos','xvel','yvel')
            
            subplot(211)
            plot(allt,tote)
            legend('total energy')
            
            subplot(212)
            plot(allt,comWR)
            hold on
            plot(allt,footpower,'r')
            plot(allt,achillespower,'c')
            plot(allt,legPower,'k')
            legend('COM WR','Foot','Achilles','Leg')
%             subplot(313)
%             plot(allt,ConstraintError)
%             title('Constraints Error')
            
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
    
    function [this] = SpringFootRunner(input)
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
      

      %Draw Lines
      %       plotter.plotLine(points.foot,points.pelvis);
      %       plotter.plotLine(points.pelvis,points.soft);
      plotter.plotLine(points.foot,points.heel);
      plotter.plotLine(points.foot,points.toe);
      
      %Draw Springs
      numcoils=3;
      springwidth=.07;
      plotter.plotSpring(points.foot(1),points.foot(2),...
          points.pelvis(1),points.pelvis(2),...
          numcoils,this.lleg,springwidth) %leg spring
      
           numcoils=3;
      springwidth=.03;
      plotter.plotSpring(points.heel(1),points.heel(2),...
          points.pelvis(1),points.pelvis(2),...
          numcoils,1,springwidth,'Color',[0 1 1]) %achilles spring
      
                 numcoils=3;
      springwidth=.02;
      plotter.plotSpring(points.heel(1),points.heel(2),...
          points.toe(1),points.toe(2),...
          numcoils,.3,springwidth,'Color',[1 0 0]) %achilles spring
      
      %Draw Masses
      plotter.plotMass(points.pelvis);
      
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
        frameskip=2;
        aviname=[];
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'frameskip'
                    interleaveAnimationFrameskip = value;
                case 'tmax'
                    tmax = value;
                case 'aviname'
                    aviname = value;
            end
        end
        
        [xf, tf, allx, allt, tair]= onestep(this, x0,'aviname',aviname,'interleaveAnimation',1,'interLeaveAnimationFrameSkip',frameskip,'tmax',tmax);
        [speed,steplength,stepfreq,airfrac] = getGaitChar(this,x0,tf,xf,tair);
        
    end
    
    function [xf,tf,allx,allt,tair,this,phasevec]= onestep(this, x0,varargin)
        %%
        RelTol = 1e-6; %10; %
        AbsTol = 1e-6; %10; %
        tmax = 5; %2; %6;
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
        
        phases = this.phases;
        
    

      %%
      this.getQandUdefs(x0);
      
      %Figure out where pelvis is using stance leg kinematics (this takes
      %care of initial conditions that aren't consistenent with stance
      %constraints).  This should ideally already have been done before
      %calling onestep.
      
      x0 = this.GoodInitialConditions(x0);
      
      points=this.getPoints(x0);
%       this.footangle = x0(5) - x0(6);
%       this.achilleslength = norm(points.pelvis - points.heel);
%       this.achillesangle = x0(3)-x0(6);
      
      landangle = x0(3);
      toeangle = x0(5);
      heelangle = x0(6);
      %% HeelToe Phase
      phase = phases{1};
      [x0,~] = this.phaseTransition(0,x0,phase);
      opts = odeset('Events', @(t,x) this.HeelOffOrFall(t,x),'RelTol',RelTol','AbsTol',AbsTol);
      [t,x] = ode45(@(t,x) this.XDoubleDot(t,x,phase),0:dt:tmax,x0,opts);
      
      allt = t; allx = x; phasevec = 1*ones(length(t),1);
      
      
      %% Toe Phase      
      phase = phases{2};
      [x0,~] = this.phaseTransition(allt(end),allx(end,:),phase);
      opts = odeset('Events', @(t,x) this.ToeOffOrFall(t,x),'RelTol',RelTol','AbsTol',AbsTol);
      [t,x] = ode45(@(t,x) this.XDoubleDot(t,x,phase),allt(end):dt:allt(end)+tmax,x0,opts);
      
      allt = [allt;t]; allx = [allx;x]; phasevec = [phasevec; 2*ones(length(t),1)];
      %% Aerial Phase
      tair = allt(end);
      phase = phases{3};
      [x0,~] = this.phaseTransition(allt(end),allx(end,:),phase);
      
      %Set Swing leg angle and length
      x0(3)=landangle;
      x0(4)=this.lleg;
      x0(5)=toeangle;
      x0(6)=heelangle;
      
      airoptions = odeset('Events', @(t,x) this.HeelStrikeOrFall(t,x,landangle),'RelTol',RelTol','AbsTol',AbsTol);
      
      [t,x] = ode45(@(t,x) this.XDoubleDot(t,x,phase),allt(end):dt:allt(end)+tmax,x0,airoptions);
      
      tf = t(end);
      allt = [allt; t];
      allx = [allx;x];
      phasevec = [phasevec; 3*ones(length(t),1)];
      %% Switch States for Next Step & Plotting
      
      %Make sure states are consistent with heel strike
      xf = switchLegs(this,allx(end,:),landangle);
      
      %Calculate states after collision
      [xf,impulses] = this.phaseTransition(tf,xf,'HeelToe');
      
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
    
    
    function [value, isTerminal, direction] = HeelOffOrFall(this,t,state)
        points = this.getPoints(state);
        
        %Lifting Off the Ground
        phase = 'HeelToe';
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
    
        function [value, isTerminal, direction] = ToeOffOrFall(this,t,state)
        points = this.getPoints(state);
        
        %Lifting Off the Ground
        phase = 'Toe';
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
        
        %Hitting the Ground
        swingfoot = points.heel(2);
        
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
       
%        newstatestruc = SpringFootRunnerState(x);
%        newstatestruc.stanceLeg.Angle = landangle;
%        newstatestruc.stanceLeg.Length = this.lleg;
       
       xnew = x;
 
    end
    
    function [speed] = getSpeed(this, x0, xf, tf)
        if isempty(xf)
            [xf,tf] = this.onestep(x0);
        end
        
        %Convert state vectors to descriptive class
        x0struc = SpringFootRunnerState(x0);
        xfstruc = SpringFootRunnerState(xf);
        
        speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
    end
    
    function [steplength] = getStepLength(this, x0, xf)
        if isempty(xf)
            [xf] = this.onestep(x0);
        end
        
        %Convert state vectors to descriptive class
        x0struc = SpringFootRunnerState(x0);
        xfstruc = SpringFootRunnerState(xf);
        
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
    
    function [MM,rhs] = getMMandRHS(this,time,state)
        %%
        this.getParams();
        this.getQandUdefs(state);
        
       c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
MM = zeros(6,6); rhs = zeros(6,1);

% Mass Matrix
MM(1,1) = mfoot + mheel + mpelvis + mtoe; MM(1,2) = 0; MM(1,3) = ...
-(q4*s3*(mfoot + mheel + mtoe)); MM(1,4) = c3*(mfoot + mheel + mtoe); MM(1,5) ...
= -(s5*ltoe*mtoe); MM(1,6) = -(s6*lheel*mheel); 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + mheel + mpelvis + mtoe; MM(2,3) = ...
c3*q4*(mfoot + mheel + mtoe); MM(2,4) = s3*(mfoot + mheel + mtoe); MM(2,5) = ...
c5*ltoe*mtoe; MM(2,6) = c6*lheel*mheel; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = (mfoot + mheel + ...
mtoe)*(q4*q4); MM(3,4) = 0; MM(3,5) = c3m5*q4*ltoe*mtoe; MM(3,6) = ...
c3m6*q4*lheel*mheel; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot + ...
mheel + mtoe; MM(4,5) = s3m5*ltoe*mtoe; MM(4,6) = s3m6*lheel*mheel; 
MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
MM(5,5) = mtoe*(ltoe*ltoe); MM(5,6) = 0; 
MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
MM(6,5) = MM(5,6); MM(6,6) = mheel*(lheel*lheel); 

% righthand side terms
rhs(1) = 2*u4*(c3*cleg + s3*u3*(mfoot + mheel + mtoe)) + c3*q4*(mfoot + mheel ...
+ mtoe)*(u3*u3) + c5*ltoe*mtoe*(u5*u5) + c6*lheel*mheel*(u6*u6) + ...
g*mfoot*sin(gslope) + g*mheel*sin(gslope) + g*mpelvis*sin(gslope) + ...
g*mtoe*sin(gslope); 
rhs(2) = 2*u4*(s3*cleg - c3*u3*(mfoot + mheel + mtoe)) - g*mfoot*cos(gslope) ...
- g*mheel*cos(gslope) - g*mpelvis*cos(gslope) - g*mtoe*cos(gslope) + ...
q4*s3*(mfoot + mheel + mtoe)*(u3*u3) + s5*ltoe*mtoe*(u5*u5) + ...
s6*lheel*mheel*(u6*u6); 
rhs(3) = -(u3*cachilles) + u6*cachilles - q3*kachilles + q6*kachilles + ...
achillesangle*kachilles - q4*(2*u3*u4*(mfoot + mheel + mtoe) + g*mfoot*cos(q3 ...
- gslope) + g*mheel*cos(q3 - gslope) + g*mtoe*cos(q3 - gslope) + ...
s3m5*ltoe*mtoe*(u5*u5) + s3m6*lheel*mheel*(u6*u6)); 
rhs(4) = u4*cleg + kleg*lleg + q4*(-kleg + (mfoot + mheel + mtoe)*(u3*u3)) + ...
c3m5*ltoe*mtoe*(u5*u5) + c3m6*lheel*mheel*(u6*u6) - g*mfoot*sin(q3 - gslope) ...
- g*mheel*sin(q3 - gslope) - g*mtoe*sin(q3 - gslope); 
rhs(5) = -(u5*cfoot) + u6*cfoot - q5*kfoot + q6*kfoot + footangle*kfoot - ...
2*c3m5*u3*u4*ltoe*mtoe - g*ltoe*mtoe*cos(q5 - gslope) + ...
q4*s3m5*ltoe*mtoe*(u3*u3); 
rhs(6) = (u3 - u6)*cachilles + (u5 - u6)*cfoot - (-q3 + q6 + ...
achillesangle)*kachilles - (-q5 + q6 + footangle)*kfoot - ...
2*c3m6*u3*u4*lheel*mheel - g*lheel*mheel*cos(q6 - gslope) + ...
q4*s3m6*lheel*mheel*(u3*u3); 

        
    end
    
    function [C, CDot] = getConstraints(this,state,phase)
        %%
        this.getParams();
        this.getQandUdefs(state);
        
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
        switch phase      
                
            case 'HeelToe'
constraintJacobianHeelToe(1,1) = 1; constraintJacobianHeelToe(1,2) = 0; ...
constraintJacobianHeelToe(1,3) = -(q4*s3); constraintJacobianHeelToe(1,4) = ...
c3; constraintJacobianHeelToe(1,5) = 0; constraintJacobianHeelToe(1,6) = ...
-(s6*lheel); 
constraintJacobianHeelToe(2,1) = 0; constraintJacobianHeelToe(2,2) = 1; ...
constraintJacobianHeelToe(2,3) = c3*q4; constraintJacobianHeelToe(2,4) = s3; ...
constraintJacobianHeelToe(2,5) = 0; constraintJacobianHeelToe(2,6) = ...
c6*lheel; 
constraintJacobianHeelToe(3,1) = 0; constraintJacobianHeelToe(3,2) = 1; ...
constraintJacobianHeelToe(3,3) = c3*q4; constraintJacobianHeelToe(3,4) = s3; ...
constraintJacobianHeelToe(3,5) = c5*ltoe; constraintJacobianHeelToe(3,6) = 0; 


constraintJacobianHeelToeDot(1,1) = 0; constraintJacobianHeelToeDot(1,2) = 0; ...
constraintJacobianHeelToeDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianHeelToeDot(1,4) = -(s3*u3); ...
constraintJacobianHeelToeDot(1,5) = 0; constraintJacobianHeelToeDot(1,6) = ...
-(c6*u6*lheel); 
constraintJacobianHeelToeDot(2,1) = 0; constraintJacobianHeelToeDot(2,2) = 0; ...
constraintJacobianHeelToeDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianHeelToeDot(2,4) = c3*u3; constraintJacobianHeelToeDot(2,5) ...
= 0; constraintJacobianHeelToeDot(2,6) = -(s6*u6*lheel); 
constraintJacobianHeelToeDot(3,1) = 0; constraintJacobianHeelToeDot(3,2) = 0; ...
constraintJacobianHeelToeDot(3,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianHeelToeDot(3,4) = c3*u3; constraintJacobianHeelToeDot(3,5) ...
= -(s5*u5*ltoe); constraintJacobianHeelToeDot(3,6) = 0; 

C = constraintJacobianHeelToe;
CDot = constraintJacobianHeelToeDot;

%%%%%%%%%%%%%%%%%%% Let both heel & toe slide in x direction
% C=C(2:3,:);
% CDot=CDot(2:3,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case 'Toe'

constraintJacobianToe(1,1) = 1; constraintJacobianToe(1,2) = 0; ...
constraintJacobianToe(1,3) = -(q4*s3); constraintJacobianToe(1,4) = c3; ...
constraintJacobianToe(1,5) = -(s5*ltoe); constraintJacobianToe(1,6) = 0; 
constraintJacobianToe(2,1) = 0; constraintJacobianToe(2,2) = 1; ...
constraintJacobianToe(2,3) = c3*q4; constraintJacobianToe(2,4) = s3; ...
constraintJacobianToe(2,5) = c5*ltoe; constraintJacobianToe(2,6) = 0; 


constraintJacobianToeDot(1,1) = 0; constraintJacobianToeDot(1,2) = 0; ...
constraintJacobianToeDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianToeDot(1,4) = -(s3*u3); constraintJacobianToeDot(1,5) = ...
-(c5*u5*ltoe); constraintJacobianToeDot(1,6) = 0; 
constraintJacobianToeDot(2,1) = 0; constraintJacobianToeDot(2,2) = 0; ...
constraintJacobianToeDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianToeDot(2,4) = c3*u3; constraintJacobianToeDot(2,5) = ...
-(s5*u5*ltoe); constraintJacobianToeDot(2,6) = 0; 


                C = constraintJacobianToe;
                CDot = constraintJacobianToeDot;
                
            case 'Aerial'
constraintJacobianAerial(1,1) = 0; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 1; constraintJacobianAerial(1,4) = 0; ...
constraintJacobianAerial(1,5) = 0; constraintJacobianAerial(1,6) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 0; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 1; ...
constraintJacobianAerial(2,5) = 0; constraintJacobianAerial(2,6) = 0; 
constraintJacobianAerial(3,1) = 0; constraintJacobianAerial(3,2) = 0; ...
constraintJacobianAerial(3,3) = 0; constraintJacobianAerial(3,4) = 0; ...
constraintJacobianAerial(3,5) = 1; constraintJacobianAerial(3,6) = 0; 
constraintJacobianAerial(4,1) = 0; constraintJacobianAerial(4,2) = 0; ...
constraintJacobianAerial(4,3) = 0; constraintJacobianAerial(4,4) = 0; ...
constraintJacobianAerial(4,5) = 0; constraintJacobianAerial(4,6) = 1; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; ...
constraintJacobianAerialDot(1,5) = 0; constraintJacobianAerialDot(1,6) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; ...
constraintJacobianAerialDot(2,5) = 0; constraintJacobianAerialDot(2,6) = 0; 
constraintJacobianAerialDot(3,1) = 0; constraintJacobianAerialDot(3,2) = 0; ...
constraintJacobianAerialDot(3,3) = 0; constraintJacobianAerialDot(3,4) = 0; ...
constraintJacobianAerialDot(3,5) = 0; constraintJacobianAerialDot(3,6) = 0; 
constraintJacobianAerialDot(4,1) = 0; constraintJacobianAerialDot(4,2) = 0; ...
constraintJacobianAerialDot(4,3) = 0; constraintJacobianAerialDot(4,4) = 0; ...
constraintJacobianAerialDot(4,5) = 0; constraintJacobianAerialDot(4,6) = 0; 

                C = constraintJacobianAerial;
                CDot = constraintJacobianAerialDot;
            otherwise
                error('Unknown phase for running model: %s', phase);
                
                
        end
        
    end
    
    function [J] = getVelJacob(this,state)
                this.getParams();
                this.getQandUdefs(state);
       c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
        
velJacobian(1,1) = 1; velJacobian(1,2) = 0; velJacobian(1,3) = 0; ...
velJacobian(1,4) = 0; velJacobian(1,5) = 0; velJacobian(1,6) = 0; 
velJacobian(2,1) = 0; velJacobian(2,2) = 1; velJacobian(2,3) = 0; ...
velJacobian(2,4) = 0; velJacobian(2,5) = 0; velJacobian(2,6) = 0; 
velJacobian(3,1) = 1; velJacobian(3,2) = 0; velJacobian(3,3) = -(q4*s3); ...
velJacobian(3,4) = c3; velJacobian(3,5) = 0; velJacobian(3,6) = 0; 
velJacobian(4,1) = 0; velJacobian(4,2) = 1; velJacobian(4,3) = c3*q4; ...
velJacobian(4,4) = s3; velJacobian(4,5) = 0; velJacobian(4,6) = 0; 
velJacobian(5,1) = 1; velJacobian(5,2) = 0; velJacobian(5,3) = -(q4*s3); ...
velJacobian(5,4) = c3; velJacobian(5,5) = 0; velJacobian(5,6) = -(s6*lheel); 
velJacobian(6,1) = 0; velJacobian(6,2) = 1; velJacobian(6,3) = c3*q4; ...
velJacobian(6,4) = s3; velJacobian(6,5) = 0; velJacobian(6,6) = c6*lheel; 
velJacobian(7,1) = 1; velJacobian(7,2) = 0; velJacobian(7,3) = -(q4*s3); ...
velJacobian(7,4) = c3; velJacobian(7,5) = -(s5*ltoe); velJacobian(7,6) = 0; 
velJacobian(8,1) = 0; velJacobian(8,2) = 1; velJacobian(8,3) = c3*q4; ...
velJacobian(8,4) = s3; velJacobian(8,5) = c5*ltoe; velJacobian(8,6) = 0; 


J=velJacobian;
    end
    function [E] = getEnergies(this,state)
        this.getParams();
        this.getQandUdefs(state);
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
kineticEnergy = (2*c3m6*q4*u3*u6*lheel*mheel + 2*s3m6*u4*u6*lheel*mheel + ...
2*c3m5*q4*u3*u5*ltoe*mtoe + 2*s3m5*u4*u5*ltoe*mtoe - 2*u1*(s6*u6*lheel*mheel ...
+ s5*u5*ltoe*mtoe + q4*s3*u3*(mfoot + mheel + mtoe) - c3*u4*(mfoot + mheel + ...
mtoe)) + 2*u2*(c6*u6*lheel*mheel + c5*u5*ltoe*mtoe + c3*q4*u3*(mfoot + mheel ...
+ mtoe) + s3*u4*(mfoot + mheel + mtoe)) + (mfoot + mheel + mpelvis + ...
mtoe)*(u1*u1) + (mfoot + mheel + mpelvis + mtoe)*(u2*u2) + ...
mfoot*(q4*q4)*(u3*u3) + mheel*(q4*q4)*(u3*u3) + mtoe*(q4*q4)*(u3*u3) + ...
mfoot*(u4*u4) + mheel*(u4*u4) + mtoe*(u4*u4) + mheel*(u6*u6)*(lheel*lheel) + ...
mtoe*(u5*u5)*(ltoe*ltoe))/2.;

potentialEnergy = (-2*q3*q6*kachilles - 2*q3*achillesangle*kachilles + ...
2*q6*achillesangle*kachilles - 2*q5*q6*kfoot - 2*q5*footangle*kfoot + ...
2*q6*footangle*kfoot - 2*q4*kleg*lleg + 2*q2*g*(mfoot + mheel + mpelvis + ...
mtoe)*cos(gslope) + kachilles*(q3*q3) + kleg*(q4*q4) + kfoot*(q5*q5) + ...
kachilles*(q6*q6) + kfoot*(q6*q6) + kachilles*(achillesangle*achillesangle) + ...
kfoot*(footangle*footangle) + kleg*(lleg*lleg) + 2*q4*g*mfoot*sin(q3 - ...
gslope) + 2*q4*g*mheel*sin(q3 - gslope) + 2*q4*g*mtoe*sin(q3 - gslope) + ...
2*g*ltoe*mtoe*sin(q5 - gslope) + 2*g*lheel*mheel*sin(q6 - gslope) - ...
2*q1*g*(mfoot + mheel + mpelvis + mtoe)*sin(gslope))/2.;

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope))) - ...
mfoot*(-(q2*g*cos(gslope)) - q4*g*sin(q3 - gslope) + q1*g*sin(gslope)) - ...
mtoe*(-(q2*g*cos(gslope)) - q4*s3*g*cos(gslope) - g*ltoe*sin(q5 - gslope) + ...
q1*g*sin(gslope) + c3*q4*g*sin(gslope)) - mheel*(-(q2*g*cos(gslope)) - ...
q4*s3*g*cos(gslope) - g*lheel*sin(q6 - gslope) + q1*g*sin(gslope) + ...
c3*q4*g*sin(gslope));

PEspring = (kachilles*((q3 - q6 - achillesangle)*(q3 - q6 - ...
achillesangle)))/2. + (kfoot*((q5 - q6 - footangle)*(q5 - q6 - ...
footangle)))/2. + (kleg*((q4 - lleg)*(q4 - lleg)))/2.;
        
        E.KE = kineticEnergy;
        E.PE = potentialEnergy;
        E.PEgrav = PEgrav;
        E.PEspring = PEspring;
        E.Total = E.KE + E.PE;
        
    end
    
    function [points] = getPoints(this, state)
        
        this.getParams();
        this.getQandUdefs(state);
        
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
points.foot(1) = q1 + c3*q4; 
points.foot(2) = q2 + q4*s3; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COMpos(1) = c6*lheel*mheel*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
c5*ltoe*mtoe*power(mfoot + mheel + mpelvis + mtoe,-1) + (q1*mfoot + q1*mheel ...
+ q1*mpelvis + q1*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
c3*(q4*mfoot + q4*mheel + q4*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1); 
points.COMpos(2) = s6*lheel*mheel*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
s5*ltoe*mtoe*power(mfoot + mheel + mpelvis + mtoe,-1) + (q2*mfoot + q2*mheel ...
+ q2*mpelvis + q2*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
s3*(q4*mfoot + q4*mheel + q4*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1); 


points.heel(1) = q1 + c3*q4 + c6*lheel; 
points.heel(2) = q2 + q4*s3 + s6*lheel; 


points.toe(1) = q1 + c3*q4 + c5*ltoe; 
points.toe(2) = q2 + q4*s3 + s5*ltoe; 
    end
    
    function [newstate] = GoodInitialConditions(this,x0)
        this.getQandUdefs(x0);
        
        %Set state velocities to be consistent with constraints while being
        %least squares close to inputted velocities
        Jc = this.getConstraints(x0,'HeelToe');
        nullJc = null(Jc);
        unew = nullJc * (nullJc \ x0(7:12)');
        newstate = [x0(1:6) unew'];
        
        %Shift body (without changing configuration) so that heel is on ground
        points = this.getPoints(newstate);
        pelvx = points.pelvis(1) - points.heel(1);
        pelvy = points.pelvis(2) - points.heel(2);
        newstate([1 2])=[pelvx pelvy];
        
        %Put toe on ground
        points = this.getPoints(newstate);
        toeang = asin(-points.foot(2)/this.ltoe);
        newstate(5) = toeang;
        
        %       u = newstate(7:12)';
        %       Jc*u; %should be zero if constraints are working
        
    end
    
    function [C,Ceq] = MassesAddToOne(this,x0,xf,tf,allx,allt,tair)
        r = this;
        C=[];
        Ceq = 1 - r.mpelvis - r.mfoot - r.mheel - r.mtoe;
    end
    
    
    function [GRF] = getGRF(this,x,phase)
        
        if size(x,1)==1
           x=x'; 
        end
        [xddot, constraintForces] = this.XDoubleDot(0,x,phase);
        
        if strcmp(phase,'Heel') || strcmp(phase,'Toe')
        GRF = [constraintForces(1);constraintForces(2)];
        elseif strcmp(phase,'HeelToe')
        GRF = [constraintForces(1);constraintForces(2) + constraintForces(3)];    
        else
           GRF=[0;0]; 
        end
    end
    
    function [comWR] = getcomWR(this,x,phase)
        this.getParams();
        this.getQandUdefs(x);
        
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
comvx = (c3*u4*mfoot + c3*u4*mheel - s6*u6*lheel*mheel + c3*u4*mtoe - ...
s5*u5*ltoe*mtoe - q4*s3*u3*(mfoot + mheel + mtoe) + u1*(mfoot + mheel + ...
mpelvis + mtoe))*power(mfoot + mheel + mpelvis + mtoe,-1);

comvy = (s3*u4*mfoot + s3*u4*mheel + c6*u6*lheel*mheel + s3*u4*mtoe + ...
c5*u5*ltoe*mtoe + c3*q4*u3*(mfoot + mheel + mtoe) + u2*(mfoot + mheel + ...
mpelvis + mtoe))*power(mfoot + mheel + mpelvis + mtoe,-1);

        
        GRF = this.getGRF(x,phase);
        
        comWR = dot(GRF,[comvx;comvy]);
    end
    
    function footPower = getFootPower(this,x)
        sz = SpringFootRunnerState(x);
        force = this.getFootForce(x);
        velocity = sz.toe.AngleDot - sz.heel.AngleDot;
        footPower = force.*velocity;
    end
    
    function footforce = getFootForce(this,x)
        sz = SpringFootRunnerState(x);
        footforce = -this.kfoot*(sz.toe.Angle - sz.heel.Angle - this.footangle) ...
            -this.cfoot*(sz.toe.AngleDot - sz.heel.AngleDot);
    end
    
    function legPower = getLegPower(this,x)
        sz = SpringFootRunnerState(x);
        force = this.getLegForce(x);
        vel = sz.stanceLeg.LengthDot;
        legPower = force.*vel;
    end
    
    function legforce = getLegForce(this,x)
        sz = SpringFootRunnerState(x);
        legforce = -this.kleg*(sz.stanceLeg.Length - this.lleg) - this.cleg*(sz.stanceLeg.LengthDot);
    end
    
    function achillesPower = getAchillesPower(this,x)
        sz = SpringFootRunnerState(x);
        force = this.getAchillesForce(x);
        velocity = sz.stanceLeg.AngleDot - sz.heel.AngleDot;
        achillesPower = force.*velocity;
    end
    
    function achillesforce = getAchillesForce(this,x)
        sz = SpringFootRunnerState(x);
        achillesforce = -this.kachilles*(sz.stanceLeg.Angle - sz.heel.Angle - this.achillesangle) ...
                        -this.cachilles*(sz.stanceLeg.AngleDot - sz.heel.AngleDot);
    end
    
    
end
    
    
end
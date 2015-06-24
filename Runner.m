classdef (Abstract) Runner
    %Generic running model with basic functions to take derivatives,
    %integrate dynamics, implement collisions, and more.  You need
    %concrete subclasses with more specific information about a particular model to
    %use this class.
    
    properties (Abstract)
        gslope %slope of the ground
        g %force of gravity
        N %number of states
        statestovary %an array of numbers corresponding to which elements of the state vector to vary when finding a limit cycle
        statestomeasure %an array of numbers corresponding to which elements of the state vector to measure when finding a limit cycle
    end
    
methods
    
    function [this] = Runner() 
       dir=cd;
       addpath([dir '\State_Definitions']);
    end
    
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
      runcharic.speed=[]; %The speed the model should run at
      runcharic.steplength=[]; %The step length the model should have
      runcharic.airfrac=[];  %From 0 to 1, the ratio of time spent in air
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
              case 'runcharic'
                  runcharic = value;
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
      
      constraintFunction = @(x) this.ConstFcn(x,runcharic,parametersToAlter,additionalConstraintFunction,initialConditionGuess,addedCostFcn);
      objectiveFunction = @(x) this.CostFcn(x,runcharic,parametersToAlter,additionalConstraintFunction,initialConditionGuess,addedCostFcn);
      
      
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
      if isempty(LB)
          for i = 1:length(parametersToAlter)
              if strcmp(parametersToAlter{i}(1),'c') ||  strcmp(parametersToAlter{i}(1),'l') || strcmp(parametersToAlter{i}(1),'k')
                  LB(length(this.statestovary)+i) = 0;
                  
              elseif  strcmp(parametersToAlter{i}(1),'m')
                  LB(length(this.statestovary)+i) = minmass;
                  
              elseif regexpi(parametersToAlter{i},'transition')
                  LB(length(this.statestovary)+i) = .8;
                  UB(length(this.statestovary)+i) = 1;
                  %          elseif regexpi(parametersToAlter{i},'impulsecoeff')
                  %              LB(length(this.statestovary)+i) = 0;
              end
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
      
      constraintFunction = @(x) this.ConstFcn(x,runcharic,parametersToAlter,additionalConstraintFunction,initialConditionGuess,addedCostFcn);
      [c, ceq, limitCycleError] = constraintFunction(finalOptimizerState);
      
      
      finalStates = initialConditionGuess;
      finalStates(this.statestovary) = finalVariedStates;
      finalStates = this.GoodInitialConditions(finalStates);
      
      newrunner = this;
      newrunner.printStepCharacteristics(finalStates);
    end
    
    function [cost] = CostFcn(this,x,runcharic,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost)
       global xLast myf myc myceq myError
       
       if ~isequal(x,xLast)
           [myc, myceq, myError, cExtra, ceqExtra, myf] = fixedPointConstraint(this,x,runcharic,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost);
           xLast = x;
       end
       
       cost = myf;
    end
    
    function [c, ceq, limitCycleError, cExtra, ceqExtra, cost] = ConstFcn(this,x,runcharic,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost)
        global xLast myf myc myceq myError
        
        if ~isequal(x,xLast)
            [myc, myceq, myError, cExtra, ceqExtra, myf] = fixedPointConstraint(this,x,runcharic,parametersToAlter,additionalConstraintFunction,initialConditionGuess,additionalCost);
            xLast = x;
        end
        
        c = myc;
        ceq = myceq;
        limitCycleError = myError;
        
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
        
        limitCycleError = xf(this.statestomeasure) - x0(this.statestomeasure)';
        
        c=[];
        ceq=limitCycleError;

        if ~isempty(runcharic.speed)
            [speed] = getSpeed(this, x0, xf, tf);
            ceq = [ceq; runcharic.speed - speed];
        end
        if ~isempty(runcharic.steplength)
            [steplength] = getStepLength(this, x0, xf);
            ceq = [ceq; runcharic.steplength - steplength];
        end
        if ~isempty(runcharic.airfrac)
            [airfrac] = getAerialFraction(this, x0, tf, tair);
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
    
    function [xddot, constraintForces] = XDoubleDot(this,time,x,phase)
        %% 
        %Phase specifies what equations to use, EG 'aerial' or 'stance'
        
        [MM,rhs] = this.getMMandRHS(time,x);
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
%        if strcmp(phase,'stance')
%        constraintForces = constraintForces - [0;rhs(7)]; 
%        end
        %Check Constraints, these should be equal to zero if constraints
        %are working.  For debugging.
        
%         constraintacc = Jc*accs +Jcdot*u;
%         constraintvel = Jc*u;
        
    end
    
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
        
        phase = phaseToSwitchTo;
        
        [MM,rhs] = this.getMMandRHS(time,x);
        [Jc,Jcdot] = this.getConstraints(x,phase);
        
        [d1,d2] = size(Jc);
        u = x(this.N/2+1:end);
        

        
        MMbig = [MM Jc'; Jc zeros(d1)];
        RHSbig = [MM*u;zeros(d1,1)];
        
        VelsAndImpulses = MMbig \ RHSbig;
        xNew = [x(1:this.N/2);VelsAndImpulses(1:this.N/2)];
        
        xNew = this.positionSwitches(xNew);
        
        Impulse = -VelsAndImpulses(this.N/2+1:end);
        
        if (sum(isnan(xNew)))
            xNew
            phaseToSwitchTo
            error('xNew cannot have any NaNs!')
        end
        
    end
    
    function xSwitched = positionSwitches(this,x)
        %Used to switch the positional states of the system during a phase
        %transition. In this superclass, this will return the same states
        %as input.  If you want to implement position switching during
        %phase transitions, write a positionSwitches method for the
        %subclass of interest
        xSwitched = x;
    end
    
    function [stepEndStates] = manystep(this, x0, N, varargin)
        %%
        
        %Take multiple steps using onestep
      interleaveAnimation = 0;
      doneFunction = @(x) 0;
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'interleaveAnimation'
            interleaveAnimation = value;
          case 'doneFunction'
            doneFunction = value;
        end
      end
      
      states = x0;
      
      [nextState, finalTime, allStates, allTimes, tair, this] = this.onestep(x0, 'interleaveAnimation', interleaveAnimation);
      states = [states; nextState'];
      
      for i = 1 : N-1
        %         [nextState] =
        [nextState, finalTime, allStates, allTimes, tair, this] = this.onestep(nextState, 'interleaveAnimation', interleaveAnimation);
        states = [states; nextState'];
        if (doneFunction(nextState))
          break;
        end

      end
      stepEndStates = states;
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
        

        limitCycleError = reshape(xf(this.statestomeasure),length(this.statestomeasure),1) - ...
                         reshape(x0(this.statestomeasure),length(this.statestomeasure),1) ;
        
            [speed] = getSpeed(this, x0, xf, tf);
            [steplength] = getStepLength(this, x0, xf);
            [airfrac] = getAerialFraction(this, x0, tf, tair);
        
        fprintf('step parameters: speed: %g, step length = %g, \n airfrac = %g, limitCycleError = %g\n', speed, steplength, airfrac, norm(limitCycleError));
    end
    
    function [] = getParams(this)
        %%
        % Add the parameter variables dynamically to the work space of the
        % function that calls this function.
        ws = 'caller';
        
        parmnames = fieldnames(this);
        for i = 1:length(parmnames)
%             pstring = sprintf(['assignin(ws,''' parmnames{i} ''', %g);'],this.(parmnames{i}));
%             eval(pstring);

assignin(ws,parmnames{i},this.(parmnames{i}));
        end
        
    end
    
    function [] = getQandUdefs(this,x)
        %%
        % Add the values for x1,x2,...,xN,u1,u2,...,uN to the work space of the
        % function that calls this function.
        ws = 'caller';
        
        numstates = this.N/2;
        for i = 1:numstates
            qstring = sprintf('assignin(ws,''q%g'',x(%g));',[i i]);
            eval(qstring);
            ustring = sprintf('assignin(ws,''u%g'',x(%g));',[i i+numstates]);
            eval(ustring);
        end
        
        if length(x)>this.N % Means constraint forces are being tracked as states(hopefully)
           for i = this.N+1:length(x)
              lstring = sprintf('assignin(ws,''l%g'',x(%g));',[i-this.N i]);
              eval(lstring);
           end
        end
        
    end
    
    function [parameterValues] = getParametersFromList(this, parameterNames)
        %%
        parameterValues = zeros(length(parameterNames), 1);
        for i = 1 : length(parameterNames)
            parameterValues(i) = this.(parameterNames{i});
        end
    end
    
    function [states, parameterValues] = separateStatesAndParameters(this, statesAndParams, parameterNames)
        %%
        numParams = length(parameterNames);
        parameterValues = statesAndParams((end-(numParams-1)) : end);
        states = statesAndParams(1 : (end - numParams));
        %         this = this.setParameterValues(parameterNames, parameterValues);
    end
    
    function [this] = setParametersFromList(this, parameterNames, parameterValues)
        %%
        for i = 1 : length(parameterNames)
            this.(parameterNames{i}) = parameterValues(i);
        end
    end
    
%     function [speed] = getSpeed(this, x0, xf, tf)
%         if isempty(xf)
%             [xf,tf] = this.onestep(x0);
%         end
%         
%         %Convert state vectors to descriptive class
%         
%         classname = class(this);
%         
%         eval(['x0struc = ' classname 'State(x0);']);
%         eval(['xfstruc = ' classname 'State(xf);']);
%         
%         speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
%     end
    
    
end
    
    
end
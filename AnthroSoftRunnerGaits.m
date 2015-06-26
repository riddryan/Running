%% Description of what this file does

%Use this script to mess around with what model parameters and IC to feed
%to an optimizer in ordere to find a gait that has characteristics similar
%to actual human running in terms of speed (dsp), step length (dsl), step
%frequency (dsf), and ratio of air-time to ground-time (daf).

% What you need to do each time you run this:
% 1. Pick what model(s) to use by filling in the array cellstouse with the
% integers corresponding to each model you want.

% 2. Decide if you want to save the result, and if you do, go to each cell
% for each model you're running, and give it the savename that you want

% 2.  For each model you are finding a limit cycle for, go to the
% corresponding cell and set the IC you want to use, and what model
% parameters you want the optimizer to have access to in a cell-list
% variable parmstovary


%% Initialization & Options
useguess=1; %set to 1 to attempt to find limit cycle using user-set initial conditions (within each cell).  
%Otherwise, try to load an existing limit cycle to start from
usenominal = 1; %Use subject 7 trial 2.  If 0, will use average running characteristics from all trials
savegait=1; %1 for saving the limit cycles.  Make sure you set the name of the save files
%within each cell that you want, each time you run it
dir = cd;
savepath = [dir '\SavedGaits\'];

% Which models are available
% 1. Soft Stomach
% 2. Soft Stomach Series
% 3. Soft Stomach Series Parallel

cellstouse=[14];

%Optimizer Constraint Tolerance
constrainttolerance = 1e-5;

useairfrac=1;
usespeed=1;
usesteplength=1;
% usestepfreq=1;
%% Set desired speed, step length, aerial fracion time
%From human running data
dsp=1.0115;
dsl=1.1371;
dsf=0.8864;
daf=.2165;

if usenominal
[datasoft,dsp,dsl,daf,dsf,datagrf] = getHumanData(7, 2);
else
 dsp=1.0115;
dsl=1.1371;
dsf=0.8864;
daf=.2165;   
end

runcharic=struct;
if usespeed==1
    runcharic.speed=dsp; else runcharic.speed=[];
end
if usesteplength==1
    runcharic.steplength=dsl; else runcharic.steplength=[];
end

if useairfrac==1
    runcharic.airfrac=daf; else runcharic.airfrac=[];
end
% if usestepfreq==1
%     runcharic.stepfreq=dsf;
% end

%% Optimizations
if any(cellstouse==1) %Soft Stomach Model
    %%
    
    if useguess
        runner = SoftStomachRunner;
        %Initial Conditions
        IC = SoftStomachRunnerState;
        
        IC.stanceLeg.Angle = -1.1129;
        IC.stanceLeg.AngleDot = -.89584;
        
        IC.stanceLeg.Length = 1;
        IC.stanceLeg.LengthDot = -.562075;
        
        IC.softstomach.stretch = .2029542911;
        IC.softstomach.stretchDot = -.07;
        
        x0 = IC.getVector();
        
        %IMPORTANT: MAKE SURE OPTIMIZER INITIAL GUESS IS CONSISTENT WITH INITIAL
        %CONSTRAINTS ON MODEL
        x0 = runner.GoodInitialConditions(x0);
        
    else
        load([savepath 'SoftStomachRunner/' 'StomachNoDamping.mat']);
        runner=r;
        x0 = xstar;
    end

% Set parameters to vary during optimization
parmstovary=[{}];


% Find Limit cycle
[finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,'parametersToAlter',parmstovary,'TolCon',constrainttolerance);

newstomach = runner.setParametersFromList(parmstovary,finalParameters);
newstomachx0 = finalStates;

figure
newstomach.onestep(newstomachx0,'interleaveAnimation',1);

r=newstomach;
xstar = newstomachx0;

if savegait
    save([savepath 'SoftStomachRunner/' 'AnthroSoftStomach.mat'],'r','xstar');
end

end

if any(cellstouse==2) %Soft Series Stomach Model
    %%
    if useguess
        runner = SoftStomachSeriesRunner;
        
        IC = SoftStomachSeriesRunnerState;
        
        
        IC.stanceLeg.Angle = -1.1129;
        IC.stanceLeg.AngleDot = -.9;
        
        IC.stanceLeg.Length = runner.lleg;
        IC.stanceLeg.LengthDot = 0;
        
        IC.softstomach.stretch = .2029542911;
        IC.softstomach.stretchDot = -.07;
        
        IC.softseries.stretch = runner.lsoftseries;
        IC.softseries.stretchDot = -.2;
        
        x0 = IC.getVector();
        
        %Make sure foot starts on ground
        x0 = runner.GoodInitialConditions(x0);
    else
        load([savepath 'SoftStomachSeriesRunner/' 'SeriesStomachDamping.mat'])
        runner=r;
        x0 = xstar;
    end
        
    
        runner.csoftseries=.3;
        runner.cleg=.3;
    
      
    
    

    
%     parmstovary=[{'gslope'} {'ksoftseries'} {'kleg'}];
%     parmstovary=[{}];
      parmstovary=[{'gslope'} {'ksoftseries'} {'kleg'}];
    
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,'parametersToAlter',parmstovary,'TolCon',constrainttolerance);
    
    newseriesstomach = runner.setParametersFromList(parmstovary,finalParameters);
    newseriesstomachx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair] = newseriesstomach.onestep(newseriesstomachx0,'interleaveAnimation',1);
    
    r=newseriesstomach;
    xstar = newseriesstomachx0;
    if savegait
        save([savepath 'SoftStomachSeriesRunner/' 'SeriesStomachDamping.mat'],'r','xstar');
    end
            
end

if any(cellstouse==3) %Soft Series Parallel Stomach Model
    %%
    if useguess
        runner = SoftStomachSeriesParallelRunner;
        
            IC = SoftStomachSeriesParallelRunnerState;
            
            %             x0 = [-.8 1 .2 0 0 -.895 -.56 -.07 0 0];
            IC.stanceLeg.Angle = -1.32;
            IC.stanceLeg.AngleDot = -.65;
            
            IC.stanceLeg.Length = runner.lleg;
            IC.stanceLeg.LengthDot = 0;
            
            IC.softstomach.stretch = runner.lsoftstomach;
            IC.softstomach.stretchDot = 0;
            
            IC.softseries.stretch = runner.lsoftseries;
            IC.softseries.stretchDot = -.45;
            
            IC.softparallel.stretch = runner.lsoftparallel;
            IC.softparallel.stretchDot = -.3;
            
            x0 = IC.getVector();
            
            %Make sure foot starts on ground
            x0 = runner.GoodInitialConditions(x0);
    else
        load([savepath 'SoftStomachSeriesParallelRunner/' 'SeriesParallelStomachDamping4.mat'])
        runner=r;
        x0 = xstar;
    end
     
    runner.csoftparallel=.8;
%     parmstovary=[{'gslope'} {'ksoftseries'} {'kleg'}];
%     parmstovary=[{}];
        parmstovary = [{'gslope'} {'kleg'}];
%       parmstovary=[{'gslope'} {'ksoftseries'} {'kleg'} {'ksoftparallel'}];
    
%       addedconstraints = @(r,x0,xf,tf,allx,allt,tair) r.TakeoffConstraints(x0,xf,tf,allx,allt,tair);
    extracost = @(r,x0,xf,tf,allx,allt,tair) r.TakeoffCost(x0,xf,tf,allx,allt,tair);
      
    % Find Limit cycle
%     [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
%                                                                                   'parametersToAlter',parmstovary,...
%                                                                                   'TolCon',constrainttolerance,...
%                                                                                    'additionalConstraintFunction',addedconstraints);
    [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
                                                                                  'parametersToAlter',parmstovary,...
                                                                                  'TolCon',constrainttolerance,...
                                                                                   'Objective',extracost);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = newx0;
    if savegait
        save([savepath 'SoftStomachSeriesParallelRunner/' 'SPSAnthro1.mat'],'r','xstar');
    end
            
end

if any(cellstouse==4) %SSSP_vert
    if useguess
        runner = SSSP_vert;
        
        IC = SSSP_vertState;
        
        %             x0 = [-.8 1 .2 0 0 -.895 -.56 -.07 0 0];
        IC.stanceLeg.Angle = -1.32;
        IC.stanceLeg.AngleDot = -.65;
        
        IC.stanceLeg.Length = runner.lleg;
        IC.stanceLeg.LengthDot = 0;
        
        IC.softstomach.stretch = runner.lsoftstomach;
        IC.softstomach.stretchDot = 0;
        
        IC.softseries.stretch = runner.lsoftseries;
        IC.softseries.stretchDot = -.45;
        
        IC.softparallel.stretch = runner.lsoftparallel;
        IC.softparallel.stretchDot = -1;
        
        x0 = IC.getVector();
        
        runner.ksoftseries = 35;
        runner.csoftparallel = .4;
        
        %Make sure foot starts on ground
        x0 = runner.GoodInitialConditions(x0);
    else
                load([savepath 'SSSP_vert/' 'SeriesParallelStomachDamping3.mat'])
                runner=r;
                x0 = xstar;
    end
     
    runner.csoftparallel=.5;
%     parmstovary=[{'gslope'} {'ksoftseries'} {'kleg'}];
%     parmstovary=[{}];
%         parmstovary = [{'gslope'}];
      parmstovary=[{'gslope'} {'ksoftseries'} {'kleg'} {'ksoftparallel'} {'csoftparallel'}];
    
      addedconstraints = @(r,x0) r.additionalConstraints(x0);
      
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
                                                                                  'parametersToAlter',parmstovary,...
                                                                                  'TolCon',constrainttolerance,...
                                                                                   'additionalConstraintFunction',addedconstraints);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = newx0;
    if savegait
        save([savepath 'SSSP_vert/' 'SSSP_vert1.mat'],'r','xstar');
    end 
end
%%
if any(cellstouse==5) %Spring Foot
    if useguess
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
        runner.kachilles = 1;
        runner.mfoot = 0;
        runner.mheel = .1;
        runner.mtoe = .1;
        runner.footangle = x0(5) - x0(6);
        runner.achillesangle = x0(3) - x0(6);
    else
                load([savepath 'SpringFootRunner/' 'SeriesParallelStomachDamping3.mat'])
                runner=r;
                x0 = xstar;
    end
    
    
    parmstovary=[{'gslope'} {'kleg'} {'kachilles'} {'kfoot'} {'achillesangle'} {'footangle'} {'cfoot'} {'cachilles'}];
%       addedconstraints = @(r,x0) r.additionalConstraints(x0);
    addedconstraints=[];
      
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
                                                                                  'parametersToAlter',parmstovary,...
                                                                                  'TolCon',constrainttolerance,...
                                                                                   'additionalConstraintFunction',addedconstraints);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = newx0;
    if savegait
        save([savepath 'SpringFootRunner/' 'SpringFoot2.mat'],'r','xstar');
    end 
end

if any(cellstouse==6) %Soft Par
    %%
    if useguess
            runner = SoftParRunner;
            
            IC = SoftParRunnerState;
            
            IC.stanceLeg.Angle = -1.1105; 
            IC.stanceLeg.AngleDot = -1;
            
            IC.stanceLeg.Length = runner.lleg;
            IC.stanceLeg.LengthDot = -.35;
            
            IC.softparallel.stretch = runner.lsoftparallel;
            IC.softparallel.stretchDot = -.47;
            
            x0 = IC.getVector();
            
            runner.mpelvis=.9;
            runner.msoftparallel=.1;
            runner.csoftparallel = 0.5;
            runner.ksoftparallel = 18;
            runner.kleg = 12.9801;
            runner.cleg = 0.1;
            runner.gslope=0;
%             
            %Make sure foot starts on ground
            x0 = runner.GoodInitialConditions(x0);
    else
        load([savepath 'SoftParRunner/' 'SoftPar1.mat'])
        runner=r;
        x0 = xstar;
    end
    
        parmstovary=[{'gslope'} {'ksoftparallel'} {'kleg'}];
%     parmstovary=[{}];
%         parmstovary = [{'gslope'}];
%       parmstovary=[{'gslope'} {'ksoftseries'} {'kleg'} {'ksoftparallel'}];
    
      addedconstraints = @(r,x0,xf,tf,allx,allt,tair) r.TakeoffConstraints(x0,xf,tf,allx,allt,tair);
% addedconstraints = [];
%     extracost = @(r,x0,xf,tf,allx,allt,tair) r.TakeoffCost(x0,xf,tf,allx,allt,tair);
      extracost = [];
      
      
    % Find Limit cycle
%     [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
%                                                                                   'parametersToAlter',parmstovary,...
%                                                                                   'TolCon',constrainttolerance,...
%                                                                                    'additionalConstraintFunction',addedconstraints);
    [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
                                                                                  'parametersToAlter',parmstovary,...
                                                                                  'TolCon',constrainttolerance,...
                                                                                   'Objective',extracost,'additionalConstraintFunction',addedconstraints);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = newx0;
    if savegait
        save([savepath 'SoftParRunner/' 'SoftPar2.mat'],'r','xstar');
    end
            
end

if any(cellstouse==7) %NonlinParSoft Runner
    %%
    if useguess
            runner = NonlinParSoftRunner;
            
             IC = NonlinParSoftRunnerState;
            
            IC.stanceLeg.Angle = -1.1105; 
            IC.stanceLeg.AngleDot = -1;
            
            IC.stanceLeg.Length = runner.lleg;
            IC.stanceLeg.LengthDot = -.4;
            
            IC.softparallel.stretch = runner.lsoftparallel;
            IC.softparallel.stretchDot = -.4;
            
            x0 = IC.getVector();
            
            runner.mpelvis=.9;
            runner.msoftparallel=.1;
            runner.csoftparallel = 0.5;
%             runner.ksoftparallel = 18;
            runner.kleg = 18;
            
            runner.kimpact = 35;
            runner.kstance = 20;
            runner.transitionstart = .97;
            runner.transitionend = .96;
            
            runner.cleg = 0.1;
            runner.gslope=0;
%             
            %Make sure foot starts on ground
            x0 = runner.GoodInitialConditions(x0);
    else
        load([savepath 'NonlinParSoftRunner/' 'NonlinPar1.mat'])
        runner=r;
        x0 = xstar;
    end
    
%         parmstovary=[{'gslope'} {'kimpact'} {'kstance'} {'kleg'} {'csoftparallel'} {'transitionstart'} {'transitionend'}];
parmstovary=[{'gslope'} {'kimpact'} {'kstance'} {'kleg'} {'csoftparallel'}];
%     parmstovary=[{}];
%         parmstovary = [{'gslope'}];
%       parmstovary=[{'gslope'} {'ksoftseries'} {'kleg'} {'ksoftparallel'}];
    
      addedconstraints = @(r,x0,xf,tf,allx,allt,tair) r.TakeoffConstraints(x0,xf,tf,allx,allt,tair);
% addedconstraints = [];
%     extracost = @(r,x0,xf,tf,allx,allt,tair) r.TakeoffCost(x0,xf,tf,allx,allt,tair);
      extracost = [];
      
      
    % Find Limit cycle
%     [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
%                                                                                   'parametersToAlter',parmstovary,...
%                                                                                   'TolCon',constrainttolerance,...
%                                                                                    'additionalConstraintFunction',addedconstraints);
    [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
                                                                                  'parametersToAlter',parmstovary,...
                                                                                  'TolCon',constrainttolerance,...
                                                                                   'Objective',extracost,'additionalConstraintFunction',addedconstraints);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = newx0;
    if savegait
        save([savepath 'NonlinParSoftRunner/' 'NonlinPar3.mat'],'r','xstar');
    end
            
end
%% 
if any(cellstouse==8) %Foot Achilles Runner
    if useguess
        runner = FootAchillesRunner;
            
            IC = FootAchillesRunnerState;
            
%             x0 = [-.8 1 .2 0 0 -.895 -.56 -.07 0 0];
            IC.stanceLeg.Angle = -1.2;
            IC.stanceLeg.AngleDot = -1.6;
            
            IC.stanceLeg.Length = 1;
            IC.stanceLeg.LengthDot = -1;
            
            IC.toe.Angle = -30 /360*2*pi;
            IC.toe.AngleDot = 0.8;
            
            IC.heel.AngleDot = -.8;
            
            IC.heel.Angle = -130 /360*2*pi; 
            
            
            x0 = IC.getVector();
            
            %Make sure foot starts on ground
            x0 = runner.GoodInitialConditions(x0);
            
            runner.kleg = 20;
            runner.cfoot =0.0;
            runner.kachilles = 0.3;
            runner.mfoot = 0.1;
            runner.mheel = 0.1;
            runner.mtoe = .001;
            runner.kimpact = 0.005;
            runner.kstance = .3;
            runner.footangle = x0(5) - x0(6);
    else
                load([savepath 'FootAchillesRunner/' 'SeriesParallelStomachDamping3.mat'])
                runner=r;
                x0 = xstar;
    end
    
    
    parmstovary=[{'gslope'} {'kleg'} {'kachilles'} {'kimpact'} {'kstance'} {'footangle'} {'cfoot'} {'cachilles'}];
%       addedconstraints = @(r,x0) r.additionalConstraints(x0);
    addedconstraints=[];
      
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
                                                                                  'parametersToAlter',parmstovary,...
                                                                                  'TolCon',constrainttolerance,...
                                                                                   'additionalConstraintFunction',addedconstraints);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = newx0;
    if savegait
        save([savepath 'FootAchillesRunner/' 'FAR1.mat'],'r','xstar');
    end 
end
%%
if any(cellstouse==9) %Massless Achilles Foot Runner
    if useguess
            runner = MasslessAchillesRunner;
            
            IC = MasslessAchillesRunnerState;
            
            IC.pelvis.x = -.1715;
            IC.pelvis.y = 1.1363;
            IC.pelvis.xDot = 1.1;
            IC.pelvis.yDot = -0.15;
            
            IC.stanceLeg.Angle = -1.2967;
            IC.stanceLeg.Length = 1;
            IC.toe.Angle = -0.7425;
            IC.heel.Angle = -2.0899;
            
            x0 = IC.getVector();
            [x0,runner] = runner.GoodInitialConditions(x0);

            odex0 = x0([1 2 7 8]);
            %Make sure foot starts on ground
            
            runner.kleg = 40;
            runner.kachilles = 0.4048;
            runner.kimpact = 0.9870;
            runner.gslope = 0;
    else
                load([savepath 'MasslessAchillesRunner/' 'MAR1.mat'])
                runner=r;
                x0 = xstar;
    end
    
%     figure
%     runner.anim(x0);
    
    parmstovary=[{'gslope'} {'kleg'} {'kachilles'} {'kimpact'}];
%       addedconstraints = @(r,x0) r.additionalConstraints(x0);
    addedconstraints=[];
      
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
         runner.findLimitCycle(x0,'runcharic',runcharic,...
        'parametersToAlter',parmstovary,...
        'TolCon',constrainttolerance,...
        'additionalConstraintFunction',addedconstraints);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair,newr,phasevec] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = allx(1,:);
    if savegait
        save([savepath 'MasslessAchillesRunner/''MAR1.mat'],'r','xstar','parmstovary','limitCycleError',...
                                   'c','ceq','eflag','optimoutput','lambda',...
                                   'xf','tf','allx','allt','tair','phasevec');
    end 
end
%%
if any(cellstouse==10) %RetractSLIP Runner
    if useguess
        
            runner = RetractSLIP;
            IC = RetractSLIPState;
            
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
                    
                    
                    
            [x0,runner] = runner.GoodInitialConditions(x0);
            
%             addedconstraints = [];
    else
                load([savepath 'RetractSLIP/' 'LockSepHips.mat'],'r','xstar')
                runner=r;
                x0 = xstar;
                
%                 runner.rigidlegimpulse = 1;
%                 runner.impulsecoeff = 0;
%                 addedconstraints = @(r,varargin) r.SetPointConstraint(varargin);
%                 addedconstraints = @(r,varargin) r.PositiveImpulse(varargin);
                   addedconstraints = @(r,varargin) r.NoTanImpulse(varargin);
                
    end
    
%     figure
%     runner.anim(x0);
    
%     parmstovary=[{'kstance'} {'kswing'} {'khip'} {'swingl'} {'hipl'} {'gslope'} {'cswing'} {'chip'} {'cstance'}];
parmstovary=[{'khip'} {'hipl'}];
%       addedconstraints = @(r,x0) r.additionalConstraints(x0);
%     addedconstraints=[];
      addedconstraints = @(r,varargin) r.PositiveImpulse(varargin);
%       runner.statestomeasure = [5 6 11 12];
%       runner.statestovary = [5 6 11 12];
      runner.statestomeasure = [3 4];
      runner.statestovary = [5 6 11 12];
      Algorithm = 'interior-point';
      MaxEvals = 1800;
      
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
         runner.findLimitCycle(x0,'runcharic',runcharic,...
        'parametersToAlter',parmstovary,...
        'TolCon',constrainttolerance,...
        'additionalConstraintFunction',addedconstraints,'Algo',Algorithm,'MaxEvals',MaxEvals);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair,newr,phasevec] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = allx(1,:);
    x0 = xstar;
    r.printStepCharacteristics(x0,xf,tf,tair);
    if savegait
        save([savepath 'RetractSLIP/' 'OnlyMeasureSLIP.mat'],'r','xstar','parmstovary','limitCycleError',...
                                   'c','ceq','eflag','optimoutput','lambda',...
                                   'xf','tf','allx','allt','tair','phasevec');
    end 
end
%%
if any(cellstouse==11) %SLIP Runner
    runner = SLIP;
    IC = SLIPState;
    if useguess
        
        runner.kstance = 12.8734; %12
        IC.stancefoot.Angle = -1.1287;
        IC.stancefoot.Length = runner.stancel;
        %
        IC.pelvis.xDot = 1.0138;
        IC.pelvis.yDot = -0.1651;
        
        IC.stancefoot.AngleDot = -0.8457;
        IC.stancefoot.LengthDot = -0.5830;
        x0 = IC.getVector();
    else
                load([savepath '/SLIP' 'SLIP_NoAerial_unmatchedSL.mat'],'r','xstar')
                runner=r;
                x0 = xstar;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%
    runcharic.airfrac = 0;
    runcharic.steplength = [];
    %%%%%%%%%%%%%%%%%%%%%%%
    
parmstovary=[{'kstance'}];
%       addedconstraints = @(r,x0) r.additionalConstraints(x0);
    addedconstraints=[];
      
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
         runner.findLimitCycle(x0,'runcharic',runcharic,...
        'parametersToAlter',parmstovary,...
        'TolCon',constrainttolerance,...
        'additionalConstraintFunction',addedconstraints,'MaxEvals',1000);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair,newr,phasevec] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = allx(1,:);
    x0 = xstar;
    r.printStepCharacteristics(x0,xf,tf,tair);
    if savegait
        save([savepath '/SLIP' 'SLIP_NoAerial_unmatchedSL2.mat'],'r','xstar','parmstovary','limitCycleError',...
                                   'c','ceq','eflag','optimoutput','lambda',...
                                   'xf','tf','allx','allt','tair','phasevec');
    end 
end
%% Swing
if any(cellstouse==12) %Swing Runner
    runner = Swing;
    IC = SwingState;
    if useguess
                   load('./SavedGaits/SLIP/SLIP_NoAerial_unmatchedSL.mat','SLIPdata')
            runner.SLIPx0 = [SLIPdata(end,4);SLIPdata(end,5)];
            runner.SLIPxf = [SLIPdata(1,4);SLIPdata(1,5)];
            %add another step to the cycle
%             finaltime = SLIPdata(end,1);
%             finalx = SLIPdata(end,6)-SLIPdata(1,6);
%             steps = size(SLIPdata,1);
%             SLIPdata = [SLIPdata;SLIPdata(2:end,:);SLIPdata(2:end,:)];
%             SLIPdata(steps+1:end,1) = SLIPdata(steps+1:end,1) + finaltime;
%             SLIPdata(steps+1:end,6) = SLIPdata(steps+1:end,6) + finalx;
%             SLIPdata(2*steps:end,1) = SLIPdata(2*steps:end,1) + finaltime;
%             SLIPdata(2*steps:end,6) = SLIPdata(2*steps:end,6) + finalx;
runner.SLIPdata = SLIPdata;


runner.kswing = 0;
runner.khip = 0;

runner.gslope = 0;
runner.swingl = 0.95;
runner.hipl = -.65;

IC.swingfoot.Angle = runner.SLIPx0(1);
IC.swingfoot.Length = runner.SLIPx0(2);

IC.swingfoot.AngleDot = 1.05;
IC.swingfoot.LengthDot = -1;

x0 = IC.getVector();

runner.statestovary = [3 4];
        
        [x0,runner] = runner.GoodInitialConditions(x0);
    else
        load([savepath 'Swing/' 'Swing_yank1.mat'],'r','xstar')
        runner=r;
        x0 = xstar;
        
%         x0(3)=0;
%         xstar(3)=0;
        runner.statestovary = [];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%
    runcharic.airfrac = 0;
    runcharic.steplength = [];
    %%%%%%%%%%%%%%%%%%%%%%%
    
    parmstovary=[];
% parmstovary=[{'kswing'} {'khip'} {'hipl'}];
    %       addedconstraints = @(r,x0) r.additionalConstraints(x0);
    addedconstraints=[];
    addedcost = @(r,varargin) r.impulsecoeff^2;
%     addedcost = [];

    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
        runner.findLimitCycle(x0,'runcharic',runcharic,...
        'parametersToAlter',parmstovary,...
        'TolCon',constrainttolerance,'Objective',addedcost,...
        'additionalConstraintFunction',addedconstraints,'MaxEvals',1000);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair,newr,phasevec] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = allx(1,:);
    x0 = xstar;
    r.printStepCharacteristics(x0,xf,tf,tair);
    if savegait
        save([savepath 'Swing/' 'NoSprings.mat'],'r','xstar','parmstovary','limitCycleError',...
                                   'c','ceq','eflag','optimoutput','lambda',...
                                   'xf','tf','allx','allt','tair','phasevec');
    end 
end
%%
if any(cellstouse==13) %RetractKneeSwing Runner
    runner = RetractKneeSwing;
    IC = RetractKneeSwingState;
    if useguess

        SLIPfname = './SavedGaits/SLIP/SLIP_NoAerial_unmatchedSL.mat';
        [ runner.SLIPdata, runner.SLIPx0, runner.SLIPxf ] = getSLIPdata( SLIPfname );
                    
                    runner.phases = {'Aerial' 'KneeLock'};
                    runner.kneelock = 1;
                    runner.sephips = 0;
                    runner.mfoot = 0.5;
                    
                    runner.kknee = 3; %0.01
                    runner.khip = 7; %0.01
                    
                    runner.gslope = 0;
                    runner.kneel = 1;
                    runner.hipl = -0.5;
                    
                    IC.foot.Angle = runner.SLIPx0(1);
                    IC.knee.Angle = runner.SLIPx0(1);
                    
                    IC.foot.AngleDot = 0;
                    IC.knee.AngleDot = 0;
                    
                    x0 = IC.getVector();
        
        
%         runner.statestovary = [3 4];
        
       [x0,runner] = runner.GoodInitialConditions(x0);
    else
                load([savepath 'RetractKneeSwing/' 'SLIP_NoAerial_unmatchedSL.mat'],'r','xstar')
                runner=r;
                x0 = xstar;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%
    runcharic.airfrac = 0;
    runcharic.steplength = [];
    %%%%%%%%%%%%%%%%%%%%%%%
    
parmstovary=[{'kknee'} {'khip'} {'hipl'} {'kneel'}];
%       addedconstraints = @(r,x0) r.additionalConstraints(x0);
    addedconstraints=[];
    
    constrainttolerance = 1e-4;
      
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
         runner.findLimitCycle(x0,'runcharic',runcharic,...
        'parametersToAlter',parmstovary,...
        'TolCon',constrainttolerance,...
        'additionalConstraintFunction',addedconstraints,'MaxEvals',1000,'TolX',1e-10,...
        'Algo','interior-point');
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair,newr,phasevec] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = allx(1,:);
    x0 = xstar;
    r.printStepCharacteristics(x0,xf,tf,tair);
    if savegait
        save([savepath 'RetractKneeSwing/' 'KneeLockUnder.mat'],'r','xstar','parmstovary','limitCycleError',...
                                   'c','ceq','eflag','optimoutput','lambda',...
                                   'xf','tf','allx','allt','tair','phasevec');
    end 
end
%% SwingSLIP
if any(cellstouse==14) 
    
    useguess = 0;
    
    runner = SwingSLIP;
    IC = SwingSLIPState;
    if useguess
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
        
        
%         runner.statestovary = [3 4];
        
       [x0,runner] = runner.GoodInitialConditions(x0);
    else
                load([savepath 'SwingSLIP/' 'NoSwingSpringLock.mat'],'r','xstar')
                runner=r;
                x0 = xstar;
    end
    
    
parmstovary=[{'kswing'} {'khip'} {'hipl'} {'impulsecoeff'}];
%       addedconstraints = @(r,x0) r.additionalConstraints(x0);
%     addedconstraints= @(r,varargin) r.floorandswinglconstraint(varargin{:});
   addedconstraints= @(r,varargin) r.floorconstraint(varargin{:});
    runner.runcharic = runcharic;
%     Objective = @(r,varargin) r.kswing^2;
Objective = @(r,varargin) r.impulsecoeff^2;
%     runner.lockable = 0;
    
    %Use these if you already have a limit cycle on SLIP states
    runner.statestovary = [];
    runner.statestomeasure = [3 4];
    
    
    constrainttolerance = 1e-4;
      
    % Find Limit cycle
    [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
         runner.findLimitCycle(x0,...
        'parametersToAlter',parmstovary,...
        'TolCon',constrainttolerance,...
        'additionalConstraintFunction',addedconstraints,'MaxEvals',1000,'TolX',1e-10,...
        'Algo','interior-point','Objective',Objective);
    
    newr = runner.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair,newr,phasevec,tstance] = newr.onestep(newx0,'interleaveAnimation',1);
    
    r=newr;
    xstar = allx(1,:);
    x0 = xstar;
    r.print(x0,xf,tf,tair,tstance,allt,allx);
    if savegait
        save([savepath 'SwingSLIP/' 'MinImpulseLock.mat'],'r','xstar','parmstovary','limitCycleError',...
                                   'c','ceq','eflag','optimoutput','lambda',...
                                   'xf','tf','allx','allt','tair','phasevec');
    end 
end
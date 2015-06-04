function MatchHumanData
%% Initialization
close all
mdir = cd;
modelfolder = [mdir '\SavedGaits\'];
% modelname = [modelfolder 'NonLinPar3.mat'];
modelname = [modelfolder 'NonlinParSoftRunner_Subject7_Trial2_softmatch1.mat'];
savepath = modelfolder;

%Optimization Options
subject=7;
trial=2;
constrainttolerance = 1e-2;
usespeed=1;
usesteplength=1;
useairfrac=1;
savegait=1;
SoftObjective = 0;
GrfObjective = 1;
scalegrfs=1;
useguess = 0;

if SoftObjective && GrfObjective
   error('Can only have one objective function to minimize'); 
end

% parmstovary = [{'msoftparallel'} {'gslope'} {'ksoftparallel'} {'csoftparallel'} {'kleg'}];
parmstovary=[{'gslope'} {'kimpact'} {'kstance'} {'kleg'} {'cleg'} {'csoftparallel'} {'transitionstart'} {'transitionend'} {'msoftparallel'} {'mpelvis'}];
% parmstovary=[{'gslope'} {'kimpact'} {'kstance'} {'kleg'}];
% parmstovary=[{'gslope'} {'kimpact'} {'kstance'} {'kleg'} {'csoftparallel'} {'cleg'} {'msoftparallel'} {'mpelvis'}];
% parmstovary=[{'gslope'} {'kimpact'} {'kstance'} {'kleg'} {'csoftparallel'}];
% parmstovary = [{'msoftstomach'} {'msoftseries'} {'msoftparallel'} {'gslope'} ...
%                     {'ksoftseries'} {'kleg'} {'ksoftstomach'} {'ksoftparallel'}...
%                     {'csoftstomach'} {'cleg'} {'csoftseries'} {'csoftparallel'}];
% parmstovary = [ {'gslope'} ...
%                     {'ksoftseries'} {'kleg'} {'ksoftparallel'}];
 
%Human data options
[datasoft,dsp,dsl,daf,dsf,datagrf] = getHumanData( subject, trial);

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

%% Optimization

if ~useguess
    load(modelname,'xstar','r');
    x0=xstar;
    runner=r;
else
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
    r = runner;
end

optimizerGuess = x0(runner.statestovary);
if (~isempty(parmstovary))
    optimizerGuess = [optimizerGuess runner.getParametersFromList(parmstovary)'];
end

if SoftObjective
    extracost = @(runner,x0,xf,tf,allx,allt,tair,phasevec) MatchSoft(runner,allx,allt,tair,datasoft,phasevec);
elseif GrfObjective
    if ~scalegrfs
        extracost = @(runner,x0,xf,tf,allx,allt,tair,phasevec) MatchGRF(runner,allx,allt,tair,datagrf,phasevec);
    else
        extracost  = @(runner,x0,xf,tf,allx,allt,tair,phasevec) MatchScaledGRF(runner,allx,allt,tair,datagrf,phasevec);
    end
else
    extracost = [];
end
extraconstraint = @(runner,x0,xf,tf,allx,allt,tair) MassesAddToOne(runner); %ensures total mass = 1
% if isa(runner,'SoftParRunner') %At takeoff, parallel soft tissue is doing no work
%     partakeoffconstraint = @(r,x0,xf,tf,allx,allt,tair) runner.TakeoffConstraints(x0,xf,tf,allx,allt,tair);
%     extraconstraint = {extraconstraint,partakeoffconstraint};
% end
% if isa(runner,'NonlinParSoftRunner') %At takeoff, parallel soft tissue is doing no work
%     nonlinspringconstraints = @(r,x0,xf,tf,allx,allt,tair) r.nonlinspringconstraints;
%     extraconstraint = {extraconstraint, nonlinspringconstraints};
% end

[finalStates, finalParameters, limitCycleError, c, ceq]=runner.findLimitCycle(x0,'runcharic',runcharic,...
    'parametersToAlter',parmstovary,...
    'TolCon',constrainttolerance,...
    'Objective',extracost,'additionalConstraintFunction',extraconstraint);

    r = runner.setParametersFromList(parmstovary,finalParameters);
    xstar = finalStates;

    if savegait
        if SoftObjective
            savename = ([savepath class(r) sprintf('_Subject%d_Trial%d_softmatch',[subject trial])]);
        elseif GrfObjective
            if ~scalegrfs
                savename = ([savepath class(r) sprintf('_Subject%d_Trial%d_grfmatch',[subject trial])]);
            else
                savename = ([savepath class(r) sprintf('_Subject%d_Trial%d_scaledgrfmatch',[subject trial])]);
            end
        else
            savename = ([savepath class(r) sprintf('_Subject%d_Trial%d_noobj',[subject trial])]);
        end
        
        FilesWithSimilarNames = dir([savename '*']);
        num2saveas = length(FilesWithSimilarNames)+1;
        save([savename sprintf('%d',num2saveas) '.mat']);
    end
    
    figure
    [xf,tf,allx,allt,tair] = r.onestep(xstar,'interleaveAnimation',1);

end

function Cost = MatchSoft(r,allx,allt,tair,softdata,phasevec)
        samps = length(softdata);
        [~,tairsamp]=min(abs(allt-tair));
        stancex=allx(1:tairsamp,:);
        modelstates = interp1(linspace(0,1,size(stancex,1)),stancex,linspace(0,1,samps));
        
        modelsoftpower = zeros(samps,1);
        for i = 1:samps
           modelsoftpower(i) = r.getSoftPower(modelstates(i,:)); 
        end
        Error = modelsoftpower-softdata';
        weights = linspace(1,0.1,length(Error));
        WeightedError = weights'.*Error;
        Cost = sum(WeightedError.^2);
end

function Cost = MatchGRF(r,allx,allt,tair,grfdata,phasevec)
        samps = length(grfdata);
        [~,tairsamp]=min(abs(allt-tair));
        stancex=allx(1:tairsamp,:);
        modelstates = interp1(linspace(0,1,size(stancex,1)),stancex,linspace(0,1,samps));
        
        modelgrfz = zeros(samps,1);
        for i = 1:samps
            phase = r.phases{phasevec(i)};
            grfs = r.getGRF(modelstates(i,:),phase);
            modelgrfz(i) = grfs(2);
        end

        Error = modelgrfz-grfdata;
        weights = linspace(1,0.1,length(Error));
        WeightedError = weights'.*Error;
        Cost = sum(WeightedError.^2);
end

function Cost = MatchScaledGRF(r,allx,allt,tair,grfdata,phasevec)
samps = length(grfdata);
[~,tairsamp]=min(abs(allt-tair));
stancex=allx(1:tairsamp,:);

modelgrfz = zeros(size(stancex,1),1);
for i = 1:size(stancex,1)
    phase = r.phases{phasevec(i)};
    grfs = r.getGRF(stancex(i,:),phase);
    modelgrfz(i,1) = grfs(2);
end
modelgrfz = interp1(linspace(0,1,size(modelgrfz,1)),modelgrfz,linspace(0,1,samps));

datascale = mean(grfdata);
modelscale = mean(modelgrfz);
modelgrfz = modelgrfz/modelscale*datascale;

Error = modelgrfz'-grfdata;
% weights = [10*ones(1,15) linspace(10,0.1,length(Error)-15)];
weights = [10*ones(1,20)];
WeightedError = weights'.*Error(1:length(weights));
Cost = sum(WeightedError.^2);
end

function [C,Ceq] = MassesAddToOne(r)
        C=[];
        if isa(r,'SoftSeriesParallelRunner')
        Ceq = 1-r.mpelvis-r.msoftparallel-r.msoftstomach-r.msoftseries;
        elseif isa(r,'SoftParRunner') || isa(r,'NonlinParSoftRunner')
            Ceq = 1-r.mpelvis-r.msoftparallel;
        end
end

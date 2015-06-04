clear all

% loadname = 'SwingSet_af_sp_sl.mat';
loadname = 'RetractSLIP1.mat';
% loadfolder = '.\ParameterStudies\RetractSLIP\';
loadfolder = '.\SavedGaits\';
load([loadfolder loadname]);

%% Options
[datasoft,dsp,dsl,daf,dsf,datagrf] = getHumanData(7, 2);
runcharic.speed = dsp;
% runcharic.speed = [];
runcharic.steplength = dsl;
% runcharic.steplength = [];
runcharic.airfrac = daf;
% runcharic.airfrac = [];
addedconstraints = [];
constrainttolerance = [];
parmstovary = [];
constrainttolerance = 1e-2;
%% RetractSLIP studies

savename = 'SwingSet_af_sp_sl2.mat'; %set to empty if you want to auto-generate a save name, otherwise input file name (without path)

parmstovary=[{'kstance'} {'kswing'} {'khip'} {'hipl'} {'swingl'}];

swingsetconstraint = @(r,varargin) deal([],r.swingl - r.stancel);
swingsetcost = @(r,varargin) (r.swingl - r.stancel).^2;

    [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
        r.findLimitCycle(xstar,...
        'runcharic',runcharic,...
        'parametersToAlter',parmstovary,...
        'TolCon',constrainttolerance,...
        'Objective',swingsetcost,...
        'additionalConstraintFunction',[],...
        'TolX',[]);
    
    oldr = r;
    oldxstar = xstar;
    r = r.setParametersFromList(parmstovary,finalParameters);
    newx0 = finalStates;
    
    figure
    [xf,tf,allx,allt,tair,newr,phasevec] = r.onestep(newx0,'interleaveAnimation',1);
    
    xstar = allx(1,:);
    x0 = xstar;
    r.printStepCharacteristics(x0,xf,tf,tair);
    

    % parmname = 'swingl';
% currentParmValue = r.(parmname);
% parmrange = linspace(currentParmValue,1,200);
% parmstovary=[{'kstance'} {'kswing'} {'khip'} {'hipl'}];

%Check that initial gait is a limit cycle and looks good
%     r.swingl = 0.4571;
%     [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
%          r.findLimitCycle(xstar,'runcharic',runcharic,...
%         'parametersToAlter',parmstovary,...
%         'TolCon',constrainttolerance,...
%         'additionalConstraintFunction',addedconstraints);

    % Run the parameter study
% [runners,xstar,cnvrg] = parmstudy1d(r,xstar,parmrange,parmname,...
%                        'runcharic',runcharic,'parmstovary',parmstovary);














%% Saving & Export

rootdir = cd;
pfolder = '\ParameterStudies';
classfolder = ['\' class(r) '\'];
if ~isempty(savename)
    save([rootdir pfolder classfolder savename])
else
    basename = 'Pstudy';
    fnames = dir([rootdir pfolder classfolder '*mat']);
    fnames = struct2cell(fnames);
    number = zeros(size(fnames,2),1);
    for i = 1:size(fnames,2)
        name = regexp(fnames{1,i},[basename '[0-9]+\.mat'],'match');
        if ~isempty(name)
            if isa(name,'cell')
               name = name{1}; 
            end
            numberdexes = regexp(name,'[0-9]');
            number(i) = str2double(name(numberdexes));
        end
    end
    biggestnum = max(number);
    charnum = num2str(biggestnum+1);
    save([rootdir pfolder classfolder basename charnum '.mat']);
end























% clearvars

% loadname = 'SwingSet_af_sp_sl.mat';
% loadname = 'Swing_noleg_sephips.mat';
% loadfolder = '.\ParameterStudies\RetractSLIP\';
% loadname = 'Swing_nohip.mat';
% 
% loadfolder = '.\SavedGaits\';
% load([loadfolder loadname]);

%% Options
[datasoft,dsp,dsl,daf,dsf,datagrf] = getHumanData(7, 2);
runcharic.speed = dsp;
runcharic.speed = [];
% runcharic.steplength = dsl;
runcharic.steplength = [];
runcharic.airfrac = 0;
% runcharic.airfrac = [];
addedconstraints = [];
parmstovary = [];
constrainttolerance = 1e-4;
MaxEvals = 1000;

loadfolder = '.\SavedGaits\Swing\';

cellstouse = [6];
%% 1D studies

if sum(cellstouse==1)
    PNAME = 'kswing';
    parmrange = sort(linspace(r.(PNAME),0,40));
    % parmrange = sort(linspace(r.(PNAME),1,40));
    % parmrange = [0.825 0.829];
    
    % savename = ['Swing_' PNAME '.mat'];
    % savename = [PNAME '_sephips_noleg_hiplconstant'];
    savename = [PNAME '_nohip_swinglconstant'];
    % parmstovary=[{'kswing'} {'khip'} {'swingl'} {'hipl'}];
    % parmstovary=[{'kswing'} {'swingl'}];
    % parmstovary=[{'khip'}];
    parmstovary=[{'kswing'}];
    parmstovary = parmstovary(~strcmp(parmstovary,PNAME));
    r.statestovary = [3 4];
    
    % extraconstraint = @(r,x0,xf,tf,allx,allt,varargin) r.floorconstraint(x0,xf,tf,allx,allt,varargin);
    extraconstraint = [];
    
    %     figure
    %     [xf,tf,allx,allt,tair,newr,phasevec] = r.onestep(xstar,'interleaveAnimation',1);
    %
    %     r.printStepCharacteristics(xstar,xf,tf,tair);
    
    %Check that initial gait is a limit cycle and looks good
    %     [finalStates, finalParameters, limitCycleError, c, ceq, eflag, optimoutput, lambda] = ...
    %          r.findLimitCycle(xstar,'runcharic',runcharic,...
    %         'parametersToAlter',parmstovary,...
    %         'TolCon',constrainttolerance,...
    %         'additionalConstraintFunction',addedconstraints);
    
    
    %     Run the parameter study
    [runners,xstar,cnvrg] = parmstudy1d(r,xstar,parmrange,PNAME,...
        'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
        'MaxEvals',1000);
    
    numparams = length(parmstovary);
    numIC = length(r.statestovary);
    numvars = numparams+numIC;
    numstudies = length(cnvrg);
    
    pvar = zeros(numstudies,1);
    resparms = zeros(numstudies,numvars);
    for i = 1:length(cnvrg)
        pvar(i) = runners(i).(PNAME);
        for j = 1:numparams
            resparms(i,j) = runners(i).(parmstovary{j});
        end
        
        resparms(i,numparams+1:end) = xstar(runners(i).statestovary,i);
    end
    figure
    titles = [parmstovary {'AngVel0' 'LengthVel0'}];
    for j = 1:numvars
        subplot(numvars,1,j)
        plot(pvar,resparms(:,j))
        ylabel(titles{j});
        if j == numvars
            xlabel(PNAME)
        end
        
    end
    
end

%% No-Hip Study
if sum(cellstouse==2)
    loadname = 'Swing_nohip.mat';
load([loadfolder loadname]);

    parmnames = {'kswing','swingl'};
    parmranges{1} = sort(linspace(40,0,30));
    parmranges{2} = sort(linspace(0.5,1,10));
%         parmranges{1} = sort(linspace(r.(parmnames{1}),0.9*r.(parmnames{1}),3));
%     parmranges{2} = sort(linspace(r.(parmnames{2}),0.3,3));
%     
    savename = ['Nohip_kswingandswingl2.mat'];

    parmstovary=[{}];
    r.statestovary = [3 4];
    
  
    extraconstraint = [];
    
    %     Run the parameter study
    [runners,xstar,cnvrg,pranges] = parmstudy2d(r,xstar,parmranges,parmnames,...
        'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
        'MaxEvals',1000);
    
    [p2,p1] = size(cnvrg);
    angvel0 = zeros(p2,p1);
    lengthvel0 = zeros(p2,p1);
    for i = 1:p1
        for j = 1:p2
            angvel0(j,i) = xstar(3,i,j);
            lengthvel0(j,i) = xstar(4,i,j);
        end
    end
    
    figure
    %     cols = colormap(jet(p1));
    cols = (cubehelix(max(p1,p2),0.58,-0.35,1.85,0.85,[0 0.63]));
    colormap(cols)
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols(i,:),'LineWidth',2)
        if i ==1
            hold on
        end
    end
    h0=colorbar;
    caxis([min(pranges{1}(1,:)) max(pranges{1}(1,:))])
    ylabel(h0,parmnames{1});
    title('No Hip State Space')
    xlabel('thetadot')
    ylabel('ldot')
    
    figure
    colormap(cols)
    for j = 1:p2
        dexes = cnvrg(j,:)==1;
        subplot(211)
        plot(pranges{1}(j,dexes),angvel0(j,dexes),'Color',cols(j,:),'LineWidth',2)
        if j ==1
            hold on
        end
        subplot(212)
        plot(pranges{1}(j,dexes),lengthvel0(j,dexes),'Color',cols(j,:),'LineWidth',2)
        if j ==1
            hold on
        end
        
    end
    subplot(211)
    h1=colorbar;
    caxis([min(pranges{2}(:,1)) max(pranges{2}(:,1))])
    ylabel(h1,parmnames{2});
    xlabel(parmnames{1})
    ylabel('thetadot')
    subplot(212)
    h2=colorbar;
    caxis([min(pranges{2}(:,1)) max(pranges{2}(:,1))])
    ylabel(h2,parmnames{2});
    xlabel(parmnames{1})
    ylabel('ldot')
    
end

%% No Leg Study
if sum(cellstouse==3)
    loadname = 'Swing_noleg.mat';
    
load([loadfolder loadname]);
    parmnames = {'khip','hipl'};
    parmranges{1} = sort(linspace(r.(parmnames{1}),0,20));
    parmranges{2} = sort(linspace(-1,1,20));
%         parmranges{1} = sort(linspace(r.(parmnames{1}),0.9*r.(parmnames{1}),3));
%     parmranges{2} = sort(linspace(r.(parmnames{2}),0.3,3));
%     
    savename = ['Noleg_khipandhipl.mat'];

    parmstovary=[{}];
    r.statestovary = [3 4];
    
  
    extraconstraint = [];
    
    %     Run the parameter study
    [runners,xstar,cnvrg,pranges] = parmstudy2d(r,xstar,parmranges,parmnames,...
        'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
        'MaxEvals',1000);
    
    [p2,p1] = size(cnvrg);
    angvel0 = zeros(p2,p1);
    lengthvel0 = zeros(p2,p1);
    for i = 1:p1
        for j = 1:p2
            angvel0(j,i) = xstar(3,i,j);
            lengthvel0(j,i) = xstar(4,i,j);
        end
    end
    
    figure
    %     cols = colormap(jet(p1));
    cols = (cubehelix(max(p1,p2),2.32,0.04,2.11,0.85,[0 0.75]));
    colormap(cols)
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols(i,:),'LineWidth',2)
        if i ==1
            hold on
        end
    end
    colorbar;
    title('No Leg State Space')
    xlabel('thetadot')
    ylabel('ldot')
    
    figure
    colormap(cols)
    for j = 1:p2
        dexes = cnvrg(j,:)==1;
        subplot(211)
        plot(pranges{1}(j,dexes),angvel0(j,dexes),'Color',cols(j,:),'LineWidth',2)
        if j ==1
            hold on
        end
        subplot(212)
        plot(pranges{1}(j,dexes),lengthvel0(j,dexes),'Color',cols(j,:),'LineWidth',2)
        if j ==1
            hold on
        end
        
    end
    subplot(211)
    h1=colorbar;
    caxis([min(pranges{2}(:,1)) max(pranges{2}(:,1))])
    ylabel(h1,parmnames{2});
    xlabel(parmnames{1})
    ylabel('thetadot')
    subplot(212)
    h2=colorbar;
    caxis([min(pranges{2}(:,1)) max(pranges{2}(:,1))])
    ylabel(h2,parmnames{2});
    xlabel(parmnames{1})
    ylabel('ldot')
    
end
%% Full Model Hip Study
if sum(cellstouse==4)
    loadname = 'Swing1.mat';

load([loadfolder loadname]);
    parmnames = {'khip','hipl'};
    parmranges{1} = sort(linspace(0,4,20));
    parmranges{2} = sort(linspace(-1,1,20));
%     parmranges{1} = sort(linspace(1.1*r.(parmnames{2}),0.9*r.(parmnames{1}),2));
%     parmranges{2} = sort(linspace(1.1*r.(parmnames{2}),0.9*r.(parmnames{2}),2));
%         parmranges{1} = sort(linspace(r.(parmnames{1}),0.9*r.(parmnames{1}),3));
%     parmranges{2} = sort(linspace(r.(parmnames{2}),0.3,3));
%     
    savename = ['khipandhipl_zerovel.mat'];

    parmstovary=[{}];
    r.statestovary = [3 4];
    r.usefloorconstraint = 0;
  
    extraconstraint = [];
    
    %     Run the parameter study
    [runners,xstar,cnvrg,pranges] = parmstudy2d(r,xstar,parmranges,parmnames,...
        'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
        'MaxEvals',500);
    
    [p2,p1] = size(cnvrg);
    angvel0 = zeros(p2,p1);
    lengthvel0 = zeros(p2,p1);
    floornegs = zeros(p2,p1);
    for i = 1:p1
        for j = 1:p2
            [~, ~, allx, allt] = runners(j,i).onestep(xstar(:,i,j));
            [~,floornegs(j,i)] = runners(j,i).floorconstraint(1,1,1,allx,allt);
            angvel0(j,i) = xstar(3,i,j);
            lengthvel0(j,i) = xstar(4,i,j);
        end
    end
    

    negtol = -0.01;
    figure
    %     cols = colormap(jet(p1));
    cols = (cubehelix(max(p1,p2),2.32,0.04,2.11,0.85,[0 0.75]));
    colormap(cols)
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols(i,:),'LineWidth',2)
        if i ==1
            hold on
        end
        posdexes = floornegs(:,i)>negtol & dexes;
        plot(angvel0(posdexes,i),lengthvel0(posdexes,i),'rx','LineWidth',2)
    end
    colorbar;
    title('State Space')
    xlabel('thetadot')
    ylabel('ldot')
    
    figure
    colormap(cols)
    for j = 1:p2
        dexes = cnvrg(j,:)==1;
        posdexes = floornegs(j,:)>negtol & dexes;
        subplot(211)
        plot(pranges{1}(j,dexes),angvel0(j,dexes),'Color',cols(j,:),'LineWidth',2)
        if j ==1
            hold on
        end
        plot(pranges{1}(j,posdexes),angvel0(j,posdexes),'rx','LineWidth',2)
        subplot(212)
        plot(pranges{1}(j,dexes),lengthvel0(j,dexes),'Color',cols(j,:),'LineWidth',2)
        if j ==1
            hold on
        end
        plot(pranges{1}(j,posdexes),lengthvel0(j,posdexes),'rx','LineWidth',2)
    end
    subplot(211)
    h1=colorbar;
    caxis([min(pranges{2}(:,1)) max(pranges{2}(:,1))])
    ylabel(h1,parmnames{2});
    xlabel(parmnames{1})
    ylabel('thetadot')
    subplot(212)
    h2=colorbar;
    caxis([min(pranges{2}(:,1)) max(pranges{2}(:,1))])
    ylabel(h2,parmnames{2});
    xlabel(parmnames{1})
    ylabel('ldot')
    
end
%% Full Model Swing Spring Study
if sum(cellstouse==5)
    loadname = 'Swing1.mat';

load([loadfolder loadname]);
    parmnames = {'kswing','swingl'};
    parmranges{1} = sort(linspace(0,40,30));
    parmranges{2} = sort(linspace(0.5,1,20));
%     parmranges{1} = sort(linspace(1.1*r.(parmnames{2}),0.9*r.(parmnames{1}),2));
%     parmranges{2} = sort(linspace(1.1*r.(parmnames{2}),0.9*r.(parmnames{2}),2));
%         parmranges{1} = sort(linspace(r.(parmnames{1}),0.9*r.(parmnames{1}),3));
%     parmranges{2} = sort(linspace(r.(parmnames{2}),0.3,3));
%     
    savename = ['kswingandswingl_zerovel.mat'];

    parmstovary=[{}];
    r.statestovary = [3 4];
    r.usefloorconstraint = 0;
  
    extraconstraint = [];
    
    %     Run the parameter study
    [runners,xstar,cnvrg,pranges] = parmstudy2d(r,xstar,parmranges,parmnames,...
        'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
        'MaxEvals',500);
    
    [p2,p1] = size(cnvrg);
    angvel0 = zeros(p2,p1);
    lengthvel0 = zeros(p2,p1);
    floornegs = zeros(p2,p1);
    for i = 1:p1
        for j = 1:p2
            if cnvrg(j,i)
            [~, ~, allx, allt] = runners(j,i).onestep(xstar(:,i,j));
            [~,floornegs(j,i)] = runners(j,i).floorconstraint(1,1,1,allx,allt);
            angvel0(j,i) = xstar(3,i,j);
            lengthvel0(j,i) = xstar(4,i,j);
            end
        end
    end
    

    negtol = -0.01;
    figure
    %     cols = colormap(jet(p1));
    cols = (cubehelix(max(p1,p2),2.32,0.04,2.11,0.85,[0 0.75]));
    colormap(cols)
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols(i,:),'LineWidth',2)
        if i ==1
            hold on
        end
        posdexes = floornegs(:,i)>negtol & dexes;
        plot(angvel0(posdexes,i),lengthvel0(posdexes,i),'rx','LineWidth',2)
    end
    colorbar;
    title('State Space')
    xlabel('thetadot')
    ylabel('ldot')
    
    figure
    colormap(cols)
    for j = 1:p2
        dexes = cnvrg(j,:)==1;
        posdexes = floornegs(j,:)>negtol & dexes;
        subplot(211)
        plot(pranges{1}(j,dexes),angvel0(j,dexes),'Color',cols(j,:),'LineWidth',2)
        if j ==1
            hold on
        end
        plot(pranges{1}(j,posdexes),angvel0(j,posdexes),'rx','LineWidth',2)
        subplot(212)
        plot(pranges{1}(j,dexes),lengthvel0(j,dexes),'Color',cols(j,:),'LineWidth',2)
        if j ==1
            hold on
        end
        plot(pranges{1}(j,posdexes),lengthvel0(j,posdexes),'rx','LineWidth',2)
    end
    subplot(211)
    h1=colorbar;
    caxis([min(pranges{2}(:,1)) max(pranges{2}(:,1))])
    ylabel(h1,parmnames{2});
    xlabel(parmnames{1})
    ylabel('thetadot')
    subplot(212)
    h2=colorbar;
    caxis([min(pranges{2}(:,1)) max(pranges{2}(:,1))])
    ylabel(h2,parmnames{2});
    xlabel(parmnames{1})
    ylabel('ldot')
    
end

%% 6: Leg Yank Impulse Study
if sum(cellstouse==6)
    loadname = 'Swing_yank1.mat';

load([loadfolder loadname]);
    PNAME = 'impulsecoeff';
    parmrange = sort(linspace(0,2,30));
%     parmranges{1} = sort(linspace(1.1*r.(parmnames{2}),0.9*r.(parmnames{1}),2));
%     parmranges{2} = sort(linspace(1.1*r.(parmnames{2}),0.9*r.(parmnames{2}),2));
%         parmranges{1} = sort(linspace(r.(parmnames{1}),0.9*r.(parmnames{1}),3));
%     parmranges{2} = sort(linspace(r.(parmnames{2}),0.3,3));
%     
    savename = ['yankimpulse_nofloor.mat'];

    parmstovary=[{'kswing'} {'khip'} {'hipl'}];
    r.usefloorconstraint = 0;
  
    extraconstraint = [];
    
    %     Run the parameter study
    [runners,xstar,cnvrg] = parmstudy1d(r,xstar,parmrange,PNAME,...
        'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
        'MaxEvals',1000);
    
    numparams = length(parmstovary);
    numIC = length(r.statestovary);
    numvars = numparams+numIC;
    numstudies = length(cnvrg);
    
    pvar = zeros(numstudies,1);
    resparms = zeros(numstudies,numvars);
    for i = 1:length(cnvrg)
        pvar(i) = runners(i).(PNAME);
        for j = 1:numparams
            resparms(i,j) = runners(i).(parmstovary{j});
        end
    end
    
end

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























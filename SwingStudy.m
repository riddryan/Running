%% Options
clear all
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
forcerun = 0; %Run cells even if results exist

LineSize=3;
TextSize=12;
fontstyle='bold';
fonttype='Times New Roman';

loadfolder = './SavedGaits/Swing/';
exportfolder = './Figures/';

cellstouse = [20];
%% 1: kswing study

if sum(cellstouse==1)
    % parmrange = sort(linspace(r.(PNAME),1,40));
    % parmrange = [0.825 0.829];
    
    % savename = ['Swing_' PNAME '.mat'];
    % savename = [PNAME '_sephips_noleg_hiplconstant'];
    savename = [PNAME '_nohip_swinglconstant'];
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
            PNAME = 'kswing';
    parmrange = sort(linspace(r.(PNAME),0,40));
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
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
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
    
end

%% 2: No-Hip Study
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
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        
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
        
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
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
    
end

%% 3: No Leg Study
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
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
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
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
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
    
end

%% 4: Full Model Hip Study
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
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
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
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
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
    
end

%% 5: Full Model Swing Spring Study
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
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
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
        
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
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
    
        rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
    
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
    
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
    end
end

%% 7: Leg Yank Impulse Study: vary all spring params
if sum(cellstouse==7)
    loadname = 'Swing1.mat';
    
    load([loadfolder loadname]);
    r.rigidlegimpulse = 1;
    r.impulsecoeff = 0;
    PNAME = 'impulsecoeff';
    parmrange = sort(linspace(0,3,30));
    %
    savename = ['impulsecoeff.mat'];
    
        rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        
    parmstovary=[{'kswing'} {'khip'} {'hipl'} {'swingl'}];
    r.usefloorconstraint = 0;
    
    extraconstraint = [];
    
    %     Run the parameter study
    [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
    figure
    for j = 1:numparams
        subplot(numparams,1,j)
        plot(prange,resparms(:,j))
        ylabel(parmstovary{j})
    end
    end
    
end

%% SWING SPRING REST LENGTH STUDIES
% 1D parameter studies on the Swing Spring Rest Length
%% 8: Swing Rest Length Study: vary all parms
% Allowing too many parameters to change causes bumps as the optimizer
% finds different combinations of parameters to meet the limit cycle
if sum(cellstouse==8)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,30));
        %
        
        
        parmstovary=[{'kswing'} {'khip'} {'hipl'} {'impulsecoeff'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange,resparms(:,j),'LineWidth',2)
            hold on
            plot(prange(tripdex),resparms(tripdex,j),'rx','LineWidth',2)
            plot(prange(abovedex),resparms(abovedex,j),'gx','LineWidth',2)
            ylabel(parmstovary{j})
%             set(gca,'XLim',[0.6 2])
        end
        legend('Good','No Floor','Foot Above Pelvis')
        xlabel(PNAME)
                
        set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
        hgexport(h,[exportfolder 'swinglStudy.bmp'])
    end
    
end

%% 8b: Swing Rest Length Study: vary all parms
% Explore the space where swing length is very short causing the hip dof
% inertia to go to zero and cause difficulties for integrator
if sum(cellstouse==8) && 0
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swinglB.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,1,30));
        %
        
        
        parmstovary=[{'kswing'} {'khip'} {'hipl'} {'impulsecoeff'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg==1)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol & cnvrg==1;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0 & cnvrg==1;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        figure
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg==1),resparms(cnvrg==1,j))
            hold on
            plot(prange(tripdex),resparms(tripdex,j),'rx')
            plot(prange(abovedex),resparms(abovedex,j),'gx')
            plot(prange(abovedex & tripdex), resparms(abovedex & tripdex,j),'mx')
            ylabel(parmstovary{j})
%             set(gca,'XLim',[0.6 2])
        end
        legend('Good','No Floor','Foot Above Pelvis')
        xlabel(PNAME)
        
    end
    
end

%% 9: Swing Rest Length Study: Vary kswing khip impulsecoeff
    %Fails at around swingl = 0.5, since the foot rebounds above the body
    %causing an instantaneous change in angle that makes the hip spring
    %extremely nonlinear.  In cell 8 where you can vary the hip spring set
    %point you can avoid this behavior.
if sum(cellstouse==9)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl2.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,30));
        %
        
        
        parmstovary=[{'kswing'} {'khip'} {'impulsecoeff'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        figure
        convergedex = cnvrg>0;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(convergedex),resparms(convergedex,j))
            ylabel(parmstovary{j})
        end
         xlabel(PNAME)
    end
    
end

%% 10: Swing Rest Length Study: Vary khip impulsecoeff
    %Instead of failing at swingl = 0.5, the optimization is able to figure
    %out how to get a limit cycle if it gives a large impulse downwards
    %along the leg so that it swings really far below the ground and then
    %springs back up in time to hit the ground from below.
if sum(cellstouse==10)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl3.mat'];
    
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,30));
        %
        
        
        parmstovary=[{'khip'} {'impulsecoeff'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        figure
        convergedex = cnvrg>0;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(convergedex),resparms(convergedex,j))
            ylabel(parmstovary{j})
        end
         xlabel(PNAME)
    end
end

%% 11: Swing Rest Length Study: Vary khip kswing
    %Has a very small range of succesful gaits.  kswing does not seem to be
    %an important parameter.
if sum(cellstouse==11)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl4.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,50));
        %
        
        
        parmstovary=[{'khip'} {'kswing'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        figure
        convergedex = cnvrg>0;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(convergedex),resparms(convergedex,j))
            ylabel(parmstovary{j})
        end
         xlabel(PNAME)
    end
end

%% 12: Swing Rest Length Study: Vary khip
    %Can not get to any other limit cycles by varying only khip.  This
    %implies that while kswing has little effect on the limit cycle, it
    %does have some effect.
if sum(cellstouse==12)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl5.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,30));
        %
        
        
        parmstovary=[{'khip'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        figure
        convergedex = cnvrg>0;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(convergedex),resparms(convergedex,j))
            ylabel(parmstovary{j})
        end
         xlabel(PNAME)
    end
end

%% 13: Swing Rest Length Study: Vary khip hipl
    %The hip spring goes to zero as the swing rest length gets shorter.
    %The hip rest angle is more forward as the swing rest length gets
    %longer as the swing rest length moves away from 1.
if sum(cellstouse==13)
       
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl6.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,30));
        %
        
        
        parmstovary=[{'khip' 'hipl'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        figure
        convergedex = cnvrg>0;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(convergedex),resparms(convergedex,j))
            ylabel(parmstovary{j})
        end
         xlabel(PNAME)
    end
end

%% 14: Swing Rest Length Study: Vary hipl impulsecoeff
if sum(cellstouse==14)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl7.mat'];
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,30));
        %
        
        
        parmstovary=[{'impulsecoeff' 'hipl'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        figure
        convergedex = cnvrg>0;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(convergedex),resparms(convergedex,j))
            ylabel(parmstovary{j})
        end
         xlabel(PNAME)
    end
    
end

%% 15: Swing Rest Length Study: Vary hipl khip kswing
if sum(cellstouse==15)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl8.mat'];
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,30));
        %
        
        
        parmstovary=[{'khip' 'kswing' 'hipl'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        figure
        convergedex = cnvrg>0;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(convergedex),resparms(convergedex,j))
            ylabel(parmstovary{j})
        end
         xlabel(PNAME)
    end
    
end
%% 16: Swing Rest Length Study: Vary khip hipl impulsecoeff
if sum(cellstouse==16)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['swingl9.mat'];
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'swingl';
        parmrange = sort(linspace(0,2,30));
        %
        
        
        parmstovary=[{'khip' 'hipl' 'impulsecoeff'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
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
            [~, ~, allx, allt] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0;
    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        figure
        convergedex = cnvrg>0;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(convergedex),resparms(convergedex,j))
            ylabel(parmstovary{j})
        end
         xlabel(PNAME)
    end
    
end

%%

%% 17: Both Set Points Study
if sum(cellstouse==17)
    loadname = 'Swing1.mat';
    
    load([loadfolder loadname]);
    parmnames = {'swingl','hipl'};
    parmranges{1} = sort(linspace(0.3,1.5,30));
    parmranges{2} = sort(linspace(-1.3,0,30));
    %     parmranges{1} = sort(linspace(1.1*r.(parmnames{2}),0.9*r.(parmnames{1}),2));
    %     parmranges{2} = sort(linspace(1.1*r.(parmnames{2}),0.9*r.(parmnames{2}),2));
    %         parmranges{1} = sort(linspace(r.(parmnames{1}),0.9*r.(parmnames{1}),3));
    %     parmranges{2} = sort(linspace(r.(parmnames{2}),0.3,3));
    %
    savename = ['swinglhipl.mat'];
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        parmstovary=[{'kswing' 'khip' 'impulsecoeff'}];
        r.usefloorconstraint = 0;
        r.rigidlegimpulse = 1;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,pranges] = parmstudy2d(r,xstar,parmranges,parmnames,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',300);
        
        
        [p2,p1] = size(cnvrg);
        floornegs = zeros(p2,p1);
        
        numparams = length(parmstovary);
        [s1,s2] = size(cnvrg);
        
        resparms = zeros(s1,s2,numparams);
        
        for i = 1:p1
            for j = 1:p2
                
                for m = 1:numparams
                    resparms(j,i,m) = runners(j,i).(parmstovary{m});
                end
                
                if sum(isnan(xstar(:,i,j)))==0
                    [~, ~, allx, allt] = runners(j,i).onestep(xstar(:,i,j));
                    [~,floornegs(j,i)] = runners(j,i).floorconstraint(1,1,1,allx,allt);
                    
                    for k = 1:size(allx(:,1))
                        pts = runners(j,i).getPoints(allt(k),allx(k,:));
                        swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
                        %             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
                        %             hipacc(k) = accs(3);
                    end
                    abovepelvis(j,i) = sum(swingfootrel(swingfootrel>0));
                end
            end
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;
        abovedex = abovepelvis>0;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
%         negtol = -0.01;
%         figure
%         %     cols = colormap(jet(p1));
%         cols = (cubehelix(max(p1,p2),2.32,0.04,2.11,0.85,[0 0.75]));
%         colormap(cols)
%         for j = 1:numparams
%             subplot(numparams,1,j)
%             contour(pranges{1},pranges{2},resparms(:,:,j));
%             title(parmstovary{j})
%             xlabel(parmnames{1})
%             ylabel(parmnames{2})
%             colorbar;
%         end
        
% figure
%         for j = 1:p2
%             dexes = cnvrg(j,:)==1;
%             negdexes = floornegs(j,:) & dexes;
%             abovedexes = abovedex(j,:) & dexes;
%             bothdexes = abovedexes & negdexes;
%             
%             plot(pranges{1}(j,dexes),pranges{2}(j,dexes),'Color',cols(j,:),'LineWidth',2)
%             if j ==1
%                 hold on
%             end
%             plot(pranges{1}(j,negdexes),pranges{2}(j,negdexes),'rx','LineWidth',2)
%             plot(pranges{1}(j,abovedexes),pranges{2}(j,abovedexes),'gx','LineWidth',2)
%             plot(pranges{1}(j,bothdexes),pranges{2}(j,bothdexes),'mx','LineWidth',2)
% 
%         end
%         xlabel(parmnames{1})
%         ylabel(parmnames{2})
        
figure
numcols = 50;
cols = (cubehelix(numcols,2.32,1.3,2.52,1.22,[0 1]));
for m = 1:numparams
    cdex = find(cnvrg==1);
    minp = min(min(resparms(cdex + (m-1)*p1*p2)));
    maxp = max(max(resparms(cdex + (m-1)*p1*p2)));
    subplot(numparams,1,m)
    for j = 1:p2
        for i = 1:p1
            if cnvrg(j,i)==1
                dist = (runners(j,i).(parmstovary{m})-minp)/(maxp-minp);
                hold on
                plot(pranges{1}(j,i),pranges{2}(j,i),'x','Color',cols(max(1,round(dist*numcols)),:))
            end
        end
    end
    
end
        
    end
    
end
%% 18: Hip Spring Study: vary all parms
% Varying the hip spring stiffness does not change the swing spring
% stiffness very much.  As the hip spring becomes stiffer, the swing set
% point gets longer.  As the hip spring becomes stiffer, the hip set point
% saturates to being equal to zero (both legs aligned).  The impulse
% coefficient saturates to zero as the hip spring gets stiffer.
if sum(cellstouse==18)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['khip.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'khip';
        parmrange = sort(linspace(0,10,30));
        %
        
        
        parmstovary=[{'kswing'} {'swingl'} {'hipl'} {'impulsecoeff'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg==1)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol & cnvrg==1;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0 & cnvrg==1;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg==1),resparms(cnvrg==1,j),'LineWidth',2)
            hold on
            plot(prange(tripdex),resparms(tripdex,j),'rx','LineWidth',2)
            plot(prange(abovedex),resparms(abovedex,j),'gx','LineWidth',2)
            plot(prange(abovedex & tripdex), resparms(abovedex & tripdex,j),'mx','LineWidth',2)
            ylabel(parmstovary{j})
%             set(gca,'XLim',[0.6 2])
        end
        legend('Good','No Floor','Foot Above Pelvis')
        xlabel(PNAME)
        
        set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
        hgexport(h,[exportfolder 'khipStudy.bmp'])
    end
    
end
%% 19: Swing Spring Study: vary all parms
% 
if sum(cellstouse==19)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['kswing.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'kswing';
        parmrange = sort(linspace(0,50,40));
        %
        
        
        parmstovary=[{'swingl'} {'khip'} {'hipl'} {'impulsecoeff'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg==1)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol & cnvrg==1;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0 & cnvrg==1;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg==1),resparms(cnvrg==1,j),'LineWidth',2)
            hold on
            plot(prange(tripdex),resparms(tripdex,j),'rx','LineWidth',2)
            plot(prange(abovedex),resparms(abovedex,j),'gx','LineWidth',2)
            plot(prange(abovedex & tripdex), resparms(abovedex & tripdex,j),'mx','LineWidth',2)
            ylabel(parmstovary{j})
%             set(gca,'XLim',[0.6 2])
        end
        legend('Good','No Floor','Foot Above Pelvis')
        xlabel(PNAME)
        
        set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
        hgexport(h,[exportfolder 'kswingStudy.bmp'])
    end
    
end
%% 20: Hip Spring Set Point Study: vary all parms
% 
if sum(cellstouse==20)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\Swing\'];
    savename = ['hipl.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'Swing1.mat';
        
        load([loadfolder loadname]);
        r.rigidlegimpulse = 1;
        r.impulsecoeff = 0;
        PNAME = 'hipl';
        parmrange = sort(linspace(-1.5,0.5,40));
        %
        
        
        parmstovary=[{'kswing'} {'swingl'} {'khip'} {'impulsecoeff'}];
        r.usefloorconstraint = 0;
        
        extraconstraint = [];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg==1)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allt(k),allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [~,floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol & cnvrg==1;
        tripswitch = find(diff(tripdex)==1)+1;
        
        abovedex = abovepelvis>0 & cnvrg==1;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg==1),resparms(cnvrg==1,j),'LineWidth',2)
            hold on
            plot(prange(tripdex),resparms(tripdex,j),'rx','LineWidth',2)
            plot(prange(abovedex),resparms(abovedex,j),'gx','LineWidth',2)
            plot(prange(abovedex & tripdex), resparms(abovedex & tripdex,j),'mx','LineWidth',2)
            ylabel(parmstovary{j})
%             set(gca,'XLim',[0.6 2])
        end
        legend('Good','No Floor','Foot Above Pelvis')
        xlabel(PNAME)
        
        set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
        hgexport(h,[exportfolder 'hiplStudy.bmp'])
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























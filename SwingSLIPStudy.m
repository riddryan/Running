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
saveflag = 1;

LineSize=3;
TextSize=12;
fontstyle='bold';
fonttype='Times New Roman';

loadfolder = './SavedGaits/SwingSLIP/';
exportfolder = './Figures/';

cellstouse = [8];
%% 1: Swingl Study: Start from No Impulse gait: Locking
% 
if sum(cellstouse==1)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['Swingl_Lock.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'NoImpulseLock.mat';
        load([loadfolder loadname]);

        
        PNAME = 'swingl';
        parmrange = sort(linspace(0.5,1,15));
        parmstovary=[{'kswing'} {'khip'} {'hipl'} {'impulsecoeff'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
        r.statestovary = [];
        r.statestomeasure = [3 4];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000,'plotiter',1);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg>=0)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg>=0),resparms(cnvrg>=0,j),'LineWidth',2)
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
        saveflag = 0;
    end
    
end

%% 2: kswing Study: Start from Fully Extended Gait: Locking
% 
if sum(cellstouse==2)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['Kswing_Lock.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'NoSwingSpringLock.mat';
        load([loadfolder loadname]);

        
        PNAME = 'kswing';
        parmrange = sort(linspace(0,15,15));
        parmstovary=[{'swingl'} {'khip'} {'hipl'} {'impulsecoeff'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
        r.statestovary = [];
        r.statestomeasure = [3 4];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000,'plotiter',1);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg>=0)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
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
        saveflag = 0;
    end
    
end

%% 3: swingl Study: Start from Fully Extended Gait
% 
if sum(cellstouse==3)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['Swingl.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'ExtendedSwingSpringNoLock.mat';
        load([loadfolder loadname]);

        
        PNAME = 'swingl';
        parmrange = sort(linspace(1,0.6,15));
        parmstovary=[{'kswing'} {'khip'} {'hipl'} {'impulsecoeff'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
        r.statestovary = [];
        r.statestomeasure = [3 4];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000,'plotiter',1);
       
        
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
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
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
        saveflag = 0;
    end
    
end

%% 4: impulse study: start from Zero Impulse Gait: Locking
% 
if sum(cellstouse==4)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['Impulse_Lock.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'NoImpulseLock.mat';
        load([loadfolder loadname]);

        
        PNAME = 'impulsecoeff';
        parmrange = sort(linspace(0,3,15));
        parmstovary=[{'kswing'} {'swingl'} {'khip'} {'hipl'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
        r.statestovary = [];
        r.statestomeasure = [3 4];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000,'plotiter',1);
       
        
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
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
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
        saveflag = 0;
    end
    
end

%% 5: Swingl Study: Start from No Impulse gait
% 
if sum(cellstouse==5)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['Swingl.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'NoImpulse.mat';
        load([loadfolder loadname]);

        
        PNAME = 'swingl';
        parmrange = sort(linspace(0.5,1,15));
        parmstovary=[{'kswing'} {'khip'} {'hipl'} {'impulsecoeff'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
        r.statestovary = [];
        r.statestomeasure = [3 4];
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',1000,'plotiter',1);
       
        
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
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
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
        saveflag = 0;
    end
    
end

%% 6: Speed Study: Start from No Impulse Gait UnLOCKED
% 
if sum(cellstouse==6)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['Speed_NoImpulse.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'NoImpulse.mat';
        load([loadfolder loadname]);

        
        PNAME = 'speed';
        parmrange = sort(linspace(0.8,1.2,15));
        parmstovary=[{'kstance'} {'swingl'} {'kswing'} {'khip'} {'hipl'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
%         r.statestovary = [];
%         r.statestomeasure = [3 4];
        r.statestovary = [3 5 7:8]; 
        r.statestomeasure = [3 4 7:8];  
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',300,'plotiter',1,'tryhard',1);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg>=0)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg>=0),resparms(cnvrg>=0,j),'LineWidth',2)
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
        saveflag = 0;
    end
    
end

%% 7: AirFrac: Start from No Impulse Gait UnLOCKED
% 
if sum(cellstouse==7)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['AirFrac_NoImpulse.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'NoImpulse.mat';
        load([loadfolder loadname]);
        
        PNAME = 'airfrac';
        parmrange = sort(linspace(0,0.5,25));
        parmstovary=[{'kstance'} {'swingl'} {'kswing'} {'khip'} {'hipl'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
%         r.statestovary = [];
%         r.statestomeasure = [3 4];
        r.statestovary = [3 5 7:8]; 
        r.statestomeasure = [3 4 7:8]; 
        MaxEvals = 300;
        tryhard = 1;
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',MaxEvals,'plotiter',1,'tryhard',tryhard);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg>=0)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
%         cnvrg(cnvrg==0) =1;
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg>=0),resparms(cnvrg>=0,j),'LineWidth',2)
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
        saveflag = 0;
    end
    
end

%% 8: Step Length: Start from No Impulse Gait UnLOCKED
% 
if sum(cellstouse==8)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['StepLength_NoImpulse.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'NoImpulse.mat';
        load([loadfolder loadname]);

        
        PNAME = 'steplength';
        parmrange = sort(linspace(0.8,1.4,25));
        parmstovary=[{'kstance'} {'swingl'} {'kswing'} {'khip'} {'hipl'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
%         r.statestovary = [];
%         r.statestomeasure = [3 4];
        r.statestovary = [3 5 7:8]; 
        r.statestomeasure = [3 4 7:8];  
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',300,'plotiter',1,'tryhard',1);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg>=0)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg>=0),resparms(cnvrg>=0,j),'LineWidth',2)
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
        saveflag = 0;
    end
    
end

%% 9: Speed Study: Start from No Impulse Gait UnLOCKED
% 
if sum(cellstouse==9)
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\SwingSLIP\'];
    savename = ['Speed_NoImpulseLock.mat'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'NoImpulseLock.mat';
        load([loadfolder loadname]);

        
        PNAME = 'speed';
        parmrange = sort(linspace(0.8,1.2,15));
        parmstovary=[{'kstance'} {'swingl'} {'kswing'} {'khip'} {'hipl'}];
        
        extraconstraint = @(r,varargin) r.floorconstraint(varargin{:});
%         r.statestovary = [];
%         r.statestomeasure = [3 4];
        r.statestovary = [3 5 7:8]; 
        r.statestomeasure = [3 4 7:8];  
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',300,'plotiter',1,'tryhard',1);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams+numIC;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        for i = find(cnvrg>=0)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
%             accs = runners(i).XDoubleDot(allt(k),allx(k,:)',runners(i).phases{phasevec(k)});
%             hipacc(k) = accs(3);
            end
            abovepelvis(i) = sum(swingfootrel(swingfootrel>0));
            [floornegs(i)] = runners(i).floorconstraint(1,1,1,allx,allt);
        end
        
        triptol = -1e-4;
        tripdex = floornegs<triptol;

        
        abovedex = abovepelvis>0;
        
                    else
        load([rootdir pfolder classfolder savename],'-regexp', '^(?!cellstouse)\w')
        
        
        h=figure;
        for j = 1:numparams
            subplot(numparams,1,j)
            plot(prange(cnvrg>=0),resparms(cnvrg>=0,j),'LineWidth',2)
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
        saveflag = 0;
    end
    
end
%% Saving & Export
if saveflag
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
end






















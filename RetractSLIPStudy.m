%% Options
clearvars
[datasoft,dsp,dsl,daf,dsf,datagrf] = getHumanData(7, 2);
runcharic.speed = dsp;
% runcharic.speed = [];
% runcharic.steplength = dsl;
runcharic.steplength = dsl;
runcharic.airfrac = daf;
% runcharic.airfrac = [];
addedconstraints = [];
parmstovary = [];
constrainttolerance = 1e-4;
MaxEvals = 1200;
forcerun = 0; %Run cells even if results exist

LineSize=3;
TextSize=12;
fontstyle='bold';
fonttype='Times New Roman';

loadfolder = './SavedGaits/RetractSLIP/';
exportfolder = './Figures/';

cellstouse = [1];
%% 1: Tan Impulse to No Tan Impulse, extended set point
% 
if sum(cellstouse==1)
    PNAME = 'tanimpulsecoeff';
    savename = [PNAME '.mat'];
    rootdir = cd;
    pfolder = '\ParameterStudies';
    classfolder = ['\RetractSLIP\'];
    
    if ~exist([rootdir pfolder classfolder savename],'file')
        loadname = 'TanImpulse.mat';
        load([loadfolder loadname]);
        
        parmrange = sort(linspace(r.tanimpulsecoeff,0,20));
        parmstovary=[{'kswing'} {'khip'} {'hipl'} {'impulsecoeff'}];

        %Extra changes to the loaded gait
        extraconstraint = @(r,varargin) r.PositiveImpulse(varargin);
%         r.rigidlegimpulse = 1;
%         r.impulsecoeff = 0;
        
        %     Run the parameter study
        [runners,xstar,cnvrg,prange] = parmstudy1d(r,xstar,parmrange,PNAME,...
            'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
            'MaxEvals',MaxEvals);
       
        
        numparams = length(parmstovary);
        numIC = length(r.statestovary);
        numvars = numparams;
        numstudies = length(cnvrg);
        
        pvar = zeros(numstudies,1);
        resparms = zeros(numstudies,numvars);
        floornegs = zeros(size(cnvrg));
        abovepelvis = zeros(size(cnvrg));
        for i = find(cnvrg==1)
            pvar(i) = runners(i).(PNAME);
            for j = 1:numparams
                resparms(i,j) = runners(i).(parmstovary{j});
            end
            [~, ~, allx, allt, tair,this,phasevec] = runners(i).onestep(xstar(:,i));
            for k = 1:size(allx(:,1))
            pts = runners(i).getPoints(allx(k,:));
            swingfootrel(k) = pts.swingfoot(2) - pts.pelvis(2);
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
        hgexport(h,[exportfolder savename '.bmp'])
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























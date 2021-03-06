% close all
clearvars

%% Initialization & Options

%within each cell that you want, each time you run it
cellstouse=[11];


saveAnimation=0;
savefigs=0;

interleaveAnimationFrameskip=2;
LineWidth=3;
LineSize=3;
TextSize=14;
fontstyle='bold';
fonttype='Times New Roman';



%Where to save things
savepath = [cd '\SavedGaits\'];
avipath = [cd '\Animations\'];
figpath = [cd '\Figures\'];

exportfolder = figpath;


%% Gaits
if any(cellstouse==1) %Soft Stomach Model
    %%
    fname = 'StomachNoDamping.mat';
    
    aviname = [avipath 'StomachDamping.avi'];
    
    load([savepath fname]);
    runner=r;
    x0 = xstar;
    
    if ~saveAnimation
        figure
    end
    
    [xf,tf,allx,allt,tair,this]= r.onestep(x0,'interleaveAnimation',~saveAnimation);
    
    if saveAnimation
        figure
        obj = VideoWriter(aviname);
        open(obj);
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            thisState = allx(i, :);
            runner.plot(thisState,'aviWriter',obj);
            pause(0.01);
        end
        close(obj);
    end
    
    for i = 1: length(allt)
        if allt(i)<tair
            phase = 'stance';
        else
            phase = 'aerial';
        end
        
        x = allx(i,:);
        GRF(i,:) = r.getGRF(x,phase);
        comWR(i,1) = r.getcomWR(x,phase);
        stomachPower(i,1) = r.getStomachPower(x);
        
    end
    
    figure
    subplot(211)
    plot(allt,GRF,'LineWidth',LineWidth)
    legend('grfX','grfY')
    title(fname(1:end-4))
    ylabel('Force (Dimensionless)')
    
    subplot(212)
    hold on
    plot(allt,comWR,'k','LineWidth',LineWidth)
    plot(allt,stomachPower,'r','LineWidth',LineWidth)
    legend('COM WR','Stomach')
    ylabel('Power (Dimensionless)')
    xlabel('Time (Dimensionless)')
    
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    figname = [figpath 'SoftStomachGRFandPower.fig'];
    if savefigs
        saveas(gcf,figname)
    end
    
end

if any(cellstouse==2) %Soft Series Stomach Model
    %%
    %         fname = 'SeriesStomachNoDamping.mat';
    fname = 'SeriesStomachDamping.mat';
    
    aviname = [avipath 'SeriesStomachDamping.avi'];
    
    load([savepath fname])
    runner=r;
    x0 = xstar;
    
    %         r.csoftseries=.3;
    %         r.cleg=.3;
    
    
    
    
    if ~saveAnimation
        figure
    end
    [xf,tf,allx,allt,tair] = r.onestep(x0,'interleaveAnimation',~saveAnimation);
    
    
    if saveAnimation
        figure
        obj = VideoWriter(aviname);
        open(obj);
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            thisState = allx(i, :);
            runner.plot(thisState,'aviWriter',obj);
            pause(0.01);
        end
        close(obj);
    end
    
    for i = 1: length(allt)
        if allt(i)<tair
            phase = 'stance';
        else
            phase = 'aerial';
        end
        
        x = allx(i,:);
        GRF(i,:) = r.getGRF(x,phase);
        comWR(i,1) = r.getcomWR(x,phase);
        stomachPower(i,1) = r.getStomachPower(x);
        seriesPower(i,1) = r.getSeriesPower(x);
        
    end
    
    figure
    subplot(211)
    plot(allt,GRF,'LineWidth',LineWidth)
    title(fname(1:end-4))
    ylabel('Force (Dimensionless)')
    legend('grfX','grfY')
    
    subplot(212)
    hold on
    plot(allt,comWR,'k','LineWidth',LineWidth)
    plot(allt,stomachPower,'r','LineWidth',LineWidth)
    plot(allt,seriesPower,'b','LineWidth',LineWidth)
    legend('COM WR','Stomach','Soft Series')
    ylabel('Power (Dimensionless)')
    xlabel('Time (Dimensionless)')
    
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    figname = [figpath 'SoftSeriesStomachGRFandPower.fig'];
    if savefigs
        saveas(gcf,figname)
    end
    
end

if any(cellstouse==3) %Soft Series Parallel Stomach Model
    %%
    %         fname = 'SeriesStomachNoDamping.mat';
    fname = 'SSSP_Subject7_Trial2_softmatch.mat';
    
    aviname = [avipath 'SSSP_Subject7_Trial2_softmatch.avi'];
    
    load([savepath fname])
    runner=r;
    x0 = xstar;
    
    %     r.csoftparallel = .5;
    %     r.ksoftparallel = 15;
    %     r.msoftparallel = .15;
    %         r.csoftseries=.3;
    %         r.cleg=.3;
    
    
    
    
    if ~saveAnimation
        figure
    end
    [xf,tf,allx,allt,tair] = r.onestep(x0,'interleaveAnimation',~saveAnimation);
    
    if saveAnimation
        figure
        obj = VideoWriter(aviname);
        open(obj);
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            thisState = allx(i, :);
            runner.plot(thisState,'aviWriter',obj);
            pause(0.01);
        end
        close(obj);
    end
    
    for i = 1: length(allt)
        if allt(i)<tair
            phase = 'stance';
        else
            phase = 'aerial';
        end
        
        x = allx(i,:);
        GRF(i,:) = r.getGRF(x,phase);
        comWR(i,1) = r.getcomWR(x,phase);
        stomachPower(i,1) = r.getStomachPower(x);
        seriesPower(i,1) = r.getSeriesPower(x);
        parallelPower(i,1) = r.getParallelPower(x);
        
        energies = runner.getEnergies(x);
        tote(i) = energies.Total;
        
        
        
    end
    
    softPower = stomachPower + seriesPower + parallelPower;
    toeoffmodel = find(allt==tair,1);
    
    subject=7; trial=2;
    [datasoft,dsp,dsl,daf,dsf] = getHumanData( subject, trial);
    
    figure
    plot(datasoft)
    hold on
    plot(interp1(linspace(1,length(datasoft),length(softPower(1:toeoffmodel))),softPower(1:toeoffmodel),1:length(datasoft)),'r')
    legend('Data','Model')
    xlabel('Sample')
    ylabel('Soft Tissue Power')
    
    figure
    plot(allt,tote)
    title('Total Energy')
    
    figure
    subplot(211)
    plot(allt,GRF,'LineWidth',LineWidth)
    title(fname(1:end-4))
    ylabel('Force (Dimensionless)')
    legend('grfX','grfY')
    
    subplot(212)
    hold on
    plot(allt,comWR,'k','LineWidth',LineWidth)
    plot(allt,stomachPower,'r','LineWidth',LineWidth)
    plot(allt,seriesPower,'b','LineWidth',LineWidth)
    plot(allt,parallelPower,'g','LineWidth',LineWidth)
    legend('COM WR','Stomach','Soft Series','SoftParallel')
    ylabel('Power (Dimensionless)')
    xlabel('Time (Dimensionless)')
    
    
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    figname = [figpath 'SoftSeriesParallelStomachGRFandPower.fig'];
    if savefigs
        saveas(gcf,figname)
    end
    
    figure
    plot(allt,parallelPower)
    
end

if any(cellstouse==4)
    %%
    %         fname = 'SeriesStomachNoDamping.mat';
    fname = 'SSSP_vert1.mat';
    
    aviname = [avipath fname(1:end-4) '.avi'];
    
    load([savepath fname])
    runner=r;
    x0 = xstar;
    
    %
    %
    %
    x0(14) = -.5;
    r.csoftparallel=1;
    r.ksoftparallel=40;
    r.msoftparallel=.2;
    %
    %
    %
    
    %     r.csoftparallel = .5;
    %     r.ksoftparallel = 15;
    %     r.msoftparallel = .15;
    %         r.csoftseries=.3;
    %         r.cleg=.3;
    
    
    
    
    if ~saveAnimation
        figure
    end
    [xf,tf,allx,allt,tair] = r.onestep(x0,'interleaveAnimation',~saveAnimation);
    
    if saveAnimation
        figure
        obj = VideoWriter(aviname);
        open(obj);
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            thisState = allx(i, :);
            runner.plot(thisState,'aviWriter',obj);
            pause(0.01);
        end
        close(obj);
    end
    
    for i = 1: length(allt)
        if allt(i)<tair
            phase = 'stance';
        else
            phase = 'aerial';
        end
        
        x = allx(i,:);
        GRF(i,:) = r.getGRF(x,phase);
        comWR(i,1) = r.getcomWR(x,phase);
        stomachPower(i,1) = r.getStomachPower(x);
        seriesPower(i,1) = r.getSeriesPower(x);
        parallelPower(i,1) = r.getParallelPower(x);
        
        energies = runner.getEnergies(x);
        tote(i) = energies.Total;
        
        
        
    end
    
    figure
    plot(allt,tote)
    title('Total Energy')
    
    figure
    subplot(211)
    plot(allt,GRF,'LineWidth',LineWidth)
    title(fname(1:end-4))
    ylabel('Force (Dimensionless)')
    legend('grfX','grfY')
    
    subplot(212)
    hold on
    plot(allt,comWR,'k','LineWidth',LineWidth)
    plot(allt,stomachPower,'r','LineWidth',LineWidth)
    plot(allt,seriesPower,'b','LineWidth',LineWidth)
    plot(allt,parallelPower,'g','LineWidth',LineWidth)
    legend('COM WR','Stomach','Soft Series','SoftParallel')
    ylabel('Power (Dimensionless)')
    xlabel('Time (Dimensionless)')
    
    
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    figname = [figpath 'SoftSeriesParallelStomachGRFandPower.fig'];
    if savefigs
        saveas(gcf,figname)
    end
    
    figure
    plot(allt,parallelPower)
    
end
if any(cellstouse==6) %Soft Par
    %%
    %         fname = 'SeriesStomachNoDamping.mat';
    %     fname = 'SoftParRunner_Subject7_Trial2_grfmatch.mat';
    %     fname = 'SoftParRunner_Subject7_Trial2_softmatch.mat';
    fname = 'SoftParRunner_Subject7_Trial2_scaledgrfmatch2.mat';
    
    aviname = [avipath 'SoftPar_SoftMatch.avi'];
    
    load([savepath fname])
    runner=r;
    x0 = xstar;
    
    %     r.csoftparallel = .5;
    %     r.ksoftparallel = 15;
    %     r.msoftparallel = .15;
    %         r.csoftseries=.3;
    %         r.cleg=.3;
    
    
    
    
    if ~saveAnimation
        figure
    end
    [xf,tf,allx,allt,tair] = r.onestep(x0,'interleaveAnimation',~saveAnimation);
    
    if saveAnimation
        figure
        obj = VideoWriter(aviname);
        open(obj);
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            thisState = allx(i, :);
            runner.plot(thisState,'aviWriter',obj);
            pause(0.01);
        end
        close(obj);
    end
    
    for i = 1: length(allt)
        if allt(i)<tair
            phase = 'stance';
        else
            phase = 'aerial';
        end
        
        x = allx(i,:);
        GRF(i,:) = r.getGRF(x,phase);
        comWR(i,1) = r.getcomWR(x,phase);
        parallelPower(i,1) = r.getParallelPower(x);
        
        energies = runner.getEnergies(x);
        tote(i) = energies.Total;
        
        
        
    end
    
    toeoffmodel = find(allt==tair,1);
    
    subject=7; trial=2;
    [datasoft,dsp,dsl,daf,dsf,datagrfz] = getHumanData( subject, trial);
    
    figure
    stancetimevec = linspace(0,(1-daf)*(1/dsf),length(datasoft));
    plot(stancetimevec,datasoft,'LineWidth',3)
    hold on
    plot(stancetimevec,interp1(linspace(1,length(datasoft),length(parallelPower(1:toeoffmodel))),parallelPower(1:toeoffmodel),1:length(datasoft))...
        ,'r','LineWidth',3)
    legend('Data','Model')
    xlabel('Time')
    ylabel('Power')
    title('Soft-Tissue Power')
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    
    figure
    plot(datagrfz)
    hold on
    plot(interp1(linspace(1,length(datagrfz),length(parallelPower(1:toeoffmodel))),GRF(1:toeoffmodel,2),1:length(datagrfz)),'r')
    legend('Data','Model')
    xlabel('Sample')
    ylabel('GRF-Z')
    
    figure
    plot(allt,tote)
    title('Total Energy')
    
    figure
    subplot(211)
    plot(allt,GRF,'LineWidth',LineWidth)
    title(fname(1:end-4))
    ylabel('Force (Dimensionless)')
    legend('grfX','grfY')
    
    subplot(212)
    hold on
    plot(allt,comWR,'k','LineWidth',LineWidth)
    plot(allt,parallelPower,'g','LineWidth',LineWidth)
    legend('COM WR','SoftParallel')
    ylabel('Power (Dimensionless)')
    xlabel('Time (Dimensionless)')
    
    
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    figname = [figpath 'SoftSeriesParallelStomachGRFandPower.fig'];
    if savefigs
        saveas(gcf,figname)
    end
    
    figure
    plot(allt,parallelPower)
    
end
if any(cellstouse==7) %Nonlin Par soft Runner
    %%
    
    % fname = 'NonlinPar3.mat';
    % fname = 'NonlinParSoftRunner_Subject7_Trial2_softmatch1.mat';
    fname = 'NonlinParSoftRunner_Subject7_Trial2_scaledgrfmatch9.mat';
    %     fname = 'NonlinParSoftRunner_Subject7_Trial27.mat';
    load([savepath fname])
    runner=r;
    x0 = xstar;
    
    aviname = 'NonlinParSoft_GrfMatch';
    
    %     r.csoftparallel = .5;
    %     r.ksoftparallel = 15;
    %     r.msoftparallel = .15;
    %         r.csoftseries=.3;
    %         r.cleg=.3;
    
    
    
    
    if ~saveAnimation
        figure
    end
    [xf,tf,allx,allt,tair] = r.onestep(x0,'interleaveAnimation',~saveAnimation);
    
    if saveAnimation
        figure
        obj = VideoWriter(aviname);
        open(obj);
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            thisState = allx(i, :);
            runner.plot(thisState,'aviWriter',obj);
            pause(0.01);
        end
        close(obj);
    end
    
    for i = 1: length(allt)
        if allt(i)<tair
            phase = 'stance';
        else
            phase = 'aerial';
        end
        
        x = allx(i,:);
        GRF(i,:) = r.getGRF(x,phase);
        comWR(i,1) = r.getcomWR(x,phase);
        parallelPower(i,1) = r.getParallelPower(x);
        
        energies = runner.getEnergies(x);
        tote(i) = energies.Total;
        
        legforce(i,1) = -r.kleg*(x(2)-r.lleg) - r.cleg*(x(7));
        kparhistory(i,1) = r.getksoftparallel(x);
        parforce(i,1) = kparhistory(i,1)*(x(3) - r.lsoftparallel) + r.csoftparallel*x(8);
        
    end
    
    toeoffmodel = find(allt==tair,1);
    
    subject=7; trial=2;
    [datasoft,dsp,dsl,daf,dsf,datagrfz] = getHumanData( subject, trial);
    
    figure
    subplot(211)
    f1=plot(allx(:,2)-r.lleg,legforce,'LineWidth',3);
    hold on
    plot(allx(1:5,2)-r.lleg,legforce(1:5),'r','LineWidth',5)
    plot([0 0],get(gca,'YLim'),'--k')
    plot(get(gca,'XLim'),[0 0],'--k')
    legend('Work Loop','Start')
    title('Leg')
    ylabel('Force')
    xlabel('Displacement')
    set(gca,'XLim',get(gca,'XLim'),'YLim',get(gca,'YLim'))
    
    subplot(212)
    plot(allx(:,3)-r.lsoftparallel,-parforce,'LineWidth',3)
    hold on
    plot(allx(1:5,3)-r.lsoftparallel,-parforce(1:5),'r','LineWidth',5)
    plot([0 0],get(gca,'YLim'),'--k')
    plot(get(gca,'XLim'),[0 0],'--k')
    %     legend('Work Loop','Start')
    xlabel('Displacement')
    ylabel('Force')
    title('Parallel Soft Tissue')
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    
    figure
    subplot(212)
    stancetimevec = linspace(0,(1-daf)*(1/dsf),length(datasoft));
    plot(stancetimevec,datasoft,'LineWidth',3)
    hold on
    plot(stancetimevec,interp1(linspace(1,length(datasoft),length(parallelPower(1:toeoffmodel))),parallelPower(1:toeoffmodel),1:length(datasoft)),'r',...
        'LineWidth',3)
    legend('Data','Model')
    xlabel('Time')
    ylabel('Power')
    title('Soft Tissue Power')
    
    subplot(211)
    stancetimevec = linspace(0,(1-daf)*(1/dsf),length(datagrfz));
    plot(stancetimevec,datagrfz,'LineWidth',3)
    hold on
    plot(stancetimevec,interp1(linspace(1,length(datagrfz),length(parallelPower(1:toeoffmodel))),GRF(1:toeoffmodel,2),1:length(datagrfz)),'r',...
        'LineWidth',3)
    legend('Data','Model')
    xlabel('Time')
    ylabel('Force')
    title('Vertical GRF')
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    
    figure
    plot(allt,tote)
    title('Total Energy')
    
    figure
    subplot(211)
    plot(allt,GRF,'LineWidth',LineWidth)
    title(fname(1:end-4))
    ylabel('Force (Dimensionless)')
    legend('grfX','grfY')
    
    subplot(212)
    hold on
    plot(allt,comWR,'k','LineWidth',LineWidth)
    plot(allt,parallelPower,'g','LineWidth',LineWidth)
    legend('COM WR','SoftParallel')
    ylabel('Power (Dimensionless)')
    xlabel('Time (Dimensionless)')
    
    
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    figname = [figpath 'SoftSeriesParallelStomachGRFandPower.fig'];
    if savefigs
        saveas(gcf,figname)
    end
    
    figure
    plot(allt,parallelPower)
    
end

if any(cellstouse==8) %Foot Achilles Runner
    %%
    
    fname = 'FAR1.mat';
    load([savepath fname])
    runner=r;
    x0 = xstar;
    
    %     r.csoftparallel = .5;
    %     r.ksoftparallel = 15;
    %     r.msoftparallel = .15;
    %         r.csoftseries=.3;
    %         r.cleg=.3;
    
    
    
    
    if ~saveAnimation
        figure
    end
    [xf,tf,allx,allt,tair,~,phasevec] = r.onestep(x0,'interleaveAnimation',~saveAnimation);
    
    if saveAnimation
        figure
        obj = VideoWriter(aviname);
        open(obj);
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            thisState = allx(i, :);
            runner.plot(thisState,'aviWriter',obj);
            pause(0.01);
        end
        close(obj);
    end
    
    for i = 1: length(allt)
        phase = r.phases{phasevec(i)};
        
        x = allx(i,:);
        GRF(i,:) = r.getGRF(x,phase);
        comWR(i,1) = r.getcomWR(x,phase);
        
        energies = runner.getEnergies(x);
        tote(i) = energies.Total;
        
        legforce(i,1) = r.getLegForce(x);
        footforce(i,1) = r.getFootForce(x);
        achillesforce(i,1) = r.getAchillesForce(x);
        
        legpower(i,1) = r.getLegPower(x);
        footpower(i,1) = r.getFootPower(x);
        achillespower(i,1) = r.getAchillesPower(x);
    end
    
    toeoffmodel = find(allt==tair,1);
    
    subject=7; trial=2;
    [datasoft,dsp,dsl,daf,dsf,datagrfz] = getHumanData( subject, trial);
    
    
    figure
    plot(allt,legpower)
    hold on
    plot(allt,footpower,'r','LineWidth',2)
    plot(allt,achillespower,'g','LineWidth',2)
    plot(allt,comWR,'k','LineWidth',3)
    legend('Leg','Foot','Achilles','COM')
    xlabel('Time')
    ylabel('Power')
    
    
    figure
    plot(allt,tote)
    title('Total Energy')
    
    figure
    subplot(211)
    plot(allt,GRF,'LineWidth',LineWidth)
    title(fname(1:end-4))
    ylabel('Force (Dimensionless)')
    legend('grfX','grfY')
    
    subplot(212)
    hold on
    plot(allt,comWR,'k','LineWidth',LineWidth)
    legend('COM WR')
    ylabel('Power (Dimensionless)')
    xlabel('Time (Dimensionless)')
    
    
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    figname = [figpath 'SoftSeriesParallelStomachGRFandPower.fig'];
    if savefigs
        saveas(gcf,figname)
    end
    
    
end

if any(cellstouse==9) %Massless Achilles Runner (MAR)
    %%
    
    fname = 'MAR1.mat';
    load([savepath fname])
    runner=r;
    x0 = xstar;
    
    [xf,tf,allx,allt,tair,runner,phasevec] = runner.onestep(xstar);
    xstar = allx(1,:);
    r = runner;
    
    figure
    runner.anim(allx)
    
    if saveAnimation
        figure
        obj = VideoWriter(aviname);
        open(obj);
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            thisState = allx(i, :);
            runner.plot(thisState,'aviWriter',obj);
            pause(0.01);
        end
        close(obj);
    end
    
    runner.printStepCharacteristics(x0,xf,tf,tair);
    u3 = zeros(length(allt),1);u4=u3;u5=u3;u6=u3;
    
    for i = 1:length(unique(phasevec))
        if i ~= length(runner.phases)
            dexes = find(phasevec==i,1):find(phasevec==i+1,1)-1;
        else
            dexes = find(phasevec==i,1):length(phasevec);
        end
        
        u3temp = diff(allx(dexes,3))./diff(allt(dexes));
        u4temp = diff(allx(dexes,4))./diff(allt(dexes));
        u5temp = diff(allx(dexes,5))./diff(allt(dexes));
        u6temp = diff(allx(dexes,6))./diff(allt(dexes));
        
        u3(dexes) = interp1(dexes(1:end-1)+0.5,u3temp,dexes,'pchip');
        u4(dexes) = interp1(dexes(1:end-1)+0.5,u4temp,dexes,'pchip');
        u5(dexes) = interp1(dexes(1:end-1)+0.5,u5temp,dexes,'pchip');
        u6(dexes) = interp1(dexes(1:end-1)+0.5,u6temp,dexes,'pchip');
    end
    
    
    
    for i = 1:size(allx,1)
        phase = runner.phases{phasevec(i)};
        PEfoot(i) = 1/2*runner.getkfoot(allx(i,:),phase)*(allx(i,5)-allx(i,6)-runner.footangle)^2;
        PEleg(i) =   1/2*runner.kleg*(allx(i,4)-runner.lleg)^2;
        PEach(i) = 1/2*runner.getachilles(allx(i,:),phase)*(allx(i,3)-allx(i,6)-runner.achillesangle)^2;
        PEsprings(i) =  PEfoot(i) + PEleg(i) + PEach(i);
        PEgrav(i) = runner.mpelvis*runner.g*cos(runner.gslope)*allx(i,2) ....
            - runner.mpelvis*runner.g*sin(runner.gslope)*allx(i,1);
        PE(i) = PEsprings(i) + PEgrav(i);
        
        vpelv(i,1:2) = [allx(i,7);allx(i,8)];
        KE(i) = 1/2*runner.mpelvis*dot(vpelv(i,1:2),vpelv(i,1:2));
        TotE(i) = PE(i) + KE(i);
        
        points=runner.getPoints(allx(i,:));
        footx(i)=points.foot(1);
        footy(i)=points.foot(2);
        heelx(i)=points.heel(1); heely(i) = points.heel(2);
        toex(i)=points.toe(1); toey(i) = points.toe(2);
        
        footangvel(i) = u5(i)-u6(i);
        footspring(i) = runner.getkfoot(allx(i,:),phase);
        footforce(i) = runner.getFootForce(allx(i,:),phase);
        footpower(i) = footforce(i)*footangvel(i);
        
        achvel(i) = u3(i) - u6(i);
        achforce(i) = runner.getAchillesForce(allx(i,:),phase);
        achpower(i) = achforce(i)*achvel(i);
        
        legvel(i) = u4(i);
        legforce(i) = runner.getLegForce(allx(i,:));
        legpower(i) = legforce(i)*legvel(i);
        
        
        legvec = points.pelvis - points.foot;
        legdist = norm(legvec);
        legang = atan2(-legvec(2),-legvec(1));
        achforcedir = [0 1;-1 0]*legvec'/legdist;
        achforcez = runner.getachilles(allx(i,:),phase)*(legang - allx(i,6) - runner.achillesangle)/legdist;
        Fres(i,1:2) = achforcez*achforcedir'+legforce(i)*[cos(allx(i,3)) sin(allx(i,3))];
        comWR(i) = dot(-Fres(i,1:2),vpelv(i,1:2));
    end
    
    AchOn = find(phasevec==2,1);
    HeelOff = find(phasevec==3,1);
     d = 1:length(allt);
     
    figure
    subplot(211)
    plot(allt,PEsprings(d))
    hold on
    plot(allt,PE(d))
    plot(allt,KE(d))
    plot(allt,TotE(d))
    legend('PEsprings','PE','KE','TotE')
    
    subplot(212)
    plot(allt,PEleg(d))
    hold on
    plot(allt,PEfoot(d))
    plot(allt,PEach(d))
    %                 plot(allt,PEgrav)
    %                 plot(allt,KE)
    legend('leg','foot','ach');
    
   
    
    figure
    plot(allt,footpower(d),'LineWidth',2)
    hold on
    plot(allt,achpower(d),'r','LineWidth',2)
    plot(allt,legpower(d),'m','LineWidth',2)
    plot(allt,comWR(d),'c','LineWidth',2);
    plot(allt(AchOn)*ones(1,2),get(gca,'YLim'),'k--')
    plot(allt(HeelOff)*ones(1,2),get(gca,'YLim'),'k--')
    plot(tair*ones(1,2),get(gca,'YLim'),'k--')
    legend('foot','achilles','leg','Com WR','Location','SouthEast')
    ylabel('Power')
    xlabel('Time')
    text(1.05*allt(AchOn),max(comWR),'Achilles On')
    text(0.98*allt(HeelOff),max(comWR),'Heel Off','HorizontalAlignment','right')
    text(1.02*tair,max(comWR),'Toe Off')
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    %                 text(1.2,0.6,...
    %                     sprintf('Hi Art, if you see this, here are \nthe massless foot power curves'),...
    %                     'FontSize',30,'FontWeight','Bold')
    
    figure
    plot(allt,allx(d,3:6),'LineWidth',3)
    legend('leg angle','leg length','toe angle','toe length')
    hold on
    plot(allt(AchOn)*ones(1,2),get(gca,'YLim'),'k--')
    plot(allt(HeelOff)*ones(1,2),get(gca,'YLim'),'k--')
    plot(tair*ones(1,2),get(gca,'YLim'),'k--')
    
    figure
    plot(allt,allx(d,4),'LineWidth',3)
    hold on
    plot(allt,allx(d,5)-allx(d,6),'LineWidth',3)
    plot(allt,allx(d,3)-allx(d,6),'LineWidth',3)
    legend('leg spring','foot spring','achilles spring')
    
    
    %     if savefigs
    %         saveas(gcf,figname)
    %     end
    
    
end

if any(cellstouse==10) %RetractSLIP Runner
    %%
    
    fname = 'RetractSLIP1.mat';
    load([savepath fname])
    runner=r;
    x0 = xstar;
%     gifname = [avipath 'RetractSLIP1.gif'];
    gifname = [avipath 'RetractSLIP1.avi'];
    
    [xf,tf,allx,allt,tair,runner,phasevec] = runner.onestep(xstar);
    xstar = allx(1,:);
    r = runner;
    
%     figure
%     runner.anim(allx)
%     
    if saveAnimation
        figure
        count = 0;
        for i = 1 : 4 : size(allx, 1)
            count = count+1;
            cla;
            thisState = allx(i, :);
            runner.plot(thisState);
            if i ==1
                f= getframe;
                [im,map] = rgb2ind(f.cdata,256,'nodither');
            else
               f = getframe;
               im(:,:,1,count) = rgb2ind(f.cdata,map,'nodither');
            end
        end
        
        imwrite(im,map,gifname,'DelayTime',0,'LoopCount',inf);
    end
    
    
    
    
    for i = 1:size(allx,1)
        phase = runner.phases{phasevec(i)};
        x = allx(i,:);
        energies = runner.getEnergies(x,phase);
        TOTE(i) = energies.Total;
        KE(i) = energies.KE;
        PE(i) = energies.PE;
        PEgrav(i) = energies.PEgrav;
        PEspring(i) = energies.PEspring;
        pts = runner.getPoints(x);
        stancefootx(i) = pts.stancefoot(1);
        stancefooty(i) = pts.stancefoot(2);
        swingfootx(i) = pts.swingfoot(1);
        swingfooty(i) = pts.swingfoot(2);
        
        vels = runner.getVels(x);
        stancefootvelx(i) = vels.stancefoot(1);
        stancefootvely(i) = vels.stancefoot(2);
        swingfootvelx(i) = vels.swingfoot(1);
        swingfootvely(i) = vels.swingfoot(2);
        
        stancepower(i) = runner.getStancePower(x,phase);
        swingpower(i) = runner.getSwingPower(x);
        hippower(i) = runner.getHipPower(x);
        GRF(i,1:2) = runner.getGRF(allt(i),x,phase);
        COMWR(i) = runner.getcomWR(x,phase);
    end
    airdex = find(allt==tair,1);
    
    figure
    plot(allt,TOTE,'LineWidth',LineWidth)
    hold on
    plot(allt,KE,'LineWidth',LineWidth)
    plot(allt,PE,'LineWidth',LineWidth)
    plot(allt,PEgrav,'LineWidth',LineWidth)
    plot(allt,PEspring,'LineWidth',LineWidth)
    legend('Tot','KE','PE','PEgrav','PEspring')
    
    figure
    plot(allt,GRF,'LineWidth',LineWidth)
    legend('GRFX','GRFY')
    
    figure
    subplot(211)
    plot(allt,stancepower,'LineWidth',LineWidth)
    hold on
    plot(allt,COMWR,'LineWidth',LineWidth)
    legend('stance leg','COM WR')
    
    subplot(212)
        plot(allt,swingpower,'LineWidth',LineWidth)
        hold on
    plot(allt,hippower,'LineWidth',LineWidth)
    plot(allt(airdex:end),stancepower(airdex:end),'LineWidth',LineWidth)
    legend('Swing Leg','Hip','Push-Off Leg')
    
end

if any(cellstouse==11) %SLIP Runner
    %%
    savepath = [savepath 'SLIP/'];
    fname = 'SLIPNominal.mat';
    load([savepath fname])
    runner=r;
    x0 = xstar;
%     gifname = [avipath 'RetractSLIP1.gif'];
    gifname = [avipath 'SLIPNominal.avi'];
    
    [xf,tf,allx,allt,tair,runner,phasevec] = runner.onestep(xstar);
    xstar = allx(1,:);
    r = runner;
    
%     figure
%     runner.anim(allx)
%     
    if saveAnimation
        figure
        count = 0;
        for i = 1 : 4 : size(allx, 1)
            count = count+1;
            cla;
            thisState = allx(i, :);
            runner.plot(thisState);
            if i ==1
                f= getframe;
                [im,map] = rgb2ind(f.cdata,256,'nodither');
            else
               f = getframe;
               im(:,:,1,count) = rgb2ind(f.cdata,map,'nodither');
            end
        end
        
        imwrite(im,map,gifname,'DelayTime',0,'LoopCount',inf);
    end
    
    
    
    
   for i = 1:size(allx,1)
                phase = runner.phases{phasevec(i)};
                energies = runner.getEnergies(allx(i,:));
                TOTE(i) = energies.Total;
                KE(i) = energies.KE;
                PE(i) = energies.PE;
                PEgrav(i) = energies.PEgrav;
                pts = runner.getPoints(allx(i,:));
                stancefootx(i) = pts.stancefoot(1);
                stancefooty(i) = pts.stancefoot(2);
                vels = runner.getVels(allx(i,:));
                stancefootvelx(i) = vels.stancefoot(1);
                stancefootvely(i) = vels.stancefoot(2);
                GRF(i,:) = runner.getGRF(allt(i),allx(i,:),phase);
            end
            

            figure
            plot(allt,TOTE)
            hold on
            plot(allt,KE)
            plot(allt,PE)
            plot(allt,PEgrav)
            legend('Tot','KE','PE','PEgrav')
            
            
           
           figure
           plot(allt,stancefootx)
           hold on
           plot(allt,stancefooty)
           plot(tair,0,'rx')
           
           thetaf=atan2(-xf(2),-xf(1));
           rf = norm(xf(1:2));
           altstate = xf;
           altstate([3 4]) = [thetaf rf];
           
           subject=7;
           trial=2;
           [datasoft,dsp,dsl,daf,dsf,datagrfz,datagrfy] = getHumanData( subject, trial);
           
           samps = length(datagrfz);
           [~,tairsamp]=min(abs(allt-tair));
           stancex=allx(1:tairsamp,:);
           modelstates = interp1(linspace(0,1,size(stancex,1)),stancex,linspace(0,1,samps));
           
           modelgrfy = zeros(samps,1);
           modelgrfx = zeros(samps,1);
           for i = 1:samps
               phase = r.phases{phasevec(i)};
               grfs = r.getGRF(0,modelstates(i,:),phase);
               modelgrfy(i) = grfs(2);
               modelgrfx(i) = grfs(1);
           end
           
           h=figure;
           stancecycle = linspace(0,100,length(modelgrfy));
           plot(stancecycle,modelgrfy,'b','LineWidth',2.5)
           hold on
           plot(stancecycle,datagrfz,'r','LineWidth',2.5)
           plot(stancecycle,modelgrfx,'b','LineWidth',2.5)
           plot(stancecycle,datagrfy,'r','LineWidth',2.5)
           legend('Model','Data')
           xlabel('Stance Phase')
           ylabel('Force')
           title('GRF')
           
        set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
        print(h,'-dbmp',[exportfolder 'SLIPModelvsData'])

    
end

if any(cellstouse==12) %Swing
    %%
    
    fname = 'Swing1.mat';
    load([savepath fname])
    runner=r;
    x0 = xstar;
%     gifname = [avipath 'RetractSLIP1.gif'];
    gifname = [avipath fname '.avi'];
    
    [xf,tf,allx,allt,tair,runner,phasevec] = runner.onestep(xstar);
    xstar = allx(1,:);
    r = runner;
    
%     figure
%     runner.anim(allx)
%     
    if saveAnimation
        figure
        count = 0;
        for i = 1 : 4 : size(allx, 1)
            count = count+1;
            cla;
            thisState = allx(i, :);
            runner.plot(thisState);
            if i ==1
                f= getframe;
                [im,map] = rgb2ind(f.cdata,256,'nodither');
            else
               f = getframe;
               im(:,:,1,count) = rgb2ind(f.cdata,map,'nodither');
            end
        end
        
        imwrite(im,map,gifname,'DelayTime',0,'LoopCount',inf);
    end
    
    for i = 1:size(allx,1)
        [xpacc(i),ypacc(i),stanceangle(i),stancelength(i),xp(i),yp(i),xvel(i),yvel(i),angvel(i),lengthvel(i)] = getSLIPstates(runner.SLIPdata,allt(i));
        phase = runner.phases{phasevec(i)};
        energies = runner.getEnergies(allt(i),allx(i,:));
        TOTE(i) = energies.Total;
        KE(i) = energies.KE;
        PE(i) = energies.PE;
        PEgrav(i) = energies.PEgrav;
        pts = runner.getPoints(allt(i),allx(i,:));
        stancefootx(i) = pts.stancefoot(1);
        stancefooty(i) = pts.stancefoot(2);
        swingfootx(i) = pts.swingfoot(1);
        swingfooty(i) = pts.swingfoot(2);
        vels = runner.getVels(allt(i),allx(i,:));
        stancefootvelx(i) = vels.stancefoot(1);
        stancefootvely(i) = vels.stancefoot(2);
        GRF(i,:) = runner.getGRF(allt(i),allx(i,:),phase);
        
        %Forces
        swingforce(i) = runner.getSwingForce(allx(i,:));
        hipforce(i) = runner.getHipForce(allx(i,:),stanceangle(i));
        
        ang = allx(i,1);
        swingforcex(i) = swingforce(i)*cos(ang);
        swingforcey(i) = swingforce(i)*sin(ang);
        
        hipforcex(i) = hipforce(i)*sin(ang);
        hipforcey(i) = -hipforce(i)*cos(ang);
        
        accs = runner.XDoubleDot(allt(i),allx(i,:)',phase);
        thetaacc(i) = accs(3);
        
        tgrav(i) = -runner.g*allx(i,2);
        tu2(i) = 2*allx(i,3)*allx(i,4);
        
        
        legacc(i) = accs(4);
    end
    

figure
plot(allt,TOTE)
hold on
plot(allt,KE)
plot(allt,PE)
plot(allt,PEgrav)
legend('Tot','KE','PE','PEgrav')


figure
plot(allt,GRF)
title('GRF')

figure
subplot(211)
plot(allt,-swingforce)
subplot(212)
plot(allt,yp)

figure
subplot(211)
plot(allt,hipforce)
subplot(212)
plot(allt,xp)

% figure
% subplot(311)
% plot(allt,swingforcex)
% hold on
% plot(allt,swingforcey)
% title('swing force')
% 
% subplot(312)
% plot(allt,hipforcex)
% hold on
% plot(allt,hipforcey)
% title('hip force')
%     
% subplot(313)
% plot(allt,hipforcex+swingforcex)
% hold on
% plot(allt,hipforcey+swingforcey)
% title('Sum')
    
    
end
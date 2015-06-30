clearvars

figpath = '.\Figures\';
SwingSlipFolder =[cd '\SavedGaits\SwingSLIP\'];
export = 0;

arrowcol = [0.2 0.9 0];
interleaveAnimationFrameskip=2;
LineWidth=3;
LineSize=3;
TextSize=14;
fontstyle='bold';
fonttype='Times New Roman';

trial = 4;

subject = 3;
[SoftPower,Speed,StepLength,AirFrac,StepFreq,grf_z,grf_y,...
    swingfootvelx,swingfootvely,hippowdata,kneepowdata] = getHumanData( subject, trial);

%% 1: NoImpulse SwingSLIP

fname = 'NoImpulse';
load([SwingSlipFolder fname])


[uniquet,uniquedex] = unique(allt);


allx = allx(uniquedex,:);
allx = interp1(uniquet,allx,linspace(0,uniquet(end),length(uniquet)));
for i = 1:length(allx)
    vels = r.getVels(allx(i,:));
    vfootx(i) = vels.swingfoot(1);
    vfooty(i) = vels.swingfoot(2);
    hippow(i) = r.getHipPower(allx(i,:));
    swingpow(i) = r.getSwingPower(allx(i,:));
    
end


timecycle = linspace(0,100,length(allx));
vfootxDATA = interp1(linspace(0,100,length(swingfootvelx)),swingfootvelx,timecycle);
vfootyDATA = interp1(linspace(0,100,length(swingfootvely)),swingfootvely,timecycle);
hippowDATA = interp1(linspace(0,100,length(swingfootvelx)),hippowdata,timecycle);
kneepowDATA = interp1(linspace(0,100,length(swingfootvely)),kneepowdata,timecycle);


figure
subplot(211)
plot(timecycle,vfootxDATA,'LineWidth',2)
hold on
plot(timecycle,vfootx,'LineWidth',2)
legend('Data','Model','Location','South')
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)


title('Swing Foot Velocity')

ylabel('Velocity Fore-Aft')
subplot(212)
plot(timecycle,vfootyDATA,'LineWidth',2)
hold on
plot(timecycle,vfooty,'LineWidth',2)
xlabel('%Swing Cycle')
ylabel('Velocity Up-Down')
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)

set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
if export
print(gcf,'-dbmp',[figpath fname '.bmp']);
end

figure
subplot(221)
plot(timecycle,hippowDATA,'LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
ylabel('Hip Power')
title('Data')

subplot(222)
plot(timecycle,hippow,'r','LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
title('Model')

subplot(223)
plot(timecycle,kneepowDATA,'LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
xlabel('%Swing Cycle')
ylabel('Knee Power')

subplot(224)
plot(timecycle,swingpow,'r','LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
xlabel('%Swing Cycle')



set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
if export
print(gcf,'-dbmp',[figpath '\SwingPower\' fname '.bmp']);
end

%% 2: No Springs SwingSLIP
% clear r xstar allt allx uniquet 
fname = 'NoSprings';
load([SwingSlipFolder fname]);

[uniquet,uniquedex] = unique(allt);

allx = allx(uniquedex,:);
allx = interp1(uniquet,allx,linspace(0,uniquet(end),length(uniquet)));
for i = 1:length(allx)
    vels = r.getVels(allx(i,:));
    vfootx2(i) = vels.swingfoot(1);
    vfooty2(i) = vels.swingfoot(2);
        hippow(i) = r.getHipPower(allx(i,:));
    swingpow(i) = r.getSwingPower(allx(i,:));
end
vfootx=vfootx2; vfooty = vfooty2;
% vfootx2 = interp1(linspace(0,100,length(uniquet)),vfootx2,linspace(0,100,length(vfootx)));
% vfooty2 = interp1(linspace(0,100,length(uniquet)),vfooty2,linspace(0,100,length(vfooty)));

% subplot(211)
% plot(vfootx2,'LineWidth',2)
% legend('Data','Spring Model','Throw Model','Location','South')
% subplot(212)
% plot(vfooty2,'LineWidth',2)

timecycle = linspace(0,100,length(allx));
vfootxDATA = interp1(linspace(0,100,length(swingfootvelx)),swingfootvelx,timecycle);
vfootyDATA = interp1(linspace(0,100,length(swingfootvely)),swingfootvely,timecycle);
hippowDATA = interp1(linspace(0,100,length(swingfootvelx)),hippowdata,timecycle);
kneepowDATA = interp1(linspace(0,100,length(swingfootvely)),kneepowdata,timecycle);

figure
subplot(211)
plot(timecycle,vfootxDATA,'LineWidth',2)
hold on
plot(timecycle,vfootx,'LineWidth',2)
legend('Data','Model','Location','South')
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)


title('Swing Foot Velocity')

ylabel('Velocity Fore-Aft')
subplot(212)
plot(timecycle,vfootyDATA,'LineWidth',2)
hold on
plot(timecycle,vfooty,'LineWidth',2)
xlabel('%Swing Cycle')
ylabel('Velocity Up-Down')
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)


set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
if export
print(gcf,'-dbmp',[figpath fname '.bmp']);
end

figure
subplot(221)
plot(timecycle,hippowDATA,'LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
ylabel('Hip Power')
title('Data')

subplot(222)
plot(timecycle,hippow,'r','LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
title('Model')

subplot(223)
plot(timecycle,kneepowDATA,'LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
xlabel('%Swing Cycle')
ylabel('Knee Power')

subplot(224)
plot(timecycle,swingpow,'r','LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
xlabel('%Swing Cycle')



set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
if export
print(gcf,'-dbmp',[figpath '\SwingPower\' fname '.bmp']);
end
%% 3 No Swing Spring + Locking SwingSLIP
% clear r xstar allt allx uniquet 
fname = 'NoSwingSpringLock';
load([SwingSlipFolder fname]);

[uniquet,uniquedex] = unique(allt);

allx = allx(uniquedex,:);
allx = interp1(uniquet,allx,linspace(0,uniquet(end),length(uniquet)));
for i = 1:length(allx)
    vels = r.getVels(allx(i,:));
    vfootx2(i) = vels.swingfoot(1);
    vfooty2(i) = vels.swingfoot(2);
            hippow(i) = r.getHipPower(allx(i,:));
    swingpow(i) = r.getSwingPower(allx(i,:));
end
vfootx=vfootx2; vfooty = vfooty2;
% vfootx2 = interp1(linspace(0,100,length(uniquet)),vfootx2,linspace(0,100,length(vfootx)));
% vfooty2 = interp1(linspace(0,100,length(uniquet)),vfooty2,linspace(0,100,length(vfooty)));

% subplot(211)
% plot(vfootx2,'LineWidth',2)
% legend('Data','Spring Model','Throw Model','Location','South')
% subplot(212)
% plot(vfooty2,'LineWidth',2)

timecycle = linspace(0,100,length(allx));
vfootxDATA = interp1(linspace(0,100,length(swingfootvelx)),swingfootvelx,timecycle);
vfootyDATA = interp1(linspace(0,100,length(swingfootvely)),swingfootvely,timecycle);
hippowDATA = interp1(linspace(0,100,length(swingfootvelx)),hippowdata,timecycle);
kneepowDATA = interp1(linspace(0,100,length(swingfootvely)),kneepowdata,timecycle);

figure
subplot(211)
plot(timecycle,vfootxDATA,'LineWidth',2)
hold on
plot(timecycle,vfootx,'LineWidth',2)
legend('Data','Model','Location','South')
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)


title('Swing Foot Velocity')

ylabel('Velocity Fore-Aft')
subplot(212)
plot(timecycle,vfootyDATA,'LineWidth',2)
hold on
plot(timecycle,vfooty,'LineWidth',2)
xlabel('%Swing Cycle')
ylabel('Velocity Up-Down')
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)


set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
if export
print(gcf,'-dbmp',[figpath fname '.bmp']);
end

figure
subplot(221)
plot(timecycle,hippowDATA,'LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
ylabel('Hip Power')
title('Data')

subplot(222)
plot(timecycle,hippow,'r','LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
title('Model')

subplot(223)
plot(timecycle,kneepowDATA,'LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
xlabel('%Swing Cycle')
ylabel('Knee Power')

subplot(224)
impdex=timecycle(find(diff(allx(:,12))<1e-8,1));
plot(timecycle,swingpow,'r','LineWidth',2)
hold on
ylims = get(gca,'YLim');
set(gca,'YLim',ylims);
% plot([impdex impdex],[ylims(1) 0],'r')
a = arrow([impdex 0],[impdex ylims(1)]);
text(0.4*impdex,-0.5,sprintf('Knee Lock\n Impulse'))
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
xlabel('%Swing Cycle')


set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
if export
print(gcf,'-dbmp',[figpath '\SwingPower\' fname '.bmp']);
end
%% No-Impulse + Weak Knee + Locking SwingKneeSLIP
% clear r xstar allt allx uniquet
SwingKneeSlipFolder = [cd '\SavedGaits\SwingKneeSLIP\'];
fname = 'LockNoImpulseWeakKnee';
load([SwingKneeSlipFolder fname]);

[uniquet,uniquedex] = unique(allt);

allx = allx(uniquedex,:);
allx = interp1(uniquet,allx,linspace(0,uniquet(end),length(uniquet)));
for i = 1:length(allx)
    vels = r.getVels(allx(i,:));
    vfootx2(i) = vels.swingfoot(1);
    vfooty2(i) = vels.swingfoot(2);
    hippow(i) = r.getHipPower(allx(i,:));
    kneepow(i) = r.getKneePower(allx(i,:));
end
vfootx=vfootx2; vfooty = vfooty2;
% vfootx2 = interp1(linspace(0,100,length(uniquet)),vfootx2,linspace(0,100,length(vfootx)));
% vfooty2 = interp1(linspace(0,100,length(uniquet)),vfooty2,linspace(0,100,length(vfooty)));

% subplot(211)
% plot(vfootx2,'LineWidth',2)
% legend('Data','Spring Model','Throw Model','Location','South')
% subplot(212)
% plot(vfooty2,'LineWidth',2)

timecycle = linspace(0,100,length(allx));
vfootxDATA = interp1(linspace(0,100,length(swingfootvelx)),swingfootvelx,timecycle);
vfootyDATA = interp1(linspace(0,100,length(swingfootvely)),swingfootvely,timecycle);
hippowDATA = interp1(linspace(0,100,length(swingfootvelx)),hippowdata,timecycle);
kneepowDATA = interp1(linspace(0,100,length(swingfootvely)),kneepowdata,timecycle);

figure
subplot(211)
plot(timecycle,vfootxDATA,'LineWidth',2)
hold on
plot(timecycle,vfootx,'LineWidth',2)
legend('Data','Model','Location','South')
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)


title('Swing Foot Velocity')

ylabel('Velocity Fore-Aft')
subplot(212)
plot(timecycle,vfootyDATA,'LineWidth',2)
hold on
plot(timecycle,vfooty,'LineWidth',2)
xlabel('%Swing Cycle')
ylabel('Velocity Up-Down')
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)


set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
if export
print(gcf,'-dbmp',[figpath fname '.bmp']);
end

figure
subplot(221)
plot(timecycle,hippowDATA,'LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
ylabel('Hip Power')
title('Data')

subplot(222)
plot(timecycle,hippow,'r','LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
title('Model')

subplot(223)
plot(timecycle,kneepowDATA,'LineWidth',2)
hold on
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
xlabel('%Swing Cycle')
ylabel('Knee Power')

subplot(224)
impdex=timecycle(find(abs(diff(allx(:,12)))>4,1));
plot(timecycle,kneepow,'r','LineWidth',2)
hold on
ylims = get(gca,'YLim');
set(gca,'YLim',ylims);
% plot([impdex impdex],[ylims(1) 0],'r')
a = arrow([impdex 0],[impdex ylims(1)]);
text(0.4*impdex,-0.5,sprintf('Knee Lock\n Impulse'))
plot(get(gca,'XLim'),[0 0],'k--','LineWidth',1.5)
xlabel('%Swing Cycle')


set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
if export
print(gcf,'-dbmp',[figpath '\SwingPower\' fname '.bmp']);
end
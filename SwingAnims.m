animpath = '.\Animations\';
SwingSlipFolder =[cd '\SavedGaits\SwingSLIP\'];

cellstouse = [3];

arrowcol = [0.2 0.9 0];
interleaveAnimationFrameskip=2;
LineWidth=3;
LineSize=3;
TextSize=14;
fontstyle='bold';
fonttype='Times New Roman';
%% No Springs
if sum(cellstouse==1) 

fname = 'NoSprings';
load([SwingSlipFolder fname])

figure
obj = VideoWriter([animpath fname '.avi']);
open(obj);

for i = 1 : 3 : length(allx)
    cla;
    r.plot(allx(i,:),'stanceleg',0);
    if i == 1
        vels = r.getVels(allx(i,:));
        pts = r.getPoints(allx(i,:));
        foot = pts.swingfoot;
        footvel = 0.4*vels.swingfoot/norm(vels.swingfoot);
        ar=arrow(foot,foot+footvel,[],[],150,2);
        set(ar,'EdgeColor',arrowcol,'FaceColor',arrowcol);
        
        t = text(pts.swingfoot(1),-0.1,'\color[rgb]{0.2 0.9 0} Impulse','FontSize',TextSize);
        
        for j = 1:20
        thisFrame = getframe(gcf);
        writeVideo(obj, thisFrame);
        end
        
    end
    
    thisFrame = getframe(gcf);
    writeVideo(obj, thisFrame);
    
end
    close(obj);
    
end
%% No Impulse
if sum(cellstouse==2) 
fname = 'NoImpulse';
load([SwingSlipFolder fname])

figure
obj = VideoWriter([animpath fname '.avi']);
open(obj);

for i = 1 : 2 : length(allx)
    cla;
    r.plot(allx(i,:),'stanceleg',0);
    
    thisFrame = getframe(gcf);
    writeVideo(obj, thisFrame);
    
end
    close(obj);
   
      
end
      
%% No Swing Spring, Locking
if sum(cellstouse==3) 
fname = 'NoSwingSpringLock';
load([SwingSlipFolder fname])

figure
obj = VideoWriter([animpath fname '.avi']);
open(obj);

for i = 1 : 2 : length(allx)
    cla;
    r.plot(allx(i,:),'stanceleg',0);
        if i == 1
        vels = r.getVels(allx(i,:));
        pts = r.getPoints(allx(i,:));
        foot = pts.swingfoot;
        footvel = 0.4*vels.swingfoot/norm(vels.swingfoot);
        ar=arrow(foot,foot+footvel,[],[],150,2);
        set(ar,'EdgeColor',arrowcol,'FaceColor',arrowcol);
        
        t = text(pts.swingfoot(1),-0.1,'\color[rgb]{0.2 0.9 0} Impulse','FontSize',TextSize);
        
        for j = 1:20
        thisFrame = getframe(gcf);
        writeVideo(obj, thisFrame);
        end
        
    end
    thisFrame = getframe(gcf);
    writeVideo(obj, thisFrame);
    
end
    close(obj);

      
end
      
      
      
      
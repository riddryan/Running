LineSize=3;
TextSize=12;
fontstyle='bold';
fonttype='Times New Roman';
%% 
if 0
    file1 = './ParameterStudies/Swing/Swing_swingl.mat';
    file2 = './ParameterStudies/Swing/Swing_swingl2.mat';
    
    load(file1);
    
    figure
    titles = [parmstovary {'AngVel0' 'LengthVel0'}];
    for j = 1:numvars
        subplot(numvars,1,j)
        plot(pvar,resparms(:,j))
        ylabel(titles{j});
        if j == numvars
            xlabel(PNAME)
            hold on
            plot(pvar,cnvrg,'bx')
        end
        
    end
    
    load(file2);
    for j = 1:numvars
        subplot(numvars,1,j)
        hold on
        plot(pvar,resparms(:,j))
        ylabel(titles{j});
        if j == numvars
            xlabel(PNAME)
            plot(pvar,cnvrg,'rx')
        end
        
    end
    
end

%%
nominalfile = './SavedGaits/Swing/Swing1.mat';
nospringsfile = './SavedGaits/Swing/Swing_nosprings.mat';
file1 = './ParameterStudies/Swing/Noleg_khipandhipl.mat';
file2 = './ParameterStudies/Swing/Nohip_kswingandswingl.mat';
file3 = './ParameterStudies/Swing/khipandhipl_zerovel.mat';
file4 = './ParameterStudies/Swing/kswingandswingl_zerovel.mat';
file5 = './ParameterStudies/Swing/yankimpulse.mat';



figure

load(file1);
cols1 = (cubehelix(max(p1,p2),0.58,-0.35,1.85,0.85,[0 0.63]));
colormap(cols1)

    for i = 1:p1
        dexes = cnvrg(:,i)==1;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols1(i,:),'LineWidth',2)
        if i ==1
            hold on
        end
        if pranges{1}(1,i) == max(max(pranges{1}))
         plot(angvel0(dexes,i),lengthvel0(dexes,i),'g','LineWidth',2)   
        end
    end
    
    text(1.5,-1,'No Leg Spring')
    
    load(file2);
    cols2 = (cubehelix(max(p1,p2),0.91,-0.24,2.37,0.61,[0 0.764]));
    colormap(cols2)
    
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols2(i,:),'LineWidth',2)
    end
    
    text(1,-0.5,'No Hip Spring')
    
    negtol = -0.01;
    
    load(file3);
    cols1 = (cubehelix(max(p1,p2),0.58,-0.35,1.85,0.85,[0 0.63]));
    colormap(cols1)
    
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
% dexes = cnvrg(:,i)==1 & floornegs(:,i)>negtol;
%         plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols1(i,:),'LineWidth',2)
        posdexes = floornegs(:,i)>negtol & dexes;
        plot(angvel0(posdexes,i),lengthvel0(posdexes,i),'bx','LineWidth',2)
    end
    
    t3 = text(0,0.5,'\color{blue} khip, hipsetpoint');
        
    load(file4);
    cols2 = (cubehelix(max(p1,p2),0.91,-0.24,2.37,0.61,[0 0.764]));
    colormap(cols2)
    
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
% dexes = cnvrg(:,i)==1 & floornegs(:,i)>negtol;
%         plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols2(i,:),'LineWidth',2)
        posdexes = floornegs(:,i)>negtol & dexes;
        plot(angvel0(posdexes,i),lengthvel0(posdexes,i),'rx','LineWidth',2)
    end
    
    t4 = text(-2,-2.5,'\color{red} kswing, swingsetpoint');
    
    load(nominalfile)
plot(xstar(3),xstar(4),'g.','LineWidth',2,'MarkerSize',40)

tnom = text(0.25,0.1,'\color{green} Nominal');

load(file5);
    
       minyankdex = find(cnvrg==1,1);
       normyankdex = length(runners);
       
       minyank = runners(minyankdex).getYankImpulse(xstar(:,minyankdex),0);
       normyank = runners(normyankdex).getYankImpulse(xstar(:,normyankdex),0);
       
       
       plot(minyank(3),minyank(4),'c.','LineWidth',2,'MarkerSize',40)
       t5_1 = text(0.15,-.6,'\color{cyan} Min Yank');
       
       plot(normyank(3),normyank(4),'m.','LineWidth',2,'MarkerSize',40)
       t5_2 = text(-.3,-1.5,'\color{magenta} Impulse Coeff = 2');
       
       
       plot(get(gca,'XLim'),[0 0],'k--')
       plot([0 0],get(gca,'YLim'),'k--')
       
       load(nospringsfile)
       plot(xstar(3),xstar(4),'y.','LineWidth',2,'MarkerSize',40)
       
       nospringstext = text(1,-1.4,'\color{yellow} No Springs');
    
    
    title('Swing Leg IC State Space')
    xlabel('\textbf{$\dot{\theta_{0}}$}','interpreter','latex')
    ylabel('\textbf{$\dot{l_{0}}$}','interpreter','latex')
    
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
    
   


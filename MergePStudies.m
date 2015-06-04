
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
nominalfile = './SavedGaits/Swing1.mat';
file1 = './ParameterStudies/Swing/Noleg_khipandhipl.mat';
file2 = './ParameterStudies/Swing/Nohip_kswingandswingl.mat';
file3 = './ParameterStudies/Swing/khipandhipl_zerovel.mat';
file4 = './ParameterStudies/Swing/kswingandswingl_zerovel.mat';



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
    
    load(file2);
    cols2 = (cubehelix(max(p1,p2),0.91,-0.24,2.37,0.61,[0 0.764]));
    colormap(cols2)
    
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols2(i,:),'LineWidth',2)
    end
    
    negtol = -0.01;
    
    load(file3);
    cols1 = (cubehelix(max(p1,p2),0.58,-0.35,1.85,0.85,[0 0.63]));
    colormap(cols1)
    
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
% dexes = cnvrg(:,i)==1 & floornegs(:,i)>negtol;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols1(i,:),'LineWidth',2)
        posdexes = floornegs(:,i)>negtol & dexes;
        plot(angvel0(posdexes,i),lengthvel0(posdexes,i),'rx','LineWidth',2)
    end
    
        
    load(file4);
    cols2 = (cubehelix(max(p1,p2),0.91,-0.24,2.37,0.61,[0 0.764]));
    colormap(cols2)
    
    for i = 1:p1
        dexes = cnvrg(:,i)==1;
% dexes = cnvrg(:,i)==1 & floornegs(:,i)>negtol;
        plot(angvel0(dexes,i),lengthvel0(dexes,i),'Color',cols2(i,:),'LineWidth',2)
        posdexes = floornegs(:,i)>negtol & dexes;
        plot(angvel0(posdexes,i),lengthvel0(posdexes,i),'rx','LineWidth',2)
    end
    
    load(nominalfile)
plot(xstar(3),xstar(4),'r.','LineWidth',2,'MarkerSize',20)
    
    title('State Space')
    xlabel('thetadot')
    ylabel('ldot')


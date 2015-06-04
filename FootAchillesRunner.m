classdef FootAchillesRunner < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
  
    properties
        mpelvis = .8; mfoot=0; mheel=1e-2; mtoe=1e-2;
        gslope = 0;
        g = 1;
        N=12;
        statestovary = [3 5 6 9 10 11 12];
        statestomeasure = [7 8 9 10 11 12];
        lleg=1; lheel = .2; ltoe = .2; Rtoe=.3;
        footangle = pi - 80/360*2*pi; 
        achillesangle = .6;
        achillessetpoint = -pi/2;
        kleg=12.9801; kachilles=0;
        cleg=0; cfoot=0; cachilles=0;
        
        kimpact = .1;  %stiffness of foot spring at impact
        kstance = .1;  %stiffness of foot after impact
        
        %How much force foot spring must be exerting before transitioning
        %into a stiffer spring (kimpact -> kstance). The leg transitions
        %between kimpact and kstance over the range set by the displacement ratios w.r.t. rest length: transitionstart
        %and transitionend.
        transitionstart = 1;
        transitionend = 1.3;
        
        
        phases = {'HeelToe' 'Bottomed' 'AchillesEngaged' 'Unbottomed' 'Toe' 'Aerial'};
    end
    
    properties (Dependent=true)
        
    end
   
    methods (Static)
       
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            savepath = [dir '\Animations\'];
            aviname = [savepath 'FootAchilles1.avi'];
            doanalysis = 1;
                
            runner = FootAchillesRunner;
            
            IC = FootAchillesRunnerState;
            
%             x0 = [-.8 1 .2 0 0 -.895 -.56 -.07 0 0];
            IC.stanceLeg.Angle = -1.2;
            IC.stanceLeg.AngleDot = -1.6;
            
            IC.stanceLeg.Length = 1;
            IC.stanceLeg.LengthDot = -1.1;
            
            IC.toe.Angle = -30 /360*2*pi;
            IC.toe.AngleDot = 1;
            
            IC.heel.AngleDot = -1;
            
            IC.heel.Angle = -130 /360*2*pi; 
            
            
            x0 = IC.getVector();
            
            %Make sure foot starts on ground
            x0 = runner.GoodInitialConditions(x0);
            
            runner.kleg = 16;
            runner.cfoot =0.0;
            runner.kachilles = .5;
            
            runner.mfoot = 0.1;
            runner.mheel = 0.1;
            runner.mtoe = .001;

%             runner.mfoot = 0.0;
%             runner.mheel = 0.0;
%             runner.mtoe = 0.0;
            
            runner.kimpact = 0.005;
            runner.kstance = .3;
            runner.footangle = x0(5) - x0(6);
% runner.footangle = x0(5) - x0(6) + .05;
            
            
            %Take a Step
            [xf,tf,allx,allt,tair,runner,phasevec] = runner.onestep(x0);
            
            
            %% Check Energy and constraints
            
            if doanalysis
            energies0 = runner.getEnergies(x0);
            energy0 = energies0.Total;
            
            N = runner.N;
            
            ts = length(allt); 
            footx = zeros(ts,1);footy = zeros(ts,1);heelx = zeros(ts,1); heely = zeros(ts,1); toex = zeros(ts,1); toey = zeros(ts,1); footvel = zeros(ts,2); heelvel = zeros(ts,2); toevel = zeros(ts,2);
%             footveldir = zeros(ts,2); heelveldir = zeros(ts,2); toeveldir=zeros(ts,2); 
            tote = zeros(ts,1); footpower=zeros(ts,1); comWR = zeros(ts,1); achillesspring=zeros(ts,1);achillesangle=zeros(ts,1);achillesangvel=zeros(ts,1);achillespower=zeros(ts,1); achillesforce=zeros(ts,1);
            footspring=zeros(ts,1);footangle=zeros(ts,1); footangvel=zeros(ts,1);footforce=zeros(ts,1); distalfootpower=zeros(ts,1); copvel=zeros(ts,2); cop=zeros(ts,2); GRF=zeros(ts,2); bottomedconstraint=zeros(ts,1);
            
            bdex = find(phasevec==2,1);
            
            for i = 1:size(allx,1)

                phase = runner.phases{phasevec(i)};
                
                Corig = runner.getConstraints(allx(i,:),phase);
                Jvel = runner.getVelJacob(allx(i,:));
                
%               Jtickankle = Jvel(4:6,:)*null(Corig);
%                 Jtickankle = Jvel(4:6,3:6)*null(Corig(:,3:6));
                                
                points=runner.getPoints(allx(i,:));
                
%                 if i ==1 %Set up inverse dynamics parameters of foot
%                    pfootnom = (points.foot + points.heel + points.toe)/3; 
%                    lt = norm(points.toe-pfootnom);
%                    alphat = atan2(points.toe(2)-pfootnom(2),points.toe(1)-pfootnom(1));
%                    lf = norm(points.foot-pfootnom);
%                    alphaf = atan2(points.foot(2)-pfootnom(2),points.foot(1)-pfootnom(1));
%                    lh = norm(points.heel-pfootnom);
%                    alphah = atan2(points.heel(2)-pfootnom(2),points.heel(1)-pfootnom(1));
%                    
%                    rcom(i,1:2) = pfootnom;
%                    thetacom(i,1) = 0;
%                 else
%                     objfun = @(x) norm(points.heel' - (x(1:2) + lh*[cos(x(3)+alphah);sin(x(3)+alphah)]))^2 +...
%                                         norm(points.toe' - (x(1:2) + lt*[cos(x(3)+alphat);sin(x(3)+alphat)]))^2  +... 
%                                         norm(points.foot' - (x(1:2) + lf*[cos(x(3)+alphaf);sin(x(3)+alphaf)]))^2;
%                                     options = optimoptions(@fminunc,'Algorithm','quasi-newton');
%                     [optimvars,fval,exitflag,output] = fminunc(objfun,[rcom(i-1,1:2)';thetacom(i-1)],options);
%                     rcom(i,1:2) = optimvars(1:2);
%                     thetacom(i,1) = optimvars(3);
%                 end
                
                footx(i)=points.foot(1);
                footy(i)=points.foot(2);
                heelx(i)=points.heel(1); heely(i) = points.heel(2);
                toex(i)=points.toe(1); toey(i) = points.toe(2);
                u = allx(i,N/2+1:end)';
                vels = Jvel*u;
                footvel(i,1:2) = vels(4:5);
                heelvel(i,1:2) = vels(7:8);
                toevel(i,1:2) = vels(10:11);
                
                
%                 footveldir = footvel(i,1:2)/norm(footvel(i,1:2));
%                 heelveldir = heelvel(i,1:2)/norm(heelvel(i,1:2));
%                 toeveldir = toevel(i,1:2)/norm(toevel(i,1:2));
                
                
                energies = runner.getEnergies(allx(i,:));
                tote(i) = energies.Total;
                
                footpower(i) = runner.getFootPower(allx(i,:));
                comWR(i) = runner.getcomWR(allx(i,:),phase);
                legPower(i) = runner.getLegPower(allx(i,:));
                
                achillesspring(i) = runner.getachilles(allx(i,:));
                achillesangle(i) = allx(i,3) - allx(i,6);
                achillesangvel(i) = allx(i,9) - allx(i,12);
                achillespower(i) = runner.getAchillesPower(allx(i,:));
                achillesforce(i) = runner.getAchillesForce(allx(i,:));
                
                
                footspring(i) = runner.getkfoot(allx(i,:));
                footangle(i) = (allx(i,5) - allx(i,6))/runner.footangle;
                footangvel(i) = allx(i,11)-allx(i,12);
                footforce(i) = runner.getFootForce(allx(i,:));
                
%                 anklemoment(i) = achillesforce(i);
%                 ankleangvel(i) = allx(i,9)-allx(i,11);
%                 anklepower(i) = anklemoment(i)*ankleangvel(i);
                
                
                GRF(i,:) = runner.getGRF(allx(i,:),phase);
                if strcmp(phase,'HeelToe') || strcmp(phase,'AchillesEngaged') || strcmp(phase,'Bottomed') || strcmp(phase,'Unbottomed')
                    theeltoe(i) = allt(i);
                    [~,cforces]=runner.XDoubleDot(allt(i),allx(i,:)',phase);
                    if ~strcmp(phase,'HeelToe') && ~strcmp(phase,'Unbottomed')
                       bottomedconstraint(i,1) = cforces(4); 
                    else
                        bottomedconstraint(i,1) = 0;
                    end
                    lambdas(i,:) = cforces(1:3);
                    heelgrfy(i,1) = cforces(2);
                    cop(i,1) = (heelx(i)*lambdas(i,2) + toex(i)*lambdas(i,3))/(lambdas(i,2)+lambdas(i,3));
                    copvel(i,1:2) = (heelvel(i,1:2)'*lambdas(i,2) + toevel(i,1:2)'*lambdas(i,3)) / (lambdas(i,2)+lambdas(i,3));
                elseif strcmp(phase,'Toe')
                    cop(i,1) = toex(i);
                    copvel(i,1:2) = toevel(i,1:2);
                    bottomedconstraint(i,1) = 0;
                    lambdas(i,[1 3]) = GRF(i,:);
                    lambdas(i,2) = 0;
                    heelgrfy(i,1) = 0;
                else
                    cop(i,1) = NaN;
                    copvel(i,1:2) = [NaN NaN];
                    bottomedconstraint(i,1) = 0;
                    lambdas(i,:) = 0;
                    heelgrfy(i,1) = 0;
                end
                cop(i,2) = 0;
                
%                 omegafoot(i) = allx(i,11);
%                 rdist(i,1:2) = cop(i,1:2)'-[footx(i);footy(i)];
%                 copvel(i,1:2) = [-omegafoot(i)*rdist(i,2);omegafoot(i)*rdist(i,1)]+footvel(i,1:2)';
                rtoefoot = 1.2*(points.toe - points.foot);
                if i>bdex
                copvel(i,1:2) = footvel(i,:) + allx(i,11)*[-rtoefoot(2) rtoefoot(1)];
                end

                distalfootpower(i,1) = dot(GRF(i,1:2),copvel(i,1:2));
                
%                 distalfootpower(i,1) = dot(GRF(i,1:2),footvel(i,1:2));
% distalfootpower(i,1) = dot(GRF(i,1:2),(heelvel(i,1:2)+footvel(i,1:2)+toevel(i,1:2))/3);
                

%                 Jc = runner.getConstraints(allx(i,:),phase);                
%                 ConstraintError(i,:) = Jc*u;
                
                %                 if abs(energies.Total - energy0) > 1e-3
                %                     error('energy conservation failed');
                %                 end
                
            end
            
%             IDfootangvel = [0;diff(thetacom)];
%             IDfootvel = [[0 0];diff(rcom)];
%             IDtoepos = lt*[cos(alphat+thetacom) sin(alphat+thetacom)];
%             IDtoevel = IDfootvel + [-IDfootangvel.*IDtoepos(:,2) IDfootangvel.*IDtoepos(:,1)];
            
            
            LineWidth=3;
LineSize=3;
TextSize=14;
fontstyle='bold';
fonttype='Times New Roman';


% figure
% plot(allt,heelx)
% hold on
% plot(allt,heely,'g')
% plot(allt,toex,'r')
% plot(allt,toey,'m')
% plot(allt,footx,'y')
% plot(allt,footy,'k')
% legend('heelx','heely','toex','toey','anklex','ankley')

            
%             figure
%             subplot(311)
%             plot(allt,footforce,'LineWidth',3)
%             ylabel('Moment')
%             title('Foot')
%             subplot(312)
%             plot(allt,footangvel,'LineWidth',3)
%             ylabel('Angular Velocity')
%                         subplot(313)
%             plot(allt,footpower,'LineWidth',3)
%             ylabel('Power')
%             xlabel('Time')
%             set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
            
            
%             figure
%             plot(theeltoe,lambdas);
%             hold on
%             plot(allt,GRF(:,1),'m')
%             plot(allt,GRF(:,2),'c')
%             plot(allt,bottomedconstraint,'k')
%             legend('HeelX','HeelY','ToeY','GrfX','GrfY','foot bottommed')
%             
            figure
            subplot(211)
            plot(allt,tote)
            legend('total energy')
            
            subplot(212)
            plot(allt,comWR,'LineWidth',3)
            hold on
            plot(allt,footpower,'r','LineWidth',3)
            plot(allt,achillespower,'c','LineWidth',5)
            plot(allt,legPower,'k','LineWidth',3)
            legend('COM WR','Foot','Achilles','Leg')
            xlabel('Time')
            ylabel('Power')
            set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
% 
%                         figure
%             subplot(211)
%             plot(allt,tote)
%             legend('total energy')
%             
%             subplot(212)
%             plot(allt,comWR,'LineWidth',3)
%             hold on
%             plot(allt,footpower+achillespower,'r','LineWidth',3)
% %             plot(allt,achillespower,'c','LineWidth',5)
%             plot(allt,legPower,'k','LineWidth',3)
%             legend('COM WR','Foot-Achilles','Leg')
%             xlabel('Time')
%             ylabel('Power')
%             set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
% 
%             
            
% figure
% subplot(211)
% plot(allt,footangvel)
% hold on
% plot(allt,footangle,'r')
% plot(allt,footspring,'g')
% plot(allt,phasevec,'k')
% legend('foot ang vel','foot ang','footspring','phasevec')
% 
% subplot(212)
% plot(allt,achillesangvel)
% hold on
% plot(allt,achillesangle,'r')
% plot(allt,achillesspring,'g')
% plot(allt,phasevec,'k')
% legend('ach ang vel','ach ang','ach spring','phasevec')

figure
subplot(311)
plot(allt,GRF,'LineWidth',3)
hold on
plot([tair tair],get(gca,'YLim'),'k--')
set(gca,'XLim',[0 tf])
title('Foot')
legend('Horizontal GRF','Vertical GRF')
ylabel('Force')
subplot(312)
plot(allt,copvel,'LineWidth',3)
hold on
plot([tair tair],get(gca,'YLim'),'k--')
set(gca,'XLim',[0 tf])
legend('Contact vel horizontal','Contact vel vertical')
ylabel('Velocity')
subplot(313)
plot(allt,distalfootpower,'LineWidth',3)
hold on
plot([tair tair],get(gca,'YLim'),'k--')
set(gca,'XLim',[0 tf])
legend('distal foot power')
ylabel('Power')
xlabel('Time')
set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)

sdex = find(allt==tair,1);
stancetime = linspace(0,100,sdex);
figure
s1=subplot(311);
plot([0 100],[0 0],'k','LineWidth',1.5)
hold on
plot(stancetime,achillespower(1:sdex),'LineWidth',2,'Color',[127 133 255]/255)

% l1=legend('Achilles/Ankle','Location','North');
ylabel('Power (Dimensionless)')
% set(l1,'Box','Off');
s2=subplot(312);
plot([0 100],[0 0],'k','LineWidth',1.5)
hold on
plot(stancetime,distalfootpower(1:sdex),'k--','LineWidth',2)
hold on
plot(stancetime,footpower(1:sdex),'r','LineWidth',2)

% l2=legend('Distal Foot','True Foot','Location','South');
% set(l2,'Box','Off');
ylabel('Power (Dimensionless)')

subplot(313)
plot([0 100],[0 0],'k','LineWidth',1.5)
hold on
plot(stancetime,distalfootpower(1:sdex)+achillespower(1:sdex),'k--','LineWidth',2)
hold on

% plot(stancetime,footpower(1:sdex)+achillespower(1:sdex),'LineWidth',3,'Color',[168 121 196]/255)
col1 = [1 0 0];
col2 = [127 133 255]/255;
for i = 1:sdex-1
    if footpower(i)~=0 || achillespower(i) ~=0
    w1 = abs(footpower(i))/(abs(footpower(i))+abs(achillespower(i)));
    w2 = abs(achillespower(i))/(abs(footpower(i))+abs(achillespower(i)));
    col = (w1*col1 + w2*col2)/(w1+w2);
    col(col>1)=1; col(col<0)=0;
    else
        col = (col1+col2)/2;
    end

    plot(stancetime(i:i+1),footpower(i:i+1)+achillespower(i:i+1),'Color',col,'LineWidth',2);
end
plot([0 100],[0 0],'k','LineWidth',1.5)
xlabel('Stance Time %')
ylabel('Power (Dimensionless)')
% l3=legend('Combined Ankle-Foot','True Ankle-Foot','Location','South');
% set(l3,'Box','Off');
xlim = get(gca,'XLim'); ylim = get(gca,'YLim');
yticks = [-.4 -.2 0 .2 .4]; ylabels = {-.4 -0.2  0  0.2 0.4};
set(gca,'YTick',yticks,'YTickLabel',ylabels,'YAxisLocation','right','Box','Off','XTickLabel',[]);
set(s1,'XLim',xlim,'YLim',ylim,'YTick',yticks,'YTickLabel',ylabels,'YAxisLocation','right','XTickLabel',[],'Box','Off'); 
set(s2,'XLim',xlim,'YLim',ylim,'YTick',yticks,'YTickLabel',ylabels,'YAxisLocation','right','XTickLabel',[],'Box','Off');

set(gcf,'Units','points')
set(gcf,'Position',[200 200 350 550])
set(findall(gcf, '-property', 'FontSize'), 'FontSize', TextSize, 'fontWeight', fontstyle,'FontName',fonttype)
% subplot(212)
% plot(allt,cop(:,1))
% hold on
% plot(allt,copvel,'k')
% plot(allt,GRF,'r')
% legend('cop','copvel','GRF')

keyboard;
            end

            %% Animation
            figure
            if saveAnimation
                obj = VideoWriter(aviname);
                open(obj);
            end
            
            for i = 1 : 4 : length(allx)
                cla;
                if saveAnimation
                    runner.plot(allx(i,:),'aviWriter',obj);
                else
                    runner.plot(allx(i, :));
                end
                pause(0.01);
                
            end
            
            if saveAnimation
                close(obj);
            end
            

        end
        

        
    end
    
methods
    %% Class Constructor and Properties-related Functions
    function [this] = FootAchillesRunner(input)
        %%
        this = this@Runner();
        
        if (nargin == 1 && ~isempty(input))
            parmnames = fieldnames('this');
            for i = 1:length(parmnames)
                pstring = sprintf(['this.' parmnames{i} ' = input.%g'],this.(parmnames{i}));
                eval(pstring);
            end
        end
        
    end
    
    function [kachilles] = getachilles(this,state)
       if state(3) > this.achillessetpoint;
           kachilles = 0;
       else
           kachilles = this.kachilles;
       end
    end
    
    function [kfoot] = getkfoot(this,state)

        pt1 = this.transitionstart;
        pt2 = this.transitionend;
        StateStruc = FootAchillesRunnerState(state);
        toe = StateStruc.toe.Angle;
        heel = StateStruc.heel.Angle;
        foot = toe - heel;
        legvel = StateStruc.stanceLeg.LengthDot;
        
        if foot/this.footangle < pt1 
            kfoot  = this.kimpact;
        elseif foot/this.footangle >= pt1 && foot/this.footangle <= pt2 % Transition Region
            kfoot  = interp1([pt1 pt2],[this.kimpact this.kstance],foot/this.footangle); %linear interpolation
        else %We are past the transition region
            kfoot  = this.kstance;
        end
        
%         if legpos/this.lleg >= pt1 && legvel<=0  %Impact phase
%             kfoot  = this.kimpact;
%         elseif legpos/this.lleg <= pt1 && legpos/this.lleg >= pt2 && legvel<=0 % Transition Region
%             kfoot  = interp1([pt1 pt2],[this.kimpact this.kstance],legpos); %linear interpolation
%         else %We are past the transition region
%             kfoot  = this.kstance;
%         end
    end
    
    %% Simulation Functions
    function [xf,tf,allx,allt,tair,this,phasevec]= onestep(this, x0,varargin)
        %%
        RelTol = 1e-6; %10; %
        AbsTol = 1e-6; %10; %
        tmax = 2; %2; %6;
        dt = 1e-2;
        interleaveAnimation = 0; %1; %
        interleaveAnimationFrameskip = 2;
        aviname = [];
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'interleaveAnimation'
                    interleaveAnimation = value;
                case 'interleaveAnimationFrameskip'
                    interleaveAnimationFrameskip = value;
                case 'tmax'
                    tmax = value;
                case 'aviname'
                    aviname=value;
            end
        end
           

      %%
      this.getQandUdefs(x0);
      x0 = this.GoodInitialConditions(x0);
      xpre = x0;
      
      sim = 1;
      phasenum= 1;
      tstart = 0;
      allt = [];
      allx = [];
      phasevec = [];
      phaseevents = { @(t,x) this.HeelToeEvents(t,x), @(t,x) this.BottomedEvents(t,x), @(t,x) this.AchillesEngagedEvents(t,x,forcethresh) ...
                      @(t,x) this.UnbottomedEvents(t,x) , @(t,x) this.ToeEvents(t,x)  , @(t,x) this.AerialEvents(t,x) };
      
                  
                  while sim
                      %% Phase transition & Integration
                      phase =  this.phases{phasenum}; %Get name of phase corresponding to phasenum
                      [x0,~] = this.phaseTransition(0,x0,phase); % Ensure initial conditions meet constraints of phase
                      if strcmp(phase,'Bottomed')
                          [~,cfs] = this.XDoubleDot(tstart,x0,phase);
                          forcethresh = cfs(4); %Keep track of force threshold for unbottoming
                          phaseevents{3} = @(t,x) this.AchillesEngagedEvents(t,x,forcethresh); %update event function
                      elseif strcmp(phase,'Aerial')
                          x0([3 5 6]) = xpre([3 5 6]); %Move leg to collision config
                          
                      elseif strcmp(phase,'AchillesEngaged')
                          this.achillesangle = x0(3) - x0(6);
                          phaseevents{4} = @(t,x) this.UnbottomedEvents(t,x);
                          phaseevents{5} = @(t,x) this.ToeEvents(t,x);
                          phaseevents{6} = @(t,x) this.AerialEvents(t,x);
                      end
                      opts = odeset('Events', phaseevents{phasenum},'RelTol',RelTol','AbsTol',AbsTol); %Set integration options
                      [t,x,~,~,ie] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tstart:dt:tstart+tmax,x0,opts); %Integrate dynamics
                      ie = ie(1);
                      
                      %%  Recording & Concatenating Integration Results
                      allt = [allt;t]; allx = [allx;x]; phasevec = [phasevec; phasenum*ones(length(t),1)];
                      tstart = allt(end);
                      x0 = allx(end,:);
                      
                      %% Decide which Phase to Move To
                      [eventpossibilities] = phaseevents{phasenum}(tstart,x0');
                      fallevent = length(eventpossibilities); %Event in which model falls for that phase
                      
                      if ie == fallevent || phasenum == 6  %Fallen or reached last phase
                          sim = 0;
                      elseif ie ==1 %First event is reserved for expected behavior
                          
                          phasenum = phasenum+1; %Move forward one phase
                          
                      elseif phasenum == 2 || phasenum == 3
                          
                          if ie==2 %heel comes off early
                              phasenum = 5; %go to toe phase
                          else %Toe comes off early
                              sim = 0; %Dont currently model heel-only contact so end simulation
                          end
                          
                      else
                          sim = 0;
                      end
                      
                  end
                  
                  xf = allx(end,:); tf = allt(end); tair = allt(find(phasevec==6,1));
                  if isempty(tair)
                      tair = tf;
                  end
                  
    
      
      if (interleaveAnimation)
          
          if ~isempty(aviname)
              obj = VideoWriter(aviname);
              open(obj);
          end
        
        for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
            cla;
            if ~isempty(aviname)
                this.plot(allx(i,:),'aviWriter',obj);
            else
                this.plot(allx(i, :));
            end
            pause(0.01);
            
        end
        
        if ~isempty(aviname)
            close(obj);
        end
        
      end
      
      
        
    end
    
    function  [value, isTerminal, direction] = Fall(this,t,state)
        points = this.getPoints(state);
        fell = points.COMpos(2);
        value=fell;
        isTerminal=1;
        direction=0;
    end
    
    function [value, isTerminal, direction]  = HeelToeEvents(this,t,state)
        
        %Bottoming Out
        value = state(11)-state(12); %foot deformation velocity
        direction = -1;
        isTerminal = 1;
        
        [~, constraintForces] = this.XDoubleDot(t,state,'HeelToe');
        %Heel Comes Off
        value(2,1) = constraintForces(2); isTerminal(2,1) = 1; direction(2,1) = -1;
        %Toe COMES Off
        value(3,1) = constraintForces(3); isTerminal(3,1) = 1; direction(3,1) = -1;
        
        
        [value(4,1),isTerminal(4,1),direction(4,1)] = this.Fall(t,state);
    end
    
    function [value, isTerminal, direction]  = BottomedEvents(this,t,state)
        
        %Achilles Engages
        value = state(3)-this.achillessetpoint;
        direction = -1;
        isTerminal = 1;
        
        [~, constraintForces] = this.XDoubleDot(t,state,'Bottomed');
        %Heel Comes Off
        value(2,1) = constraintForces(2); isTerminal(2,1) = 1; direction(2,1) = -1;
        %Toe COMES Off
        value(3,1) = constraintForces(3); isTerminal(3,1) = 1; direction(3,1) = -1;
        
        [value(4,1),isTerminal(4,1),direction(4,1)] = this.Fall(t,state);
        
    end
        
    function [value, isTerminal, direction] = AchillesEngagedEvents(this,t,state,forcethreshold)
            
            [~, constraintForces] = this.XDoubleDot(t,state,'AchillesEngaged');
            %Foot Unbottoms Comes off
            value = forcethreshold - constraintForces(4); isTerminal = 1; direction = -1;
            %Heel Comes off
            value(2,1) = constraintForces(2); isTerminal(2,1) = 1; direction(2,1) = 0;
            %Toe Comes off
            value(3,1) = constraintForces(3); isTerminal(3,1) = 1; direction(3,1) = 0;
            
            %Falling
            [value(4,1),isTerminal(4,1),direction(4,1)] = this.Fall(t,state);
            
        end
        
     function [value, isTerminal, direction] = UnbottomedEvents(this,t,state)
        
        [~, constraintForces] = this.XDoubleDot(t,state,'Unbottomed');
        %Heel Comes off
        value = constraintForces(2); isTerminal = 1; direction = 0;
        %Toe Comes off
        value(2,1) = constraintForces(3); isTerminal(2,1) = 1; direction(2,1) = -1;
        
        %Falling
        [value(3,1),isTerminal(3,1),direction(3,1)] = this.Fall(t,state);
        
    end
    
    function [value, isTerminal, direction] = ToeEvents(this,t,state)
        
        [~, constraintForces] = this.XDoubleDot(t,state,'Toe');
        %Toe Comes off
        value = constraintForces(2); isTerminal = 1; direction = -1;
        
        %Falling
        [value(2,1),isTerminal(2,1),direction(2,1)] = this.Fall(t,state);
        
        
    end
    
    function [value, isTerminal, direction] = AerialEvents(this,t,state)
        points = this.getPoints(state);
        
        %Hitting the Ground
        value = points.heel(2);
        isTerminal = 1;
        direction = -1;
        
        %Falling
        [value(2,1),isTerminal(2,1),direction(2,1)] = this.Fall(t,state);
    end
    
    function [xnew] = switchLegs(this,x,landangle)
       %Switch the leg angle at collision to be the touchdown angle, and
       %the length of the leg spring to be at rest.
       
%        newstatestruc = FootAchillesRunnerState(x);
%        newstatestruc.stanceLeg.Angle = landangle;
%        newstatestruc.stanceLeg.Length = this.lleg;
       
       xnew = x;
 
    end
    
    function [newstate] = GoodInitialConditions(this,x0)
        this.getQandUdefs(x0);
        
        %Set state velocities to be consistent with constraints while being
        %least squares close to inputted velocities
        Jc = this.getConstraints(x0,'HeelToe');
        nullJc = null(Jc);
        unew = nullJc * (nullJc \ x0(7:12)');
        newstate = [x0(1:6) unew'];
        
        %Shift body (without changing configuration) so that heel is on ground
        points = this.getPoints(newstate);
        pelvx = points.pelvis(1) - points.heel(1);
        pelvy = points.pelvis(2) - points.heel(2);
        newstate([1 2])=[pelvx pelvy];
        
        %Put toe on ground
        points = this.getPoints(newstate);
        toeang = asin(-points.foot(2)/this.ltoe);
        newstate(5) = toeang;
        
        %       u = newstate(7:12)';
        %       Jc*u; %should be zero if constraints are working
        
    end
    
    %% Graphical Functions 
    function plot(this, state, varargin)
        %%
        
        aviWriter = [];
        shouldAntiAlias = 0;
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'aviWriter'
                    aviWriter = value;
                case 'shouldAntiAlias'
                    shouldAntiAlias = value;
                    
            end
        end
        
        %%
        
        
        points = this.getPoints(state);
        
        plotter = RunnerPlotter;
        
        %Draw Ground
        plot([-5 5],[0 0],'k','LineWidth',2)
        
        hold on
        
        
        %Draw Wall Lines
        %       linesStart = floor(xLims(1) * 5) / 5;
        %       linesEnd = floor(xLims(2) * 5) / 5;
        %       linesToDraw = linesStart : 0.2 : linesEnd;
        %
        %       for x = linesToDraw
        %           line([x x], [0 yLims(2)], 'color', ones(3,1) * 0.3)
        %       end
        
        
        %Draw Lines
        %       plotter.plotLine(points.foot,points.pelvis);
        %       plotter.plotLine(points.pelvis,points.soft);
        plotter.plotLine(points.foot,points.heel);
        plotter.plotLine(points.foot,points.toe);
        
        %Draw Springs
        numcoils=3;
        springwidth=.07;
        plotter.plotSpring(points.foot(1),points.foot(2),...
            points.pelvis(1),points.pelvis(2),...
            numcoils,this.lleg,springwidth) %leg spring
        
        numcoils=3;
        springwidth=.02;
        
        pdir = (points.pelvis - points.foot)/norm(points.pelvis - points.foot);
        achpoint = points.foot + .25 * this.lleg * pdir;
        
        plotter.plotAngSpring(achpoint,points.heel,points.foot,2,.05,'Color',[127 133 255]/255) %achilles spring
        
        numcoils=3;
        springwidth=.02;
        linrestlength = sqrt(this.lheel^2 + this.ltoe^2 - 2*this.lheel*this.ltoe*cos(this.footangle));
%         plotter.plotAngSpring(points.heel,points.toe,points.foot,2,.05,'Color',[1 0 0]) %achilles spring
        plotter.plotSpring(points.heel(1),points.heel(2),points.toe(1),points.toe(2),...
                           numcoils,linrestlength,springwidth,'Color',[1 0 0]);

        %Draw Masses
        plotter.plotMass(points.pelvis);
%         plotter.plotMass(points.foot,'scaling',this.mfoot/this.mpelvis);
%         plotter.plotMass(points.heel,'scaling',this.mheel/this.mpelvis);
        
        axis equal;
        
        %Set Axis Limits
        xLims = [points.pelvis(1)]+ [-1 1];
        xlim(xLims);
        
        yLims = [points.pelvis(2)] + [-1.5 .5];
        ylim(yLims);
        
        
        
        
        set(gcf, 'color', 'w');
        
        axis off;
        box off;
        
        set(gca, 'ActivePositionProperty', 'OuterPosition');
        set(gca, 'Position', [0, 0, 1, 1]);
        
        
        if (~isempty(aviWriter))
            if (shouldAntiAlias)
                antiAliasedFig = myaa();
                drawnow;
                pause(0.01);
                thisFrame = getframe(antiAliasedFig);
                close(antiAliasedFig);
            else
                thisFrame = getframe(gcf);
            end
            writeVideo(aviWriter, thisFrame);
        end
        
        
    end
    
    function [speed,stepfreq,steplength,airfrac] = anim(this,x0,varargin)
        tmax=5;
        frameskip=2;
        aviname=[];
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'frameskip'
                    interleaveAnimationFrameskip = value;
                case 'tmax'
                    tmax = value;
                case 'aviname'
                    aviname = value;
            end
        end
        
        [xf, tf, allx, allt, tair]= onestep(this, x0,'aviname',aviname,'interleaveAnimation',1,'interLeaveAnimationFrameSkip',frameskip,'tmax',tmax);
        [speed,steplength,stepfreq,airfrac] = getGaitChar(this,x0,tf,xf,tair);
        
    end

    
    %% Mathematica Output (Copy and paste appropiate matrices into these functions)
    function [MM,rhs] = getMMandRHS(this,time,state)
        %%
        this.getParams();
        this.getQandUdefs(state);
        [kachilles] = this.getachilles(state);
        kfoot = this.getkfoot(state);
        
       c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
MM = zeros(6,6); rhs = zeros(6,1);

% Mass Matrix
MM(1,1) = mfoot + mheel + mpelvis + mtoe; MM(1,2) = 0; MM(1,3) = ...
-(q4*s3*(mfoot + mheel + mtoe)); MM(1,4) = c3*(mfoot + mheel + mtoe); MM(1,5) ...
= -(s5*ltoe*mtoe); MM(1,6) = -(s6*lheel*mheel); 
MM(2,1) = MM(1,2); MM(2,2) = mfoot + mheel + mpelvis + mtoe; MM(2,3) = ...
c3*q4*(mfoot + mheel + mtoe); MM(2,4) = s3*(mfoot + mheel + mtoe); MM(2,5) = ...
c5*ltoe*mtoe; MM(2,6) = c6*lheel*mheel; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = (mfoot + mheel + ...
mtoe)*(q4*q4); MM(3,4) = 0; MM(3,5) = c3m5*q4*ltoe*mtoe; MM(3,6) = ...
c3m6*q4*lheel*mheel; 
MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = mfoot + ...
mheel + mtoe; MM(4,5) = s3m5*ltoe*mtoe; MM(4,6) = s3m6*lheel*mheel; 
MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
MM(5,5) = mtoe*(ltoe*ltoe); MM(5,6) = 0; 
MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
MM(6,5) = MM(5,6); MM(6,6) = mheel*(lheel*lheel); 

% righthand side terms
rhs(1) = 2*u4*(c3*cleg + s3*u3*(mfoot + mheel + mtoe)) + c3*q4*(mfoot + mheel ...
+ mtoe)*(u3*u3) + c5*ltoe*mtoe*(u5*u5) + c6*lheel*mheel*(u6*u6) + ...
g*mfoot*sin(gslope) + g*mheel*sin(gslope) + g*mpelvis*sin(gslope) + ...
g*mtoe*sin(gslope); 
rhs(2) = 2*u4*(s3*cleg - c3*u3*(mfoot + mheel + mtoe)) - g*mfoot*cos(gslope) ...
- g*mheel*cos(gslope) - g*mpelvis*cos(gslope) - g*mtoe*cos(gslope) + ...
q4*s3*(mfoot + mheel + mtoe)*(u3*u3) + s5*ltoe*mtoe*(u5*u5) + ...
s6*lheel*mheel*(u6*u6); 
rhs(3) = -(u3*cachilles) + u6*cachilles - q3*kachilles + q6*kachilles + ...
achillesangle*kachilles - q4*(2*u3*u4*(mfoot + mheel + mtoe) + g*mfoot*cos(q3 ...
- gslope) + g*mheel*cos(q3 - gslope) + g*mtoe*cos(q3 - gslope) + ...
s3m5*ltoe*mtoe*(u5*u5) + s3m6*lheel*mheel*(u6*u6)); 
rhs(4) = u4*cleg + kleg*lleg + q4*(-kleg + (mfoot + mheel + mtoe)*(u3*u3)) + ...
c3m5*ltoe*mtoe*(u5*u5) + c3m6*lheel*mheel*(u6*u6) - g*mfoot*sin(q3 - gslope) ...
- g*mheel*sin(q3 - gslope) - g*mtoe*sin(q3 - gslope); 
rhs(5) = -(u5*cfoot) + u6*cfoot - q5*kfoot + q6*kfoot + footangle*kfoot - ...
2*c3m5*u3*u4*ltoe*mtoe - g*ltoe*mtoe*cos(q5 - gslope) + ...
q4*s3m5*ltoe*mtoe*(u3*u3); 
rhs(6) = (u3 - u6)*cachilles + (u5 - u6)*cfoot - (-q3 + q6 + ...
achillesangle)*kachilles - (-q5 + q6 + footangle)*kfoot - ...
2*c3m6*u3*u4*lheel*mheel - g*lheel*mheel*cos(q6 - gslope) + ...
q4*s3m6*lheel*mheel*(u3*u3); 

        
    end
    
    function [C, CDot] = getConstraints(this,state,phase)
        %%
        this.getParams();
        this.getQandUdefs(state);
        [kachilles] = this.getachilles(state);
        kfoot = this.getkfoot(state);
        
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
        switch phase      
                
            case {'HeelToe'}
constraintJacobianHeelToe(1,1) = 1; constraintJacobianHeelToe(1,2) = 0; ...
constraintJacobianHeelToe(1,3) = -(q4*s3); constraintJacobianHeelToe(1,4) = ...
c3; constraintJacobianHeelToe(1,5) = 0; constraintJacobianHeelToe(1,6) = ...
-(s6*lheel); 
constraintJacobianHeelToe(2,1) = 0; constraintJacobianHeelToe(2,2) = 1; ...
constraintJacobianHeelToe(2,3) = c3*q4; constraintJacobianHeelToe(2,4) = s3; ...
constraintJacobianHeelToe(2,5) = 0; constraintJacobianHeelToe(2,6) = ...
c6*lheel; 
constraintJacobianHeelToe(3,1) = 0; constraintJacobianHeelToe(3,2) = 1; ...
constraintJacobianHeelToe(3,3) = c3*q4; constraintJacobianHeelToe(3,4) = s3; ...
constraintJacobianHeelToe(3,5) = c5*ltoe; constraintJacobianHeelToe(3,6) = 0; 


constraintJacobianHeelToeDot(1,1) = 0; constraintJacobianHeelToeDot(1,2) = 0; ...
constraintJacobianHeelToeDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianHeelToeDot(1,4) = -(s3*u3); ...
constraintJacobianHeelToeDot(1,5) = 0; constraintJacobianHeelToeDot(1,6) = ...
-(c6*u6*lheel); 
constraintJacobianHeelToeDot(2,1) = 0; constraintJacobianHeelToeDot(2,2) = 0; ...
constraintJacobianHeelToeDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianHeelToeDot(2,4) = c3*u3; constraintJacobianHeelToeDot(2,5) ...
= 0; constraintJacobianHeelToeDot(2,6) = -(s6*u6*lheel); 
constraintJacobianHeelToeDot(3,1) = 0; constraintJacobianHeelToeDot(3,2) = 0; ...
constraintJacobianHeelToeDot(3,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianHeelToeDot(3,4) = c3*u3; constraintJacobianHeelToeDot(3,5) ...
= -(s5*u5*ltoe); constraintJacobianHeelToeDot(3,6) = 0; 



C = constraintJacobianHeelToe;
CDot = constraintJacobianHeelToeDot;

            case {'Bottomed' , 'AchillesEngaged', 'Unbottomed'}
                
constraintJacobianBottomed(1,1) = 1; constraintJacobianBottomed(1,2) = 0; ...
constraintJacobianBottomed(1,3) = -(q4*s3); constraintJacobianBottomed(1,4) = ...
c3; constraintJacobianBottomed(1,5) = -(s5*ltoe); ...
constraintJacobianBottomed(1,6) = 0; 
constraintJacobianBottomed(2,1) = 0; constraintJacobianBottomed(2,2) = 1; ...
constraintJacobianBottomed(2,3) = c3*q4; constraintJacobianBottomed(2,4) = ...
s3; constraintJacobianBottomed(2,5) = 0; constraintJacobianBottomed(2,6) = ...
c6*lheel; 
constraintJacobianBottomed(3,1) = 0; constraintJacobianBottomed(3,2) = 1; ...
constraintJacobianBottomed(3,3) = c3*q4; constraintJacobianBottomed(3,4) = ...
s3; constraintJacobianBottomed(3,5) = c5*ltoe; ...
constraintJacobianBottomed(3,6) = 0; 
constraintJacobianBottomed(4,1) = 0; constraintJacobianBottomed(4,2) = 0; ...
constraintJacobianBottomed(4,3) = 0; constraintJacobianBottomed(4,4) = 0; ...
constraintJacobianBottomed(4,5) = 1; constraintJacobianBottomed(4,6) = -1; 


constraintJacobianBottomedDot(1,1) = 0; constraintJacobianBottomedDot(1,2) = ...
0; constraintJacobianBottomedDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianBottomedDot(1,4) = -(s3*u3); ...
constraintJacobianBottomedDot(1,5) = -(c5*u5*ltoe); ...
constraintJacobianBottomedDot(1,6) = 0; 
constraintJacobianBottomedDot(2,1) = 0; constraintJacobianBottomedDot(2,2) = ...
0; constraintJacobianBottomedDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianBottomedDot(2,4) = c3*u3; ...
constraintJacobianBottomedDot(2,5) = 0; constraintJacobianBottomedDot(2,6) = ...
-(s6*u6*lheel); 
constraintJacobianBottomedDot(3,1) = 0; constraintJacobianBottomedDot(3,2) = ...
0; constraintJacobianBottomedDot(3,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianBottomedDot(3,4) = c3*u3; ...
constraintJacobianBottomedDot(3,5) = -(s5*u5*ltoe); ...
constraintJacobianBottomedDot(3,6) = 0; 
constraintJacobianBottomedDot(4,1) = 0; constraintJacobianBottomedDot(4,2) = ...
0; constraintJacobianBottomedDot(4,3) = 0; constraintJacobianBottomedDot(4,4) ...
= 0; constraintJacobianBottomedDot(4,5) = 0; ...
constraintJacobianBottomedDot(4,6) = 0; 

C = constraintJacobianBottomed;
CDot = constraintJacobianBottomedDot;

if strcmp(phase,'Unbottomed')
   C=C(1:3,:); CDot=CDot(1:3,:); 
end

%%%%%%%%%%%%%%%%%%% Let both heel & toe slide in x direction
% C=C(2:3,:);
% CDot=CDot(2:3,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case 'Toe'

% can't move either direction

% constraintJacobianToe(1,1) = 1; constraintJacobianToe(1,2) = 0; ...
% constraintJacobianToe(1,3) = -(q4*s3); constraintJacobianToe(1,4) = c3; ...
% constraintJacobianToe(1,5) = -(s5*ltoe); constraintJacobianToe(1,6) = 0; 
% constraintJacobianToe(2,1) = 0; constraintJacobianToe(2,2) = 1; ...
% constraintJacobianToe(2,3) = c3*q4; constraintJacobianToe(2,4) = s3; ...
% constraintJacobianToe(2,5) = c5*ltoe; constraintJacobianToe(2,6) = 0; 
% 
% 
% constraintJacobianToeDot(1,1) = 0; constraintJacobianToeDot(1,2) = 0; ...
% constraintJacobianToeDot(1,3) = -(c3*q4*u3) - s3*u4; ...
% constraintJacobianToeDot(1,4) = -(s3*u3); constraintJacobianToeDot(1,5) = ...
% -(c5*u5*ltoe); constraintJacobianToeDot(1,6) = 0; 
% constraintJacobianToeDot(2,1) = 0; constraintJacobianToeDot(2,2) = 0; ...
% constraintJacobianToeDot(2,3) = -(q4*s3*u3) + c3*u4; ...
% constraintJacobianToeDot(2,4) = c3*u3; constraintJacobianToeDot(2,5) = ...
% -(s5*u5*ltoe); constraintJacobianToeDot(2,6) = 0; 

%rolling constraint

constraintJacobianToe(1,1) = 1; constraintJacobianToe(1,2) = 0; ...
constraintJacobianToe(1,3) = -(q4*s3); constraintJacobianToe(1,4) = c3; ...
constraintJacobianToe(1,5) = -(s5*ltoe); constraintJacobianToe(1,6) = 0; 
constraintJacobianToe(2,1) = 0; constraintJacobianToe(2,2) = 1; ...
constraintJacobianToe(2,3) = c3*q4; constraintJacobianToe(2,4) = s3; ...
constraintJacobianToe(2,5) = c5*ltoe; constraintJacobianToe(2,6) = 0; 


% constraintJacobianToe(3,1) = 0; constraintJacobianToe(3,2) = 0; ...
% constraintJacobianToe(3,3) = -kachilles; constraintJacobianToe(3,4) = 0; ...
% constraintJacobianToe(3,5) = kfoot; constraintJacobianToe(3,6) = kachilles - ...
% kfoot; 


constraintJacobianToeDot(1,1) = 0; constraintJacobianToeDot(1,2) = 0; ...
constraintJacobianToeDot(1,3) = -(c3*q4*u3) - s3*u4; ...
constraintJacobianToeDot(1,4) = -(s3*u3); constraintJacobianToeDot(1,5) = ...
-(c5*u5*ltoe); constraintJacobianToeDot(1,6) = 0; 
constraintJacobianToeDot(2,1) = 0; constraintJacobianToeDot(2,2) = 0; ...
constraintJacobianToeDot(2,3) = -(q4*s3*u3) + c3*u4; ...
constraintJacobianToeDot(2,4) = c3*u3; constraintJacobianToeDot(2,5) = ...
-(s5*u5*ltoe); constraintJacobianToeDot(2,6) = 0; 


% constraintJacobianToeDot(3,1) = 0; constraintJacobianToeDot(3,2) = 0; ...
% constraintJacobianToeDot(3,3) = 0; constraintJacobianToeDot(3,4) = 0; ...
% constraintJacobianToeDot(3,5) = 0; constraintJacobianToeDot(3,6) = 0; 



                C = constraintJacobianToe;
                CDot = constraintJacobianToeDot;
                
            case 'Aerial'
constraintJacobianAerial(1,1) = 0; constraintJacobianAerial(1,2) = 0; ...
constraintJacobianAerial(1,3) = 1; constraintJacobianAerial(1,4) = 0; ...
constraintJacobianAerial(1,5) = 0; constraintJacobianAerial(1,6) = 0; 
constraintJacobianAerial(2,1) = 0; constraintJacobianAerial(2,2) = 0; ...
constraintJacobianAerial(2,3) = 0; constraintJacobianAerial(2,4) = 1; ...
constraintJacobianAerial(2,5) = 0; constraintJacobianAerial(2,6) = 0; 
constraintJacobianAerial(3,1) = 0; constraintJacobianAerial(3,2) = 0; ...
constraintJacobianAerial(3,3) = 0; constraintJacobianAerial(3,4) = 0; ...
constraintJacobianAerial(3,5) = 1; constraintJacobianAerial(3,6) = 0; 
constraintJacobianAerial(4,1) = 0; constraintJacobianAerial(4,2) = 0; ...
constraintJacobianAerial(4,3) = 0; constraintJacobianAerial(4,4) = 0; ...
constraintJacobianAerial(4,5) = 0; constraintJacobianAerial(4,6) = 1; 


constraintJacobianAerialDot(1,1) = 0; constraintJacobianAerialDot(1,2) = 0; ...
constraintJacobianAerialDot(1,3) = 0; constraintJacobianAerialDot(1,4) = 0; ...
constraintJacobianAerialDot(1,5) = 0; constraintJacobianAerialDot(1,6) = 0; 
constraintJacobianAerialDot(2,1) = 0; constraintJacobianAerialDot(2,2) = 0; ...
constraintJacobianAerialDot(2,3) = 0; constraintJacobianAerialDot(2,4) = 0; ...
constraintJacobianAerialDot(2,5) = 0; constraintJacobianAerialDot(2,6) = 0; 
constraintJacobianAerialDot(3,1) = 0; constraintJacobianAerialDot(3,2) = 0; ...
constraintJacobianAerialDot(3,3) = 0; constraintJacobianAerialDot(3,4) = 0; ...
constraintJacobianAerialDot(3,5) = 0; constraintJacobianAerialDot(3,6) = 0; 
constraintJacobianAerialDot(4,1) = 0; constraintJacobianAerialDot(4,2) = 0; ...
constraintJacobianAerialDot(4,3) = 0; constraintJacobianAerialDot(4,4) = 0; ...
constraintJacobianAerialDot(4,5) = 0; constraintJacobianAerialDot(4,6) = 0; 

                C = constraintJacobianAerial;
                CDot = constraintJacobianAerialDot;
            otherwise
                error('Unknown phase for running model: %s', phase);
                
                
        end
        
    end
    
    function [J] = getVelJacob(this,state)
                this.getParams();
                this.getQandUdefs(state);

       c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
        
velJacobian(1,1) = 1; velJacobian(1,2) = 0; velJacobian(1,3) = 0; ...
velJacobian(1,4) = 0; velJacobian(1,5) = 0; velJacobian(1,6) = 0; 
velJacobian(2,1) = 0; velJacobian(2,2) = 1; velJacobian(2,3) = 0; ...
velJacobian(2,4) = 0; velJacobian(2,5) = 0; velJacobian(2,6) = 0; 
velJacobian(3,1) = 0; velJacobian(3,2) = 0; velJacobian(3,3) = 0; ...
velJacobian(3,4) = 0; velJacobian(3,5) = 0; velJacobian(3,6) = 0; 
velJacobian(4,1) = 1; velJacobian(4,2) = 0; velJacobian(4,3) = -(q4*s3); ...
velJacobian(4,4) = c3; velJacobian(4,5) = 0; velJacobian(4,6) = 0; 
velJacobian(5,1) = 0; velJacobian(5,2) = 1; velJacobian(5,3) = c3*q4; ...
velJacobian(5,4) = s3; velJacobian(5,5) = 0; velJacobian(5,6) = 0; 
velJacobian(6,1) = 0; velJacobian(6,2) = 0; velJacobian(6,3) = 1; ...
velJacobian(6,4) = 0; velJacobian(6,5) = 0; velJacobian(6,6) = 0; 
velJacobian(7,1) = 1; velJacobian(7,2) = 0; velJacobian(7,3) = -(q4*s3); ...
velJacobian(7,4) = c3; velJacobian(7,5) = 0; velJacobian(7,6) = -(s6*lheel); 
velJacobian(8,1) = 0; velJacobian(8,2) = 1; velJacobian(8,3) = c3*q4; ...
velJacobian(8,4) = s3; velJacobian(8,5) = 0; velJacobian(8,6) = c6*lheel; 
velJacobian(9,1) = 0; velJacobian(9,2) = 0; velJacobian(9,3) = 0; ...
velJacobian(9,4) = 0; velJacobian(9,5) = 0; velJacobian(9,6) = 1; 
velJacobian(10,1) = 1; velJacobian(10,2) = 0; velJacobian(10,3) = -(q4*s3); ...
velJacobian(10,4) = c3; velJacobian(10,5) = -(s5*ltoe); velJacobian(10,6) = ...
0; 
velJacobian(11,1) = 0; velJacobian(11,2) = 1; velJacobian(11,3) = c3*q4; ...
velJacobian(11,4) = s3; velJacobian(11,5) = c5*ltoe; velJacobian(11,6) = 0; 
velJacobian(12,1) = 0; velJacobian(12,2) = 0; velJacobian(12,3) = 0; ...
velJacobian(12,4) = 0; velJacobian(12,5) = 1; velJacobian(12,6) = 0; 


J=velJacobian;
    end
    function [E] = getEnergies(this,state)
        this.getParams();
        this.getQandUdefs(state);
        [kachilles] = this.getachilles(state);
        kfoot = this.getkfoot(state);
        
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
kineticEnergy = (2*c3m6*q4*u3*u6*lheel*mheel + 2*s3m6*u4*u6*lheel*mheel + ...
2*c3m5*q4*u3*u5*ltoe*mtoe + 2*s3m5*u4*u5*ltoe*mtoe - 2*u1*(s6*u6*lheel*mheel ...
+ s5*u5*ltoe*mtoe + q4*s3*u3*(mfoot + mheel + mtoe) - c3*u4*(mfoot + mheel + ...
mtoe)) + 2*u2*(c6*u6*lheel*mheel + c5*u5*ltoe*mtoe + c3*q4*u3*(mfoot + mheel ...
+ mtoe) + s3*u4*(mfoot + mheel + mtoe)) + (mfoot + mheel + mpelvis + ...
mtoe)*(u1*u1) + (mfoot + mheel + mpelvis + mtoe)*(u2*u2) + ...
mfoot*(q4*q4)*(u3*u3) + mheel*(q4*q4)*(u3*u3) + mtoe*(q4*q4)*(u3*u3) + ...
mfoot*(u4*u4) + mheel*(u4*u4) + mtoe*(u4*u4) + mheel*(u6*u6)*(lheel*lheel) + ...
mtoe*(u5*u5)*(ltoe*ltoe))/2.;

potentialEnergy = (-2*q3*q6*kachilles - 2*q3*achillesangle*kachilles + ...
2*q6*achillesangle*kachilles - 2*q5*q6*kfoot - 2*q5*footangle*kfoot + ...
2*q6*footangle*kfoot - 2*q4*kleg*lleg + 2*q2*g*(mfoot + mheel + mpelvis + ...
mtoe)*cos(gslope) + kachilles*(q3*q3) + kleg*(q4*q4) + kfoot*(q5*q5) + ...
kachilles*(q6*q6) + kfoot*(q6*q6) + kachilles*(achillesangle*achillesangle) + ...
kfoot*(footangle*footangle) + kleg*(lleg*lleg) + 2*q4*g*mfoot*sin(q3 - ...
gslope) + 2*q4*g*mheel*sin(q3 - gslope) + 2*q4*g*mtoe*sin(q3 - gslope) + ...
2*g*ltoe*mtoe*sin(q5 - gslope) + 2*g*lheel*mheel*sin(q6 - gslope) - ...
2*q1*g*(mfoot + mheel + mpelvis + mtoe)*sin(gslope))/2.;

PEgrav = -(mpelvis*(-(q2*g*cos(gslope)) + q1*g*sin(gslope))) - ...
mfoot*(-(q2*g*cos(gslope)) - q4*g*sin(q3 - gslope) + q1*g*sin(gslope)) - ...
mtoe*(-(q2*g*cos(gslope)) - q4*s3*g*cos(gslope) - g*ltoe*sin(q5 - gslope) + ...
q1*g*sin(gslope) + c3*q4*g*sin(gslope)) - mheel*(-(q2*g*cos(gslope)) - ...
q4*s3*g*cos(gslope) - g*lheel*sin(q6 - gslope) + q1*g*sin(gslope) + ...
c3*q4*g*sin(gslope));

PEspring = (kachilles*((q3 - q6 - achillesangle)*(q3 - q6 - ...
achillesangle)))/2. + (kfoot*((q5 - q6 - footangle)*(q5 - q6 - ...
footangle)))/2. + (kleg*((q4 - lleg)*(q4 - lleg)))/2.;
        
        E.KE = kineticEnergy;
        E.PE = potentialEnergy;
        E.PEgrav = PEgrav;
        E.PEspring = PEspring;
        E.Total = E.KE + E.PE;
        
    end
    
    function [points] = getPoints(this, state)
        
        this.getParams();
        this.getQandUdefs(state);
        
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
points.foot(1) = q1 + c3*q4; 
points.foot(2) = q2 + q4*s3; 


points.pelvis(1) = q1; 
points.pelvis(2) = q2; 


points.COMpos(1) = c6*lheel*mheel*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
c5*ltoe*mtoe*power(mfoot + mheel + mpelvis + mtoe,-1) + (q1*mfoot + q1*mheel ...
+ q1*mpelvis + q1*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
c3*(q4*mfoot + q4*mheel + q4*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1); 
points.COMpos(2) = s6*lheel*mheel*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
s5*ltoe*mtoe*power(mfoot + mheel + mpelvis + mtoe,-1) + (q2*mfoot + q2*mheel ...
+ q2*mpelvis + q2*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1) + ...
s3*(q4*mfoot + q4*mheel + q4*mtoe)*power(mfoot + mheel + mpelvis + mtoe,-1); 


points.heel(1) = q1 + c3*q4 + c6*lheel; 
points.heel(2) = q2 + q4*s3 + s6*lheel; 


points.toe(1) = q1 + c3*q4 + c5*ltoe; 
points.toe(2) = q2 + q4*s3 + s5*ltoe; 
    end
    
    function [C,Ceq] = MassesAddToOne(this,varargin)
        r = this;
        C=[];
        Ceq = 1 - r.mpelvis - r.mfoot - r.mheel - r.mtoe;
    end
    
    function [comWR] = getcomWR(this,x,phase)
        this.getParams();
        this.getQandUdefs(x);
        
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6); 
        
comvx = (c3*u4*mfoot + c3*u4*mheel - s6*u6*lheel*mheel + c3*u4*mtoe - ...
s5*u5*ltoe*mtoe - q4*s3*u3*(mfoot + mheel + mtoe) + u1*(mfoot + mheel + ...
mpelvis + mtoe))*power(mfoot + mheel + mpelvis + mtoe,-1);

comvy = (s3*u4*mfoot + s3*u4*mheel + c6*u6*lheel*mheel + s3*u4*mtoe + ...
c5*u5*ltoe*mtoe + c3*q4*u3*(mfoot + mheel + mtoe) + u2*(mfoot + mheel + ...
mpelvis + mtoe))*power(mfoot + mheel + mpelvis + mtoe,-1);

        
        GRF = this.getGRF(x,phase);
        
        comWR = dot(GRF,[comvx;comvy]);
    end
    
    %% Additional Dynamics Calculations
    
    function [GRF] = getGRF(this,x,phase)
        
        if size(x,1)==1
           x=x'; 
        end
        [xddot, constraintForces] = this.XDoubleDot(0,x,phase);
        
        if strcmp(phase,'Heel') || strcmp(phase,'Toe')
        GRF = [constraintForces(1);constraintForces(2)];
        elseif strcmp(phase,'HeelToe') || strcmp(phase,'AchillesEngaged') || strcmp(phase,'Bottomed') || strcmp(phase,'Unbottomed')
        GRF = [constraintForces(1);constraintForces(2) + constraintForces(3)];    
        else
           GRF=[0;0]; 
        end
    end
    
    function footPower = getFootPower(this,x)
        sz = FootAchillesRunnerState(x);
        force = this.getFootForce(x);
        velocity = sz.toe.AngleDot - sz.heel.AngleDot;
        footPower = force.*velocity;
    end
    
    function footforce = getFootForce(this,x)
        sz = FootAchillesRunnerState(x);
        kfoot = this.getkfoot(x);
        footforce = -kfoot*(sz.toe.Angle - sz.heel.Angle - this.footangle) ...
            -this.cfoot*(sz.toe.AngleDot - sz.heel.AngleDot);
    end
    
    function legPower = getLegPower(this,x)
        sz = FootAchillesRunnerState(x);
        force = this.getLegForce(x);
        vel = sz.stanceLeg.LengthDot;
        legPower = force.*vel;
    end
    
    function legforce = getLegForce(this,x)
        sz = FootAchillesRunnerState(x);
        legforce = -this.kleg*(sz.stanceLeg.Length - this.lleg) - this.cleg*(sz.stanceLeg.LengthDot);
    end
    
    function achillesPower = getAchillesPower(this,x)
        sz = FootAchillesRunnerState(x);
        force = this.getAchillesForce(x);
        velocity = sz.stanceLeg.AngleDot - sz.heel.AngleDot;
        achillesPower = force.*velocity;
    end
    
    function achillesforce = getAchillesForce(this,x)
        sz = FootAchillesRunnerState(x);
        [kachilles] = this.getachilles(x);
        achillesforce = -kachilles*(sz.stanceLeg.Angle - sz.heel.Angle - this.achillesangle) ...
                        -this.cachilles*(sz.stanceLeg.AngleDot - sz.heel.AngleDot);
    end
    
    %% Other Gait Information
    
        function [speed] = getSpeed(this, x0, xf, tf)
        if isempty(xf)
            [xf,tf] = this.onestep(x0);
        end
        
        %Convert state vectors to descriptive class
        x0struc = FootAchillesRunnerState(x0);
        xfstruc = FootAchillesRunnerState(xf);
        
        speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
    end
    
    function [steplength] = getStepLength(this, x0, xf)
        if isempty(xf)
            [xf] = this.onestep(x0);
        end
        
        %Convert state vectors to descriptive class
        x0struc = FootAchillesRunnerState(x0);
        xfstruc = FootAchillesRunnerState(xf);
        
        steplength = (xfstruc.pelvis.x - x0struc.pelvis.x);
    end
    
    function [airfrac] = getAerialFraction(this, x0, tf, tair)
        if isempty(tair) || isempty(tair)
            [xf,tf,allx,allt,tair] = this.onestep(x0);
        end
        airfrac = (tf-tair)/tf;
    end
    
    function [speed,steplength,stepfreq,airfrac] = getGaitChar(this,x0,tf,xf,tair)
            [speed] = this.getSpeed(x0, xf, tf);
            [steplength] = this.getStepLength(x0, xf);
            stepfreq = speed/steplength;
            [airfrac] = this.getAerialFraction(x0, tf, tair);
    end
    
        function [A, B] = linearizeOneStepReturnMap(this, x0, parameters, varargin)
        %%
        perturbationAmount = 1e-4;
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'perturbationAmount'
                    perturbationAmount = value;
            end
        end
        
        if (isempty(varargin))
            varargin = {'', {}};
        end
        
        %%
        for i = 1 : length(parameters)
            B(i, :) = this.getSensitivityToParameters(x0, parameters{i}, ...
                'perturbationAmount', perturbationAmount, varargin{:})';
        end
        
        A = this.getSensitivtyToStates(x0, 'perturbationAmount', perturbationAmount, varargin{:});
    end
    
    function [AMatrix] = getSensitivityToStates(this, x0, varargin)
        %%
        perturbationAmount = 1e-4;
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'perturbationAmount'
                    perturbationAmount = value;
            end
        end
        
        %%
        if (isempty(varargin))
            varargin = {'', {}};
        end
        
        AMatrix = ones(length(x0)) * NaN;
        xNextUnperturbed = this.oneStep(x0, varargin{:});
        for i = 1 : length(x0)
            x0Perturbed = x0;
            x0Perturbed(i) = x0Perturbed(i) + perturbationAmount;
            xNextPerturbed = this.oneStep(x0Perturbed, varargin{:});
            AMatrix(:, i) = (xNextPerturbed - xNextUnperturbed) / perturbationAmount;
        end
    end
    
    function [BMatrix] = getSensitivityToParameters(this, x0, parameterName, varargin)
        %%
        perturbationAmount = 1e-4;
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'perturbationAmount'
                    perturbationAmount = value;
            end
        end
        
        if (isempty(varargin))
            varargin = {'', {}};
        end
        
        xNextUnperturbed = this.oneStep(x0, varargin{:});
        this.(parameterName) = this.(parameterName) + perturbationAmount;
        xNextPerturbed = this.oneStep(x0, varargin{:});
        
        BMatrix = (xNextPerturbed - xNextUnperturbed) / perturbationAmount;
    end
    
    
    
end
    
    
end
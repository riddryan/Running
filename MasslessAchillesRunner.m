classdef MasslessAchillesRunner < Runner
    %Running Model with a spring leg, and a soft-stomach: a mass on a
    %spring above the pelvis
    
    properties
        mpelvis = 1; mfoot=0; mheel=0; mtoe=0;
        gslope = 0;
        g = 1;
        N = 8;
        statestovary = [2 3 5 6 7 8];
        statestomeasure = [7 8];
        lleg=1; lheel = .2; ltoe = .2; Rtoe=.3;
        footangle = pi - 80/360*2*pi;
        achillesangle = .6;
        achillessetpoint = -1.48;
        kleg=12.9801; kachilles=0;
        cleg=0; cfoot=0; cachilles=0;
        
        kimpact = .01;  %stiffness of foot spring at impact
%         kstance = 0.1;
        
        mstate = zeros(8,1);
        footbottomstate = zeros(8,1);
        aerialstate = zeros(8,1);
        %How much force foot spring must be exerting before transitioning
        %into a stiffer spring (kimpact -> kstance). The leg transitions
        %between kimpact and kstance over the range set by the displacement ratios w.r.t. rest length: transitionstart
        %and transitionend.
        transitionstart = 1;
        transitionend = 1.3;
        footforcethreshold = 0;
        
        
        phases = {'HeelToe' 'AchillesEngaged' 'Toe' 'Aerial'};
    end
    
    properties (Dependent=true)
        kstance
    end
    
    methods (Static)
        
        function [] = test()
            %%
            dir = cd;
            saveAnimation=1;
            savepath = [dir '\Animations\'];
            aviname = [savepath 'FootAchilles1.avi'];
            onephasesim = 0;
            
            LineWidth=3;
            LineSize=3;
            TextSize=14;
            fontstyle='bold';
            fonttype='Times New Roman';
            
            runner = MasslessAchillesRunner;
            
            IC = MasslessAchillesRunnerState;
            
            IC.pelvis.x = -.1715;
            IC.pelvis.y = 1.1363;
            IC.pelvis.xDot = 1.1;
            IC.pelvis.yDot = -0.15;
            
            IC.stanceLeg.Angle = -1.2967;
            IC.stanceLeg.Length = 1;
            IC.toe.Angle = -0.7425;
            IC.heel.Angle = -2.0899;
            
            x0 = IC.getVector();
            [x0,runner] = runner.GoodInitialConditions(x0);

            odex0 = x0([1 2 7 8]);
            %Make sure foot starts on ground
            
            runner.kleg = 40;
            runner.kachilles = 0.4048;
            runner.kimpact = 0.9870;
            runner.gslope = 0;
%             runner.kstance = 0.3;
%             runner.footangle = x0(5) - x0(6)+.3;
            
            
            
            %% DAE Simulation
            if onephasesim
                phase = 'Bottomed';
                tspan = [0 1];
                figure
                options = odeset('AbsTol',[],'RelTol',[],'OutputFcn',@odeplot);
                [allt,allx] = ode45(@(t,x) runner.XDoubleDot(t,x,phase),tspan,odex0);
                
                %Resolve for foot states
                footstates = zeros(length(allt),4);
                for i = 1:length(allt)
                    [~,footstates(i,:)] = runner.XDoubleDot(allt(i),allx(i,:),phase);
                end
                allx = [allx(:,1:2) footstates allx(:,3:4)];
                
                figure
                runner.anim(allx);
                
                for i = 1:length(allt)
                    footspring(i) = runner.getkfoot(allx(i,:),phase);
                    footang(i) = allx(i,5) - allx(i,6);
                    footforce(i) = -footspring(i)*(footang(i)-runner.footangle);
                    
                    PEsprings(i) = 1/2*footspring(i)*(footang(i)-runner.footangle)^2 + ...
                        1/2*runner.kleg*(allx(i,4)-runner.lleg)^2;
                    
                    if strcmp(phase,'AchillesEngaged') || strcmp(phase,'Unbottomed') || strcmp(phase,'Toe')
                        PEsprings(i) = PEsprings(i) + 1/2*runner.kachilles*(allx(i,3)-allx(i,6)-runner.achillesangle)^2;
                    end
                    
                    PE(i) = PEsprings(i) + runner.mpelvis*runner.g*cos(runner.gslope)*allx(i,2) ....
                        - runner.mpelvis*runner.g*sin(runner.gslope)*allx(i,1);
                    
                    vpelv = [allx(i,7);allx(i,8)];
                    KE(i) = 1/2*runner.mpelvis*dot(vpelv,vpelv);
                    TotE(i) = PE(i) + KE(i);
                    
                    points=runner.getPoints(allx(i,:));
                    footx(i)=points.foot(1);
                    footy(i)=points.foot(2);
                    heelx(i)=points.heel(1); heely(i) = points.heel(2);
                    toex(i)=points.toe(1); toey(i) = points.toe(2);
                end
                
                figure
                plot(allt,PEsprings)
                hold on
                plot(allt,PE)
                plot(allt,KE)
                plot(allt,TotE)
                legend('PEsprings','PE','KE','TotE')
                
                keyboard;
            end
            %% Take a Step
            figure
            tic
            [xf,tf,allx,allt,tair,runner,phasevec] = runner.onestep(x0,'interleaveAnimation',1);
            toc
            runner.printStepCharacteristics(x0,xf,tf,tair);
            
%             keyboard;
            %% Check Energy and constraints
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
                Fres(i,1:2) = -achforcez*achforcedir'+legforce(i)*[cos(allx(i,3)) sin(allx(i,3))];
                comWR(i) = dot(-Fres(i,1:2),vpelv(i,1:2));
            end
                
            AchOn = find(phasevec==2,1);
            HeelOff = find(phasevec==3,1);
            
                figure
                subplot(211)
                plot(allt,PEsprings)
                hold on
                plot(allt,PE)
                plot(allt,KE)
                plot(allt,TotE)
                legend('PEsprings','PE','KE','TotE')
                
                subplot(212)
                plot(allt,PEleg)
                hold on
                plot(allt,PEfoot)
                plot(allt,PEach)
%                 plot(allt,PEgrav)
%                 plot(allt,KE)
                legend('leg','foot','ach')
                
                figure
                plot(allt,footpower,'LineWidth',2)
                hold on
                plot(allt,achpower,'r','LineWidth',2)
                plot(allt,legpower,'m','LineWidth',2)
                plot(allt,comWR,'c','LineWidth',2);
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
                plot(allt,allx(:,3:6),'LineWidth',3)
                legend('leg angle','leg length','toe angle','toe length')
                hold on
                plot(allt(AchOn)*ones(1,2),get(gca,'YLim'),'k--')
                plot(allt(HeelOff)*ones(1,2),get(gca,'YLim'),'k--')
                plot(tair*ones(1,2),get(gca,'YLim'),'k--')
                
                figure
                plot(allt,allx(:,4),'LineWidth',3)
                hold on
                plot(allt,allx(:,5)-allx(:,6),'LineWidth',3)
                plot(allt,allx(:,3)-allx(:,6),'LineWidth',3)
                legend('leg spring','foot spring','achilles spring')
                
                figure
                plot(allt,footangvel,'LineWidth',3)
                hold on
                plot(allt,achvel,'r','LineWidth',3)
                plot(allt,legvel,'m','LineWidth',3)
                plot(allt,allx(:,7:8),'c','LineWidth',3);
                legend('foot vel','ach vel','leg vel','comvels')
%                 
                
            %% Animation
            keyboard;
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
        function [this] = MasslessAchillesRunner(input)
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
        
        function kstance = get.kstance(this)
           kstance = this.kimpact; 
        end
        
        function [kachilles] = getachilles(this,state,phase)
            if strcmp(phase,'HeelToe') || strcmp(phase,'Aerial')
                kachilles = 0;
                %             kachilles = this.kachilles;
            else
                kachilles = this.kachilles;
            end
        end
        
        function [kfoot] = getkfoot(this,state,phase)
            
            kfoot = this.kimpact;
            return;
            
            pt1 = this.transitionstart;
            pt2 = this.transitionend;
            StateStruc = MasslessAchillesRunnerState(state);
            toe = StateStruc.toe.Angle;
            heel = StateStruc.heel.Angle;
            foot = toe - heel;
            
            if strcmp(phase,'HeelToe')
                if foot/this.footangle < pt1
                    kfoot  = this.kimpact;
                elseif foot/this.footangle >= pt1 && foot/this.footangle <= pt2 % Transition Region
                    kfoot  = interp1([pt1 pt2],[this.kimpact this.kstance],foot/this.footangle); %linear interpolation
                else %We are past the transition region
                    kfoot  = this.kstance;
                end
            else
                kfoot = this.kstance;
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
        
        
        function [xddot,footstates,Fres] = XDoubleDot(this,time,y,phase,varargin)
            %%
            % Put any argument after phase to clear global variable
            % lastfoot

            if strcmp(phase,'Aerial')
                this.getParams();
                M = eye(4); M(3,3) = this.mpelvis; M(4,4) = this.mpelvis;
                rhs = zeros(4,1);
                rhs(1)=y(3);
                rhs(2)=y(4);
                rhs(3) = g*mpelvis*sin(gslope);
                rhs(4) = -g*mpelvis*cos(gslope);
                xddot = M \ rhs;
                footstates = this.aerialstate(3:6);
                Fres = [0;0];
                return;
                
            end
            %Options
            UserCostGradient = 1; %Use analytical gradient for cost functions
            UserConstrGradient = 1; %Use analytical gradient for constraints
            UserHessian = 0;
            
            % global var lastfoot is used to store the optimizatino result of solving
            % for the foot state, to use as the initial guess for the
            % optimization at the next time step.  Is there a better way to
            % do this?
            global lastfoot 
            if time==0 || ~isempty(varargin)
               lastfoot = []; 
            end
            
            x = this.mstate;
            x([1 2 7 8]) = y;
            if ~isempty(lastfoot)
                x([3 4 5 6]) = lastfoot;
            end
            
            this.getParams();
            this.getQandUdefs(x);
            M = eye(4); M(3,3) = this.mpelvis; M(4,4) = this.mpelvis;
            [kachilles] = this.getachilles(x,phase);
            kfoot = this.getkfoot(x,phase);
            
            Cost = @(x,this,p,phase) 1/2*kleg*(x(2)-lleg)^2 + ...
                1/2*this.getkfoot([p(1:2) x p(7:8)],phase)*(x(3)-x(4)-footangle)^2 + ...
                1/2*this.getachilles([p(1:2) x p(3:4)],phase)*(x(1)-x(4)-achillesangle)^2;
            
            if UserCostGradient == 1;
                CostGrad = @(x,this,p,phase)...
                    [this.getachilles([p(1:2) x p(3:4)],phase)*(x(1)-x(4)-achillesangle);...
                    kleg*(x(2)-lleg);...
                    this.getkfoot([p(1:2) x p(7:8)],phase)*(x(3)-x(4)-footangle);...
                    -this.getkfoot([p(1:2) x p(7:8)],phase)*(x(3)-x(4)-footangle) - this.getachilles([p(1:2) x p(3:4)],phase)*(x(1)-x(4)-achillesangle)];
                
                Cost = @(x,this,p,phase) deal(Cost(x,this,p,phase),CostGrad(x,this,p,phase));
            end
            
            if UserHessian
                HF = @(x,lambda,this,p,phase) ...
                    [this.getachilles([p(1:2) x p(3:4)],phase) 0 0 -this.getachilles([p(1:2) x p(3:4)],phase);...
                    0 kleg 0 0;...
                    0 0 this.getkfoot([p(1:2) x p(7:8)],phase) -this.getkfoot([p(1:2) x p(7:8)],phase);...
                    -this.getachilles([p(1:2) x p(3:4)],phase) 0 -this.getkfoot([p(1:2) x p(7:8)],phase) this.getkfoot([p(1:2) x p(7:8)],phase)+this.getachilles([p(1:2) x p(3:4)],phase)];
            end
            
            if strcmp(phase,'HeelToe')
                pts = this.getPoints(this.mstate);
                toex = pts.toe(1);
                
                c = @(x) [];
                ceq = @(x) [q1 + x(2)*cos(x(1)) + ltoe*cos(x(3))-toex;... toex
                    q2 + x(2)*sin(x(1)) + ltoe*sin(x(3));... %toe y
                    q2 + x(2)*sin(x(1)) + lheel*sin(x(4))]; %heel y
                
                if UserConstrGradient
                    dc = @(x) [];
                    dceq = @(x) [-x(2)*sin(x(1)) x(2)*cos(x(1)) x(2)*cos(x(1));...
                        cos(x(1))        sin(x(1)) sin(x(1));...
                        -ltoe*sin(x(3)) ltoe*cos(x(3)) 0;...
                        0               0 lheel*cos(x(4))];
                    Constraints = @(x) deal(c(x),ceq(x),dc(x),dceq(x));
                else
                    Constraints = @(x) deal(c(x),ceq(x));
                end
                
                if UserHessian
                    Hceq2 = @(x,lambda) lambda.eqnonlin(2) * ...
                        [-x(2)*sin(x(1)) cos(x(1)) 0 0;...
                        cos(x(1)) 0 0 0;...
                        0 0 0 0;...
                        0 0 0 -lheel*sin(x(4))];
                    Hc1 = @(x,lambda) lambda.ineqnonlin(1)* ...
                        -[-x(2)*sin(x(1)) cos(x(1)) 0 0;...
                        cos(x(1)) 0 0 0;...
                        0 0 -ltoe*sin(x(3)) 0;...
                        0 0 0 0];
                    Hceq1 = @(x,lambda) lambda.eqnonlin(1) * ...
                        [-x(2)*cos(x(1)) -sin(x(1)) 0 0;...
                        -sin(x(1)) 0 0 0 ;...
                        0 0 0 0;...
                        0 0 0 -lheel*cos(x(4))];
                    
                    hessianfcn = @(x,lambda,this,p,phase) HF(x,lambda,this,p,phase) + ...
                        Hc1(x,lambda) + Hc2(x,lambda) + Hceq1(x,lambda);
                end
                
                
            elseif strcmp(phase,'AchillesEngaged') && 0
                pts = this.getPoints(this.footbottomstate);
                toex = pts.toe(1);
                
                c = @(x) -[q2 + x(2)*sin(x(1)) + lheel*sin(x(4));... %heely
                    q2 + x(2)*sin(x(1)) + ltoe*sin(x(3))]; %toey
                
                ceq = @(x) [q1 + x(2)*cos(x(1)) + lheel*cos(x(4))]; %heelx
%                 ceq = @(x) [q1 + x(2)*cos(x(1)) + ltoe*cos(x(3))-toex]; %toex
                    
                if UserConstrGradient
                    dc = @(x) -[x(2)*cos(x(1)) x(2)*cos(x(1));...
                        sin(x(1))      sin(x(1)); ...
                        0              ltoe*cos(x(3)); ...
                        lheel*cos(x(4)) 0];
                    
                    dceq = @(x) [-x(2)*sin(x(1)); cos(x(1)); 0; -lheel*sin(x(4))];
%                     dceq = @(x) [-x(2)*sin(x(1));...
%                         cos(x(1));...
%                         -ltoe*sin(x(3));...
%                         0];
                        
                    Constraints = @(x) deal(c(x),ceq(x),dc(x),dceq(x));
                else
                    Constraints = @(x) deal(c(x),ceq(x));
                end
                
                if UserHessian
                    Hc1 = @(x,lambda) lambda.ineqnonlin(1) * ...
                          -[-x(2)*sin(x(1)) cos(x(1)) 0 0;...
                           cos(x(1)) 0 0 0;...
                           0 0 0 0;...
                           0 0 0 -lheel*sin(x(4))];
                    Hc2 = @(x,lambda) lambda.ineqnonlin(2)* ...
                          -[-x(2)*sin(x(1)) cos(x(1)) 0 0;...
                          cos(x(1)) 0 0 0;...
                          0 0 -ltoe*sin(x(3)) 0;...
                          0 0 0 0];
                    Hceq1 = @(x,lambda) lambda.eqnonlin(1) * ...
                            [-x(2)*cos(x(1)) -sin(x(1)) 0 0;...
                             -sin(x(1)) 0 0 0 ;...
                             0 0 0 0;...
                             0 0 0 -lheel*cos(x(4))];
                         
                    hessianfcn = @(x,lambda,this,p,phase) HF(x,lambda,this,p,phase) + ...
                                      Hc1(x,lambda) + Hc2(x,lambda) + Hceq1(x,lambda);
                end
                
            elseif strcmp(phase,'Toe') || strcmp(phase,'AchillesEngaged')
                pts = this.getPoints(this.footbottomstate);
                toex = pts.toe(1);
                
                c = @(x) -[q2 + x(2)*sin(x(1)) + lheel*sin(x(4))]; %heel y
                
                ceq = @(x) [q1 + x(2)*cos(x(1)) + ltoe*cos(x(3))-toex;... toex
                    q2 + x(2)*sin(x(1)) + ltoe*sin(x(3))]; %toe y
                
                if UserConstrGradient
                    dc = @(x) -[x(2)*cos(x(1)); sin(x(1)); 0; lheel*cos(x(4))];
                    dceq = @(x) [-x(2)*sin(x(1)) x(2)*cos(x(1));...
                        cos(x(1))        sin(x(1));...
                        -ltoe*sin(x(3)) ltoe*cos(x(3));...
                        0               0];
                    Constraints = @(x) deal(c(x),ceq(x),dc(x),dceq(x));
                else
                    Constraints = @(x) deal(c(x),ceq(x));
                end
                
                                
                if UserHessian
                    Hc1 = @(x,lambda) lambda.ineqnonlin(1) * ...
                          -[-x(2)*sin(x(1)) cos(x(1)) 0 0;...
                           cos(x(1)) 0 0 0;...
                           0 0 0 0;...
                           0 0 0 -lheel*sin(x(4))];
                    Hceq2 = @(x,lambda) lambda.eqnonlin(2)* ...
                          [-x(2)*sin(x(1)) cos(x(1)) 0 0;...
                          cos(x(1)) 0 0 0;...
                          0 0 -ltoe*sin(x(3)) 0;...
                          0 0 0 0];
                    Hceq1 = @(x,lambda) lambda.eqnonlin(1) * ...
                            [-x(2)*cos(x(1)) -sin(x(1)) 0 0;...
                             -sin(x(1)) 0 0 0;...
                             0 0 -ltoe*cos(x(3)) 0;...
                             0 0 0 0];
                         
                    hessianfcn = @(x,lambda,this,p,phase) HF(x,lambda,this,p,phase) + ...
                                      Hc1(x,lambda) + Hceq1(x,lambda) + Hceq2(x,lambda);
                end
            end
            
            z0 = x(3:6);
            opt = optimset('Display','Off','Algorithm','interior-point');
            if UserCostGradient
               opt.GradObj = 'on';
%                opt.DerivativeCheck = 'on';
            end
            
            if UserConstrGradient
               opt.GradConstr = 'on'; 
            end
            
            if UserHessian
               opt.Hessian = 'user-supplied';
               opt.HessFcn = @(z,lambda) hessianfcn(z',lambda,this,x,phase);
            end

            [zstar,fval,eflag,output] = fmincon(@(z) Cost(z,this,x,phase),z0,[],[],[],[],[],[],@(z) Constraints(z),opt);
            x(3:6) = zstar;
            
            Fres = kleg*(x(4)-lleg)*[cos(x(3));sin(x(3))];
            
            rhs = zeros(4,1);
            rhs(1) = x(7);
            rhs(2) = x(8);
            rhs(3) = g*mpelvis*sin(gslope) + Fres(1);
            rhs(4) = -g*mpelvis*cos(gslope) + Fres(2);
            
            if ~strcmp(phase,'HeelToe') || 1
                points = this.getPoints(x);
                legvec = points.pelvis - points.foot;
                legdist = norm(legvec);
                legang = atan2(-legvec(2),-legvec(1));
                achforcedir = [0 1;-1 0]*legvec'/legdist;
                achforce = kachilles*(legang - x(6) - achillesangle)/legdist;
                rhs([3 4]) = rhs([3 4]) + achforce*achforcedir;
                Fres = Fres + achforce*achforcedir;
            end
            
            xddot = M \ rhs;
            footstates = zstar;
            lastfoot = footstates;
            
            
            
        end
        
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
            [x0,this] = this.GoodInitialConditions(x0);
            this.mstate = x0;
            xpre = x0;
            sim = 1;
            phasenum= 1;
            tstart = 0;
            allt = [];
            allx = [];
            phasevec = [];
            %             phaseevents = { @(t,x) this.HeelToeEvents(t,x), @(t,x) this.BottomedEvents(t,x),...
            %                 @(t,x) this.ToeEvents(t,x)  , @(t,x) this.AerialEvents(t,x) };
            phaseevents = { @(t,x) this.HeelToeEvents(t,x), @(t,x) this.AchillesEvents(t,x) ...
                            @(t,x) this.ToeEvents(t,x), @(t,x) this.AerialEvents(t,x) };
            
            %
            
            while sim
                %% Phase transition & Integration
                phase =  this.phases{phasenum}; %Get name of phase corresponding to phasenum
                if strcmp(phase,'AchillesEngaged')
                    this.achillesangle = x0(3)-x0(6);
                    this.achillessetpoint = x0(3);
                    this.footbottomstate=x0;
                    phaseevents{phasenum} = @(t,x) this.AchillesEvents(t,x);
                elseif strcmp(phase,'Toe')
                    this.footbottomstate=x0;
                    phaseevents{phasenum} = @(t,x) this.ToeEvents(t,x);
                elseif strcmp(phase,'Aerial')
                    this.aerialstate = x0;
                    this.aerialstate([3 4 5 6]) = xpre([3 4 5 6]);
                    x0([3 4 5 6]) = xpre([3 4 5 6]);
                    phaseevents{phasenum} = @(t,x) this.AerialEvents(t,x);
                end
                odex0 = x0([1 2 7 8]);
                opts = odeset('Events', phaseevents{phasenum},'RelTol',RelTol','AbsTol',AbsTol); %Set integration options
                [t,x,~,~,ie] = ode45(@(t,x) this.XDoubleDot(t,x,phase),tstart:dt:tstart+tmax,odex0,opts); %Integrate dynamics
                
                footstates = zeros(length(t),4);
                for i = 1:length(t) %Calc footstates for pelvis trajectory solved by ode45
                    if i ==1
                    this.mstate = x0;
                    [~,footstates(i,:)] = this.XDoubleDot(t(i),x(i,:),phase,1);
                    else
                      [~,footstates(i,:)] = this.XDoubleDot(t(i),x(i,:),phase);  
                    end
                end
                x = [x(:,1:2) footstates x(:,3:4)];
                if ~isempty(ie)
                    ie = ie(1);
                end
                
                %%  Recording & Concatenating Integration Results
                if phasenum == 1 || 1
                    dexes=1:length(t);
                else
                    dexes = 2:length(t);
                end
                allt = [allt;t(dexes)]; allx = [allx;x(dexes,:)]; phasevec = [phasevec; phasenum*ones(length(dexes),1)];
                tstart = allt(end);
                x0 = allx(end,:);
                
                %% Decide which Phase to Move To
                [eventpossibilities] = phaseevents{phasenum}(tstart,x0([3 4 5 6])');
                fallevent = length(eventpossibilities); %Event in which model falls for that phase
                
                if isempty(ie)
                    sim = 0;
                    break;
                end
                if ie == fallevent || phasenum == length(this.phases) %Fallen or reached last phase
                    sim = 0;
                elseif ie ==1 %First event is reserved for expected behavior
                    phasenum = phasenum+1; %Move forward one phase
                else
                    sim = 0;
                end
                
            end
            xf = allx(end,:); tf = allt(end); tair = allt(find(phasevec==4,1));
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
            global lastfoot
            state = [state(1:2) ;lastfoot'; state(3:4)];
            ss = MasslessAchillesRunnerState(state);
            fell = ss.pelvis.y;
            value=fell;
            isTerminal=1;
            direction=0;
        end
        
        function [value, isTerminal, direction]  = HeelToeEvents(this,t,state)
            x0 = state;
            [xdot0,f0,Fres] = this.XDoubleDot(t,x0,'HeelToe');
                lowerbodystates = f0;
               
                deltat = 1e-3;
                x1 = x0 + xdot0*deltat;
              [~,f1,~] = this.XDoubleDot(t+deltat,x1,'HeelToe');  
%               value = ((f1(3)-f1(4))-(f0(3)-f0(4)))/deltat;
%                 value = (f1(2)-f0(2))/deltat;
              
            value = lowerbodystates(1) - this.achillessetpoint;
%             value = dot(Fres,[state(3);state(4)]);
            direction = 0;
            isTerminal = 1;
            
            
            [value(2,1),isTerminal(2,1),direction(2,1)] = this.Fall(t,state);
        end
        

        function [value, isTerminal, direction]  = AchillesEvents(this,t,state)
            
            if length(this.phases)==3
                [value,isTerminal,direction] = ToeEvents(this,t,state);
                return;
            end
            
            [~,lowerbodystates] = this.XDoubleDot(t,state,'AchillesEngaged');
            state = [state(1:2) ;lowerbodystates'; state(3:4)];
            pts = this.getPoints(state);
            
            value = pts.heel(2)-1e-4;
            direction = 0;
            isTerminal = 1;
            
            [value(2,1),isTerminal(2,1),direction(2,1)] = this.Fall(t,state);
            
        end
        
        function [value, isTerminal, direction] = ToeEvents(this,t,state)
            [~,lowerbodystates] = this.XDoubleDot(t,state,'Toe');
            
            value = lowerbodystates(2)-this.lleg;
            
            isTerminal = 1; direction = 0;
            
            %Falling
            [value(2,1),isTerminal(2,1),direction(2,1)] = this.Fall(t,state);
            
        end
        
        
        function [value, isTerminal, direction] = AerialEvents(this,t,state)
            foot = this.aerialstate(3:6);
            state = [state(1:2) ;foot'; state(3:4)];
            pts = this.getPoints(state);
            
            %Hitting the Ground
            value = pts.heel(2);
            isTerminal = 1;
            direction = -1;
            
            %Falling
            [value(2,1),isTerminal(2,1),direction(2,1)] = this.Fall(t,state);
        end
        
        function [xnew] = switchLegs(this,x,landangle)
            %Switch the leg angle at collision to be the touchdown angle, and
            %the length of the leg spring to be at rest.
            
            %        newstatestruc = MasslessAchillesRunnerState(x);
            %        newstatestruc.stanceLeg.Angle = landangle;
            %        newstatestruc.stanceLeg.Length = this.lleg;
            
            xnew = x;
            
        end
        
        function [newstate,this] = GoodInitialConditions(this,x0)
            this.getQandUdefs(x0);
            
            newstate = x0;
            %Shift body (without changing configuration) so that heel is on ground
            points = this.getPoints(newstate);
            pelvx = points.pelvis(1) - points.heel(1);
            pelvy = points.pelvis(2) - points.heel(2);
            newstate([1 2])=[pelvx pelvy];
            
            %Put toe on ground
            points = this.getPoints(newstate);
            toeang = asin(-points.foot(2)/this.ltoe);
            newstate(5) = toeang;
            
            this.footangle = newstate(5)-newstate(6);
            this.mstate = newstate;
            [~,finalfoot] = this.XDoubleDot(0,newstate([1 2 7 8]),'HeelToe');
            newstate([3 4 5 6]) = finalfoot;
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
            interleaveAnimationFrameskip=2;
            aviname=[];
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'interleaveAnimationFrameskip'
                        interleaveAnimationFrameskip = value;
                    case 'tmax'
                        tmax = value;
                    case 'aviname'
                        aviname = value;
                end
            end
            [h,w]=size(x0);
            if h==1 || w==1
                [xf, tf, allx, allt, tair]= onestep(this, x0,'aviname',aviname,'interleaveAnimation',1,'interLeaveAnimationFrameSkip',interleaveAnimationFrameskip,'tmax',tmax);
                [speed,steplength,stepfreq,airfrac] = getGaitChar(this,x0,tf,xf,tair);
            else %Already given states
                allx = x0;
                for i = 1 : interleaveAnimationFrameskip : size(allx, 1)
                    cla;
                    this.plot(allx(i, :));
                    pause(0.01);
                end
                speed=[]; steplength=[]; stepfreq=[]; airfrac=[];
            end
            
            
        end
        
        
        %% Mathematica Output
      
        function [E] = getEnergies(this,state)
            this.getParams();
            this.getQandUdefs(state);
            [kachilles] = this.getachilles(state,phase);
            kfoot = this.getkfoot(state,phase);
            
            c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6);
            
            kineticEnergy = 1/2*mpelvis*dot([u1 u2],[u1 u2]);
            
            potentialEnergy = (-2*q3*q6*kachilles - 2*q3*achillesangle*kachilles + ...
                2*q6*achillesangle*kachilles - 2*q5*q6*kfoot - 2*q5*footangle*kfoot + ...
                2*q6*footangle*kfoot - 2*q4*kleg*lleg + 2*q2*g*(mfoot + mheel + mpelvis + ...
                mtoe)*cos(gslope) + kachilles*(q3*q3) + kleg*(q4*q4) + kfoot*(q5*q5) + ...
                kachilles*(q6*q6) + kfoot*(q6*q6) + kachilles*(achillesangle*achillesangle) + ...
                kfoot*(footangle*footangle) + kleg*(lleg*lleg) + 2*q4*g*mfoot*sin(q3 - ...
                gslope) + 2*q4*g*mheel*sin(q3 - gslope) + 2*q4*g*mtoe*sin(q3 - gslope) + ...
                2*g*ltoe*mtoe*sin(q5 - gslope) + 2*g*lheel*mheel*sin(q6 - gslope) - ...
                2*q1*g*(mfoot + mheel + mpelvis + mtoe)*sin(gslope))/2.;
            
            PEgrav = mpelvis*g*q2*cos(gslope);
            
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
            
            
            points.COMpos(1) = q1;
            points.COMpos(2) = q2;
            
            
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
            
            if ~strcmp('Aerial',phase)
                [~,GRF] = this.getMMandRHS(x);
            else
                GRF=[0;0];
            end
        end
        
        function footPower = getFootPower(this,x,phase)
            sz = MasslessAchillesRunnerState(x);
            force = this.getFootForce(x,phase);
            velocity = sz.toe.AngleDot - sz.heel.AngleDot;
            footPower = force.*velocity;
        end
        
        function footforce = getFootForce(this,x,phase)
            sz = MasslessAchillesRunnerState(x);
            kfoot = this.getkfoot(x,phase);
            footforce = -kfoot*(sz.toe.Angle - sz.heel.Angle - this.footangle);
        end
        
        function legPower = getLegPower(this,x)
            sz = MasslessAchillesRunnerState(x);
            force = this.getLegForce(x);
            vel = sz.stanceLeg.LengthDot;
            legPower = force.*vel;
        end
        
        function legforce = getLegForce(this,x)
            sz = MasslessAchillesRunnerState(x);
            legforce = -this.kleg*(sz.stanceLeg.Length - this.lleg);
        end
        
        function achillesPower = getAchillesPower(this,x,phase)
            sz = MasslessAchillesRunnerState(x);
            force = this.getAchillesForce(x,phase);
            velocity = sz.stanceLeg.AngleDot - sz.heel.AngleDot;
            achillesPower = force.*velocity;
        end
        
        function achillesforce = getAchillesForce(this,x,phase)
            sz = MasslessAchillesRunnerState(x);
            [kachilles] = this.getachilles(x,phase);
            achillesforce = -kachilles*(sz.stanceLeg.Angle - sz.heel.Angle - this.achillesangle);
        end
        
        %% Other Gait Information
        
        function [speed] = getSpeed(this, x0, xf, tf)
            if isempty(xf)
                [xf,tf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = MasslessAchillesRunnerState(x0);
            xfstruc = MasslessAchillesRunnerState(xf);
            
            speed = (xfstruc.pelvis.x - x0struc.pelvis.x) / tf;
        end
        
        function [steplength] = getStepLength(this, x0, xf)
            if isempty(xf)
                [xf] = this.onestep(x0);
            end
            
            %Convert state vectors to descriptive class
            x0struc = MasslessAchillesRunnerState(x0);
            xfstruc = MasslessAchillesRunnerState(xf);
            
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
        
        
        function [] = getQandUdefs(this,x)
            %%
            % Add the values for x1,x2,...,xN,u1,u2,...,uN to the work space of the
            % function that calls this function.
            ws = 'caller';
            
            for i = 1:6
                qstring = sprintf('assignin(ws,''q%g'',x(%g));',[i i]);
                eval(qstring);
            end
            
            ustring = sprintf('assignin(ws,''u%g'',x(%g));',[1 7]);
            eval(ustring);
            ustring = sprintf('assignin(ws,''u%g'',x(%g));',[2 8]);
            eval(ustring);
        end
        
        
        
    end
    
    
end

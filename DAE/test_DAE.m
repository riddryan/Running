%A script that plays with solving DAEs (differential algebraic equations
%using ode45)
clear all
close all

celltouse = 5;

%% 1 Single Mass System
if celltouse==1
    %System is a single mass with two states: vertical position, vertical
    %velocity
    
    %To this system add a third dummy state equal to the position squared (This
    %is the algebraic part)
    
    M = [1 0 0;0 1 0;0 0 0]; %This mass matrix is singular
    x0 = [0;0;0];
    tspan = [0 3];
    options = odeset('Mass',M,'MassSingular','yes','Vectorized','On');
    % options = odeset('Mass',M,'MassSingular','yes');
    g = -1;
    
    myfunc = @(t,x) [x(2,:);g;x(3,:)-x(1,:).^(2)];
    tic
    [t,x] = ode15s(myfunc,tspan,x0,options);
    toc
    figure
    plot(t,x)
end

%% 2 Two mass-springs: No Constraint Matrix

if celltouse == 2
    reduceto1 = 0;
    usemassmatrix = 1;
    
    g = 1;
    M = sym(eye(4,4)); M(3,3) = 0;
    k1 = 1;
    k2 = 2;
    l1 = 1;
    l2 = 2;
    
    syms t q1(t) q2(t) u1(t) u2(t)
    vars = [q1;q2;u1;u2];
    F = [u1;u2; -k1*(q1-l1) + k2*(q2-q1-l2); -g - k2*(q2-q1-l2)];
    
    eqs = M*[diff(q1,t);diff(q2,t);diff(u1,t);diff(u2,t)] == F;
    
    if ~reduceto1
        [ODEs,constraints] = reduceDAEToODE(eqs,vars);
        F = daeFunction(ODEs, vars);
        y0est = [1;1;0;0]; yp0est = zeros(4,1);
        opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
        [y0, yp0] = decic(ODEs, vars, constraints, 0, y0est, [1,0,1,0], yp0est, opt);
        [tt,xx]=ode15i(F,[0 5], y0, yp0, opt);
        
    else
        [DAEs,DAEvars] = reduceDAEIndex(eqs,vars);
        [DAEs,DAEvars] = reduceRedundancies(DAEs,DAEvars);
        
        if ~usemassmatrix
            f = daeFunction(DAEs,DAEvars);
            y0est = [1;0;0]; yp0est = zeros(3,1);
            opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
            [y0, yp0] = decic(f, 0, y0est, [], yp0est, [], opt);
            [tt,xx]=ode15i(f,[0 5], y0, yp0, opt);
        else
            [M,F] = massMatrixForm(DAEs,DAEvars);
            tempvars = sym('Y', [numel(DAEvars) 1]);
            M = subs(M, DAEvars, tempvars);
            F = subs(F, DAEvars, tempvars);
            M = matlabFunction(M, 'vars', {t, tempvars});
            F = matlabFunction(F, 'vars', {t, tempvars});
            y0est = [1;0;0]; yp0est = zeros(3,1);
            opt = odeset('Mass', M, 'InitialSlope', yp0est,...
                'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
            [y0, yp0] = decic(@(t,y,yp) M(t,y)*yp - F(t,y), 0, y0est, [], yp0est, [], opt);
            [tt,xx]=ode15i(@(t,y,yp) M(t,y)*yp - F(t,y),[0 5], y0, yp0, opt);
        end
    end
    
    figure
    for i = 1:length(tt)
        cla;
        plot(0,xx(i,1),'ro')
        hold on
        plot(0,xx(i,2),'bo')
        plot(get(gca,'XLim'),[0 0],'k')
        ylim([-3 3]);
        pause(0.1);
    end
    
end

%% 3 Two mass-springs: With Constraint Matrix

if celltouse == 3
    reduceto1 = 0;
    usemassmatrix = 1;
    
    g = 1;
    M = sym(eye(4,4)); M(3,3) = 0;
    k1 = 1;
    k2 = 2;
    l1 = 1;
    l2 = 2;
    
    syms t q1(t) q2(t) q3(t) u1(t) u2(t) u3(t) L(t)
    vars = [q1;q2;q3;u1;u2;u3;L];
    
    
    C = [0 0 1];
    CDot = [0 0 0];
    
    DOF = 3;
    CNUM = 1;
    MM = zeros(3,3); MM(2,2) = 1;
    M = [eye(DOF)   zeros(DOF)  zeros(DOF,CNUM)  ;...
        zeros(DOF)       MM           -C.'       ; ...
        zeros(CNUM,DOF)  C     zeros(CNUM,CNUM)];
    
    F=[u1;u2;u3;-k1*(q1-l1) + k2*(q2-q1-l2); -1*g - k2*(q2-q1-l2);k1*(q1-l1);-CDot*[u1;u2;u3]];
    
    eqs = M*[diff(q1,t);diff(q2,t);diff(q3,t);diff(u1,t);diff(u2,t);diff(u3,t);L] == F;
    isLowIndexDAE(eqs,vars)
    
    if ~reduceto1
        if usemassmatrix
            [ODEs,constraints] = reduceDAEToODE(eqs,vars);
            [M,F] = massMatrixForm(ODEs,vars);
            tempvars = sym('Y', [numel(vars(t)) 1]);
            M = subs(M, vars, tempvars);
            F = subs(F, vars, tempvars);
            M = matlabFunction(M, 'vars', {t, tempvars});
            F = matlabFunction(F, 'vars', {t, tempvars});
            y0est = [1;3;0;-1;-1;0;1]; yp0est = zeros(7,1);
            opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
            [y0, yp0] = decic(ODEs,vars,constraints, 0, y0est, [], yp0est, opt);
            [tt,xx]=ode15i(@(t,y,yp) M(t,y)*yp - F(t,y),[0 5], y0, yp0, opt);
        end
    else
        [DAEs,DAEvars] = reduceDAEIndex(eqs,vars);
        [DAEs,DAEvars] = reduceRedundancies(DAEs,DAEvars);
        if ~usemassmatrix
        else
            [M,F] = massMatrixForm(DAEs,DAEvars);
            tempvars = sym('Y', [numel(DAEvars) 1]);
            M = subs(M, DAEvars, tempvars);
            F = subs(F, DAEvars, tempvars);
            M = matlabFunction(M, 'vars', {t, tempvars});
            F = matlabFunction(F, 'vars', {t, tempvars});
            y0est = [1;0;-1;-1;0]; yp0est = zeros(5,1);
            opt = odeset('Mass', M, 'InitialSlope', yp0est,...
                'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
            [y0, yp0] = decic(@(t,y,yp) M(t,y)*yp - F(t,y), 0, y0est, [], yp0est, [], opt);
            [tt,xx]=ode15i(@(t,y,yp) M(t,y)*yp - F(t,y),[0 5], y0, yp0, opt);
        end
    end
    
    figure
    animstep = ceil(length(tt)/20);
    for i = 1:animstep:length(tt)
        cla;
        plot(0,xx(i,1),'ro')
        hold on
        plot(0,xx(i,2),'bo')
        plot(0,xx(i,3),'go')
        plot(get(gca,'XLim'),[0 0],'k')
        ylim([-3 3]);
        pause(0.1);
    end
    
    %     options = odeset('Mass',M,'MassSingular','yes');
    %     x0 = [1;1;-.1;-.1];
    %     [t,x] = ode15s(F,[0 3],x0,options);
    %     figure
    %     plot(t,x)
    
end

%% 4 SpringFootRunner System, pelvis rooted, export code
if celltouse == 4
    
    usemassmatrix = 0;
    exportcode = 0;
    
    syms mpelvis
    syms lleg lheel ltoe achillesangle footangle
    syms gslope g
    syms kfoot kleg kachilles cfoot cleg cachilles
    syms t q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
    syms u1(t) u2(t) u3(t) u4(t) u5(t) u6(t)
    syms l1(t) l2(t) l3(t)
    paramlist = ['mpelvis,lleg,lheel,ltoe,achillesangle,footangle,gslope,g,kfoot,kleg,kachilles,cfoot,cleg,cachilles'];
    
    if 1
        s3 = sin(q3); c3 = sin(q5); s5 = sin(q5); c5 = cos(q5); s6 = sin(q6); c6 = cos(q6);
        c3m5 = cos(q3-q5); c3m6 = cos(q3-q6); s3m5 = sin(q3-q5); s3m6 = sin(q3-q6);
        
        mheel = 0; mtoe = 0; mfoot = 0;
        
        MM = sym(zeros(6,6)); rhs = sym(zeros(6,1)); constraintJacobianHeelToe=sym(zeros(3,6)); constraintJacobianHeelToeDot =sym(zeros(3,6));
        % MM = eye(6,6); rhs = zeros(6,1);
        
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
        
    end
    
    DOF = 6; CNUM = 3;
    
    bigM = [eye(DOF)   zeros(DOF)  zeros(DOF,CNUM)  ;...
        zeros(DOF)       MM           -C.'       ; ...
        zeros(CNUM,DOF)  C     zeros(CNUM,CNUM)];
    
    dqdt = [diff(q1,t);diff(q2,t);diff(q3,t);diff(q4,t);diff(q5,t);diff(q6,t)];
    dudt = [diff(u1,t);diff(u2,t);diff(u3,t);diff(u4,t);diff(u5,t);diff(u6,t)];
    u = [u1;u2;u3;u4;u5;u6];
    
    bigRHS = [u;rhs;-CDot*u];
    
    eqs = bigM*[dqdt;dudt;l1;l2;l3] == bigRHS;
    vars = [q1(t);q2(t);q3(t);q4(t);q5(t);q6(t);u1(t);u2(t);u3(t);u4(t);u5(t);u6(t);l1(t);l2(t);l3(t)];
    
    [DAEs,DAEvars,R,index] = reduceDAEIndex(eqs,vars);
    [DAEs,DAEvars] = reduceRedundancies(DAEs,DAEvars);
    
    mpelvis = 1; mfoot=0; mheel=0; mtoe=0;
    gslope = 0;
    g = 1;
    lleg=1; lheel = .2; ltoe = .2; Rtoe=.3;
    footangle = pi - 80/360*2*pi;
    achillesangle = .6;
    achillessetpoint = -pi/2;
    kleg=12.9801; kachilles=0.1; kfoot = .1;
    cleg=0; cfoot=0; cachilles=0;
    
    DAEs = subs(DAEs);
    
    f = daeFunction(DAEs,DAEvars);
    y0est = [-.2338 1.0852 -1.2 1 -.8727 -2.2689 ...
        1.4881 -.3934 -1.2036 -.8316 .3144 -.4236 ...
        zeros(1,3)]';
    yp0est = [ 1.0214 -.2869 -1.6 -1.1 1 0 ...
        zeros(1,6) ...
        zeros(1,3)]';
    opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    [y0, yp0] = decic(f, 0, y0est, [], yp0est, [], opt);
    [tt,xx]=ode15i(f,[0 1], y0, yp0, opt);
    
    r = MasslessAchillesRunner;
    
    figure
    r.anim(xx(:,1:12));
    
    
    [ODEs,constraints,index] = reduceDAEToODE(eqs,vars);
    % [M,F] = massMatrixForm(ODEs,vars);
    % tempvars = sym('Y', [numel(vars) 1]);
    % M = subs(M, vars, tempvars);
    % F = subs(F, vars, tempvars);
    
    mpelvis = 1; mfoot=0; mheel=0; mtoe=0;
    gslope = 0;
    g = 1;
    lleg=1; lheel = .2; ltoe = .2; Rtoe=.3;
    footangle = pi - 80/360*2*pi;
    achillesangle = .6;
    achillessetpoint = -pi/2;
    kleg=12.9801; kachilles=0.1; kfoot = .1;
    cleg=0.1; cfoot=0.1; cachilles=0.1;
    
    %         M = subs(M); F = subs(F);
    % M = matlabFunction(M, 'vars', {t, tempvars});
    % F = matlabFunction(F, 'vars', {t, tempvars});
    
    ODEs = subs(ODEs); constraints = subs(constraints);
    F = daeFunction(ODEs, vars);
    
    y0est = [-.2338 1.0852 -1.2 1 -.8727 -2.2689 ...
        1.4881 -.3934 -1.2036 -.8316 .3144 -.4236 ...
        zeros(1,3)]';
    yp0est = [ 1.0214 -.2869 -1.6 -1.1 1 0 ...
        zeros(1,6) ...
        zeros(1,3)]';
    
    opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    [y0, yp0] = decic(ODEs, vars, constraints, 0, y0est, [ones(1,11) zeros(1,4)], yp0est, opt);
    [tt,xx]=ode15i(F,[0 1], y0, yp0, opt);
    
    
    %      opt = odeset('Mass', M, 'InitialSlope', yp0est,...
    %          'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    %      [y0, yp0] = decic(@(t,y,yp) M(t,y)*yp - F(t,y), 0, y0est, [ones(1,6) zeros(1,9)], yp0est, [], opt);
    %      opt = odeset('Mass', M, 'InitialSlope', yp0,...
    %          'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    %      [tt,xx]=ode15i(@(t,y,yp) M(t,y)*yp - F(t,y),[0 1], y0, yp0, opt);
    
    % opt = odeset('Mass', M,'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    % ODEs = subs(ODEs); constraints = subs(constraints);
    % [y0, yp0] = decic(ODEs,vars,constraints, 0, y0est, [], yp0est, opt);
    % [tt,xx] = ode15s(F,[0 1],y0,opt);
    
    r = MasslessAchillesRunner;
    
    figure
    r.anim(xx(:,1:12));
    
    
    if exportcode
        tempvars = sym('Y', [numel(vars) 1]);
        M = subs(M,vars,tempvars);
        F = subs(F,vars,tempvars);
        
        clear q1 q2 q3 q4 q5 q6 u1 u2 u3 u4 u5 u6 l1 l2 l3
        syms q1 q2 q3 q4 q5 q6 u1 u2 u3 u4 u5 u6 l1 l2 l3
        varsFinal = [q1;q2;q3;q4;q5;q6;u1;u2;u3;u4;u5;u6;l1;l2;l3];
        M = subs(M,tempvars,varsFinal);
        F = subs(F,tempvars,varsFinal);
        
        Mstring = func2str(matlabFunction(M));
        startdex = strfind(Mstring,')') + 1;
        Mstring = Mstring(startdex:end);
        
        Fstring = func2str(matlabFunction(F));
        startdex = strfind(Fstring,')') + 1;
        Fstring = Fstring(startdex:end);
        
        fid = fopen('DAE_MM.m','w');
        fprintf(fid,'function [MM] = DAE_MM(this,t,x)\n\n');
        fprintf(fid,'this.getQandUdefs(x);\n');
        fprintf(fid,'this.getParams;\n');
        fprintf(fid,'[kachilles] = this.getachilles(x);\n');
        fprintf(fid,'[kfoot] = this.getkfoot(x);\n');
        % fprintf(fid,'[kachilles] = this.getachilles(x);\n');
        % fprintf(fid,'[kfoot] = this.getkfoot(x);\n');
        fprintf(fid,['MM = ' Mstring ';\n\nend']);
        fclose(fid);
        
        fid = fopen('DAE_F.m','w');
        fprintf(fid,'function [F] = DAE_F(this,t,x)\n\n');
        fprintf(fid,'this.getQandUdefs(x);\n');
        fprintf(fid,'this.getParams;\n');
        fprintf(fid,'[kachilles] = this.getachilles(x);\n');
        fprintf(fid,'[kfoot] = this.getkfoot(x);\n');
        fprintf(fid,['F = ' Fstring ';\n\nend']);
        fclose(fid);
        
        
        y0est = [-.2338 1.0852 -1.2 1 -30/360*2*pi -130/360*2*pi ...
            1.0214 -.2869 -1.6 -1.1 1 0 ...
            zeros(1,3)]';
        yp0est = [ 1.0214 -.2869 -1.6 -1.1 1 0 ...
            zeros(1,6) ...
            zeros(1,3)]';
        
        opt = odeset('Mass', M, 'InitialSlope', yp0est,...
            'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
        
    end
    
    
end

%% 5 Reduced SpringFootRunner System, ground rooted
if celltouse == 5
    
    usemassmatrix = 0;
    exportcode = 0;
    tspan = [0 .1];
    
    if ~exportcode
        mpelvis = 1;
        gslope = 0;
        g = 1;
        lleg=1; lheel = .2; ltoe = .2;
        footangle = pi/4;
        kleg=0; kachilles=1; kfoot = 0.1;
        cleg=0.5; cfoot=0.5;
    else
        syms mpelvis gslope g lleg lheel ltoe footangle achillesangle achillessetpoint kleg kachilles kfoot
        syms cleg cfoot cachilles
    end
    paramlist = ['mpelvis,lleg,lheel,ltoe,achillesangle,achillessetpoint,footangle,gslope,g,kfoot,kleg,kachilles,cfoot,cleg,cachilles'];
    plistvector = {'mpelvis' 'lleg' 'lheel' 'ltoe' 'achillesangle' 'achillessetpoint' 'footangle' 'gslope' 'g' 'kfoot' 'kleg' 'kachilles' 'cfoot' 'cleg' 'cachilles'};
    syms t q1(t) q2(t) q3(t)
    syms u1(t) u2(t) u3(t)
    
    if 1
        c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2);
        
        MM = sym(zeros(3,3)); rhs = sym(zeros(3,1));
        

% Mass Matrix
MM(1,1) = mpelvis*(lheel*lheel); MM(1,2) = c1m2*q3*lheel*mpelvis; MM(1,3) = ...
-(s1m2*lheel*mpelvis); 
MM(2,1) = MM(1,2); MM(2,2) = mpelvis*(q3*q3); MM(2,3) = 0; 
MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mpelvis; 

% righthand side terms
rhs(1) = -(u1*cfoot) - q1*kfoot + footangle*kfoot - ...
2*c1m2*u2*u3*lheel*mpelvis - g*lheel*mpelvis*cos(q1 - gslope) - ...
q3*s1m2*lheel*mpelvis*(u2*u2); 
rhs(2) = q3*mpelvis*(-2*u2*u3 - g*cos(q2 - gslope) + s1m2*lheel*(u1*u1)); 
rhs(3) = -(u3*cleg) + kleg*lleg + c1m2*lheel*mpelvis*(u1*u1) + q3*(-kleg + ...
mpelvis*(u2*u2)) - g*mpelvis*sin(q2 - gslope); 
        
        
        DOF = 3;
        
        bigM = [eye(DOF)   zeros(DOF);...
            zeros(DOF)       MM ];
        
    end
    
    dqdt = [diff(q1,t);diff(q2,t);diff(q3,t)];
    dudt = [diff(u1,t);diff(u2,t);diff(u3,t)];
    u = [u1;u2;u3];
    
    bigRHS = [u;rhs];
    
    eqs = bigM*[dqdt;dudt] == bigRHS;
    vars = [q1;q2;q3;u];
    IsItLowIndex = isLowIndexDAE(eqs,vars)
    [DAEeqs,DAEvars,R,index]=reduceDAEIndex(eqs,vars);
    if ~exportcode
        
        if ~usemassmatrix
            f = daeFunction(eqs,vars);
            
            y0est = [pi/4 1.8 1 ...
                0 0 0]';
            yp0est = [0 0 0 ...
                zeros(1,3)]';
            
            opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
            [y0, yp0] = decic(f, 0, y0est, [], yp0est, [], opt);
            [tt,xx]=ode15i(f,tspan, y0, yp0, opt);
            y0est = [0 1.8 1 ...
                0 0 0]';
            yp0est = [0 0 0 ...
                zeros(1,3)]';
            
            opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
            [y0, yp0] = decic(f, 0, y0est, [], yp0est, [], opt);
            [tt,xx]=ode15i(f,tspan, y0, yp0, opt);
            %
        else
            [M,F] = massMatrixForm(eqs,vars);
            tempvars = sym('Y', [numel(vars(t)) 1]);
            M = subs(M, vars, tempvars);
            F = subs(F, vars, tempvars);
            M = matlabFunction(M, 'vars', {t, tempvars});
            F = matlabFunction(F, 'vars', {t, tempvars});
            
            y0est = [pi/4 pi/2 1 ...
                -.2 -.2 -.2]';
            yp0est = [-.2 -.2 -.2 ...
                zeros(1,3)]';
            
            opt = odeset('Mass',M,'InitialSlope',yp0est,'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
            [y0, yp0] = decic(@(t,y,yp) M(t,y)*yp - F(t,y), 0, y0est, [], yp0est, [], opt);
            [tt,xx]=ode15s(F,tspan, y0, opt);
        end
        
        figure
        plotter = RunnerPlotter;
        
        animstep = ceil(length(tt)/20);
        count = 0;
        for i = 1:animstep:length(tt)
            count = count+1;
            cla;
            x = xx(i,:);
            q1 = x(1); q2 = x(2); q3 = x(3);
            u1 = x(4); u2 = x(5); u3 = x(6);
            
            c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2);
            foot = [cos(q1)*lheel;sin(q1)*lheel];
            pelvis = foot + q3*[cos(q2);sin(q2)];
            
            plot([-5 5],[0 0],'k','LineWidth',2)
            hold on
            
            plotter.plotLine([0;0],foot);
            numcoils=3;
            springwidth=.03;
            plotter.plotSpring(foot(1),foot(2),...
                pelvis(1),pelvis(2),...
                numcoils,1,springwidth,'Color',[0 1 1])
            
            plotter.plotSpring(foot(1),0,foot(1),foot(2),3,.3,.02,'Color',[0 1 1]);
            plotter.plotMass(pelvis);
            xLims = pelvis(1) + [-1 1];
            xlim(xLims);
            
            yLims = pelvis(2) + [-1.5 .5];
            ylim(yLims);
            pause(0.1);
            
            kineticEnergy(count) = (mpelvis*(2*u1*(c1m2*q3*u2 - s1m2*u3)*lheel + q3*q3*(u2*u2) + ...
                u3*u3 + u1*u1*(lheel*lheel)))/2.;
            
            potentialEnergy(count) = (kfoot*((-q1 + footangle)*(-q1 + footangle)) + kleg*((-q3 + ...
                lleg)*(-q3 + lleg)) - 2*g*mpelvis*(-(lheel*sin(q1 - gslope)) - q3*sin(q2 - ...
                gslope)))/2.;
            
        end
        TotE = kineticEnergy + potentialEnergy;
        figure
        plot(TotE)
    else
        f = eval(['daeFunction(eqs,vars,' paramlist ')']);
        Fstring = func2str(f);
        startdex = strfind(Fstring,')') + 1;
        Fstring = Fstring(startdex:end);
        
        fid = fopen('DAE_HeelToe.m','w');
        fprintf(fid,'function F = DAE_HeelToe(this,t,in2,in3)\n\n');
        fprintf(fid,'this.getParams;\n');
        fprintf(fid,'[kachilles] = this.getachilles(x);\n');
        fprintf(fid,'[kfoot] = this.getkfoot(x);\n');
        fprintf(fid,['F = ' Fstring ';\n\nend']);
        fclose(fid);
        
        fid = fopen('DAE_HeelToe.m','r');
        textf = fread(fid,'*char')';
        fclose(fid);
        
        for i = length(plistvector):-1:1
            textf = strrep(textf,sprintf('param%d',i),plistvector{i});
        end
        
        fid = fopen('DAE_HeelToe.m','w+');
        fprintf(fid,'%s',textf);
        fclose(fid);
    end
    
    
    
    
    %      opt = odeset('Mass', M, 'InitialSlope', yp0est,...
    %          'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    %      [y0, yp0] = decic(@(t,y,yp) M(t,y)*yp - F(t,y), 0, y0est, [ones(1,6) zeros(1,9)], yp0est, [], opt);
    %      opt = odeset('Mass', M, 'InitialSlope', yp0,...
    %          'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    %      [tt,xx]=ode15i(@(t,y,yp) M(t,y)*yp - F(t,y),[0 1], y0, yp0, opt);
    
    % opt = odeset('Mass', M,'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    % ODEs = subs(ODEs); constraints = subs(constraints);
    % [y0, yp0] = decic(ODEs,vars,constraints, 0, y0est, [], yp0est, opt);
    % [tt,xx] = ode15s(F,[0 1],y0,opt);
    
    
    
    
end

%% 6 Two Link Pendulum with one angular spring
if celltouse == 6
    
    
    usemassmatrix = 0;
    exportcode = 0;
    
    mpelvis = 1; mfoot=0; mheel=0; mtoe=0;
    gslope = 0;
    g = 1;
    lleg=1; lheel = .2; ltoe = .2; Rtoe=.3;
    footangle = .7854;
    achillesangle = .6;
    achillessetpoint = -pi/2;
    kleg=1; kachilles=1; kfoot = .5;
    cleg=0.1; cfoot=0.0; cachilles=0.1;
    
    syms t q1(t) q2(t)
    syms u1(t) u2(t)
    
    if 1
        c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2);
        MM = sym(zeros(2,2)); rhs = sym(zeros(2,1));
        
        % Mass Matrix
        MM(1,1) = mpelvis*(lheel*lheel); MM(1,2) = c1m2*lheel*lleg*mpelvis;
        MM(2,1) = MM(1,2); MM(2,2) = mpelvis*(lleg*lleg);
        
        % righthand side terms
        rhs(1) = -(u1*cfoot) - q1*kfoot + footangle*kfoot - g*lheel*mpelvis*cos(q1 - ...
            gslope) - s1m2*lheel*lleg*mpelvis*(u2*u2);
        rhs(2) = lleg*mpelvis*(-(g*cos(q2 - gslope)) + s1m2*lheel*(u1*u1));
        
        
        DOF = 2;
        
        bigM = [eye(DOF)   zeros(DOF);...
            zeros(DOF)       MM ];
        
    end
    
    dqdt = [diff(q1,t);diff(q2,t)];
    dudt = [diff(u1,t);diff(u2,t)];
    u = [u1;u2];
    
    bigRHS = [u;rhs];
    
    eqs = bigM*[dqdt;dudt] == bigRHS;
    vars = [q1;q2;u];
    IsItLowIndex = isLowIndexDAE(eqs,vars)
    
    %     [M,F] = massMatrixForm(eqs,vars);
    %     tempvars = sym('Y', [numel(vars(t)) 1]);
    %     M = subs(M, vars, tempvars);
    %     F = subs(F, vars, tempvars);
    %     M = matlabFunction(M, 'vars', {t, tempvars});
    % F = matlabFunction(F, 'vars', {t, tempvars});
    %     y0est = [0.7854 1.7 ...
    %         -.8 0]';
    %     yp0est = [ -.8 0 ...
    %         zeros(1,2)]';
    %     opt = odeset('Mass', M, 'InitialSlope', yp0est,...
    %                 'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7),'Vectorized','On');
    %             [y0, yp0] = decic(@(t,y,yp) M(t,y)*yp - F(t,y), 0, y0est, [], yp0est, [], opt);
    %      [tt,xx]=ode15s(F, [0, 3], y0, opt);
    
    %
    f = daeFunction(eqs,vars);
    
    y0est = [0.7854 1.7 ...
        -.8 0]';
    yp0est = [ -.8 0 ...
        zeros(1,2)]';
    
    opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    [y0, yp0] = decic(f, 0, y0est, [], yp0est, [], opt);
    [tt,xx]=ode15i(f,[0 3], y0, yp0, opt);
    %
    figure
    plotter = RunnerPlotter;
    
    animstep = ceil(length(tt)/20);
    count = 0;
    for i = 1:animstep:length(tt)
        count = count+1;
        cla;
        x = xx(i,:);
        q1 = x(1); q2 = x(2);
        u1 = x(3); u2 = x(4);
        c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2);
        foot = [cos(q1)*lheel;sin(q1)*lheel];
        pelvis = foot + lleg*[cos(q2);sin(q2)];
        
        plot([-5 5],[0 0],'k','LineWidth',2)
        hold on
        
        plotter.plotLine([0;0],foot);
        plotter.plotLine(foot,pelvis);
        plotter.plotMass(pelvis);
        xLims = pelvis(1) + [-1 1];
        xlim(xLims);
        
        yLims = pelvis(2) + [-1.5 .5];
        ylim(yLims);
        pause(0.1);
        
        kineticEnergy(count) = (mpelvis*(2*c1m2*u1*u2*lheel*lleg + u1*u1*(lheel*lheel) + ...
            u2*u2*(lleg*lleg)))/2.;
        
        potentialEnergy(count) = (kfoot*((-q1 + footangle)*(-q1 + footangle)))/2. - ...
            g*mpelvis*(-(lheel*sin(q1 - gslope)) - lleg*sin(q2 - gslope));
    end
    TotE = kineticEnergy + potentialEnergy;
    
    figure
    plot(TotE)
    
    
    %      opt = odeset('Mass', M, 'InitialSlope', yp0est,...
    %          'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    %      [y0, yp0] = decic(@(t,y,yp) M(t,y)*yp - F(t,y), 0, y0est, [ones(1,6) zeros(1,9)], yp0est, [], opt);
    %      opt = odeset('Mass', M, 'InitialSlope', yp0,...
    %          'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    %      [tt,xx]=ode15i(@(t,y,yp) M(t,y)*yp - F(t,y),[0 1], y0, yp0, opt);
    
    % opt = odeset('Mass', M,'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    % ODEs = subs(ODEs); constraints = subs(constraints);
    % [y0, yp0] = decic(ODEs,vars,constraints, 0, y0est, [], yp0est, opt);
    % [tt,xx] = ode15s(F,[0 1],y0,opt);
    
    
    
    
end

%% 7 SpringFootRunner System, pelvis rooted
if celltouse == 7
    
    usemassmatrix = 0;
    exportcode = 0;
    
    mpelvis = 1; mfoot=0; mheel=0; mtoe=0;
    gslope = 0;
    g = 1;
    lleg=1; lheel = .2; ltoe = .2; Rtoe=.3;
    footangle = .7854;
    achillesangle = .6;
    achillessetpoint = -pi/2;
    kleg=5; kachilles=1; kfoot = 0.1;
    cleg=5; cfoot=5; cachilles=0.1;
    
    syms t q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
    syms u1(t) u2(t) u3(t) u4(t) u5(t) u6(t)
    syms l1(t) l2(t) l3(t)
    
    if 1
        s3 = sin(q3); c3 = sin(q5); s5 = sin(q5); c5 = cos(q5); s6 = sin(q6); c6 = cos(q6);
        c3m5 = cos(q3-q5); c3m6 = cos(q3-q6); s3m5 = sin(q3-q5); s3m6 = sin(q3-q6);
        
        mheel = 0; mtoe = 0; mfoot = 0;
        
        MM = sym(zeros(6,6)); rhs = sym(zeros(6,1)); constraintJacobianHeelToe=sym(zeros(3,6)); constraintJacobianHeelToeDot =sym(zeros(3,6));
        % MM = eye(6,6); rhs = zeros(6,1);
        
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
        
    end
    
    DOF = 6; CNUM = 3;
    
    bigM = [eye(DOF)   zeros(DOF)  zeros(DOF,CNUM)  ;...
        zeros(DOF)       MM           -C.'       ; ...
        zeros(CNUM,DOF)  C     zeros(CNUM,CNUM)];
    
    dqdt = [diff(q1,t);diff(q2,t);diff(q3,t);diff(q4,t);diff(q5,t);diff(q6,t)];
    dudt = [diff(u1,t);diff(u2,t);diff(u3,t);diff(u4,t);diff(u5,t);diff(u6,t)];
    u = [u1;u2;u3;u4;u5;u6];
    
    bigRHS = [u;rhs;-CDot*u];
    
    posHeelToeconstraints(1,1) = q1 + c3*q4 + c6*lheel == 0;
    posHeelToeconstraints(2,1) = q2 + q4*s3 + s6*lheel == 0;
    posHeelToeconstraints(3,1) = q2 + s3*q4 + s5*ltoe == 0;
    
    eqs = bigM*[dqdt;dudt;l1;l2;l3] == bigRHS;
    vars = [q1;q2;q3;q4;q5;q6;u;l1;l2;l3];
    
    f = daeFunction(eqs,vars);
    
    constraints = @(x) [x(1) + cos(x(3))*x(4) + cos(x(6))*lheel;...
        x(2) + x(4)*sin(x(3)) + sin(x(6))*lheel;...
        x(2) + sin(x(3))*x(4) + sin(x(5))*ltoe];
    
    y0est = [-.2338 1.0852 -1.4416 1 -pi/4 pi/4-pi ...
        0 0 0 0 0 0 ...
        zeros(1,3)]';
    yp0est = [ 0 0 0 0 0 0 ...
        zeros(1,6) ...
        zeros(1,3)]';
    
    opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    [y0, yp0] = getDAE_IC(f, 0, y0est, [], yp0est, [], opt, constraints);
    %     [y0, yp0] = decic(f, 0, y0est, [], yp0est, [], opt););
    
    if 0
        x = y0(1:12);
        q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6);
        u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12);
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6);
        
        clear constraintJacobianHeelToe
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
        
        Jc = constraintJacobianHeelToe;
        nullJc = null(Jc);
        unew = nullJc * (nullJc \ x(7:12));
        newstate = [x(1:6) ; unew];
        
        points.foot(1) = q1 + c3*q4;
        points.foot(2) = q2 + q4*s3;
        points.pelvis(1) = q1;
        points.pelvis(2) = q2;
        points.heel(1) = q1 + c3*q4 + c6*lheel;
        points.heel(2) = q2 + q4*s3 + s6*lheel;
        points.toe(1) = q1 + c3*q4 + c5*ltoe;
        points.toe(2) = q2 + q4*s3 + s5*ltoe;
        
        %Shift body (without changing configuration) so that heel is on ground
        pelvx = points.pelvis(1) - points.heel(1);
        pelvy = points.pelvis(2) - points.heel(2);
        newstate([1 2])=[pelvx pelvy];
        
        %Put toe on ground
        x = newstate;
        q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6);
        u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12);
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6);
        points.foot(2) = q2 + q4*s3;
        
        toeang = asin(-points.foot(2)/ltoe);
        newstate(5) = toeang;
        
        y0(1:12) = newstate;
        
    end
    
    
    
    %
    [tt,xx]=ode15i(f,[0 3], y0, yp0, opt);
    
    
    
    figure
    plotter = RunnerPlotter;
    
    animstep = ceil(length(tt)/20);
    count = 0;
    for i = 1:animstep:length(tt)
        count = count+1;
        cla;
        x = xx(i,:);
        q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4); q5 = x(5); q6 = x(6);
        u1 = x(7); u2 = x(8); u3 = x(9); u4 = x(10); u5 = x(11); u6 = x(12);
        
        c3 = cos(q3); c5 = cos(q5); c6 = cos(q6); s3 = sin(q3); s5 = sin(q5); s6 = sin(q6); c3m5 = cos(q3 - q5); c3m6 = cos(q3 - q6); s3m5 = sin(q3 - q5); s3m6 = sin(q3 - q6);
        points.foot(1) = q1 + c3*q4;
        points.foot(2) = q2 + q4*s3;
        points.pelvis(1) = q1;
        points.pelvis(2) = q2;
        points.heel(1) = q1 + c3*q4 + c6*lheel;
        points.heel(2) = q2 + q4*s3 + s6*lheel;
        points.toe(1) = q1 + c3*q4 + c5*ltoe;
        points.toe(2) = q2 + q4*s3 + s5*ltoe;
        
        foot = [cos(q1)*lheel;sin(q1)*lheel];
        pelvis = foot + lleg*[cos(q2);sin(q2)];
        
        plot([-5 5],[0 0],'k','LineWidth',2)
        hold on
        
        plotter.plotLine(points.foot,points.heel);
        plotter.plotLine(points.foot,points.toe);
        
        %Draw Springs
        numcoils=3;
        springwidth=.07;
        plotter.plotSpring(points.foot(1),points.foot(2),...
            points.pelvis(1),points.pelvis(2),...
            numcoils,lleg,springwidth) %leg spring
        
        numcoils=3;
        springwidth=.03;
        plotter.plotSpring(points.heel(1),points.heel(2),...
            points.pelvis(1),points.pelvis(2),...
            numcoils,1,springwidth,'Color',[0 1 1]) %achilles spring
        
        numcoils=3;
        springwidth=.02;
        plotter.plotSpring(points.heel(1),points.heel(2),...
            points.toe(1),points.toe(2),...
            numcoils,.3,springwidth,'Color',[1 0 0]) %achilles spring
        
        %Draw Masses
        plotter.plotMass(points.pelvis);
        plotter.plotMass(points.foot);
        plotter.plotMass(points.heel);
        
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
        
        pause(0.1);
        
        %         kineticEnergy(count) = (mpelvis*(2*c1m2*u1*u2*lheel*lleg + u1*u1*(lheel*lheel) + ...
        % u2*u2*(lleg*lleg)))/2.;
        %
        % potentialEnergy(count) = (kfoot*((-q1 + footangle)*(-q1 + footangle)))/2. - ...
        % g*mpelvis*(-(lheel*sin(q1 - gslope)) - lleg*sin(q2 - gslope));
    end
    %     TotE = kineticEnergy + potentialEnergy;
    
    
    
    
end

%% 8 Two Link Pendulum with one angular spring, Pelvis Rooted
if celltouse == 8
    
    
    usemassmatrix = 0;
    exportcode = 0;
    
    mpelvis = 1; mfoot=0; mheel=0; mtoe=0;
    gslope = 0;
    g = 1;
    lleg=1; lheel = .2; ltoe = .2; Rtoe=.3;
    footangle = -3*pi/4;
    achillesangle = .6;
    achillessetpoint = -pi/2;
    kleg=1; kachilles=1; kfoot = 1;
    cleg=0.1; cfoot=0; cachilles=0.1;
    
    syms t q1(t) q2(t) q3(t) q4(t)
    syms u1(t) u2(t) u3(t) u4(t)
    syms l1(t) l2(t)
    
    if 1
        c3 = cos(q3); c4 = cos(q4); s3 = sin(q3); s4 = sin(q4); c3m4 = cos(q3 - q4); s3m4 = sin(q3 - q4);
        MM = sym(zeros(4,4)); rhs = sym(zeros(4,1));
        
        % Mass Matrix
        MM(1,1) = mheel + mpelvis; MM(1,2) = 0; MM(1,3) = -(s3*lleg*mheel); MM(1,4) = ...
            -(s4*lheel*mheel);
        MM(2,1) = MM(1,2); MM(2,2) = mheel + mpelvis; MM(2,3) = c3*lleg*mheel; ...
            MM(2,4) = c4*lheel*mheel;
        MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = mheel*(lleg*lleg); MM(3,4) = ...
            c3m4*lheel*lleg*mheel;
        MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = ...
            mheel*(lheel*lheel);
        
        % righthand side terms
        rhs(1) = c3*lleg*mheel*(u3*u3) + c4*lheel*mheel*(u4*u4) + ...
            g*mpelvis*sin(gslope);
        rhs(2) = -(g*mpelvis*cos(gslope)) + s3*lleg*mheel*(u3*u3) + ...
            s4*lheel*mheel*(u4*u4);
        rhs(3) = -(s3m4*lheel*lleg*mheel*(u4*u4));
        rhs(4) = -(u4*cfoot) - q4*kfoot + footangle*kfoot + ...
            s3m4*lheel*lleg*mheel*(u3*u3);
        
        C = sym(zeros(2,4)); CDot = C;
        C(1,1) = 1; C(1,2) = 0; C(1,3) = -(s3*lleg); C(1,4) = -(s4*lheel);
        C(2,1) = 0; C(2,2) = 1; C(2,3) = c3*lleg; C(2,4) = c4*lheel;
        
        CDot(1,1) = 0; CDot(1,2) = 0; CDot(1,3) = -(c3*u3*lleg); CDot(1,4) = ...
            -(c4*u4*lheel);
        CDot(2,1) = 0; CDot(2,2) = 0; CDot(2,3) = -(s3*u3*lleg); CDot(2,4) = ...
            -(s4*u4*lheel);
        
        
        DOF = 4; CNUM = 2;
        
        bigM = [eye(DOF)   zeros(DOF)  zeros(DOF,CNUM)  ;...
            zeros(DOF)       MM           -C.'       ; ...
            zeros(CNUM,DOF)  C     zeros(CNUM,CNUM)];
        
        
    end
    
    dqdt = [diff(q1,t);diff(q2,t);diff(q3,t);diff(q4,t)];
    dudt = [diff(u1,t);diff(u2,t);diff(u3,t);diff(u4,t)];
    u = [u1;u2;u3;u4];
    
    bigRHS = [u;rhs;-CDot*u];
    
    rhs2 = subs(rhs2,tempvars,varsFinal);
    
    eqs = bigM*[dqdt;dudt;l1;l2] == bigRHS;
    vars = [q1;q2;q3;q4;u;l1;l2];
    IsItLowIndex = isLowIndexDAE(eqs,vars)
    
    
    %------------------------------
    f = daeFunction(eqs,vars);
    footangle = pi/4;
    legangle = 1.8;
    pelvx = lheel*cos(footangle) + lleg*cos(legangle);
    pelvy = lheel*sin(footangle) + lleg*sin(legangle);
    
    y0est = [pelvx pelvy -(pi-legangle) -(pi-footangle) ...
        0 0 0 0 ...
        0  0]';
    yp0est = [ 0 0 0 0 ...
        zeros(1,4) ...
        0 0]';
    
    opt = odeset('RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    [y0, yp0] = decic(f, 0, y0est, [ones(1,4) zeros(1,6)], yp0est, [], opt);
    [tt,xx]=ode15i(f,[0 3], y0, yp0, opt);
    %----------------------------
    
    
    %
    figure
    plotter = RunnerPlotter;
    
    animstep = ceil(length(tt)/30);
    count = 0;
    for i = 1:animstep:length(tt)
        count = count+1;
        cla;
        x = xx(i,:);
        q1 = x(1); q2 = x(2); q3 = x(3); q4 = x(4);
        u1 = x(5); u2 = x(6); u3 = x(7); u4 = x(8);
        
        c3 = cos(q3); c4 = cos(q4); s3 = sin(q3); s4 = sin(q4); c3m4 = cos(q3 - q4); s3m4 = sin(q3 - q4);
        pelvis = [q1;q2];
        foot = pelvis + lleg*[cos(q3);sin(q3)];
        heel = foot + lheel*[cos(q4);sin(q4)];
        
        %         err = heel;
        %         heel = heel-err; foot = foot-err; pelvis = pelvis - err;
        
        plot([-5 5],[0 0],'k','LineWidth',2)
        hold on
        
        plotter.plotLine(heel,foot);
        plotter.plotLine(foot,pelvis);
        plotter.plotMass(pelvis);
        plotter.plotMass(heel,'scaling',.1);
        plotter.plotMass(foot,'scaling',.1);
        xLims = pelvis(1) + [-1 1];
        xlim(xLims);
        
        yLims = pelvis(2) + [-1.5 .5];
        ylim(yLims);
        pause(0.1);
        
        kineticEnergy(i) = (mpelvis*(u1*u1 + u2*u2))/2.;
        
        potentialEnergy(i) = (kfoot*((-q4 + footangle)*(-q4 + footangle)))/2. + ...
            g*mpelvis*(q2*cos(gslope) - q1*sin(gslope));
    end
    TotE = kineticEnergy + potentialEnergy;
    figure
    plot(TotE)
    
    
    
    
    %      opt = odeset('Mass', M, 'InitialSlope', yp0est,...
    %          'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    %      [y0, yp0] = decic(@(t,y,yp) M(t,y)*yp - F(t,y), 0, y0est, [ones(1,6) zeros(1,9)], yp0est, [], opt);
    %      opt = odeset('Mass', M, 'InitialSlope', yp0,...
    %          'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    %      [tt,xx]=ode15i(@(t,y,yp) M(t,y)*yp - F(t,y),[0 1], y0, yp0, opt);
    
    % opt = odeset('Mass', M,'RelTol', 10.0^(-7), 'AbsTol' , 10.0^(-7));
    % ODEs = subs(ODEs); constraints = subs(constraints);
    % [y0, yp0] = decic(ODEs,vars,constraints, 0, y0est, [], yp0est, opt);
    % [tt,xx] = ode15s(F,[0 1],y0,opt);
    
    
    
    
end
classdef RunnerPlotter
  %Helps draw models
  
  properties
    
    vectorColor = [38 57 61]/255;
    lineWidth = 2;
    footColor = [204 87 83]/255;
    linkColor = [38 57 61]/255;
    dotColor = [222 129 31]/255;
    
  end
  
  methods
    
    
    function [] = plotVectorFromPoint(this, point, vector, varargin)
      
      scaleFactor = 1.0;
      for i = 1 : 2 : length(varargin)
        if(strcmp(varargin{i}, 'scaleFactor'))
          scaleFactor = varargin{i+1};
        end
      end
      
      line([point(1) point(1) + scaleFactor * vector(1)], ...
        [point(2), point(2) + scaleFactor * vector(2)], ...
        'Color', this.vectorColor, 'LineWidth', this.lineWidth);
      hold on;
      
      dotColor = this.dotColor; %'k';
      plot(point(1), point(2), ...
        'o', 'MarkerFaceColor', dotColor, 'MarkerEdgeColor', dotColor, 'MarkerSize', 6);
      axis equal;
      
    end
    
    function plotLine(this, from, to,varargin)
        
        color = this.linkColor;
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'Color'
                    color = value;
            end
        end
    
        line( ...
            [from(1) to(1)], ...
            [from(2), to(2)], ...
            'Color', color, 'LineWidth', this.lineWidth);
        hold on;

    end
    
    function plotArc(this,radius,center,direction,angleexcursion)
        startangle = atan(direction(2)/direction(1));
        ang = startangle:.01:startangle+angleexcursion;
        xp=radius*cos(ang);
        yp=radius*sin(ang);
        plot(center(1)+xp,center(2)+yp,'Color',this.linkColor,'LineWidth',this.lineWidth);
    end
    
    function plotSpring(this,xa,ya,xb,yb,ne,a,r0,varargin)
        % SPRING         Calculates the position of a 2D spring
        %    [XS YS] = SPRING(XA,YA,XB,YB,NE,A,R0) calculates the position of
        %    points XS, YS of a spring with ends in (XA,YA) and (XB,YB), number
        %    of coils equal to NE, natural length a, and natural radius r0.
        %    Useful for mass-spring oscillation animations.
        %    ne: number of coils , a = natural length , r0 = natural radius
        
        color = this.vectorColor;
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'Color'
                    color = value;
            end
        end
        
        R = [xb yb] - [xa ya]; mod_R = norm(R); % relative position between "end_B" and "end_A" 
        
        
        %Ranges for straight and spring segments
        springrange = 1/3;
        straightrange = (1 - springrange) / 2;
        pt1=[xa ya]; pt2=[xb yb];
        xvals = pt2(1) - pt1(1);
        yvals = pt2(2) - pt1(2);
        
        %Redifine the actual starting points of the spring to start
        %somewhere in between the original input points
        xa = straightrange*xvals+pt1(1); xb = (straightrange+springrange)*xvals+pt1(1);
        ya = straightrange*yvals+pt1(2); yb = (straightrange+springrange)*yvals+pt1(2);
        
        %Straight lines that don't make up part of the spring
        line1x = [pt1(1) xa]; line1y = [pt1(2) ya];
        line2x = [pt2(1) xb]; line2y = [pt2(2) yb];
        
        %Redefine rest length and current length for these points
        R = [xb yb] - [xa ya]; mod_R = norm(R);
        a = springrange*a;
        
        
        
        
            
            Li_2 =  (a/(4*ne))^2 + r0^2;                % (large of a quarter of coil)^2
            ei = 0:(2*ne+1);                            % vector of longitudinal positions
            j = 0:2*ne-1; b = [0 (-ones(1,2*ne)).^j 0]; % vector of transversal positions
        
        L_2 = (mod_R/(4*ne))^2; % (actual longitudinal extensión of a coil )^2
        if L_2 > Li_2
%             error('Spring:TooEnlargement', ...
%                 'Initial conditions cause pulling the spring beyond its maximum large. \n Try reducing these conditions.')
             r = 0;
        else
            r = sqrt(Li_2 - L_2);   %actual radius
        end
        c = r*b;    % vector of transversal positions
        u1 = R/mod_R; u2 = [-u1(2) u1(1)]; % unitary longitudinal and transversal vectors
        xs = xa + u1(1)*(mod_R/(2*ne+1)).*ei + u2(1)*c; % horizontal coordinates
        ys = ya + u1(2)*(mod_R/(2*ne+1)).*ei + u2(2)*c; % vertical coordinates
        
        plot(xs,ys,'Color',color,'LineWidth',this.lineWidth);
        hold on
        plot(line1x,line1y,'Color',color,'LineWidth',this.lineWidth);
        plot(line2x,line2y,'Color',color,'LineWidth',this.lineWidth);
        
    end
    
    function plotAngSpring(this,r1,r2,c,numcoils,coilrad,varargin)
        % SPRING         Calculates the position of a 2D spring
        %    plotANGSPRING(r1,r2,c,ne,a,r0) draws an angular spring b/w
        %    2D points r1, and r2, with the center of the ellipse at point
        %    c, with numcoils # of coils, rest angle restang, and coilrad natural radius
        %    of the spring coils
        
        color = this.vectorColor;
        dbgplot = 0;
        CCW = 0; %enforce draw counterclockwise
        CW = 0; %enforce draw clockwise
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'Color'
                    color = value;
                case 'CCW'
                    CCW = value;
            end
        end
        if ~iscolumn(r1)
            r1 = r1';
        end
        if ~iscolumn(r2)
            r2 = r2';
        end
        if ~iscolumn(c)
            c = c';
        end
        
        v1 = r1 - c; v2 = r2 - c;
        rotang = -atan2(v1(2),v1(1));
        RotE = [cos(rotang) -sin(rotang); sin(rotang) cos(rotang)];
        v1 = RotE*v1; v2 = RotE*v2;
        l1 = norm(v1); l2 = norm(v2);
        ang1 = 0; ang2 = atan2(v2(2),v2(1));
        
        springstart = 1/3;
        pts = 30;
        
        l2p = sqrt(v2(2)^2/(1 - v2(1)^2/l1^2));
%         ang2p = acos(v2(1)/l1);
%         ang2p = ang2;
        ang2p = sign(ang2)*acos(v2(1)/l1);
        if CCW && ang2p<0
            ang2p = 2*pi + ang2p;
        end
        
        arc1range = linspace(ang1,springstart*ang2p,pts);
        springrange = linspace(springstart*ang2p,(1-springstart)*ang2p,pts);
        arc2range = linspace((1-springstart)*ang2p,ang2p,pts);
        
        
        xarc1 = l1*cos(arc1range); xarc2 = l1*cos(arc2range);
        yarc1 = l2p*sin(arc1range); yarc2 = l2p*sin(arc2range);
        
        xpts = l1*cos(linspace(springrange(1),springrange(end),2*numcoils+1));
        ypts = l2p*sin(linspace(springrange(1),springrange(end),2*numcoils+1));
        arcvecs = diff([xpts; ypts],[],2);
        
        xloc = zeros(1,2*numcoils + 2); yloc = xloc;
        
        arcangles = bsxfun(@atan2,arcvecs(2,:),arcvecs(1,:));
        exangles = acos(bsxfun(@hypot,arcvecs(1,:),arcvecs(2,:))/2/coilrad);
        
        A = (-1).^([1:length(arcangles)]);
        xloc = [xpts(1) xpts(1:2*numcoils)+coilrad*cos(arcangles + A.*exangles) xpts(end)];
        yloc = [ypts(1) ypts(1:2*numcoils)+coilrad*sin(arcangles + A.*exangles) ypts(end)];
        
%         For loop version, probably much slower, but easier to read
%         for i = 1:2*numcoils
%             if norm(arcvecs(:,i))<=2*coilrad
%             arcangle = atan2(arcvecs(2,i),arcvecs(1,i)); %angle from
%             horizontal of line connecting the two arc points
%             exangle = acos(norm(arcvecs(:,i))/2/coilrad); %angle to add
%             to arcangle such that it forms an isoscoles triangle with the
%             two equal lengths equal to coil rad
%             xhalf = xpts(i) + coilrad*cos(arcangle + (-1)^i * exangle);
%             yhalf = ypts(i) + coilrad*sin(arcangle + (-1)^i * exangle);
%             else
%                 xhalf = xpts(i) + coilrad*cos(arcangle);
%                 yhalf = ypts(i) + coilrad*sin(arcangle);
%             end
%             if i == 1
%                xloc(i) = xpts(i);
%                yloc(i) = ypts(i);
%             end
%             xloc(1,i+1) = xhalf;
%             yloc(1,i+1) = yhalf;
%
%             if i == 2*numcoils
%                 xloc(end) = xpts(i+1);
%                 yloc(end) = ypts(i+1);
%             end
%         end

Rt = RotE.';
pts = [xarc1 xloc xarc2; yarc1 yloc yarc2];
pts = Rt*pts + repmat(c,1,size(pts,2));


plot(pts(1,:),pts(2,:),'Color',color,'LineWidth',this.lineWidth);

if dbgplot
    figure
    plot([c(1) r1(1)],[c(2) r1(2)],'b')
    hold on
    plot([c(1) r2(1)],[c(2) r2(2)],'b')
    plot([0 v1(1)],[0 v1(2)],'r')
    hold on
    plot([0 v2(1)],[0 v2(2)],'r')
    
    prepts = [xarc1 xloc xarc2; yarc1 yloc yarc2];
    plot(prepts(1,:),prepts(2,:),'r');
    
    plot(pts(1,:),pts(2,:),'Color',color,'LineWidth',this.lineWidth);
end

blah =1;
        
    end
    %%
    
    function plotCircSpring(this,r1,r2,rad,dir,numcoils,coilrad,stretch,varargin)
        % SPRING         Calculates the position of a 2D spring
        %    plotANGSPRING(r1,r2,c,ne,a,r0) draws an angular spring b/w
        %    2D points r1, and r2, with the center of the ellipse at point
        %    c, with numcoils # of coils, rest angle restang, and coilrad natural radius
        %    of the spring coils.  dir = 1 draws CCW, dir = 2 draws CW
        
        color = this.vectorColor;
        dbgplot = 0;
        CCW = 1; %draw counterclockwise
        whichroot = 0;
        pts = 40;
        springstart = 1/3;
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'Color'
                    color = value;
                case 'CCW'
                    CCW = value;
            end
        end
        
        if dir ==2
            CCW=0;
        end
        
        if ~iscolumn(r1)
            r1 = r1';
        end
        if ~iscolumn(r2)
            r2 = r2';
        end
        
        
        %Dist between two points on the circle
        R = sqrt( (r2(2) -r1 (2))^2 + (r2(1) - r1(1))^2 );
        
        %If desired radius of circle too small, make it the min size
        %possible
        if (rad/R)^2 < 1/4
            rad = 1/2*R;
        end
        
        %mid point
        m = [(r1(1) + r2(1))/2 ; (r1(2) + r2(2))/2];
        
        %perp dir
        d = [ r1(2) - r2(2) ; r2(1) - r1(1) ];
        
        if ~whichroot
           t = sqrt( (rad/R)^2 - 1/4 ); 
        else
           t = -sqrt( (rad/R)^2 - 1/4 ); 
        end
        
        %Center of circle
        c = m + t*d;
        
        ang1 = atan2(r1(2)-c(2),r1(1)-c(1));
        ang2 = atan2(r2(2)-c(2),r2(1)-c(1));
        
        if ang1<0, ang1 = ang1 + 2*pi; end;  if ang2<0, ang2 = ang2 + 2*pi; end;
        
        if CCW
           if ang2<ang1
               ang2 = ang2 + 2*pi;
           end
        else
            if ang2>ang1
                ang2 = ang2 - 2*pi;
            end
        end
        
        %Change coilrad & spring range accoridng to stretch
        
        coilrad = (1-stretch/(3*numcoils))*coilrad;
        springstart = (1-stretch/(2*3*numcoils))*springstart;
        
        arc1range = linspace(ang1,ang1 + springstart*(ang2-ang1),pts);
        springrange = linspace(ang1 + springstart*(ang2-ang1),ang1+(1-springstart)*(ang2-ang1),pts);
        arc2range = linspace(ang1+(1-springstart)*(ang2-ang1),ang2,pts);
        
        
        xarc1 = c(1) + rad*cos(arc1range); xarc2 = c(1) + rad*cos(arc2range);
        yarc1 = c(2) + rad*sin(arc1range); yarc2 = c(2) + rad*sin(arc2range);
        
        xpts = c(1) + repmat([rad-coilrad rad+coilrad],1,numcoils).*cos(linspace(springrange(1),springrange(end),2*numcoils));
        ypts = c(2) + repmat([rad-coilrad rad+coilrad],1,numcoils).*sin(linspace(springrange(1),springrange(end),2*numcoils));
        
        xpts = c(1) + rad*cos(linspace(springrange(1),springrange(end),2*numcoils+1));
        ypts = c(2) + rad*sin(linspace(springrange(1),springrange(end),2*numcoils+1));
        arcvecs = diff([xpts; ypts],[],2);
        
        xloc = zeros(1,2*numcoils + 2); yloc = xloc;
        
        arcangles = bsxfun(@atan2,arcvecs(2,:),arcvecs(1,:));
        exangles = acos(bsxfun(@hypot,arcvecs(1,:),arcvecs(2,:))/2/coilrad);
        
        A = (-1).^([1:length(arcangles)]);
        xloc = [xpts(1) xpts(1:2*numcoils)+coilrad*cos(arcangles + A.*exangles) xpts(end)];
        yloc = [ypts(1) ypts(1:2*numcoils)+coilrad*sin(arcangles + A.*exangles) ypts(end)];
        

pts = [xarc1 xloc xarc2; yarc1 yloc yarc2];
% pts = [xarc1 xpts xarc2; yarc1 ypts yarc2];


plot(pts(1,:),pts(2,:),'Color',color,'LineWidth',this.lineWidth);

        
    end
    
    function [] = plotFoot(this, contactpoint, stanceangle, varargin)
        %%
        shoeheight = .1;
        footlength = .15;
        
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'shoeheight'
                    shoeheight = value;
                case 'footlength'
                    footlength = value;
            end
        end
        
        legwidth=footlength/3;
        footheight=shoeheight/3;
        
        footangle = stanceangle+pi/2;
        
        %Points that define the vertices of the shoe
        xpoints = [0 0 ...
            legwidth legwidth...
            footlength footlength]-legwidth/2;
        
        ypoints = [0 shoeheight ...
            shoeheight footheight...
            footheight 0];
        
        VecsPreRot = [xpoints;ypoints];
        
        RotMatrix = [cos(footangle) -sin(footangle); sin(footangle) cos(footangle)];
        
        Vecs = RotMatrix*VecsPreRot + repmat(contactpoint',1,length(xpoints));
        
        fill(Vecs(1,:), Vecs(2,:), this.footColor)
        hold on;
        
    end
    
    
    function plotMass(this, point, varargin)
        
        scaling = 1;
        dotColor = this.dotColor;
        for i = 1 : 2 : length(varargin)
            option = varargin{i};
            value = varargin{i + 1};
            switch option
                case 'scaling'
                    scaling = value;
            end
        end
        
        marksize = 5 + scaling*15;
        
              plot(point(1), point(2),...
        'o', 'MarkerFaceColor', dotColor, 'MarkerEdgeColor', this.vectorColor, 'MarkerSize', marksize);
        hold on
        
    end
    
    
  end
  
end

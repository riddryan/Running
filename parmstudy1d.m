function [runners,xstars,cnvrg,myparmrange,index0] = parmstudy1d(runner, x0, parmrange, parmname, varargin)
% ws = parmstudy1d(w, parmrange, 'parmname') performs a one-dimensional
%   parameter study of the class w, setting the parameter named in 'parmname'
%   to the values in the array parmrange. The output argument ws is an array
%   of walking objects, which are referred to be index, e.g. ws(1).
% [ws, parmrangesucc, cnvrg] = parmstudy1d... also returns two arrays, 
%   parmrangesucc being the parameter range that was actually successful, and 
%   cnvrg an array of same size, with exitflags according to fmincon. In cases of "failure"
%   there should be a gait at the extreme of the parameter range, just not
%   at the intended value. So cnvrg(1) and cnvrg(end) should indicate
%   whether either end of desired range was not met, and parmrangesucc(1)
%   and end should indicate what those parameters were.  
% Note that 

% Added by Art July 2008

% CHANGES
%   Modified by Art to include an array of cnvrg indicators of success
%   (6/2009)
%   Modified by Shawn to include parmrangesucc (the succ suffix added
%   by Art

% Example: This performs a parameter study varying gravity 'g' from
% 0.16 - 2, and returns an array of walking objects
%   ws = parmstudy1d(walk2('normal'), linspace(0.16, 2, 20), 'g');
runcharic.speed=[];runcharic.steplength=[];runcharic.airfrac=[];
constrainttolerance = 1e-5;
extraconstraint = [];
parmstovary = [];
initialguessrule = [];
plotiter = 0;
MaxEvals = [];
algorithm = [];

for i = 1 : 2 : length(varargin)
    option = varargin{i};
    value = varargin{i + 1};
    switch option
        case 'extraconstraint'
            extraconstraint = value;
        case 'runcharic'
            runcharic = value;
        case 'parmstovary'
            parmstovary = value;
        case 'MaxEvals'
            MaxEvals = value;
        case 'TolCon'
           constrainttolerance = value;
        case 'Algo'
            algorithm = value;
    end
end

p0 = runner.(parmname);

% We start with a runner that is presumably somewhere within parmrange. 
% Use lowindices and highindices to break the parms into two ranges, 
% below and above the starting gait.
parmrange = parmrange(parmrange~=p0);
lowindices = find(p0 > parmrange);
highindices = find(p0 < parmrange);
% put the known w smack in the middle
parmrange = [parmrange(lowindices) p0 parmrange(highindices)];
highindices = highindices + 1;
index0 = length(lowindices) + 1;
classname = class(runner);
runners(length(parmrange)) = eval(classname); % put the starting point in the array
runners(index0) = runner;
cnvrg = zeros(size(parmrange)); cnvrg(index0) = 1;
ibegin = index0; iend = index0; % try to keep track of the gaits that fail completely
xstars = zeros(runner.N,length(parmrange));
xstars(:,index0) = x0;

% Descending ladder
for i = fliplr(lowindices)
    
    runner.(parmname) = parmrange(i);
    clear -global xLast
    [xstars(:,i),finalPs,~,~,~,cnvrg(i)]=runner.findLimitCycle(x0,'runcharic',runcharic,...
        'TolCon',constrainttolerance,'additionalConstraintFunction',extraconstraint,...
        'plotiter',plotiter,'parametersToAlter',parmstovary,'MaxEvals',MaxEvals,'Algo',algorithm);
    if ~isempty(parmstovary)
        runner = runner.setParametersFromList(parmstovary,finalPs);
    end
    runner
    runners(i)=runner;
    x0 = xstars(:,i)';
    if cnvrg(i)<=0 
       cnvrg(i+1:lowindices(1)) = NaN;
       xstars(:,i+1:lowindices(1)) = NaN;
        break; 
    end;  % give up if we didn't converge
end

% Ascending ladder
runner = runners(index0);
x0 = xstars(:,index0)';
for i = highindices
    runner.(parmname) = parmrange(i);
    clear -global xLast
    [xstars(:,i),finalPs,~,~,~,cnvrg(i)]=runner.findLimitCycle(x0,'runcharic',runcharic,...
        'TolCon',constrainttolerance,'additionalConstraintFunction',extraconstraint,...
        'plotiter',plotiter,'parametersToAlter',parmstovary,'MaxEvals',MaxEvals,'Algo',algorithm);
    if ~isempty(parmstovary)
        runner = runner.setParametersFromList(parmstovary,finalPs);
    end
    runner
    runners(i)=runner;
    x0 = xstars(:,i)';
    if cnvrg(i)<=0 
               cnvrg(i+1:highindices(end)) = NaN;
       xstars(:,i+1:highindices(end)) = NaN;
        break; 
    end;  % give up if we didn't converge
end

myparmrange = parmrange;

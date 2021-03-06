function [runners,xstars,cnvrg,pranges] = parmstudy2d(runner, x0, parmranges, parmnames, varargin)
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
MaxEvals = [];

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
    end
end

[runners,xstars,cnvrg,parmrange1d] = parmstudy1d(runner,x0,parmranges{1},parmnames{1},...
    'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
    'MaxEvals',MaxEvals);

if parmranges{2}(parmranges{2}==runner.(parmnames{2}))
d2 = length(parmranges{2});
n2opts = d2-1;
else
 d2 = length(parmranges{2}) + 1;   
 n2opts = d2;
end

firstrunners = runners;

if length(parmrange1d)>length(parmranges{1})
n1opts = length(parmranges{1});
else
    n1opts = length(parmranges{1})-1;
end

for i = find(cnvrg==1)
    fprintf(1,'Starting optimization %d out of %d\n\n\n',...
             [n1opts + (i-1)*n2opts + 1 n1opts*n2opts]);
    runner = firstrunners(i);
    x0 = xstars(:,i);
    
%     [rtest,xtest,ctest] = parmstudy1d(runner,x0,parmranges{2},parmnames{2},...
%         'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
%         'MaxEvals',MaxEvals);

    [runners(1:d2,i),xstars(:,i,1:d2),cnvrg(1:d2,i),pranges{2}(1:d2,i)] = parmstudy1d(runner,x0,parmranges{2},parmnames{2},...
    'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
    'MaxEvals',MaxEvals);


end

pranges{1} = repmat(parmrange1d,size(cnvrg,1),1);
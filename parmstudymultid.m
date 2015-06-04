function [runners,xstars,cnvrg,pranges,index0] = parmstudymultid(runner, x0, parmranges, parmnames, varargin)
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

pdim = length(parmnames);

if pdim==1
    [runners,xstars,cnvrg,pranges,index0] = parmstudy1d(runner,x0,parmranges{1},parmnames{1},...
    'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
    'MaxEvals',MaxEvals);
return;
end

[runners,xstars,cnvrg,pranges,index0] = parmstudymultid(runner, x0, parmranges{1:end-1}, parmnames{1:end-1}, varargin{:});

n = ndims(cnvrg);
prange = parmranges{end};
pname = parmnames{end};
xsz = size(xstars,1);

if prange(prange==runner.(pname))
dim = length(parmranges{2});
else
 dim = length(parmranges{2}) + 1;   
end

convergearray = reshape(cnvrg,numel(cnvrg),1);
ind = sub2ind(size(cnvrg),
for i = find(convergearray==1)
    runner = runners(index0);
    for j = 1:xsz
        x0(j,1) = xstars(j,i);
    end
    
    [r1,x1,c1] = parmstudy1d(runner,x0,parmranges{2},parmnames{2},...
        'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
        'MaxEvals',MaxEvals);

[runners(1:dim,i),xstars(:,i,1:dim),cnvrg(1:dim,i),pranges{2}(1:dim,i)] = parmstudy1d(runner,x0,parmranges{2},parmnames{2},...
    'runcharic',runcharic,'parmstovary',parmstovary,'extraconstraint',extraconstraint,'TolCon',constrainttolerance,...
    'MaxEvals',MaxEvals);


end
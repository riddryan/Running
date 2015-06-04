function [y0,yp0,resnrm] = getDAE_IC(odefun,t0,y0,fixed_y0,yp0,fixed_yp0,options,constraints,varargin)
%getDAE_IC  Compute consistent initial conditions for ODE15I.
%   [Y0MOD,YP0MOD] = DECIC(ODEFUN,T0,Y0,YP0) uses the input
%   Y0,YP0 as initial guesses for an iteration to find output values such
%   that ODEFUN(T0,Y0MOD,YP0MOD) = 0. ODEFUN is a function handle.  T0 is 
%   a scalar, Y0 and YP0 are column vectors.
%

% 
%   [Y0MOD,YP0MOD] = DECIC(ODEFUN,T0,Y0,YP0,OPTIONS)
%   computes as above with default values of integration tolerances replaced 
%   by the values in OPTIONS, a structure created with the ODESET function. 

%   [Y0MOD,YP0MOD] = DECIC(ODEFUN,T0,Y0,YP0,OPTIONS,CONSTRAINTS)
%   CONSTRAINTS is an additional function such that CONSTRAINTS(Y0) = 0 is
%   desired.
%
%   [Y0MOD,YP0MOD,RESNRM] = DECIC(ODEFUN,T0,Y0,YP0...)
%   returns the norm of ODEFUN(T0,Y0MOD,YP0MOD) as RESNRM. If the norm 
%   seems unduly large, use OPTIONS to specify smaller RelTol (1e-3 by default).
%
%   See also ODE15I, ODESET, IHB1DAE, IBURGERSODE, FUNCTION_HANDLE.

%   Jacek Kierzenka and Lawrence F. Shampine

%   Modified by Ryan Riddick to solve for consistent initial conditions
%   that also satisfy additional constraints, instead of holding certain
%   variables fixed
neq = length(y0);
if isempty(fixed_y0)
  free_y = 1:neq;
else
  free_y = find(fixed_y0 == 0);
end
if isempty(fixed_yp0)
  free_yp = 1:neq;
else
  free_yp = find(fixed_yp0 == 0);
end
if length(free_y) + length(free_yp) < neq
  error(message('MATLAB:decic:TooManySpecified',sprintf('%i', neq)));
end

if nargin < 7
  options = [];
end

if nargin<8
   constraints = []; 
end

rtol = odeget(options,'RelTol',1e-3,'fast');
if (length(rtol) ~= 1) || (rtol <= 0)
  error(message('MATLAB:decic:OptRelTolNotPosScalar'));
end
if rtol < 100 * eps 
  rtol = 100 * eps;
  warning(message('MATLAB:decic:RelTolIncrease',sprintf('%g',rtol)));
end
atol = odeget(options,'AbsTol',1e-6,'fast');
if any(atol <= 0)
  error(message('MATLAB:decic:OptAbsTolNotPos'));
end

y0 = y0(:);
yp0 = yp0(:);
res = feval(odefun,t0,y0,yp0,varargin{:});

% Initialize the partial derivatives
[Jac,dfdy,dfdyp,~,dfdy_options,dfdyp_options] = ...
    ode15ipdinit(odefun,t0,y0,yp0,res,options,varargin);

if ~isempty(constraints)
    dCdy = jacobianest(constraints,y0);
    dfdy = [dfdy;dCdy];
    dfdyp = [dfdyp;zeros(size(dCdy))];
    resC = feval(constraints,y0);
else
    resC = [];
end

resnrm0 = norm([res;resC]);


for counter = 1:10
    
  for chord = 1:3
    [dy,dyp] = sls([res;resC],dfdy,dfdyp,neq,free_y,free_yp);
    
    % If the increments are too big, limit the change
    % in norm to a factor of 2--trust region.
    nrmv = max(norm([y0; yp0]),norm(atol));
    nrmdv = norm([dy; dyp]);
    if nrmdv > 2*nrmv
      factor = 2*nrmv/nrmdv;
      dy = factor*dy;
      dyp = factor*dyp;
      nrmdv = factor*nrmdv;
    end
    y0 = y0 + dy;
    yp0 = yp0 + dyp;
    res = feval(odefun,t0,y0,yp0,varargin{:});
    if ~isempty(constraints)
        resC = feval(constraints,y0);
    end
    resnrm = norm([res;resC]);
    
    % Test for convergence.  The norm of the residual must
    % be no greater than the initial guess and the norm of
    % the increments must be small in a relative sense.
    if (resnrm <= resnrm0) && (nrmdv <= 1e-3*rtol*nrmv)
      return;
    end   
  end 
  
  [dfdy,dfdyp,dfdy_options,dfdyp_options] = ...
      ode15ipdupdate(Jac,odefun,t0,y0,yp0,res,dfdy,dfdyp,...
                     dfdy_options,dfdyp_options,varargin);
                 
                 
if ~isempty(constraints)
    dCdy = jacobianest(constraints,y0);
    dfdy = [dfdy;dCdy];
    dfdyp = [dfdyp;zeros(size(dCdy))];
end
  
end


%---------------------------------------------------------------------------

function [dy,dyp] = sls(res,dfdy,dfdyp,neq,free_y,free_yp)
% Solve the underdetermined system 
%           0 = res + dfdyp*dyp + dfdy*dy
% A solution is obtained with as many components as
% possible of (transformed) dy and dyp set to zero.

dy = zeros(neq,1);
dyp = zeros(neq,1);

fixed = (neq - length(free_y)) + (neq - length(free_yp));

if isempty(free_y)  % Solve 0 = res + dfdyp*dyp
  [rank,Q,R,E] = qrank(dfdyp);
  rankdef = neq - rank;
  if rankdef > 0
    if rankdef <= fixed
      error(message('MATLAB:decic:TooManyFixed',sprintf('%i',rankdef)));
    else
      error(message('MATLAB:decic:IndexGTOne'));
    end
  end
  d = - Q' * res;
  dyp = E * (R \ d);
  return
end

if isempty(free_yp)  % Solve 0 = res + dfdy*dy
  [rank,Q,R,E] = qrank(dfdy);
  rankdef = neq - rank;
  if rankdef > 0
    if rankdef <= fixed
      error(message('MATLAB:decic:TooManyFixed',sprintf('%i',rankdef)));
    else
      error(message('MATLAB:decic:IndexGTOne'));
    end
  end
  d = - Q' * res;
  dy = E * (R \ d);
  return
end


% Eliminate variables that are not free.
dfdy = dfdy(:,free_y);
dfdyp = dfdyp(:,free_yp);

[rank,Q,R,E] = qrank(dfdyp);
d = - Q' * res;
if rank == neq
  dy(free_y) = 0;
  dyp(free_yp) = E * (R \ d);
else
  S = Q' * dfdy;
  Srank = qrank(S(rank+1:end,:));
  rankdef = neq - (rank + Srank);
  if rankdef > 0
    if rankdef <= fixed
      error(message('MATLAB:decic:TooManyFixed',sprintf('%i',rankdef)));            
    else
      error(message('MATLAB:decic:IndexGTOne'));
    end
  end
  w = S(rank+1:end,:) \ d(rank+1:end); 
  w1p = R(1:rank,1:rank) \ (d(1:rank) - S(1:rank,:) * w);
  dy(free_y) = w;
  nfree_yp = length(free_yp); 
  dyp(free_yp) = E * [w1p; zeros(nfree_yp-rank,1)];
end

%---------------------------------------------------------------------------

function [rank,Q,R,E] = qrank(A)
% QR decomposition with column pivoting and rank determination.
% Have to make A full so that pivoting can be done.
[Q,R,E] = qr(full(A));
tol = max(size(A),[],2)*eps*abs(R(1,1));
rank = nnz(abs(diag(R)) > tol);  % Account for R that are neq x 1.


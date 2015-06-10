%% Options
[datasoft,dsp,dsl,daf,dsf,datagrf] = getHumanData(7, 2);
runcharic.speed = dsp;
runcharic.speed = [];
% runcharic.steplength = dsl;
runcharic.steplength = [];
runcharic.airfrac = 0;
% runcharic.airfrac = [];
addedconstraints = [];
parmstovary = [];
constrainttolerance = 1e-4;
MaxEvals = 1000;

loadfolder = '.\SavedGaits\Swing\';

%% Nominal Swing Model (No impulse)

loadname = 'Swing1.mat';
load([loadfolder loadname]);

parameters = {'g' 'swingl' 'hipl' 'kswing' 'khip'};
[A, B] = r.linearizeOneStepReturnMap(xstar, parameters);

[V,D] = eig(A);
eigvals = diag(D);


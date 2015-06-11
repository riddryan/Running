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
savename = 'PertSize.mat';

load([loadfolder loadname]);
savefolder = ['.\ReturnMapStudies\' class(r) '\'];

r.useHSevent = 1;

parameters = {'gslope' 'g' 'swingl' 'hipl' 'kswing' 'khip'};

perts = linspace(-1e-2,1e-2,20);
perts(perts==0)=[];

A = zeros(4,4,length(perts));
B = zeros(length(parameters),4,length(perts));
V = A;
D = A;
eigvals = zeros(4,length(perts));

for i = 1:length(perts) 
    [A(:,:,i),B(:,:,i)] = r.linearizeOneStepReturnMap(xstar, parameters,'perturbationAmount',perts(i));
    
    [V(:,:,i),D(:,:,i)] = eig(A(:,:,i));
    eigvals(:,i) = diag(D(:,:,i));
end

figure
subplot(221)
plot(perts,eigvals(1,:));
ylabel('eigenvalue')
subplot(222)
plot(perts,eigvals(2,:));
subplot(223)
plot(perts,eigvals(3,:));
xlabel('Perturbation Amt')
ylabel('eigenvalue')
subplot(224)
plot(perts,eigvals(4,:));
xlabel('Perturbation Amt')


save([savefolder savename]);

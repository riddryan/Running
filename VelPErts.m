clearvars



currdir = cd;

load([cd '\SavedGaits\StomachDamping.mat'])

xstar0 = xstar;

xstarf = onestep(r,xstar0);

energystar0 = r.getEnergies(xstar0).Total;
energystarf = r.getEnergies(xstarf).Total;

for i = 1:20
    
randdist = normrnd([0 0 0],.05);
    
xpert0 = xstar + [randdist(1) 0 0 0 0 ...
                 randdist(2) randdist(3) 0 0 0];
             
             xpert0 = r.GoodInitialConditions(xpert0);
             
             
             EnergyPert0(i) = r.getEnergies(xpert0).Total;
             
             
xpertf = onestep(r,xpert0);

EnergyPertf(i) = r.getEnergies(xpertf).Total;
    
end
               

figure
plot(EnergyPert0,EnergyPertf,'x')

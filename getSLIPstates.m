function [xpacc,ypacc,stanceangle,stancelength,xp,yp,xvel,yvel,angvel,lengthvel] = getSLIPstates(SLIPdata,targettimes)

SLIPtimes = SLIPdata(:,1);
SLIPstates = SLIPdata(:,2:end);
interpstates = interp1(SLIPtimes,SLIPstates,targettimes,'spline');

xpacc = interpstates(:,1);
ypacc = interpstates(:,2);
stanceangle = interpstates(:,3);
stancelength = interpstates(:,4);
xp = interpstates(:,5);
yp = interpstates(:,6);
xvel = interpstates(:,7);
yvel = interpstates(:,8);
angvel = interpstates(:,9);
lengthvel = interpstates(:,10);
end
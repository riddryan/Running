function [xpacc,ypacc,stanceangle,stancelength,xp,yp,xvel,yvel,angvel,lengthvel] = getSLIPstates(SLIPdata,targettimes)

% load('./SavedGaits/SLIP_NoAerial_unmatchedSL')
% SLIPtimes = allt;
% SLIPstates = allx;
% [SLIPtimes,dexes] = unique(SLIPtimes);
% SLIPstates = SLIPstates(dexes,:);
% phasevec = phasevec(dexes);
% 
% accs = zeros(length(SLIPtimes),8);
% for i = 1:length(SLIPtimes)
%   accs(i,:) = r.XDoubleDot(allt(i),allx(i,:)',r.phases{phasevec(i)});  
% end
% 
% SLIPdata = [allt accs(:,[1 2]) SLIPstates(:,3:4) SLIPstates(:,1:2) SLIPstates(:,5:8)];
% 
% %Make as numerically correct the foot position as possible (should be at
% %0,0)
% thetaf=atan2(-xf(2),-xf(1));
% rf = norm(xf(1:2));
% SLIPdata(end,[4 5]) = [thetaf rf];
%            
% clear SLIPtimes targettimes SLIPstates dexes i
% save('./SavedGaits/SLIP_NoAerial_unmatchedSL')

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
function [ SLIPdata, SLIPx0, SLIPxf ] = getSLIPdata( fname )
%GETSLIPDATA compiles all state & acceleration during a limit cycle of a
%SLIP model gait given in the file fname and saves it back into the same
%file.  If the SLIP information already exists in the file, it returns the
%output variables without recalculating them.

load(fname)

if exist('SLIPdata','var') && exist('SLIPx0','var') && exist('SLIPxf','var')
    return;
end

SLIPtimes = allt;
SLIPstates = allx;
[SLIPtimes,dexes] = unique(SLIPtimes);
SLIPstates = SLIPstates(dexes,:);
phasevec = phasevec(dexes);

accs = zeros(length(SLIPtimes),8);
for i = 1:length(SLIPtimes)
  accs(i,:) = r.XDoubleDot(allt(i),allx(i,:)',r.phases{phasevec(i)});  
end

SLIPdata = [allt accs(:,[1 2]) SLIPstates(:,3:4) SLIPstates(:,1:2) SLIPstates(:,5:8)];

%Make as numerically correct the foot position as possible (should be at
%0,0)
thetaf=atan2(-xf(2),-xf(1));
rf = norm(xf(1:2));
SLIPdata(end,[4 5]) = [thetaf rf];
SLIPx0 = [SLIPdata(end,4);SLIPdata(end,5)];
SLIPxf = [SLIPdata(1,4);SLIPdata(1,5)];
           
clear SLIPtimes targettimes SLIPstates dexes i
save(fname)


end


function [SoftPower,Speed,StepLength,AirFrac,StepFreq,grf_z,grf_y] = getHumanData( subject, trial)
%getHumanData outputs soft tissue power & gait characterstics from human
%running experiment on treadmill.

% This function filters and interpolates the soft tissue power data to a
% fewer number of points.
% rootdir = fullfile(cd,'../../../../../');
load('AllSubjectsPower.mat')
% subject=7;
% trial=2;
normfiltfreq=0.35;
samplestouse=100;

%% Get desired gait characterstics (step freq, soft power, speed, etc.)
datasoftpower = fp.softpower{trial,subject};
%soft power from heelstrike to toe off
toeoff = round(fp.stancetime{trial,subject}*fp.GlobalSampleFreq{trial,subject});
%Smooth the data
[B,A] = butter(3,normfiltfreq);
filtsoft = filtfilt(B,A,datasoftpower(1:toeoff));
%Interpolate the data to fewer points
filtsoft = interp1(linspace(0,1,length(filtsoft)),filtsoft,linspace(0,1,samplestouse));
filtsoft = filtsoft*fp.powernorm{trial,subject};
grf_z = fp.mRgrf{trial,subject}(1:toeoff,3)/fp.m{trial,subject}/9.81;
grf_y = fp.mRgrf{trial,subject}(1:toeoff,2)/fp.m{trial,subject}/9.81;

SoftPower = filtsoft;

legL = fp.velnorm{trial,subject}^-2/9.81;
tnorm = fp.velnorm{trial,subject}*legL;
Speed = fp.Speeds{trial,subject} * fp.velnorm{trial,subject};
steptime = fp.Strtime{trial,subject}/2/tnorm;
StepFreq = 1/steptime;
StepLength = Speed/StepFreq;
AirFrac = mean(fp.LHS{trial,subject}-fp.RTO{trial,subject})/mean(fp.LHS{trial,subject}-fp.RHS{trial,subject});


end


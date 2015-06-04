function r = Gauss7Kronrod15
% Gauss-Kronrod (7,15) pair.
nodes = [ ...
    -0.9914553711208126, -0.9491079123427585, -0.8648644233597691, ...
    -0.7415311855993944, -0.5860872354676911, -0.4058451513773972, ...
    -0.2077849550078985, 0, 0.2077849550078985, ...
    0.4058451513773972, 0.5860872354676911, 0.7415311855993944, ...
    0.8648644233597691, 0.9491079123427585, 0.9914553711208126];
wt15 = [ ...
    0.02293532201052922, 0.06309209262997855, 0.1047900103222502, ...
    0.1406532597155259, 0.1690047266392679, 0.1903505780647854, ...
    0.2044329400752989, 0.2094821410847278, 0.2044329400752989, ...
    0.1903505780647854, 0.1690047266392679, 0.1406532597155259, ...
    0.1047900103222502, 0.06309209262997855, 0.02293532201052922];
wt7 = [ ...
    0, 0.1294849661688697, 0, ...
    0.2797053914892767, 0, 0.3818300505051189, ...
    0, 0.4179591836734694, 0, ...
    0.3818300505051189, 0, 0.2797053914892767, ...
    0, 0.1294849661688697, 0];
r = struct('Nodes',nodes,'HighWeights',wt15,'LowWeights',wt7);

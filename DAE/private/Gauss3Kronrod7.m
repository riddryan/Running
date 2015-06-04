function r = Gauss3Kronrod7
% Gauss-Kronrod (3,7) pair.
nodes = [ ...
    -0.9604912687080202,-0.7745966692414834,-0.4342437493468026, ...
    0, ...
    0.4342437493468026,0.7745966692414834,0.9604912687080202];
wt3 = [0, 5/9, 0, 8/9, 0, 5/9, 0];
wt7 = [ ...
    0.1046562260264672,0.2684880898683334,0.4013974147759622, ...
    0.4509165386584744, ...
    0.4013974147759622,0.2684880898683334,0.1046562260264672];
r = struct('Nodes',nodes,'HighWeights',wt7,'LowWeights',wt3);

function  cost = generate_cost(pDes, st, ed, F, verbose)
% GENERATE_COST() generates cost map according to given feature H
% Written by Qiong Wang at University of Pennsylvania
% 04/06/2014

% INPUT
% pDes   -- Desired path
% st, ed -- Start and end point
% F      -- Feature

% Initialize
if nargin < 3
    verbose = false;
end
fNum   = size(F, 3);
w      = zeros(1, 1, fNum);
alpha  = ;
lambda = ;

cost = 1 ./ (exp(bsxfun(@times, F, w)) + 1);
ctg  = dijkstra_matrix(costs, ed(1), ed(2));
pMin = dijkstra_path2(ctg, costs, st(1), st(2));


if ~verbose
    return;
end

figure(4); imagesc(cost); title('Cost Map');
end

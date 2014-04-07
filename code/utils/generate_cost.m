function  cost = generate_cost(pDes, sts, eds, F, sz, verbose)
% GENERATE_COST() generates cost map according to given feature H
% Written by Qiong Wang at University of Pennsylvania
% 04/06/2014

% INPUT
% pDes   -- Desired path
% st, ed -- Start and end point
% sz     -- Size of the map
% F      -- Feature

% Initialize
if nargin < 6
    verbose = false;
end
fNum   = size(F, 1);            % Feature Number
N      = size(sts, 1);          % Paths Number
alpha  = 1;                     % Learning Rate
lambda = 1;
T      = 10;                   % Maximum Iteration

% Precompute pDes visitation vector
pIdxDes = zeros(size(F, 2), N);
pIdxMin = zeros(size(F, 2), 1);
for i = 1 : N
    entries = sub2ind(sz, pDes{i}(:, 2), pDes{i}(:, 1));
    pIdxDes(entries, i) = 1;
end

% Maximum Margin Planning
t = 1;
w = zeros(fNum, 1);
g = 0;
while t < 2
    t
    cost    = reshape(1 ./ (exp(w'* F) + 1), sz(1), sz(2));
    for i = 1 : N
        % Compute cost and min path
        ctg     = dijkstra_matrix(cost, eds(i, 1), eds(i, 2));
        pMin    = dijkstra_path2(ctg, cost, sts(i, 1), sts(1, 2));
        indices = sub2ind(sz, pMin(:, 2), pMin(:, 1));
        pIdxMin(indices) = 1;
        
        % Compute the descent gradient
        beta = 1 ./ sum(sqrt(sum((pDes{i}(1 : end - 1, :) - pDes{i}(2 : end, :)).^2, 2)));
        l    = pIdxMin & pIdxDes(:, i);
        tmp  = 2 * beta * ((w' * F + l') * pIdxDes(:, i) - w' * F * pIdxMin) ...
               .* F * (pIdxDes(:, i) - pIdxMin);
        g  = g + tmp;
    end
    g     = g / N + lambda .* w;
    w     = w - alpha .* g ;
    t     = t + 1;
    alpha = alpha / t; 
end

if ~verbose
    return;
end

figure(4); imagesc(cost); title('Cost Map');
end
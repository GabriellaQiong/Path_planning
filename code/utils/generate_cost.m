function [cost, w] = generate_cost(pDes, sts, eds, F, sz, verbose)
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
T      = 10;                    % Maximum Iteration

% Precompute pDes visitation vector and beta
pIdxDes = zeros(size(F, 2), N);
beta    = zeros(N, 1);
for i = 1 : N
    beta(i) = 1 ./ sum(sqrt(sum((pDes{i}(1 : end - 1, :) - pDes{i}(2 : end, :)).^2, 2)));
    entries = sub2ind(sz, pDes{i}(:, 2), pDes{i}(:, 1));
    pIdxDes(entries, i) = 1;
end

% Maximum Margin Planning
t    = 0;
w    = zeros(fNum, 1);
while t < T
    t
    for i = 1 : N
        % Compute cost and min path
        l       = ~ pIdxDes(:, i);
        cost    = reshape(exp(w'* F) - l', sz(1), sz(2));
        ctg     = dijkstra_matrix(cost, eds(i, 2), eds(i, 1));
        pMin    = dijkstra_path2(ctg, cost, sts(i, 2), sts(i, 1));
        indices = round(sub2ind(sz, pMin(:, 1), pMin(:, 2)));
        pIdxMin = zeros(size(F, 2), 1);
        pIdxMin(indices) = 1;
        h       = F * pIdxMin - F * pIdxDes(:, i);
    end
    w     = w + alpha .* h ;
    t     = t + 1;
    alpha = alpha / t; 
end

if ~verbose
    return;
end

% cost    = reshape(1 ./ (exp(w'* F) + 1), sz(1), sz(2));
figure(4); imshow(cost); title('Cost Map');
hold on; c = 'rgbcy';
for i = 1 : N
    ctg     = dijkstra_matrix(cost, eds(i, 2), eds(i, 1));
    pMin    = dijkstra_path2(ctg, cost, sts(i, 2), sts(i, 1));
    plot(pMin(:, 2), pMin(:, 1), [c(i), 'o']);
end
end
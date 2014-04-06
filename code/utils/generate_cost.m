function  cost = generate_cost(path, H, verbose)
% GENERATE_COST() generates cost map according to given feature H
% Written by Qiong Wang at University of Pennsylvania
% 04/06/2014

if nargin < 3
    verbose = false;
end

fNum = size(H, 3);
w    = zeros(1, 1, fNum);
cost = 1 / (exp(bsxfun(@times, H, w)) + 1);

if ~verbose
    return;
end

figure(4); imagesc(cost); title('Cost Map');
end

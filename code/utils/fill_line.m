function [x, y] = fill_line(pts)
% FILL_LINE() take the waypoints to make line
% Written by Qiong Wang at University of Pennsylvania
% 04/04/2014

% Split into waypoints lines
wNum = size(pts, 1);    % Number of waypoints
tNum = 0;               % Total pixel number

% Loop each line
for i = 1 : wNum - 1
    % Compute gradient
    st   = pts(i, :);
    ed   = pts(i + 1, :);
    grad = (ed(2) - st(2)) / (ed(1) - st(1));
    pNum = max(ed(2) - st(2), ed(1) - st(1)) - 1;  % Pixel number per line
    init = tNum + 1;
    tNum = tNum + pNum;
    
    if grad == Inf
        % Vertical line
        x(init : tNum) = st(1);
        y(init : tNum) = round(linspace(st(2), ed(2), pNum));
    elseif grad == 0
        % Horizontal line
        x(init : tNum) = round(linspace(st(1), ed(1), pNum));
        y(init : tNum) = st(2);
    else
        x(init : tNum) = round(linspace(st(1), ed(1), pNum));
        y(init : tNum) = round(grad * (x(init : tNum) - st(1)) + st(2));
    end
end
end
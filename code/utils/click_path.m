function path = click_path(map, st, ed, verbose)
% CLICK_PATH() gets the path after clicking waypoints from start to end
% Written by Qiong Wang at University of Pennsylvania
% 04/04/2014

if nargin < 4
    verbose = false;
end

% Select way points
h = figure; imshow(map);
if ~isempty(st) && ~isempty(ed)
    hold on; 
    plot(st(1), st(2), 'r+', 'MarkerSize', 10); plot(st(1), st(2), 'bx', 'MarkerSize', 10);
    plot(ed(1), ed(2), 'm+', 'MarkerSize', 10); plot(ed(1), ed(2), 'gx', 'MarkerSize', 10);
    hold off;
end
[wx, wy] = getline(h);
pts      = round([st; [wx, wy]; ed]);
[px, py] = fill_line(pts);
path     = [px, py];
if ~verbose
    return;
end
hold on;plot(px, py, 'co');
end
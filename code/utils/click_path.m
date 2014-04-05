function path = click_path(st, ed, type, h, verbose)
% CLICK_PATH() gets the path after clicking waypoints from start to end
% Written by Qiong Wang at University of Pennsylvania
% 04/04/2014

if nargin < 5
    verbose = false;
end
assert(ishandle(h), 'The input should include a figure handle');

% Select way points
if ~isempty(st) && ~isempty(ed)
    plot(st(1), st(2), 'r+', 'MarkerSize', 10); plot(st(1), st(2), 'bx', 'MarkerSize', 10);
    ts = text(st(1) + 40, st(2), 'Start', 'FontSize', 14, 'FontWeight', 'bold', 'BackgroundColor',[.7 .7 .9]);
    plot(ed(1), ed(2), 'm+', 'MarkerSize', 10); plot(ed(1), ed(2), 'gx', 'MarkerSize', 10);
    te = text(ed(1) + 40, ed(2), 'End', 'FontSize', 14, 'FontWeight', 'bold', 'BackgroundColor',[.7 .7 .9]);
end
[wx, wy] = getline(h);
pts      = round([st; [wx, wy]; ed]);
[px, py] = fill_line(pts);
path     = [px, py];

if ~verbose
    return;
end

if strcmp(type, 'p')
    c = 'c';
elseif strcmp(type, 'v')
    c = 'y';
else
    c = [1, 153/255, 204/255];
end
pl = plot(px, py, 'o'); set(pl, 'Color', c);
uistack(ts, 'top'); uistack(te, 'top');
end
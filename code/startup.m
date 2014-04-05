% Startup Script for Path Planning for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 04/05/2014

files = strread(genpath(scriptDir),'%s','delimiter',':'); %#ok<FPARK>
for i = 1 : numel(files)
    files{i} = [files{i}, '/*.m~'];
end
cellfun(@delete, files);
% Run Script for SLAM for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 03/17/2014

%% Clear up
clear all;
close all;
clc;

%% Parameters
% Flags
verbose = true;                  % Whether to show the details
train   = true;                  % Whether to train
test    = false;                 % Whether to test

%% Paths
scriptDir  = fileparts(mfilename('fullpath'));
outputDir  = fullfile(scriptDir, '../results');
if ~exist(outputDir, 'dir')
    mkdir(outputDir); 
end
addpath(genpath(scriptDir));

%% Load data
map = imread('aerial_color.jpg');
C   = makecform('srgb2lab');
lab = applycform(map, C);
vec = reshape(lab, [], 3);
        
%% Parse data
[dc, alpha, tsEn]  = parse_encoders(Encoders);
[tilt, rpy, tsImu] = parse_imu(Imu, verbose);
indices            = sync_time(tsImu, tsEn);
rpy                = rpy(:, indices);
theta              = angle_fuse(alpha, rpy(3, :), tsEn, verbose);

%% Dead Reckoning
% Initialize
[x, y, theta] = dead_reckoning(dc, alpha, theta, tsEn, verbose);

% Sync the time
tsHo   = Hokuyo.ts;
ranges = Hokuyo.ranges;
angles = Hokuyo.angles;

if length(tsEn) > length(tsHo)
    ts  = tsHo;
    ind = sync_time(tsEn, ts);
    x   = x(ind);
    y   = y(ind);
    rpy = rpy(:, ind);
else
    ts     = tsEn;
    ind    = sync_time(tsHo, ts);
    ranges = ranges(:, ind);
end
numData = length(ts);

% Test the results
if check
    testMapCorrelation
end

%% SLAM
MCL_SLAM;

%% Integrate Kinect data
if ~kinect
    return;
end

% Initialize and load data
cMap = zeros(size(MAP.map), 3);
k    = load(fullfile(dataDir, ['kinect', num2str(dataIdx)]));

% Process the kinect data
process_kinect;

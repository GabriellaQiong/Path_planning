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

% Params
featNum = 5;                     % Number of features
H       = cell(featNum, 1);

%% Paths
scriptDir  = fileparts(mfilename('fullpath'));
outputDir  = fullfile(scriptDir, '../results');
if ~exist(outputDir, 'dir')
    mkdir(outputDir); 
end
addpath(genpath(scriptDir));

%% Load Data and Preprocess Data
% Load original map
orig = imread('aerial_color.jpg');

% Preprocess map
% map  = medfilt2(orig);
map  = orig;
se   = strel('ball', 5, 5);
map  = imdilate(map, se);
map  = imerode(map, se);

% Map data into Lab space
C    = makecform('srgb2lab');
lab  = applycform(map, C);
as   = lab(:, :, 2);
bs   = lab(:, :, 3);
        
%% Train


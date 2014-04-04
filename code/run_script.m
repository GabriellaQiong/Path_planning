% Run Script for Path Planning for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 04/04/2014

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
pathNum = 5;

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
orig = imresize(orig, 1/2);

% Remove noise
map  = zeros(size(orig), 'uint8' );
for i = 1 : 3
    map(:, :, i)  = medfilt2(orig(:, :, i));
end

% Fuzzy the map
se   = strel('ball', 5, 5);
map  = imdilate(map, se);
map  = imerode(map, se);
if verbose
    h1 = figure(1); imshow(map);
end
bw   = rgb2gray(map);
H{5} = edge(bw, 'sobel');

% Map data into Lab space
C    = makecform('srgb2lab');
lab  = applycform(map, C);
as   = lab(:, :, 2);
bs   = lab(:, :, 3);
        
%% Train
if train
    % Set start and end points
    sts = zeros(pathNum, 2);
    eds = zeros(pathNum, 2);
    for i = 1 : pathNum
        if ~exist('h', 'var')
            h = figure; imshow(map);
        end
        [x, y] = getpts(h);
        sts(i, :) = round([x(1), y(1)]);
        eds(i, :) = round([x(2), y(2)]);
    end
%%  
    % Click the path
    h2 = figure(2);
    pd = []; vh = [];
    for i = 1 : pathNum
        clf; imshow(map);
        t = title('Please click the path for {\bfpedestrian} :)');
        p1 = click_path(sts(i, :), eds(i, :), 'p', h2, verbose);
        set(t, 'String', ('Please click the path for {\bfvehicle} :)'));
        p2 = click_path(sts(i, :), eds(i, :), 'v', h2, verbose);
        set(t, 'String', ('Please press {\itEnter} for the next path set :)'));
        pause;
        pd = [pd; p1];
        vh = [vh; p2];
    end
end
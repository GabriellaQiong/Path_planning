% Run Script for Path Planning for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 04/04/2014

%% Clear up
clear all;
close all;
clc;

%% Parameters
% Flags
verbose = true;               
% Whether to show the details
train   = true;                  % Whether to train
test    = false;                 % Whether to test

% Params
featNum = 5;                     % Number of features
pathNum = 5;
T       = 5;
thresh  = 20;

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
% if verbose
%     h1 = figure(1); imshow(map);
% end
bw   = rgb2gray(map);
sz   = size(bw);

% Map data into Lab space
C    = makecform('srgb2lab');
lab  = applycform(map, C);
as   = lab(:, :, 2);
bs   = lab(:, :, 3);
cVec = double(reshape(lab, [], 3));
        
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
    % Choose the region
    disp('Please choose the sidewalk');
    GY = [];
    for i = 1 : T
        mask = roipoly(map);
        GY = [GY; cVec(reshape(mask, [], 1), :)];
    end
    
    disp('Please choose the road');
    AS = [];
    for i = 1 : T
        mask = roipoly(map);
        AS = [AS; cVec(reshape(mask, [], 1), :)];
    end
    
    disp('Please choose the green area');
    GN = [];
    for i = 1 : T
        mask = roipoly(map);
        GN = [GN; cVec(reshape(mask, [], 1), :)];
    end
    close(gcf);
    
    %%
    % Add region feature
    grey    = mean(GY, 1);
    asphalt = mean(AS, 1);
    green   = mean(GN, 1);
    
    walk = sum(bsxfun(@minus, cVec, grey).^2, 2) <= thresh;
    road = sum(bsxfun(@minus, cVec, asphalt).^2, 2) <= thresh;
    lawn = sum(bsxfun(@minus, cVec, green).^2, 2) <= thresh;
    
    F       = zeros(featNum, numel(bw));
    F(1, :) = walk;
    F(2, :) = road;
    F(3, :) = lawn;
    F(4, :) = ~(walk | road | lawn); 
    F(5, :) = reshape(edge(bw, 'sobel'), [], 1);
    clear walk road lawn grey asphalt green mask GY AS GN;
    
    %%
    % Click the path and train cost map
    h2 = figure(2);
    pd = cell(pathNum, 1); vh = cell(pathNum, 1);
    for i = 1 : pathNum
        clf; imshow(map); hold on;
        t     = title('Please click the path for {\bfpedestrian} :)');
        pd{i} = click_path(sts(i, :), eds(i, :), 'p', h2, verbose);
        set(t, 'String', ('Please click the path for {\bfvehicle} :)'));
        vh{i} = click_path(sts(i, :), eds(i, :), 'v', h2, verbose);
        set(t, 'String', ('Please press {\itEnter} for the next path set :)'));
        pause;
    end
    close(h2);
    %%
    cp = generate_cost(pd, sts, eds, F, sz, verbose);
%     cv = generate_cost(vh, sts, eds, F, sz, verbose);
end
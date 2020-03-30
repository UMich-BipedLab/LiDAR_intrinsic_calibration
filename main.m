%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

%% Check data (DO NOT add 'clear' here)
clc
if exist('pc','var') &&  exist('data', 'var') && exist('mat_files', 'var')
% if exist('pc','var') 
    disp("Data have been loaded, it may take a while to reload them.")
    prompt = "Are you sure you want to reload them again? [Y] [N]";
    urs_ans = input(prompt, 's');
    if contains(urs_ans, 'N','IgnoreCase', true)
        disp('Reload datasets cancelled')
        return
    else
        clear
        disp("ALL varialbes cleared! ")
        disp("Keep reloading datasets...")
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parameters of user setting  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% utils_path: path of utils functions
% opts.mat_path: directory of saved mat files
% opts.load_all(0/1): 
%               0: only load the files specified in mat_files structure
%               1: load all the files from the opts.mat_path
% opts.datatype: 1: experiment data
%                2: simulation data
% opts.show_results(0/1): plot the calibrated results on figure


matlab_utils = "./matlab_utils";
opts.mat_path = "../intrinsic_latest/data/";
opts.load_all = 1;
opt_datatype = ["Experiment", "Simulation"];
opts.datatype = 1;
opts.show_results = 1;
opts.lidar.type = 1; % for spinning lidar


opt_formulation = ["Lie","BaseLine1","BaseLine2","BaseLine3"]; % Lie group, 3 params, 6 params, 8 params
opts.method = 1; % "Lie","BaseLine1","BaseLine2", "BaseLine3"
opts.iterative = 0;

opts.num_beams = 32;
opts.num_scans = 1;
opts.num_iters = 10; % user defined iterations
opts.threshold = 0;

% opts.save_path = ".\results\scan1\training_data\Lie\";
opts.save_path = "./results/scan1/training_data/Lie/";
addpath(genpath(matlab_utils))
checkDirectory(opts.save_path)
checkDirectory(opts.mat_path, 0)


%% Load datasets
% Single variable called point_cloud

disp("Loading names of data sets...")
if ~opts.load_all
    mat_files = struct('file_name', {opts.mat_path+'velodyne_points-Intrinsic-LargeTag--2019-11-21-22-04.mat';...
                     opts.mat_path+'velodyne_points-Intrinsic-SmallTag--2019-11-21-22-00.mat';...
                     opts.mat_path+'velodyne_points-Intrinsic4-SmallTag--2019-11-22-22-54.mat';...
                     opts.mat_path+'velodyne_points-Intrinsic5-LargeTag--2019-11-22-23-02.mat';...
                     opts.mat_path+'velodyne_points-Intrinsic5-SmallTag--2019-11-22-23-00.mat';...
                     opts.mat_path+'velodyne_points-Intrinsic-further-LargeTag--2019-11-22-23-05.mat';...
                     opts.mat_path+'velodyne_points-Intrinsic-further-SmallTag--2019-11-22-23-09.mat';...
                     opts.mat_path+'velodyne_points-Intrinsic-further2-LargeTag--2019-11-22-23-15.mat';...
                     opts.mat_path+'velodyne_points-Intrinsic-further2-SmallTag--2019-11-22-23-17.mat';...
                     opts.mat_path+'velodyne_points-upper1-SmallTag--2019-12-05-20-13.mat';...
                     opts.mat_path+'velodyne_points-upper2-SmallTag--2019-12-05-20-16.mat';...
                     opts.mat_path+'velodyne_points-upper3-SmallTag--2019-12-05-20-19.mat';...
                     opts.mat_path+'velodyne_points-upper4-SmallTag--2019-12-05-20-22.mat';...
                     opts.mat_path+'velodyne_points-upper5-SmallTag--2019-12-05-20-23.mat';...
                     opts.mat_path+'velodyne_points-upper6-SmallTag--2019-12-05-20-26.mat';...
                     opts.mat_path+'velodyne_points-upper7-SmallTag--2019-12-05-20-29.mat';...
                     opts.mat_path+'velodyne_points-upper8-SmallTag--2019-12-05-20-29.mat'});
    for file = 1:length(mat_files)
        mat_files(file).tag_size = identifyTagSizeFromAName(mat_files(file).file_name);
    end
else
    mat_files = loadFilesFromAFolder(opts.mat_path, '*Tag*.mat');
end

num_targets = length(mat_files);

disp("Loading point cloud from .mat files")
pc = struct('point_cloud', cell(1,num_targets));
for t = 1:num_targets
    pc(t).point_cloud = loadPointCloud(mat_files(t).file_name);
end

disp("Pre-processing payload points...")
data = struct('point_cloud', cell(1,num_targets), 'tag_size', cell(1,num_targets));% XYZIR 

for t = 1:num_targets
    for iter = 1: opts.num_scans
        data(t).payload_points = getAccumulatedPayload(pc(t).point_cloud, iter, 1);
        data(t).tag_size = mat_files(t).tag_size;
    end
end

disp("Done loading data!")

%% Calibrate intrinsic parameters 
% Optimize intrinsic parameters
[calib_param, plane, distance_original, distance, original_points, updated_points] = intrinsicCalibration(opts, data, num_targets);
save(opts.save_path + "realExpDelta.mat", 'calib_param');

disp("Done calibrating")
%% Show results
% graphical results
if opts.show_results
    disp("Now plotting....")
    plotCalibratedResults(num_targets, plane, updated_points, data);
    disp("Done plotting!")
end

% show numerical results
results = showCalibratedNumericalResults(distance, distance_original);

%% check if ring mis-ordered
disp("If the rings are mis-ordered...")
% test = [1, 8 ,16, 18, 20, 21, 33, 35];
% test = [25, 37];
% checkRingOrderWithOriginalPerTarget(original_points, updated_points, 1, opts.num_beams, 'display');
% for i = 1:length(test)
%     obect = test(i);
for i = 1:num_targets
    obect = i;
    checkRingOrderWithOriginalPerTarget(original_points, updated_points, obect, opts.num_beams, 'not display');
end

%% check if ring mis-ordered
% disp("If the rings are mis-ordered...")
% [order, mis_ordered_list] = checkRingOrder(updated_points, calib_param{1}, num_targets, opts.num_beams, opts);
% save(opts.save_path + "results.mat", "results");
disp("All processes has finished")

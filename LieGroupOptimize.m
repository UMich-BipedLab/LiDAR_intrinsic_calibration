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

clear, clc; % DO NOT add 'clear' here

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

% Single variable called point_cloud

% opts.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/slider/intrinsic_latest/data/";
% opts.path = "./data/";
opts.path = "..\intrinsic_lidar_calibration\intrinsic_calibration_mat\";
opts.load_all = 1;
opts.datatype = "Experiment";
opts.show_results = 0;
opt_formulation = ["Lie","BaseLine1","BaseLine3"]; % Lie or Spherical
opts.method = 1; % "Lie","BaseLine1","BaseLine3"
opts.iterative = 0;

opts.num_beams = 32;
opts.num_scans = 1;
opts.num_iters = 10; % user defined iterations

% path = "/home/chenxif/Documents/me590/Calibration/IntrinsicCalibration/extracted_tags/";

disp("Loading names of data sets...")
if ~opts.load_all
    mat_files = struct('file_name', {opts.path+'velodyne_points-Intrinsic-LargeTag--2019-11-21-22-04.mat';...
                     opts.path+'velodyne_points-Intrinsic-SmallTag--2019-11-21-22-00.mat';...
                     opts.path+'velodyne_points-Intrinsic4-SmallTag--2019-11-22-22-54.mat';...
                     opts.path+'velodyne_points-Intrinsic5-LargeTag--2019-11-22-23-02.mat';...
                     opts.path+'velodyne_points-Intrinsic5-SmallTag--2019-11-22-23-00.mat';...
                     opts.path+'velodyne_points-Intrinsic-further-LargeTag--2019-11-22-23-05.mat';...
                     opts.path+'velodyne_points-Intrinsic-further-SmallTag--2019-11-22-23-09.mat';...
                     opts.path+'velodyne_points-Intrinsic-further2-LargeTag--2019-11-22-23-15.mat';...
                     opts.path+'velodyne_points-Intrinsic-further2-SmallTag--2019-11-22-23-17.mat';...
                     opts.path+'velodyne_points-upper1-SmallTag--2019-12-05-20-13.mat';...
                     opts.path+'velodyne_points-upper2-SmallTag--2019-12-05-20-16.mat';...
                     opts.path+'velodyne_points-upper3-SmallTag--2019-12-05-20-19.mat';...
                     opts.path+'velodyne_points-upper4-SmallTag--2019-12-05-20-22.mat';...
                     opts.path+'velodyne_points-upper5-SmallTag--2019-12-05-20-23.mat';...
                     opts.path+'velodyne_points-upper6-SmallTag--2019-12-05-20-26.mat';...
                     opts.path+'velodyne_points-upper7-SmallTag--2019-12-05-20-29.mat';...
                     opts.path+'velodyne_points-upper8-SmallTag--2019-12-05-20-29.mat'});
    for file = 1:length(mat_files)
        mat_files(file).tag_size = identifyTagSizeFromAName(mat_files(file).file_name);
    end
else
    mat_files = loadFilesFromAFolder(opts.path, '*Tag*.mat');
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
        data(t).payload_points = getPayload(pc(t).point_cloud, iter, 1);
        data(t).tag_size = mat_files(t).tag_size;
    end
end

disp("Done loading data!")
%%
opts.iterative = 0;
opts.method = 1; %  ["Lie","BaseLine1","BaeLine3"]
opts.num_iters = 10;
opts.save_path = ".\results\scan1\training_data\Lie\";

disp('Computing planes...')
opt.corners.rpy_init = [45 2 3];
opt.corners.T_init = [2, 0, 0];
opt.corners.H_init = eye(4);
opt.corners.method = "Constraint Customize"; %% will add more later on
opt.corners.UseCentroid = 1;
plane = cell(1,num_targets);

for t = 1:num_targets
    X = [];
    for j = 1: opts.num_beams
        X = [X, data(t).payload_points];
    end
    [plane{t}, ~] = estimateNormal(opt.corners, X(1:3, :), 1.5);
end

data_split_with_ring_cartesian = cell(1,num_targets);

disp("Parsing data...")
for t = 1:num_targets
    data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(data(t).payload_points, opts.num_beams, opts.datatype);
end 
%%
distance_original = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, num_targets); 

delta(opts.num_beams) = struct();

for ring = 1: opts.num_beams
    delta(ring).Affine = GaussNewtonSolver(data_split_with_ring_cartesian, plane, ring);
end
data_split_with_ring_cartesian = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring_cartesian, delta, 1, opt_formulation(opts.method));
distance = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, num_targets);
plotCalibratedResults(num_targets, plane, data_split_with_ring_cartesian, data);

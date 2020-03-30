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
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND
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


%Validate real experiment data
clear, clc
opts.num_beams = 32;
opts.num_scans = 1;
opts.datatype = "Experiment";
opt_formulation = ["Lie","BaseLine1","BaseLine3"]; % Lie or Spherical
opts.method = 3;
opts.show_results = 1;
opts.path = ".esultstargets\Validation\BL3\";
opts.load_all = 1;
opts.threshold = -1;

t_path = ".esultstargets	raining_data\BL3\";
v_path = "..\intrinsic_lidar_calibration\Feb2020alidation_bag\";

load(t_path + 'realExpDelta.mat');
data = parseValidationbag(v_path, "*.bag",5,1);
num_targets = size(data,2);

%%
data_split_with_ring_cartesian = cell(1,num_targets);
data_split_with_ring = cell(1, num_targets);
spherical_data = cell(1,num_targets);
disp("Parsing data...")
for t = 1:num_targets
    spherical_data{t} = Cartesian2Spherical(data(t).payload_points);
    data_split_with_ring{t} = splitPointsBasedOnRing(spherical_data{t}, opts.num_beams, opts.datatype);
    data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(data(t).payload_points, opts.num_beams, opts.datatype);
end

[~, plane1, valid_rings_and_targets] = estimateIntrinsicLie(opts.num_beams, num_targets, opts.num_scans, data_split_with_ring_cartesian,opts.threshold);
distance_original = point2PlaneDistance(data_split_with_ring_cartesian, plane1, opts.num_beams, num_targets); 

if opt_formulation(opts.method) == "Lie"
    data_split_with_ring_cartesian = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring_cartesian, delta, valid_rings_and_targets, opt_formulation(opts.method));    
elseif opt_formulation(opts.method) == "BaseLine1"
    data_split_with_ring = updateDatacFromMechanicalModel(opts.num_beams, num_targets, data_split_with_ring, delta, valid_rings_and_targets);
    data_split_with_ring_cartesian = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring, delta, valid_rings_and_targets, opt_formulation(opts.method));
elseif opt_formulation(opts.method) == "BaseLine3"
    data_split_with_ring_cartesian = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring, delta, valid_rings_and_targets, opt_formulation(opts.method));
end
[~, plane2, valid_rings_and_targets] = estimateIntrinsicLie(opts.num_beams, num_targets, opts.num_scans, data_split_with_ring_cartesian, opts.threshold);
distance = point2PlaneDistance(data_split_with_ring_cartesian, plane2, opts.num_beams, num_targets);

%% Show graphical results
if opts.show_results
    disp("Now plotting....")
    plotCalibratedResults(3, plane2, data_split_with_ring_cartesian, data);
%     plotCalibratedResults(3, plane2, data_split_with_ring_cartesian, data);
%     plotCalibratedResults(num_targets, plane, data_split_with_ring_cartesian, data, opt_formulation(opts.method));
    disp("Done plotting!")
end


disp("Showing numerical results...")
disp("Showing current estimate")
results = struct('ring', {distance(end).ring(:).ring}, ...
                 'num_points', {distance(end).ring(:).num_points}, ...
                 'mean_original', {distance_original.ring(:).mean}, ...
                 'mean_calibrated', {distance(end).ring(:).mean}, ...
                 'mean_percentage', num2cell((abs([distance_original.ring(:).mean]) - abs([distance(end).ring(:).mean])) ./ abs([distance_original.ring(:).mean])), ...
                 'mean_diff', num2cell([distance_original.ring(:).mean] - [distance(end).ring(:).mean]), ...
                 'mean_diff_in_mm', num2cell(([distance_original.ring(:).mean] - [distance(end).ring(:).mean]) * 1e3), ...
                 'std_original', {distance_original.ring(:).std}, ...
                 'std_calibrated', {distance(end).ring(:).std}, ...
                 'std_diff', num2cell([distance_original.ring(:).std] - [distance(end).ring(:).std]), ...
                 'std_diff_in_mm', num2cell(([distance_original.ring(:).std] - [distance(end).ring(:).std])* 1e3));
struct2table(distance(end).ring(:))
disp("Showing comparison")

struct2table(results)
if ~exist(opts.path, 'dir')
    mkdir(opts.path)
end
save(opts.path + 'results.mat', 'results');
save(opts.path + 'data_split_with_ring_cartesian.mat', 'data_split_with_ring_cartesian');

% check if ring mis-ordered
disp("If the rings are mis-ordered...")
[order mis_ordered_list] = checkRingOrder(data_split_with_ring_cartesian, delta, 3, opts.num_beams, opts);

disp("All processes has finished")
mean_array = [results(:).mean_calibrated];
nonzero_index = find(mean_array);
nonzeros_mean = mean_array(nonzero_index);
mean(abs(nonzeros_mean))

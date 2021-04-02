%% parameters of user setting %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% opts.path: directory of saved mat files
% opts.load_all(0/1): 
%               0: only load the files specified in mat_files structure
%               1: load all the files from the opts.path
% opts.datatype: 0: experiment data
%                1: simulation data
% opts.show_results(0/1): plot the calibrated results on figure
clear all, clc

%
% mat_path = 'F:\Calibration\matlab_utils\general';
% addpath("F:\Calibration\convex_relaxation_sim3");
% opts.path = "F:\Calibration\intrinsic_lidar_calibration\intrinsic_calibration_mat\robotics_building\tetrahedron4\";
% v_path = "F:\Calibration\intrinsic_lidar_calibration\intrinsic_calibration_mat\robotics_building\validation\t4\";
% opts.save_path = ".\results\tetrahedron4\Lie\";


mat_path = '/home/brucebot/workspace/matlab_utils/general';
addpath(genpath("/home/brucebot/workspace/test/CVPR17"));
opts.path = "/home/brucebot/workspace/lc-calibration/LiDAR_intrinsic_calibration/tetrehedron_data/tetrahedron4/";
v_path = "/home/brucebot/workspace/lc-calibration/LiDAR_intrinsic_calibration/tetrehedron_data/validation/t4/";
opts.save_path = "./paper_results/tetrahedron4/Lie/";
addpath(mat_path);


%% 
opts.load_all = 1;
opt_datatype = ["Experiment", "Simulation"];
opts.datatype = 1;
opts.show_results = 1;
opts.estimatePlaneMean = 0;
opts.calibrateMean = 0;
opts.fixXY = 0;
opt_noise_type = ["3_param", "6_param", "sim3"];
opts.noise_type = 3;
opts.noise_level = 0;


opt_formulation = ["Lie","BaseLine1","BaseLine2","BaseLine3"]; % Lie group, 3 params, 6 params, 8 params
opts.method = 1; % "Lie","BaseLine1","BaseLine2", "BaseLine3"
opts.iterative = 0;

opts.num_beams = 32;
opts.num_scans = 1;
opts.num_iters = 200; % user defined iterations
opts.threshold = 0;


%% Load datasets
clc; % DO NOT add 'clear' here

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
    mat_files = loadFilesFromAFolder(opts.path, '*.mat');
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
        data(t).scan(iter).payload_points = getPayload(pc(t).point_cloud, iter);
        data(t).tag_size = mat_files(t).tag_size;
    end
end

disp("Done loading data!")


%% post process data, remove outliers with prior knowledge
for iter = 1: opts.num_scans
    %filter out outliers in tetrahedron1
%     data(1).scan(iter).payload_points = data(1).scan(iter).payload_points(:, ~(data(1).scan(iter).payload_points(5,:)== 6 & data(1).scan(iter).payload_points(1,:) > 4.22));
%     for n = 7:8
%        data(1).scan(iter).payload_points = data(1).scan(iter).payload_points(:,~(data(1).scan(iter).payload_points(5,:)== n & data(1).scan(iter).payload_points(1,:)> 4.3));
%     end
%     for n = 12
%        data(1).scan(iter).payload_points = data(1).scan(iter).payload_points(:,~(data(1).scan(iter).payload_points(5,:)== n & data(1).scan(iter).payload_points(1,:)> 4.4));
%     end
%     for n = 9:11
%        data(1).scan(iter).payload_points = data(1).scan(iter).payload_points(:,~(data(1).scan(iter).payload_points(5,:)== n & data(1).scan(iter).payload_points(1,:)> 4.38));
%     end
%     data(2).scan(iter).payload_points = data(2).scan(iter).payload_points(:,~(data(2).scan(iter).payload_points(5,:)== 7 & data(2).scan(iter).payload_points(1,:)> 4.1));
%     data(2).scan(iter).payload_points = data(2).scan(iter).payload_points(:,~(data(2).scan(iter).payload_points(5,:)== 8 & data(2).scan(iter).payload_points(1,:)> 4));
%     for n = 6:7
%        data(4).scan(iter).payload_points = data(4).scan(iter).payload_points(:,~(data(4).scan(iter).payload_points(5,:)== n & data(4).scan(iter).payload_points(2,:)> 1.14));
%     end
%     for n = 8:10
%        data(4).scan(iter).payload_points = data(4).scan(iter).payload_points(:,~(data(4).scan(iter).payload_points(5,:)== n & data(4).scan(iter).payload_points(2,:)> 1.11));
%     end
%     for n = 11:14
%        data(4).scan(iter).payload_points = data(4).scan(iter).payload_points(:,~(data(4).scan(iter).payload_points(5,:)== n & data(4).scan(iter).payload_points(2,:)> 1.067));
%     end
%     for n = 15:17
%        data(4).scan(iter).payload_points = data(4).scan(iter).payload_points(:,~(data(4).scan(iter).payload_points(5,:)== n & data(4).scan(iter).payload_points(2,:)> 1.02));
%     end
%     for n = 18:20
%        data(4).scan(iter).payload_points = data(4).scan(iter).payload_points(:,~(data(4).scan(iter).payload_points(5,:)== n & data(4).scan(iter).payload_points(2,:)> 0.99));
%     end
%     for n = 21:22
%        data(4).scan(iter).payload_points = data(4).scan(iter).payload_points(:,~(data(4).scan(iter).payload_points(5,:)== n & data(4).scan(iter).payload_points(2,:)> 0.96));
%     end
%     for n = 23:25
%        data(4).scan(iter).payload_points = data(4).scan(iter).payload_points(:,~(data(4).scan(iter).payload_points(5,:)== n & data(4).scan(iter).payload_points(2,:)> 0.93));
%     end
%     for n = 26:28
%        data(4).scan(iter).payload_points = data(4).scan(iter).payload_points(:,~(data(4).scan(iter).payload_points(5,:)== n & data(4).scan(iter).payload_points(2,:)> 0.87));
%     end

    %filter out outliers in tetrahedron2
%     for i = 1:4
%          data(i).scan(iter).payload_points = data(i).scan(iter).payload_points(:,~(data(i).scan(iter).payload_points(5,:)== 0));
%     end
    %filter by intensity
%     for i = 1:1
%          data(i).scan(iter).payload_points = data(i).scan(iter).payload_points(:,~(data(i).scan(iter).payload_points(4,:) > 50));
%     end
end

%% Optimize intrinsic parameters
clc
%target 16 and 25 have some problems
% pick_targets = [ 26    28    32    36];
% normal CN  3.0612
% base CN    5.8373
% num_targets = length(pick_targets);
% pick_data = data(pick_targets);
[calib_param, plane_original, plane, distance_original, distance, ...
    original_points, updated_points] = intrinsicCalibration(...
    opts, data, num_targets);

if ~exist(opts.save_path, 'dir')
    mkdir(opts.save_path)
end
final_parameters = computeCalibParameters(calib_param, opts.num_beams, opts.method);
save(opts.save_path + "realExpDelta.mat", 'calib_param');
%% Show graphical results
% if opts.show_results
%     disp("Now plotting....")
    plotCalibratedResults(num_targets, opts.num_scans, ...
        plane_original, original_points);
    plotCalibratedResults(num_targets, opts.num_scans, ...
        plane, updated_points);
%     disp("Done plotting!")
% 
% end

%%
for i = 1:4
    object = i;
    checkRingOrderWithOriginalPerTarget(original_points, updated_points, object, opts.num_beams, opts.num_scans, 'display');
end

%% show numerical results
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
%  results = struct('mean_original', {distance_original.ring(:).mean});
struct2table(distance(end).ring(:))
disp("Showing comparison")
struct2table(results)

% check if ring mis-ordered
disp("If the rings are mis-ordered...")
[order mis_ordered_list] = checkRingOrder(updated_points, ...
    final_parameters, num_targets, opts.num_beams, opts);
save(opts.save_path + "training_results.mat", "results");

%% Validation

validation_mat_files = loadFilesFromAFolder(v_path, '*.mat');
v_num_targets = length(validation_mat_files);
v_pc = struct('point_cloud', cell(1,v_num_targets));
for t = 1:v_num_targets
    v_pc(t).point_cloud = loadPointCloud(validation_mat_files(t).file_name);
end

v_data = struct('point_cloud', cell(1,v_num_targets), ...
                'tag_size', cell(1,v_num_targets));% XYZIR 

for t = 1:v_num_targets
    for iter = 1: opts.num_scans
        v_data(t).scan(iter).payload_points = getPayload(v_pc(t).point_cloud, iter);
        v_data(t).tag_size = validation_mat_files(t).tag_size;
%         v_data(t).scan(iter).payload_points = v_data(t).scan(iter).payload_points(:,~(v_data(t).scan(iter).payload_points(4,:)> 50));
%         v_data(t).scan(iter).payload_points = v_data(t).scan(iter).payload_points(:,~(v_data(t).scan(iter).payload_points(5,:)== 0));
    end  
end


% validation_data = parseValidationbag(v_path, "*.bag", 1, opts.num_scans);
[validation_results] = validation(v_data, final_parameters, opts);
summary1 =  struct(  'mean_original', mean([validation_results.mean_original]), ...
                     'mean_calibrated', mean([validation_results.mean_calibrated]), ...
                     'mean_percentage', (mean([validation_results.mean_original])- mean([validation_results.mean_calibrated]))/mean([validation_results.mean_original]), ...
                     'std_original', mean([validation_results.std_original]), ...
                     'std_calibrated', mean([validation_results.std_calibrated]),...
                     'thickness_original',mean([validation_results.thickness_original]),...
                     'thickness_calibrated',mean([validation_results.thickness_calibrated]));
struct2table(summary1)

std_mean = mean([validation_results.std_original]);
for k = 1:3
threshold = k*std_mean;
index = [validation_results.mean_original] > threshold;
mean_original = [validation_results.mean_original];
mean_calibrated = [validation_results.mean_calibrated];
std_original = [validation_results.std_original];
std_calibrated = [validation_results.std_calibrated];
summary2 =  struct(   'mean_original', mean(mean_original(index)), ...
                     'mean_calibrated', mean(mean_calibrated(index)), ...
                     'mean_percentage', (mean(mean_original(index))- mean(mean_calibrated(index)))/mean(mean_original(index)), ...
                     'std_original', mean(std_original(index)), ...
                     'std_calibrated', mean(std_calibrated(index)));

struct2table(summary2)
end
disp("All processes has finished")

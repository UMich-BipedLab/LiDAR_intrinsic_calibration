%% parameters of user setting %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% opts.path: directory of saved mat files
% opts.load_all(0/1): 
%               0: only load the files specified in mat_files structure
%               1: load all the files from the opts.path
% opts.datatype: 0: experiment data
%               1: simulation data
% opts.show_results(0/1): plot the calibrated results on figure
clear, clc


mat_path = 'F:\Calibration\matlab_utils\general';
opts.path = "F:\Calibration\intrinsic_lidar_calibration\intrinsic_calibration_mat\robotics_building\tetrahedron4\";
opts.load_all = 1;
opt_datatype = ["Experiment", "Simulation"];
opts.datatype = 1;
opts.show_results = 0;
opts.estimatePlaneMean = 0;
opts.calibrateMean = 0;
opts.fixXY = 0;
opt_noise_type = ["3_param", "6_param", "sim3"];
opts.noise_type = 3;


opt_formulation = ["Lie","BaseLine1","BaseLine2","BaseLine3"]; % Lie group, 3 params, 6 params, 8 params
opts.iterative = 0;

opts.num_beams = 32;
opts.num_scans = 1;
opts.num_iters = 2; % user defined iterations
opts.threshold = 0;
opts.save_path = ".\results\tetrahedron4\Lie\";
addpath(mat_path);
%% Load datasets
clc; % DO NOT add 'clear' here

% Single variable called point_cloud
% path = "/home/chenxif/Documents/me590/Calibration/IntrinsicCalibration/extracted_tags/";

disp("Loading names of data sets...")
mat_files = loadFilesFromAFolder(opts.path, '*.mat');
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


v_path = "F:\Calibration\intrinsic_lidar_calibration\intrinsic_calibration_mat\robotics_building\validation\t4\";
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
    end  
end
disp("Done loading data!")

%% Optimize intrinsic parameters

table = zeros(20,1);
level = [0,1,2,3,4,5,6,7];
for i = 8:length(level) % different noise level(0-7), our method encounter problems when noise level = 2,3,5,6
opts.noise_level = level(i);
for j = 1:3 % 3 calibration method : "Lie","BaseLine1","BaseLine2",
opts.method = j; % "Lie","BaseLine1","BaseLine2", "BaseLine3"
[calib_param, plane_original, plane, distance_original, distance, ...
    original_points, updated_points] = intrinsicCalibration(...
    opts, data, num_targets);

if ~exist(opts.save_path, 'dir')
    mkdir(opts.save_path)
end
final_parameters = computeCalibParameters(calib_param, opts.num_beams, opts.method);
%% Show graphical results
if opts.show_results
    disp("Now plotting....")
    plotCalibratedResults(num_targets, opts.num_scans, ...
        plane_original, plane, original_points, updated_points);
    disp("Done plotting!")

end

%% Validation
% validation_data = parseValidationbag(v_path, "*.bag", 1, opts.num_scans);
[validation_results] = validation(v_data, final_parameters, opts);

table(1,i) = mean([validation_results.mean_original]);
table(17,i) = mean([validation_results.std_original]);
table(21,i) = mean([validation_results.thickness_original]);
table(25,i) = mean([validation_results.sum_original]);
if j == 2 
    table(2,i) = mean([validation_results.mean_calibrated]);
    table(18,i) = mean([validation_results.std_calibrated]);
    table(22,i) = mean([validation_results.thickness_calibrated]);
    table(26,i) = mean([validation_results.sum_calibrated]);
elseif j ==3
    table(3,i) = mean([validation_results.mean_calibrated]);
    table(19,i) = mean([validation_results.std_calibrated]);
    table(23,i) = mean([validation_results.thickness_calibrated]);
    table(27,i) = mean([validation_results.sum_calibrated]);
elseif j ==1
    table(4,i) = mean([validation_results.mean_calibrated]);
    table(20,i) = mean([validation_results.std_calibrated]);
    table(24,i) = mean([validation_results.thickness_calibrated]);
    table(28,i) = mean([validation_results.sum_calibrated]);
end
std_original = [validation_results.std_original];
std_calibrated = [validation_results.std_calibrated];
std_mean = mean(std_original);

for k = 1:3
    threshold = k*std_mean;
    index = [validation_results.mean_original] > threshold;
    mean_original = [validation_results.mean_original];
    mean_calibrated = [validation_results.mean_calibrated];
    summary2 =  struct(   'mean_original', mean(mean_original(index)), ...
                         'mean_calibrated', mean(mean_calibrated(index)), ...
                         'mean_percentage', (mean(mean_original(index))- mean(mean_calibrated(index)))/mean(mean_original(index)), ...
                         'std_original', mean(std_original(index)), ...
                         'std_calibrated', mean(std_calibrated(index)));
    table(4*k+1, i) =  mean(mean_original(index));               
    if j == 2 
    table(4*k+2,i) = mean(mean_calibrated(index));
    elseif j ==3
    table(4*k+3,i) = mean(mean_calibrated(index));
    elseif j ==1
    table(4*k+4,i) = mean(mean_calibrated(index));
    end
end
end
end

table
disp("All processes has finished")
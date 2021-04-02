clc, clear
addpath(genpath("F:\Calibration\matlab_utils"))
addpath(genpath("F:\Calibration\lidar_simulator"))
opts.mat_path ="F:\Calibration\intrinsic_lidar_calibration\intrinsic_calibration_mat\robotics_building\tetrahedron1\";
opts.save_path = ".\results\";
opts.num_beams = 32;
opts.num_scans = 1;
opts.reload = 1;
opt_datatype = ["Experiment", "Simulation"];
opts.datatype = 1;
opts.test_ring = 15;
opts.process_output = 0;
opts.numerical_analysis = 0;

if opts.reload
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
    mat_files = loadFilesFromAFolder(opts.mat_path, '*.mat');
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
    save("all_mats_data-" + num2str(opts.num_scans) + "scans.mat", 'data')
else
    load("all_mats_data-" + num2str(opts.num_scans) + "scans.mat")
    num_targets = length(data);
end
disp("Done loading data!")

%% esimate planes
opt.corners.rpy_init = [45 2 3];
opt.corners.T_init = [2, 0, 0];
opt.corners.H_init = eye(4);
opt.corners.method = "Constraint Customize"; %% will add more later on
opt.corners.UseCentroid = 1;
plane = cell(1,num_targets); 
for t = 1:num_targets            
    [plane{t}, ~] = estimateNormal(opt.corners, data(t).scan(1).payload_points(1:3,:), 0.7);
end
[data_split_with_ring_cartesian, data_split_with_ring] = splitDataBasedOnRing(data, opts, num_targets);
plotCalibratedResults(num_targets, 1, plane, data_split_with_ring_cartesian, data)

%% pick 4 targets with best CN
LiDAR_opts.color_list = getColors(4);
LiDAR_opts.pose.centriod = [0 0 0];
LiDAR_opts.properties.beam = 32;
LiDAR_ring_points(opts.test_ring).points = [rand(2, 10); zeros(1, 10)];

num_obj = 4; 
combos = nchoosek(1:num_targets,num_obj);
total_counts = size(combos,1);
scene_t(total_counts) = struct('object_list', [], 'ring_t', [], 'plane_intersect_targets', [], ...
                  'target_intersection', [], 'plane_intersect', [], 'bases', [], ...
                  'suitable_targets', [], 'results', [], 'condition_matrix', [], 'ojb1', [], 'ojb2', [], 'ojb3', [], 'ojb4', []);

 best_normal_comb= [1,2,3,4];
 best_base_combo= [1,2,3,4];
 best_normal_CN = 100;
 best_base_CN = 100;
for count = 1: total_counts
%     status_name =  "status-" + num2str(count) + ".mat";
    combo = combos(count,:);
    object_list(num_obj) = struct("object_vertices_mat_h", [], "object_vertices_mat", [], ...
                                  "object_vertices", [], "target_size", [], ...
                                  "rpy", [], "xyz", [], "H", [], "H_inv", [], ...
                                  "normal", [], "centroid", [], ...
                                  'connected_lines', []);
    for i = 1:num_obj
        object_list(i).target_size = data(combo(i)).tag_size;
        object_list(i).object_vertices_mat_h = plane{1,combo(i)}.corners;
        object_list(i).object_vertices_mat = plane{1,combo(i)}.corners(1:3,:);
        object_list(i).object_vertices = convertXYZmatrixToXYZstruct(object_list(i).object_vertices_mat_h);
        object_list(i).normal =  plane{1,combo(i)}.normals;
        object_list(i).centroid =  plane{1,combo(i)}.centroid(1:3);
    end
 
    
     [scene_t(count).object_list, scene_t(count).ring_t, ...
     scene_t(count).plane_intersect_targets, scene_t(count).target_intersection, ...
     scene_t(count).plane_intersect, scene_t(count).bases, ...
     scene_t(count).suitable_targets, scene_t(count).results] = validateIntrinsicTargetPlacement(opts, object_list, LiDAR_ring_points, LiDAR_opts);
%      tmp = scene_t(count);
%      save(opts.save_path +status_name, "tmp");
    if(best_normal_CN > scene_t(count).results.normal.CNs_mean)
        best_normal_combo = combo;
%         best_normal_CN = [scene_t(count).results.normal.CNs_mean;scene_t(count).results.base.CNs_mean];
        best_normal_CN = [scene_t(count).results.normal;scene_t(count).results.base];
    end
    if(best_base_CN > scene_t(count).results.base.CNs_mean)
        best_base_combo = combo;
%         best_base_CN = [scene_t(count).results.normal.CNs_mean;scene_t(count).results.base.CNs_mean];
        best_base_CN = [scene_t(count).results.normal;scene_t(count).results.base];
    end
    
end

 best_normal_combo
 best_base_combo
 best_normal_CN 
 best_base_CN 
 
 
 display("done")




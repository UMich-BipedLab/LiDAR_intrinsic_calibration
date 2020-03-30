%This function is the portal for intrinsic Calibration
%Input: optimization parameters set by users, the processed
%points and the number of targets used in calibration scene.
%Output: calibration parameter,estimated plane, point to plane distance, and the updated points 

function [calib_param, plane, distance_original, p2p_dist, original_points, updated_points] = intrinsicCalibration(opts, data, num_targets)
    
    if opts.lidar.type == 1
        [data_split_with_ring_cartesian, data_split_with_ring] = splitDataBasedOnRing(data, opts, num_targets);
    else opts.lidar.type == 2
        error("Not supported yet, will be completed by 4/1/2020")
%         [data_split_with_ring_cartesian, data_split_with_ring] = splitDataBasedOnSegment(data, opts, num_targets);
    end
    original_points = data_split_with_ring_cartesian;
    
    if ~opts.iterative
       opts.num_iters = 1;
    end
    [calib_param, updated_points, plane, distance_original, p2p_dist] = intrinsicIterations(opts, num_targets, data_split_with_ring_cartesian, data_split_with_ring, data);  
    disp('Done optimization')
end
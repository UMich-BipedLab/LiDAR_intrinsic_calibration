function [calib_param, data_split_with_ring_cartesian, plane_original, plane, distance_original, distance] = intrinsicIterations(opts, num_targets, data_split_with_ring_cartesian, data_split_with_ring, object_list)
    calib_param = cell(1,opts.num_iters);
    distance = []; % if re-run, it will show error of "Subscripted assignment between dissimilar structures"
    distance(opts.num_iters).ring(opts.num_beams) = struct();
    distance(opts.num_iters).mean = 0;
    plane = estimatePlane(opts.datatype, data_split_with_ring_cartesian, data_split_with_ring, num_targets, opts.num_beams, opts.num_scans, opts.estimatePlaneMean, object_list);
    plane_original = plane;
    for k = 1: opts.num_iters
        fprintf("--- Working on %i/%i\n", k, opts.num_iters)
        [delta] = estimateIntrinsic(opts, num_targets, data_split_with_ring_cartesian, data_split_with_ring, plane);
        if k == 1
            distance_original = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, opts.num_scans, num_targets); 
        end
        if opts.method == 1
            data_split_with_ring_cartesian = updateCartesianData(opts.num_beams, opts.num_scans, num_targets, data_split_with_ring_cartesian, delta);
        elseif (opts.method == 2 || opts.method == 3 || opts.method == 4)
            data_split_with_ring_cartesian = updateDataBLModel(opts, num_targets, data_split_with_ring, data_split_with_ring_cartesian, delta);   
        end
        data_split_with_ring = DataFromCartesian2Spherical(opts.num_beams, opts.num_scans, num_targets, data_split_with_ring_cartesian);
        
        calib_param{k} = delta;
        plane = estimatePlane(opts.datatype, data_split_with_ring_cartesian, data_split_with_ring, num_targets, opts.num_beams, opts.num_scans, opts.estimatePlaneMean, object_list);
        distance(k) = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, opts.num_scans, num_targets); 
    end
end


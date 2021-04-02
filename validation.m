function [results] = validation(data, delta, opts)

    num_targets = length(data);
    [data_split_with_ring_cartesian, data_split_with_ring] = splitDataBasedOnRing(data, opts, num_targets);

    plane1 = estimatePlane(opts.datatype, data_split_with_ring_cartesian, data_split_with_ring, num_targets, opts.num_beams, opts.num_scans, opts.estimatePlaneMean);
    distance_original = point2PlaneDistance(data_split_with_ring_cartesian, plane1, opts.num_beams, opts.num_scans, num_targets); 

    if opts.method == 1
        updated_data_split_with_ring_cartesian = updateCartesianData(opts.num_beams, opts.num_scans, num_targets, data_split_with_ring_cartesian, delta);
    elseif (opts.method == 2 || opts.method == 3 || opts.method == 4)
        updated_data_split_with_ring_cartesian = updateDataBLModel(opts, num_targets, data_split_with_ring, data_split_with_ring_cartesian, delta);   
    end
    updated_data_split_with_ring = DataFromCartesian2Spherical(opts.num_beams, opts.num_scans, num_targets, updated_data_split_with_ring_cartesian);
    plane2 = estimatePlane(opts.datatype, updated_data_split_with_ring_cartesian, updated_data_split_with_ring, num_targets, opts.num_beams, opts.num_scans, opts.estimatePlaneMean);
    distance = point2PlaneDistance(updated_data_split_with_ring_cartesian, plane2, opts.num_beams, opts.num_scans, num_targets); 

    %% Show graphical results
    if opts.show_results
        disp("Now plotting....")
        plotCalibratedResults(num_targets, 1, plane1, data_split_with_ring_cartesian);
        plotCalibratedResults(num_targets, 1, plane2, updated_data_split_with_ring_cartesian);
        disp("Done plotting!")
    end


    disp("Showing numerical results...")
    disp("Showing current estimate")
    results = struct('ring', {distance(end).ring(:).ring}, ...
                     'num_points', {distance(end).ring(:).num_points}, ...
                     'mean_original', {distance_original.ring(:).mean}, ...
                     'mean_calibrated', {distance(end).ring(:).mean}, ...
                     'mean_percentage', num2cell((abs([distance_original.ring(:).mean]) - abs([distance(end).ring(:).mean])) ./ abs([distance_original.ring(:).mean])), ...
                     'sum_original', {distance_original.ring(:).sum}, ...
                     'sum_calibrated', {distance(end).ring(:).sum}, ...
                     'sum_percentage', num2cell((abs([distance_original.ring(:).sum]) - abs([distance(end).ring(:).sum])) ./ abs([distance_original.ring(:).sum])), ...
                     'std_original', {distance_original.ring(:).std}, ...
                     'std_calibrated', {distance(end).ring(:).std}, ...
                     'thickness_original', mean(plane1{1}.thickness), ...
                     'thickness_calibrated', mean(plane2{1}.thickness));
%     struct2table(distance(end).ring(:))
    disp("Showing comparison")

    struct2table(results)
    if ~exist(opts.save_path, 'dir')
        mkdir(opts.save_path)
    end
    save(opts.save_path + 'validation_results.mat', 'results');
    save(opts.save_path + 'data_split_with_ring_cartesian.mat', 'data_split_with_ring_cartesian');

    % check if ring mis-ordered
    disp("If the rings are mis-ordered...")
    [order mis_ordered_list] = checkRingOrder(data_split_with_ring_cartesian, delta, 3, opts.num_beams, opts);

    disp("All processes has finished")

end
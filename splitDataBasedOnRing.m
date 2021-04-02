function [data_split_with_ring_cartesian, data_split_with_ring] = splitDataBasedOnRing(data, opts, num_targets)
    spherical_data = struct();
    data_split_with_ring = struct();
    data_split_with_ring_cartesian = struct();

    disp("Parsing data...")
    for t = 1:num_targets
        for s = 1:opts.num_scans
            if isfield(data(t).scan,'payload_points')
                points = data(t).scan(s).payload_points;
            elseif isfield(data,'points_mat')
                points = data(t).scans(s).points_mat;
            end            
            if opts.noise_level == 0
                data_split_with_ring_cartesian(t).scan(s).ring = splitPointsBasedOnRing(points, opts.num_beams, opts.datatype);
            else
                spherical_data(t).scan(s).ring = Cartesian2Spherical(points);
                data_split_with_ring(t).scan(s).ring = splitPointsBasedOnRing(spherical_data(t).scan(s).ring, opts.num_beams, opts.datatype);
                data_split_with_ring_cartesian(t).scan(s).ring = imposeNoise(data_split_with_ring(t).scan(s).ring, opts.noise_type, opts.num_beams, opts.noise_level);
            end
        end
    end
    data_split_with_ring = DataFromCartesian2Spherical(opts.num_beams, opts.num_scans, num_targets, data_split_with_ring_cartesian);
end
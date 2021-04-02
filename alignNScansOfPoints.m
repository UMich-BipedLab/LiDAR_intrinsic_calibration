function data = alignNScansOfPoints(sorted_spherical_data, num_scans, num_beams, min_azimuth, min_num_points)
    data(num_beams) = struct();
    
    for ring = 1: num_beams
        accumulated_points = [];
        for s =1:num_scans
            raw_points = sorted_spherical_data(s).ring(ring).points;
            if isempty(raw_points)
                break;
            end
            % left align the points
            while(raw_points(3,1) < min_azimuth(ring)) && size(raw_points,2) > min_num_points(ring)
                raw_points = raw_points(:,2:end);               
            end
            %truncate the right part if too long
            if size(raw_points,2) > min_num_points(ring)
                raw_points = raw_points(:,1:min_num_points(ring));
            end
            accumulated_points(s,:,:) = raw_points;
        end
        data(ring).points = mean(accumulated_points,1);
        data(ring).points = reshape(data(ring).points, size(data(ring).points,2,3));
    end   
    
end
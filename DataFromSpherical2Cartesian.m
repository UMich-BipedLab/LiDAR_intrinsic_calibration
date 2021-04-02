function cartesian = DataFromSpherical2Cartesian(num_beams, num_scans, num_targets, spherical)
    cartesian = struct();    
    for i =1:num_targets
        for ring = 1: num_beams
            for scan = 1:num_scans
                if(~isempty(spherical(i).scan(scan).ring(ring).points))
                    cartesian(i).scan(scan).ring(ring).points = Spherical2Cartesian(spherical(i).scan(scan).ring(ring).points);
                    cartesian(i).scan(scan).ring(ring).points_with_I = [cartesian(i).scan(scan).ring(ring).points(1:3,:);spherical(i).scan(scan).ring(ring).points_with_I(4:5,:)];
                else
                    cartesian(i).scan(scan).ring(ring).points = [];
                end
            end
        end
    end
end
function spherical = DataFromCartesian2Spherical(num_beams, num_scans, num_targets, cartesian)
    spherical = struct();    
    for i =1:num_targets
        for ring = 1: num_beams
            for scan = 1:num_scans
                if(~isempty(cartesian(i).scan(scan).ring(ring).points))
                    spherical(i).scan(scan).ring(ring).points = Cartesian2Spherical(cartesian(i).scan(scan).ring(ring).points);
                else
                    spherical(i).scan(scan).ring(ring).points = [];
                end
            end
        end
    end
end
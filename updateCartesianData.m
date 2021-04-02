function data = updateCartesianData(num_beams, num_scans, num_targets, data_split_with_ring, delta)
    data = struct('scan', cell(1,num_targets));
    for i =1:num_targets
        for scan  =1:num_scans
            for ring = 1: num_beams
                if (isempty(data_split_with_ring(i).scan(scan).ring(ring).points))
                      data(i).scan(scan).ring(ring).points= [];
                      continue;
                else
                    data(i).scan(scan).ring(ring).points =  delta(ring).Affine * data_split_with_ring(i).scan(scan).ring(ring).points;  
                end  
            end
        end
    end
end
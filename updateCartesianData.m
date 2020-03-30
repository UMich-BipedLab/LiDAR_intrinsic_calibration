function data = updateCartesianData(num_beams, num_targets, data_split_with_ring, delta)
    data = cell(size(data_split_with_ring));
    for i =1:num_targets
        for ring = 1: num_beams
            if (isempty(data_split_with_ring{i}(ring).points))
                  data{i}(ring).points= [];
                  continue;
            else
                data{i}(ring).points =  delta(ring).Affine * data_split_with_ring{i}(ring).points;  
            end  
        end
    end
end
function data = updateDataBLModel(num_beams, num_targets, data_split_with_ring, data_split_with_ring_cartesian,delta, valid_rings_targets, method)
    %Note: if a ring should be skipped, there won't be any corresponding 
    %delta parameter, thus we should keep it as it was
    data = data_split_with_ring_cartesian;
    for i =1:num_targets
        for ring = 1: num_beams
            if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
                warning("Ring %i has been skipped in update function", ring)
                  continue;
            else
                if (method == 2)
                    data{i}(ring).points = updateBL1(data_split_with_ring{i}(ring).points, delta(ring));
                elseif (method == 3)
                     data{i}(ring).points = updateBL2(data_split_with_ring{i}(ring).points, delta(ring));                    
                elseif (method == 4)
                     data{i}(ring).points = updateBL3(data_split_with_ring{i}(ring).points, delta(ring));
                end            
            end
        end
    end
end
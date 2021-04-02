function data = updateDatacFromBL2(num_beams, num_targets, data_split_with_ring, delta, valid_rings_targets)
%     data = cell(1,17);
    data = data_split_with_ring;
    for i =1:num_targets
        for ring = 1: num_beams
            if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
                  continue;
            else
%                 data = zeros(size(data_split_with_ring{i}(ring).points));
                data{i}(ring).points(1,:) = data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D;
                data{i}(ring).points(2,:) = data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta;
                data{i}(ring).points(3,:) = data_split_with_ring{i}(ring).points(3,:)+delta(ring).phi;
            end
        end
    end
end
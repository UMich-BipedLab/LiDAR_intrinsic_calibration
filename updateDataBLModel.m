function data = updateDataBLModel(opts, num_targets, data_split_with_ring, data_split_with_ring_cartesian, delta)
    %Note: if a ring should be skipped, there won't be any corresponding 
    %delta parameter, thus we should keep it as it was
    data = data_split_with_ring_cartesian;
    for i =1:num_targets
        for s = 1:opts.num_scans
            for ring = 1: opts.num_beams
%                 if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i, s))
% %                     warning("Ring %i has been skipped in update function", ring)
%                       continue;
%                 else
                    if (opts.method == 2)
                        data(i).scan(s).ring(ring).points = updateBL1(data_split_with_ring(i).scan(s).ring(ring).points, delta(ring));
                    elseif (opts.method == 3)
                        data(i).scan(s).ring(ring).points = updateBL2(data_split_with_ring(i).scan(s).ring(ring).points, delta(ring));                    
                    elseif (opts.method == 4)
                        data(i).scan(s).ring(ring).points = updateBL3(data_split_with_ring(i).scan(s).ring(ring).points, delta(ring));
                    end            
%                 end
            end
        end
    end
end
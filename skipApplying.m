function skip_applying = skipApplying(valid_rings_targets, ring_number, data_split_with_ring, target_number)
    skip_applying = false;
    if(valid_rings_targets(ring_number).skip || isempty(data_split_with_ring{target_number}(ring_number).points))
        skip_applying = true;
    end
end
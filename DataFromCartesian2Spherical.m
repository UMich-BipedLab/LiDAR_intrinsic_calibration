function spherical = DataFromCartesian2Spherical(num_beams, num_targets, cartesian)
    spherical = cell(size(cartesian));    
    for i =1:num_targets
        for ring = 1: num_beams
            if(~isempty(cartesian{i}(ring).points))
                Cartesian2Spherical(cartesian{i}(ring).points);
            end
        end
    end
end
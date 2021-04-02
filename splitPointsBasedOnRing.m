function data = splitPointsBasedOnRing(points, num_beams, datatype)
    data(num_beams) = struct();
    total = 0;
    if isempty(points)
        warning("No point is on this target")
        data(num_beams).points = [];
        data(num_beams).points_with_I = [];
        return
    end
    % Note: Here we assume the lidar ring in experiment is 0 indexing, so we shift it to
    % fit the matlab routine. However, I fyou use the simulator, the lidar
    % ring will be 1 indexing. Be very careful with this.
    if(datatype == 1)%Experiment
        for n = 0:num_beams-1
            if ~any(points(5,:)==n)
                continue
            end
            ring_point = points(:, points(5,:)==n);
            m = n+1;
            data(m).points = [ring_point(1:3,:); ones(1,size(ring_point,2))];
            data(m).points_with_I = ring_point;
            data(m).points_with_I(5,:) = data(m).points_with_I(5,:) + 1;
            total= total+size(ring_point,2);
        end
    elseif datatype == 2%"Simulation"
        for n = 1:num_beams
            if ~any(points(5,:)==n)
                continue
            end
            ring_point = points(:, points(5,:)==n);
            data(n).points = [ring_point(1:3,:); ones(1,size(ring_point,2))];
            data(n).points_with_I = ring_point;
            total= total+size(ring_point,2);
        end     
    end
    if total ~= size(points,2)
        warning("inconsistent number of poins in splitPointsBasedOnRing.m");
        warning("split total: %i", total)
        warning("original total %i", size(points,2))
    end
end
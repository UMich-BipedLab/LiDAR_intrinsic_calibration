function cost = optimizeMultiIntrinsicCostLie (X, valid_targets, plane, ring, theta_x, theta_y, theta_z, T, S)
    cost = 0;
    for i = 1: length(X)
        if valid_targets.valid(1, i)
            cost = cost + optimizeIntrinsicCostLie(X{i}(ring), plane{i}, theta_x, theta_y, theta_z, T, S);
        end
    end
end

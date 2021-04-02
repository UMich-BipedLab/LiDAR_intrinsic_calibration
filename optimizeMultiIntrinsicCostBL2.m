function cost = optimizeMultiIntrinsicCostBL2 (X, valid_targets, plane, ring, D_s, D, A_c, V_c, H_oc, V_oc)
    cost = 0;
    num_targets = length(X); 
    for i = 1: num_targets
        if valid_targets.valid(1, i)
            cost = cost + optimizeIntrinsicCostBL2(X(i).ring(ring), plane{i}, D_s, D, A_c, V_c, H_oc, V_oc);
        end
    end
end
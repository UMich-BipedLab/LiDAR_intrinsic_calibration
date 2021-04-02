function cost = optimizeMultiIntrinsicCostBL3 (X, valid_targets, plane, ring, D_s, D, A_c, S_vc, C_vc, H_oc, S_voc, C_voc)
    cost = 0;
    num_targets = length(X); 
    for i = 1: num_targets
        if valid_targets.valid(1, i)
            cost = cost + optimizeIntrinsicCostBL3(X(i).ring(ring), plane{i}, D_s, D, A_c, S_vc, C_vc, H_oc, S_voc,C_voc);
        end
    end
end
function [delta] = estimateIntrinsic(opts, num_targets, data_split_with_ring_cartesian, data_split_with_ring, plane)    
    processed_data_split_with_ring_cartesian = struct();
    processed_data_split_with_ring = struct();
    if ~opts.calibrateMean
        for t = 1:num_targets
            processed_data_split_with_ring_cartesian(t).ring = accumulateNScans(data_split_with_ring_cartesian(t).scan(:),opts.num_beams, opts.num_scans);
            processed_data_split_with_ring(t).ring = accumulateNScans(data_split_with_ring(t).scan(:),opts.num_beams, opts.num_scans);
        end
    else
        for t = 1:num_targets
            [processed_data_split_with_ring(t).ring, processed_data_split_with_ring_cartesian(t).ring] = findMeanOfNScans(data_split_with_ring(t).scan(:),opts.num_beams, opts.num_scans);
        end
    end
        
    if (opts.method == 1)
        delta(opts.num_beams).H = struct();
        delta(opts.num_beams).Affine = struct();
        opt.delta.rpy_init = [0 0 0];
        opt.delta.T_init = [0, 0, 0];
        opt.delta.scale_init = 1;
        opt.delta.H_init = eye(4);
        opt.delta.fixXY = opts.fixXY;
%         [delta, ~, ~] = estimateDeltaLie(opt.delta, processed_data_split_with_ring_cartesian, plane, delta(opts.num_beams), opts.num_beams, num_targets, opts.threshold);
        delta = estimateDeltaLieWithCVXSolver(processed_data_split_with_ring_cartesian, plane, delta, opts.num_beams, num_targets);
    elseif (opts.method == 2)
        delta(opts.num_beams).D = struct();
        delta(opts.num_beams).theta = struct();
        delta(opts.num_beams).phi = struct();
        opt.delta.D_corr_init = 0;
        opt.delta.theta_corr_init = 0;
        opt.delta.phi_corr_init = 0;
        [delta, ~, ~] = estimateDeltaBL1(opt.delta, processed_data_split_with_ring, plane, delta(opts.num_beams), opts.num_beams, num_targets, opts.threshold);
    elseif (opts.method == 3)
        delta(opts.num_beams).D_s = struct();
        delta(opts.num_beams).D = struct();
        delta(opts.num_beams).A_c = struct();%azimuth correction
        delta(opts.num_beams).V_c = struct();%elevation correction
        delta(opts.num_beams).H_oc = struct();
        delta(opts.num_beams).V_oc = struct();
        opt.delta.D_s_init = 1;
        opt.delta.D_init = 0;
        opt.delta.A_c_init = 0;
        opt.delta.V_c_init = 0;%TODO: find a more reasonable initial guess
        opt.delta.H_oc_init = 0;
        opt.delta.V_oc_init = 0;
        [delta, ~, ~] = estimateDeltaBL2(opt.delta, processed_data_split_with_ring, plane, delta(opts.num_beams), opts.num_beams, num_targets, opts.threshold);
    elseif (opts.method == 4)
        delta(opts.num_beams).D_s = struct();
        delta(opts.num_beams).D = struct();
        delta(opts.num_beams).A_c = struct();
        delta(opts.num_beams).S_vc = struct();
        delta(opts.num_beams).C_vc = struct();
        delta(opts.num_beams).H_oc = struct();
        delta(opts.num_beams).S_voc = struct();
        delta(opts.num_beams).C_voc = struct();
        opt.delta.D_s_init = 1;
        opt.delta.D_init = 0;
        opt.delta.A_c_init = 0;
        opt.delta.S_vc_init = 0;%TODO: find a more reasonable initial guess
        opt.delta.C_vc_init = 0;%TODO: find a more reasonable initial guess
        opt.delta.H_oc_init = 0;
        opt.delta.S_voc_init = 0;
        opt.delta.C_voc_init = 0;
        [delta, ~, valid_targets] = estimateDeltaBL3(opt.delta, processed_data_split_with_ring, plane, delta(opts.num_beams), opts.num_beams, num_targets, opts.threshold);
    end

end
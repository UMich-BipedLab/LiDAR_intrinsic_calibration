function [delta, opt, valid_targets]= estimateDeltaBL2(opt, data, plane, delta, num_beams, num_targets, threshold)
    tic;
    valid_targets(num_beams).skip = [];
    valid_targets(num_beams).valid = [];
    valid_targets(num_beams).num_points = [];
    for ring = 1:num_beams
        valid_targets(ring) = checkRingsCrossDataset(data, plane, threshold, num_targets, ring);
        if ~valid_targets(ring).skip
            D_s = optimvar('D_s', 1, 1,'LowerBound',0,'UpperBound',2);
            D = optimvar('D', 1, 1,'LowerBound',-0.5,'UpperBound',0.5);
            A_c = optimvar('A_c', 1, 1,'LowerBound',deg2rad(-20),'UpperBound',deg2rad(20));
            V_c = optimvar('V_c', 1, 1,'LowerBound',deg2rad(-180),'UpperBound',deg2rad(180));
            H_oc = optimvar('H_oc', 1, 1,'LowerBound',-0.5,'UpperBound',0.5);
            V_oc = optimvar('V_oc', 1, 1,'LowerBound',-0.5,'UpperBound',0.5);


            prob = optimproblem;
            f = fcn2optimexpr(@optimizeMultiIntrinsicCostBL2, data,  valid_targets(ring), plane, ring,...
                               D_s, D, A_c, V_c, H_oc, V_oc);
            prob.Objective = f;
            x0.D_s = opt.D_s_init;
            x0.D = opt.D_init;
            x0.A_c = opt.A_c_init;
            x0.V_c = opt.V_c_init;
            x0.H_oc = opt.H_oc_init;
            x0.V_oc = opt.V_oc_init;


            % 'Display','iter'
            options = optimoptions('fmincon', 'MaxIter',5e2, 'Display','off', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxFunctionEvaluations', 3e4);
            max_trail = 5;
            num_tried = 1;
            status = 0;
            while status <=0 
                [sol, fval, status, ~] = solve(prob, x0, 'Options', options);
                if status <=0 
                    warning("optimization failed")
                end
                num_tried = num_tried + 1;
                if (num_tried + 1 > max_trail)
                    warning("tried too many time, optimization still failed, current status:")
                    disp(status)
                    break;
                end
            end
    %         R_final = rotx(sol.theta_x) * roty(sol.theta_y) * rotz(sol.theta_z);
    %         delta(ring).H = eye(4);
    %         delta(ring).H(1:3, 1:3) = R_final;
    %         delta(ring).H(1:3, 4) = sol.T;
            delta(ring).D_s = sol.D_s;
            delta(ring).D = sol.D;
            delta(ring).A_c = sol.A_c;
            delta(ring).V_c = sol.V_c;
            delta(ring).H_oc = sol.H_oc;
            delta(ring).V_oc = sol.V_oc;
            delta(ring).opt_total_cost = fval;
            opt.computation_time(ring).time = toc;
        else
            delta(ring).D_s = 1;
            delta(ring).D = 0;
            delta(ring).A_c = 0;
            delta(ring).V_c = 0;
            delta(ring).H_oc = 0;
            delta(ring).V_oc = 0;
            delta(ring).opt_total_cost = 0;
        end
    end
end
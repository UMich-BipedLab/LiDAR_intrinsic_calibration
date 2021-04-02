function delta = estimateDeltaLieWithCVXSolver(data, plane, delta, num_beams, num_targets)
    opts.solver = 2;  % 1 bisection, 2 dense-search
    if opts.solver == 1
        opts.max_iter = 30; % for 30 for bisection; 200 for dense search
    else
        opts.max_iter = 200; % for 30 for bisection; 200 for dense search
        opts.s_interval = 0.1;
    end
    opts.s_lower = 0.1; % 0.5
    opts.s_upper = 2.5; % 1.5
    opts.cost_threshold = 1e-4;
    opts.s_grad_threshold = 1e-6;
    opts.quite = 1;
    opts.show_statistic = 1; % 0: no outputs, 1: rough outputs 2: detailed
    opts.draw_results = 0;   
    opts.plotting.plane_boundary = 1.5;
    color_list = getColors(num_beams);
    fig_t(num_beams-1) = struct('fig_h', [""], 'fig_txt', [""]);
    for ring = 1:num_beams
        [out_t,  data_t, res_t] = solveByConvexSolvers(...
            opts, data, plane, ring, num_targets);
        
        delta(ring).res_t = res_t;
        delta(ring).out_t = out_t;
        delta(ring).data_t = data_t;
        delta(ring).H = out_t.H.T;
        Scaling = [out_t.s    0        0        0
                   0        out_t.s    0        0
                   0          0      out_t.s    0
                   0          0        0        1];
        delta(ring).Scaling = Scaling;
        delta(ring).Affine = delta(ring).H * Scaling;
        delta(ring).opt_total_cost = out_t.sum_cost;
        clc
        disp("='='='='='='='='='='='='='='='=")
        fprintf("ring: %i\n", ring)
        fprintf("original cost: %.4f\n", out_t.original_cost)
        fprintf("cost: %.4f\n", out_t.sum_cost)
        fprintf("scaling: %.4f\n", out_t.s)
        fprintf("d_star: %.4f\n", out_t.dstar)
        fprintf("f_star: %.4f\n", out_t.f)
        disp("='='='='='='='='='='='='='='='=")
%         txt = "Ring " + num2str(ring);
%         color = color_list{ring};
%         if ring ~= 1
%             fig_t(ring-1) = plotConvexity([], [], color, ...
%                 txt, data_t, res_t);
%             delta(ring-1).fig_t = fig_t(ring-1);
%         end
        if ring == 1 % 7
            plotConvexity([], [], [1 0 0], ...
                "", data_t, res_t);
        end

%         if ring == 2
%             data_t = plotConvexSolverResults(opts, [], [], [], [], [], ...
%                       data_t, out_t, []);
%         end
    end
end
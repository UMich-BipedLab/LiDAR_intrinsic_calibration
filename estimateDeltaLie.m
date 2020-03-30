%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function [delta, opt, valid_targets] = estimateDeltaLie(opt, data, plane, delta, num_beams, num_targets, threshold)
    tic;
    valid_targets(num_beams).skip = [];
    valid_targets(num_beams).valid = [];
    valid_targets(num_beams).num_points = [];
    for ring = 1:num_beams
        valid_targets(ring) = checkRingsCrossDataset(data, plane, threshold, num_targets, ring);
        if ~valid_targets(ring).skip

    %         ring
    %         dbstop in estimateDelta.m at 13 if ring>=32
            theta_x = optimvar('theta_x', 1, 1,'LowerBound',-0.0,'UpperBound',0.0); % 1x1
            theta_y = optimvar('theta_y', 1, 1,'LowerBound',-0.0,'UpperBound',0.0); % 1x1
            theta_z = optimvar('theta_z', 1, 1,'LowerBound',-0.5,'UpperBound',0.5); % 1x1
            T = optimvar('T', 1, 3,'LowerBound', -0.05,'UpperBound',0.05); % 1x3
            S = optimvar('S', 1, 1,'LowerBound', 0.9,'UpperBound',1.1);

%             theta_x = 0;
%             theta_y = 0;
%             theta_z = 0;
%             S = 1;
%             T = [0 0 0];
%             cost = optimizeMultiIntrinsicCostLie(data, valid_targets(ring), plane, ring, theta_x, theta_y, theta_z, T, S);

            prob = optimproblem;
            f = fcn2optimexpr(@optimizeMultiIntrinsicCostLie, data, valid_targets(ring), ...
                                        plane, ring, theta_x, theta_y, theta_z, T, S);
            prob.Objective = f;
            x0.theta_x = opt.rpy_init(1);
            x0.theta_y = opt.rpy_init(2);
            x0.theta_z = opt.rpy_init(3);
            x0.S = opt.scale_init;
            x0.T = opt.T_init;

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
            R_final = rotx(sol.theta_x) * roty(sol.theta_y) * rotz(sol.theta_z);
            delta(ring).H = eye(4);
            delta(ring).H(1:3, 1:3) = R_final;
            delta(ring).H(1:3, 4) = sol.T;
            Scaling = [sol.S    0        0        0
                       0        sol.S    0        0
                       0        0        sol.S    0
                       0        0        0        1];
            delta(ring).Scaling = Scaling;
            delta(ring).Affine = Scaling * delta(ring).H;

            delta(ring).opt_total_cost = fval;
            opt.computation_time = toc;
        else
            delta(ring).H = eye(4);
            delta(ring).Scaling = eye(4);
            delta(ring).Affine = eye(4);
            delta(ring).opt_total_cost = 0;
        end
    end
end

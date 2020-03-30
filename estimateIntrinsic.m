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

function [delta, valid_targets] = estimateIntrinsic(opts, num_targets, data_split_with_ring_cartesian,data_split_with_ring, plane)    
    if (opts.method == 1)
        delta(opts.num_beams).H = struct();
        delta(opts.num_beams).Affine = struct();
        opt.delta.rpy_init = [0 0 0];
        opt.delta.T_init = [0, 0, 0];
        opt.delta.scale_init = 1;
        opt.delta.H_init = eye(4);
        [delta, ~, valid_targets] = estimateDeltaLie(opt.delta, data_split_with_ring_cartesian, plane, delta(opts.num_beams), opts.num_beams, num_targets, opts.threshold);
    elseif (opts.method == 2)
        delta(opts.num_beams).D = struct();
        delta(opts.num_beams).theta = struct();
        delta(opts.num_beams).phi = struct();
        opt.delta.D_corr_init = 0;
        opt.delta.theta_corr_init = 0;
        opt.delta.phi_corr_init = 0;
        [delta, ~, valid_targets] = estimateDeltaBL1(opt.delta, data_split_with_ring, plane, delta(opts.num_beams), opts.num_beams, num_targets, opts.threshold);
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
        [delta, ~, valid_targets] = estimateDeltaBL2(opt.delta, data_split_with_ring, plane, delta(opts.num_beams), opts.num_beams, num_targets, opts.threshold);
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
        [delta, ~, valid_targets] = estimateDeltaBL3(opt.delta, data_split_with_ring, plane, delta(opts.num_beams), opts.num_beams, num_targets, opts.threshold);
    end

end

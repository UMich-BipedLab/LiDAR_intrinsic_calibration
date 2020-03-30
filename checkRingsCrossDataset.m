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

function valid_targets = checkRingsCrossDataset(data, plane, threshold, num_targets, ring, minimun_target_for_this_ring, minimun_point_on_a_target)
    % If not 'enough' data for a ring, don't try to calibrate it
    %
    % For enough data, by default we mean :
    % (1) on a target, a ring has to have a least 10 points to calibrate
    % (2) on the whole scene (containing many many targets), the ring has
    % to appear on targets at least 5 times.
    
    if ~exist('minimun_point_on_a_target', 'var') 
        minimun_point_on_a_target = 10;
    end
    
    if ~exist('minimun_target_for_this_ring', 'var') 
        minimun_target_for_this_ring = 3;
    end
    
    valid_targets.skip = false;
    valid_targets.valid = zeros(1, num_targets);
    valid_targets.num_points = zeros(1, num_targets);
    
    for target = 1:num_targets
        % minimun 10 points
        num_point_on_this_target = size(data{target}(ring).points, 2);
        valid_targets.num_points(1, target) = num_point_on_this_target;
        if  num_point_on_this_target >= minimun_point_on_a_target
            valid_targets.valid(1, target) = 1;
        end
    end
    
%     if valid_target_num < min(3, num_targets) % less than this, skip the ring
%     if sum(valid_targets.valid) < minimun_target_for_this_ring
%         valid_targets.skip = true;
%         warning("Ring #%i has been skipped due to not enough valid targets %i!", ring, sum(valid_targets.valid));
%     end

    target_pose.x = 0;
    target_pose.y = 0;
    target_pose.z = 0;
    for j = 1: num_targets
        if valid_targets.valid(1, j)
            x=  abs(plane{j}.unit_normals(1));
            y=  abs(plane{j}.unit_normals(2));
            z=  abs(plane{j}.unit_normals(3));
            if (x > threshold)
                target_pose.x = 1;
            end
            if (y > threshold)
                target_pose.y = 1;
            end
            if (z > threshold)
                target_pose.z = 1;
            end
        end
    end
    
    if ~(target_pose.x && target_pose.y && target_pose.z)
        valid_targets.skip = true;
    end
end

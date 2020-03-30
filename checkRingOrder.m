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

function [ring_ordered, ring_misorder_list] = checkRingOrder(data_split_with_ring, delta, num_targets, num_ring, opts)
    ring_order(num_ring) = struct();
    for ring =1:num_ring
        ring_points = [];
        %TODO: we should check the ring order for every sing target, not
        %the mean od all of targets.
        for i =1:1  
%             if (checkRingsCrossDataset(data_split_with_ring, num_targets, ring))
%                 continue;
%             end
            if (isempty(data_split_with_ring{i}(ring).points))
                continue;
            else
                ring_points = [ring_points, data_split_with_ring{i}(ring).points];
%                 if opts.show_results
%                     scatter3(0,0,0,50, 'b.')
%                     origin = [0 0 0 0 0 1;
%                               0 1 0 0 0 0;
%                               0 0 0 1 0 0];
%                     plot3(origin(1,:), origin(2,:), origin(3,:))
%                     scatter3(data_split_with_ring{i}(ring).points(1,:),...
%                              data_split_with_ring{i}(ring).points(2,:),...
%                              data_split_with_ring{i}(ring).points(3,:), 50, 'k.');
%                     text(mean(data_split_with_ring{i}(ring).points(1,:)), ...
%                          mean(data_split_with_ring{i}(ring).points(2,:)), ...
%                          mean(data_split_with_ring{i}(ring).points(3,:)), num2str(ring), ...
%                          'Color','black','FontSize',8);
%                     transformedPoints = zeros(size(data_split_with_ring{i}(ring).points));
%                     transformedPoints(1,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*sin(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta).*cos(data_split_with_ring{i}(ring).points(3,:)+ delta(ring).phi);
%                     transformedPoints(2,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*sin(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta).*sin(data_split_with_ring{i}(ring).points(3,:)+ delta(ring).phi);
%                     transformedPoints(3,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*cos(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta);
%                     text(mean(transformedPoints(1,:)), ...
%                          mean(transformedPoints(2,:)), ...
%                          mean(transformedPoints(3,:)), num2str(ring), ...
%                          'Color','red','FontSize',8);
%                     scatter3(transformedPoints(1,:),transformedPoints(2,:),transformedPoints(3,:), 50, 'r.');
%                 end
            end
        end
        
        ring_order(ring).ring = ring;
        if ~isempty(ring_points)
            ring_order(ring).ring_mean = mean(ring_points(1:3, :), 2)';
            ring_order(ring).z_axis = ring_order(ring).ring_mean(3);
        else
            ring_order(ring).ring_mean = [0 0 0];
            ring_order(ring).z_axis = 0;
        end
    end
    
    z_axis = [ring_order(:).z_axis];
    ind_non_zeros = find(z_axis);
    ring_ordered = issorted(z_axis(ind_non_zeros));
    if ring_ordered
        disp("Rings are ordered!")
        ring_misorder_list = ring_order;
    else
        [x,idx]=sort([ring_order(:).z_axis]);
        ring_misorder_list=ring_order(idx);
        warning("Rings are mis-ordered.....")
        [ring_order(:).ring; ring_misorder_list.ring]'
    end
end

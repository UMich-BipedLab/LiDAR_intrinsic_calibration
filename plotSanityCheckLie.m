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
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND
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


%% plot
% clc, close all;
% num_targets=1;
function plotSanityCheckLie(num_targets, plane, data, data_split_with_ring)
    mesh_size = [-1, 1];
    for i =1:num_targets
        w = null(plane{i}.unit_normals'); % Find two orthonormal vectors which are orthogonal to v
        [P,Q] = meshgrid(mesh_size(1):mesh_size(2)); % Provide a gridwork (you choose the size)
        X = plane{i}.centriod(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
        Y = plane{i}.centriod(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
        Z = plane{i}.centriod(3)+w(3,1)*P+w(3,2)*Q;
        figure(1);
        surf(X,Y,Z);
        hold on;
%         axis equal
        scatter3(data(i).payload_points(1,:), data(i).payload_points(2,:), data(i).payload_points(3,:), 30, 'b.');
        for ring =1:32
%             if(checkRingsCrossDataset(data_split_with_ring, num_targets, ring))
%                 continue
%             end
            if (isempty(data_split_with_ring{i}(ring).points))
                continue;
            else
                scatter3(data_split_with_ring{i}(ring).points(1,:),...
                         data_split_with_ring{i}(ring).points(2,:),...
                         data_split_with_ring{i}(ring).points(3,:), 50, 'r.');
            end
        end   
    end
end

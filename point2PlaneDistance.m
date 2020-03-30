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


function distance = point2PlaneDistance(data, plane, num_beams, num_targets)
    distance.ring(num_beams) = struct();
    for n = 1:num_beams
        X = [];
        points_array = [];
        sum_points = 0;
        for t = 1:num_targets
            point = data{t}(n).points;
            num_points = size(point, 2); 
            if num_points > 0
                plane_centroids = repmat(plane{t}.centroid, [1, num_points]);
                points_array = [points_array point];
                diff = [point - plane_centroids];
                normals = repmat(plane{t}.unit_normals, [1, num_points]);
                X = [X (dot(normals, diff(1:3,:)))];
            end
            sum_points = sum_points + num_points;
        end
        distance.ring(n).ring = n;
        distance.ring(n).num_points = sum_points;
        if sum_points > 0
            distance.ring(n).mean = mean(X,2);
            ring_centroid = mean(points_array, 2);
            distance.ring(n).ring_centroid = ring_centroid(1:3)';
            distance.ring(n).z_axis = ring_centroid(3);
            distance.ring(n).std = std(X);
        else
            distance.ring(n).ring_centroid = [0, 0, 0];
            distance.ring(n).z_axis = 0;
            distance.ring(n).mean = 0;
            distance.ring(n).std = 0;
        end
        distance.ring(n).mean_in_mm = distance.ring(n).mean * 1e3;
        distance.ring(n).std_in_mm = distance.ring(n).std * 1e3;
    end
    distance.mean = mean(abs([distance.ring(:).mean]));
%     distance.std = std([distance.ring(:).mean]);
end

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


clear; clc;

% Single variable called point_cloud
mat_file_path = '../repo/LiDARTag_data/velodyne_points-lab8-closer-big--2019-09-06-15-28.mat';
pc = loadPointCloud(mat_file_path);
scans = 1;
data = getPayload(pc, 1 , 1); % XYZIR 
[values, points_per_scan] = size(data);

%% Step 1: Calculate the d, theta, phi from the x, y, z of point  cloud
% d     - length of beam
% theta - azimuth angle off of x axis
% phi   - elevation angle
xs = data(1,:);
ys = data(2,:);
zs = data(3,:);
ring = data(5,:);

d     = sqrt(xs.^2 + ys.^2 + zs.^2);
phi   = atan2(zs , sqrt(xs.^2 + ys.^2));
theta = atan2(ys , xs);

%% Step 2: Calculate 'ground truth' points by projecting the angle onto the
% normal plane
%
% Assumption: we have the normal plane at this step in the form:
% plane_normal = [nx ny nz]

% example normal lies directly on the x axis
opt.rpy_init = [45 2 3];
opt.T_init = [2, 0, 0];
opt.H_init = eye(4);
opt.method = "Constraint Customize"; %% will add more later on
opt.UseCentroid = 1;

[plane_normal, magnitude_plane_normal] = estimateNormal(opt, data(1:3, :), 0.8051);

angles = zeros(points_per_scan, 3);
angles(:,1) = xs ./ d;
angles(:,2) = ys ./ d;
angles(:,3) = zs ./ d;

normals = zeros(points_per_scan, 3);
normals(:,1) = plane_normal(1)*ones(scans, points_per_scan);
normals(:,2) = plane_normal(2)*ones(scans, points_per_scan);
normals(:,3) = plane_normal(3)*ones(scans, points_per_scan);

numerator = magnitude_plane_normal.^2;
denominator = dot(normals', angles', 1);
d_hat = numerator ./ denominator;

% d_hat now holds the projection of a point P onto the plane normal

%% Step 3: Find the delta_d to minimize the projection error
% This delta_d represents the range offset in the LiDAR

num_beams = 32; % config of lidar
delta_D = zeros(num_beams, 1);

for n = 1:num_beams
    sorted_d = [];
    sorted_d_hat = [];
    if ~any(ring==n)
        continue
    end
    delta_D(n) = optimizeDistance(d(ring==n), d_hat(ring == n));
%     for i=1:scans
%         for j = 1:points_per_scan
%             if(ring(j)== n)
%                 sorted_d = [sorted_d, d(i,j)];
%                 sorted_d_hat = [sorted_d_hat, d_hat(i,j)];
%             end
%         end
%     end    
%     delta_D(n)= c(sorted_d, sorted_d_hat);
end
disp('done')
disp(delta_D)
std(delta_D(delta_D~=0))
plot(delta_D)

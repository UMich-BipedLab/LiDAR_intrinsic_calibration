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

function cost = optimizeIntrinsicCostBL1(X, plane, D_corr, theta_corr, phi_corr)
%     H = eye(4);
%     H(1:3,1:3) = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
%     H(1:3,4) = T';
%     X_prime = H * X.points;
    X_prime = zeros(size(X.points));
    X_prime(1,:) = (X.points(1,:)+ D_corr).*sin(X.points(2,:)+theta_corr).*cos(X.points(3,:)+ phi_corr);
    X_prime(2,:) = (X.points(1,:)+ D_corr).*sin(X.points(2,:)+theta_corr).*sin(X.points(3,:)+ phi_corr);
    X_prime(3,:) = (X.points(1,:)+ D_corr).*cos(X.points(2,:)+theta_corr);
    
    plane_centroids = repmat(plane.centroid, [1,size(X_prime, 2)]);
%     diff = abs([X_prime - plane_centroids]);
    diff = X_prime - plane_centroids;
    normals = repmat(plane.unit_normals, [1,size(X_prime, 2)]);
%     cost = norm([plane.normals .* diff(1:3,:)],'fro');
    cost = sum(abs(dot(normals, diff(1:3,:))));
end

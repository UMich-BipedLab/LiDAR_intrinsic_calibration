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


clc; clear; close all

% generate some points on a plane 1
[x,z] = meshgrid(-1:0.1:1);
p1 = [x(:), zeros(size(x(:))), z(:)];
% normal vector
n1 = [0; 1; 0; 0];

figure; 
plot3(p1(:,1), p1(:,2), p1(:,3), '.', 'markersize', 12)
hold on
quiver3(0, 0, 0, n1(1), n1(2), n1(3), 'linewidth', 3)

% generate some points on a plane 2
[y,z] = meshgrid(1:0.1:3, -1:0.1:1);
p2 = [-2*ones(size(y(:))), y(:), z(:)];
% normal vector
n2 = [1; 0; 0; 0];

plot3(p2(:,1), p2(:,2), p2(:,3), '.', 'markersize', 12)
hold on
quiver3(-2, 2, 0, n2(1), n2(2), n2(3), 'linewidth', 3)
view(145,30)
axis equal, grid on

% create perturbed points using a Sim(3) tf
T = expm( hat( randn(7,1)*0.1 ) );

disp('Ground truth tf: ')
disp(T)

q1 = zeros(size(p1,1), 4);
q2 = zeros(size(p2,1), 4);
for i = 1:size(p1,1)
    q1(i,:) = ( expm(hat(randn(7,1)*0.02)) * T * [p1(i,:)'; 1] )';
    q2(i,:) = ( expm(hat(randn(7,1)*0.02)) * T * [p2(i,:)'; 1] )';
end

% plot perturbed points
plot3(q1(:,1), q1(:,2), q1(:,3), '.k', 'markersize', 12)
plot3(q2(:,1), q2(:,2), q2(:,3), '.k', 'markersize', 12)


% solve for the new transformation
% Gauss-Newton solver over Sim(3)
MAX_ITER = 100;
eps_Jr = 1e-6;
ITER = 0;
H = eye(4); % initial guess
while ITER < MAX_ITER
    ITER = ITER + 1;
    % solve normal equations
    [A1,b1] = compute_jacobian(H, q1, n1);
    [A2,b2] = compute_jacobian(H, q1, n2);
    A = A1 + A2;
    b = b1 + b2;
    
    if rank(A) < 7
        disp('---------------------------------------------------')
        fprintf('Hessian matrix A = J^T * J 


')
        disp(A)
        disp('A is low-rank. The problem has no unique solution.')
        disp('---------------------------------------------------')
        break
    end
    
    dx = A \ b;
    % retract and update the estimate
    H = expm( hat(dx) ) * H;
    
    disp(['GN Iter: '     num2str(ITER)])
    
    % check if converged
    if norm(b) < eps_Jr
        break;
    end
end

% disp('GN estimate: ')
% disp(H)
% 
% disp('Error: ')
% disp(norm(logm(T \ H)))



% A and b for GN
function [A,b] = compute_jacobian(X,q,n)
    A = zeros(7);
    b = zeros(7,1);
    % residual
    z = q * X';
    r =  z * n;
    for i = 1:size(r,1)
        % Jacobian
        J = n(1:3)' * [-skew(z(i,1:3)'), eye(3), z(i,1:3)'];
        % Left hand side matrix A x = b
        A = A + J' * J;
        % Right hand side vector A x = b
        b = b - J' * r(i);
    end
end

function X = skew(x)
% vector to skew R^3 -> so(3)
X = [   0,  -x(3),  x(2);
    x(3),      0,  -x(1);
    -x(2), x(1),   0];
end

function X = hat(x)
% hat: R^7 -> sim(3)
X = [x(7) * eye(3) + skew(x(1:3)), x(4:6); 0 0 0 0];
end

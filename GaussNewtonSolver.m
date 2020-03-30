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

function H = GaussNewtonSolver(data, plane,ring)
    % input: 
    %      1. points of a ring
    %      2. all targets that this ring lies on
    % objective: compute a H for points of a ring on all targets 
    MAX_ITER = 100;
    eps_Jr = 1e-6;
    ITER = 0;
    H = eye(4); % initial guess
    while ITER < MAX_ITER
        ITER = ITER + 1;
        % solve normal equations
%         [A1,b1] = compute_jacobian(H, q1, n1);
%         [A2,b2] = compute_jacobian(H, q1, n2);
%         A = A1 + A2;
%         b = b1 + b2;
        A = 0;
        b = 0;
        for m = 1: length(data) % number of targets this ring lies on
            q = data{m}(ring).points';  % points on the ring associated with the target
            if ~isempty(q)
                [A_m, b_m] = compute_jacobian(H, q, [plane{m}.unit_normals;1]);
                A = A + A_m; % correction for all of the target in Lie Algebra 
                b = b + b_m; % correction for all of the target in Lie Algebra 
            end
        end  

        if rank(A) < 7
            disp('---------------------------------------------------')
            fprintf('Hessian matrix A = J^T * J \n\n\n')
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

end

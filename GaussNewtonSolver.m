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
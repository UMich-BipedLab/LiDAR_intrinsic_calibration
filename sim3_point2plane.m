clc; clear; close all

% generate some points on a plane
[x,z] = meshgrid(-1:0.1:1);
p = [x(:), zeros(size(x(:))), z(:)];

% normal vector
n = [0; 1; 0];

figure; 
plot3(p(:,1), p(:,2), p(:,3), '.', 'markersize', 12)
hold on
quiver3(0, 0, 0, n(1), n(2), n(3), 'linewidth', 3)
view(-145,30)
axis equal, grid on

% create perturbed points using a Sim(3) tf
T = expm( hat( randn(7,1)*0.1 ) );

disp('Ground truth tf: ')
disp(T)

q = zeros(size(p,1), 4);
for i = 1:size(p,1)
    q(i,:) = ( expm(hat(randn(7,1)*0.02)) * T * [p(i,:)'; 1] )';
end

% plot perturbed points
plot3(q(:,1), q(:,2), q(:,3), '.k', 'markersize', 12)


% solve for the new transformation
% Gauss-Newton solver over Sim(3)
MAX_ITER = 100;
eps_Jr = 1e-6;
ITER = 0;
H = eye(4); % initial guess
while ITER < MAX_ITER
    ITER = ITER + 1;
    % solve normal equations
    [A,b] = compute_jacobian(H,q);
    
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

% disp('GN estimate: ')
% disp(H)
% 
% disp('Error: ')
% disp(norm(logm(T \ H)))



% A and b for GN
function [A,b] = compute_jacobian(X,q)
    A = zeros(7);
    b = zeros(7,1);
    % residual
    n = [0; 1; 0; 0];
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
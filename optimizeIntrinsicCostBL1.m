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
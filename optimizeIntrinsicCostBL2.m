function cost = optimizeIntrinsicCostBL2(X, plane,D_s, D, A_c, V_c, H_oc, V_oc)
    %Note: the elevation angle and azimuth angle is defined differently as
    %the original paper. The expression is reformulated according to their
    %defination of elevation and azimuth 
    X_prime = zeros(size(X.points));
    X_prime(1,:) = (X.points(1,:)*D_s + D).*sin(V_c) .* cos(X.points(3,:)- A_c)- H_oc *sin(X.points(3,:)- A_c);
    X_prime(2,:) = (X.points(1,:)*D_s + D).*sin(V_c) .* sin(X.points(3,:)- A_c)+ H_oc *cos(X.points(3,:)- A_c);
    X_prime(3,:) = (X.points(1,:)*D_s + D).*cos(V_c) + V_oc;
    
    plane_centroids = repmat(plane.centroid, [1,size(X_prime, 2)]);
    diff = X_prime - plane_centroids;
    normals = repmat(plane.unit_normals, [1,size(X_prime, 2)]);
    cost = sum(abs(dot(normals, diff(1:3,:))));
end
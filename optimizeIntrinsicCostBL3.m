function cost = optimizeIntrinsicCostBL3(X, plane,D_s, D, A_c, S_vc, C_vc, H_oc, S_voc, C_voc)
    %Note: the elevation angle and azimuth angle is defined differently as
    %the original paper. The expression is reformulated according to their
    %defination of elevation and azimuth 
    X_prime = zeros(size(X.points));
    dxy = (X.points(1,:)*D_s + D)*S_vc -C_voc;
    X_prime(1,:) = dxy .* cos(X.points(3,:)- A_c)- H_oc *sin(X.points(3,:)- A_c);
    X_prime(2,:) = dxy .* sin(X.points(3,:)- A_c)+ H_oc *cos(X.points(3,:)- A_c);
    X_prime(3,:) = (X.points(1,:)+ D)*C_vc + S_voc;
    
    plane_centroids = repmat(plane.centroid, [1,size(X_prime, 2)]);
%     diff = abs([X_prime - plane_centroids]);
    diff = X_prime - plane_centroids;
    normals = repmat(plane.unit_normals, [1,size(X_prime, 2)]);
%     cost = norm([plane.normals .* diff(1:3,:)],'fro');
    dist = abs(dot(normals, diff(1:3,:)));
    cost = sum(sqrt(dist.^2));
end
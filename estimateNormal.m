function [plane, distance]= estimateNormal(opt, X, target_len)
    opt = optimizeCost(opt, X, target_len);
    if (opt.opt_total_cost/size(X,2) > 0.02)
        warning("Estimated plane is not accurate");
    end
    target_lidar = [0 -target_len/2 -target_len/2 1;
                    0 -target_len/2  target_len/2 1;
                    0  target_len/2  target_len/2 1;
                    0  target_len/2 -target_len/2 1]';
    corners = inv(opt.H_opt) * target_lidar;
    corners = sortrows(corners', 3, 'descend')';
    plane.corners = corners;
    plane.centroid = mean(corners, 2);
    normals = cross(corners(1:3,1)-corners(1:3,2), corners(1:3,1)-corners(1:3,3));
    normals = normals/(norm(normals));
    plane.unit_normals = normals;
    distance = dot(normals, corners(1:3,1));
    plane.normals = distance * normals;
    plane.corners = corners;
    %compute thickness
    transformed_X = opt.H_opt * [X;ones(1,size(X,2))];
    max_x = max(transformed_X(1,:));
    min_x = min(transformed_X(1,:));
    thickness = max_x - min_x;
    plane.thickness = thickness;
    if normals(1) < 0
        plane.normals = -normals;
    end
end
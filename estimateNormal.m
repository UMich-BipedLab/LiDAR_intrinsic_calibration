function [plane, distance]= estimateNormal(opt, X, target_len)
    opt = optimizeCost(opt, X, target_len);
    target_lidar = [0 -target_len/2 -target_len/2 1;
                    0 -target_len/2  target_len/2 1;
                    0  target_len/2  target_len/2 1;
                    0  target_len/2 -target_len/2 1]';
    corners = inv(opt.H_opt) * target_lidar;
    corners = sortrows(corners', 3, 'descend')';
    plane.centroid = mean(corners, 2);
    normals = cross(corners(1:3,1)-corners(1:3,2), corners(1:3,1)-corners(1:3,3));
    normals = normals/(norm(normals));
    plane.unit_normals = normals;
    distance = dot(normals, corners(1:3,1));
    plane.normals = distance * normals;
    
    if normals(1) < 0
        plane.normals = -normals;
    end
end
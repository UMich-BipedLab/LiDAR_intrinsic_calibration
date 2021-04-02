%% plot
% clc, close all;
% num_targets=1;
function plotSanityCheckLie(num_targets, plane, data, data_split_with_ring)
    mesh_size = [-1, 1];
    for i =1:num_targets
        w = null(plane{i}.unit_normals'); % Find two orthonormal vectors which are orthogonal to v
        [P,Q] = meshgrid(mesh_size(1):mesh_size(2)); % Provide a gridwork (you choose the size)
        X = plane{i}.centriod(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
        Y = plane{i}.centriod(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
        Z = plane{i}.centriod(3)+w(3,1)*P+w(3,2)*Q;
        figure(1);
        surf(X,Y,Z);
        hold on;
%         axis equal
        scatter3(data(i).payload_points(1,:), data(i).payload_points(2,:), data(i).payload_points(3,:), 30, 'b.');
        for ring =1:32
%             if(checkRingsCrossDataset(data_split_with_ring, num_targets, ring))
%                 continue
%             end
            if (isempty(data_split_with_ring{i}(ring).points))
                continue;
            else
                scatter3(data_split_with_ring{i}(ring).points(1,:),...
                         data_split_with_ring{i}(ring).points(2,:),...
                         data_split_with_ring{i}(ring).points(3,:), 50, 'r.');
            end
        end   
    end
end
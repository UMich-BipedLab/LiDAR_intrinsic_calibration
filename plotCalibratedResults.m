% function plotCalibratedResults(num_targets, num_scans, plane_original, plane, data_split_with_ring, data)
 function plotCalibratedResults(num_targets, num_scans, plane, data_split_with_ring)   
%     if isfield(data(1).scan, 'payload_points')
%         % do nothing 
%     elseif ~isfield(data(1).scan, 'payload_points') && isfield(data(1).scan, 'points_mat')
%         [data.('payload_points')] = data.('points_mat');
%     else
%         error("input data has not field of payload_points nor points_mat, please check again")
%     end

    image_handle = figure('Name', "Calibrated Results", 'Visible', 'off');
    image_handle = axes('parent', image_handle);
    set(gca,'visible','off')
    set(gca,'xtick',[])
    quiver3(image_handle, zeros(3,1),zeros(3,1),zeros(3,1),[1;0;0],[0;1;0],[0;0;1], '-b', 'fill', 'LineWidth', 5)
    hold(image_handle, 'on');
    axis(image_handle, 'equal');
    scatter3(image_handle, 0, 0, 0, 100, 'r','fill')
    mesh_size = [-1, 1];
    [P,Q] = meshgrid(mesh_size(1):mesh_size(2)); % Provide a gridwork (you choose the size)
    
    for i =1:num_targets
%         w = null(plane_original{i}.unit_normals'); % Find two orthonormal vectors which are orthogonal to v
%         X = plane_original{i}.centroid(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
%         Y = plane_original{i}.centroid(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
%         Z = plane_original{i}.centroid(3)+w(3,1)*P+w(3,2)*Q;
%         surf(image_handle, X, Y, Z);
%         colormap default
%         
%         alpha 0.5
        
%         w = null(plane{i}.unit_normals'); % Find two orthonormal vectors which are orthogonal to v
%         X = plane{i}.centroid(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
%         Y = plane{i}.centroid(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
%         Z = plane{i}.centroid(3)+w(3,1)*P+w(3,2)*Q;
%         surf(image_handle, X, Y, Z);
% %         colormap summer
%         alpha 0.5
        plotConnectedVerticesStructure(image_handle,  ...
            convertXYZmatrixToXYZstruct(plane{i}.corners(:, [1 2 4 3])))
        for s = 1:1
%             if ~isempty(data(i).scan(s).payload_points)
%                 scatter3(image_handle, data(i).scan(s).payload_points(1,:), data(i).scan(s).payload_points(2,:), data(i).scan(s).payload_points(3,:), 30, 'b.');
%             end
            for ring =1:32
                if (isempty(data_split_with_ring(i).scan(s).ring(ring).points))
                    continue;
                else
%                     scatter3(image_handle, data(i).scan(s).ring(ring).points(1,:),...
%                              data(i).scan(s).ring(ring).points(2,:),...
%                              data(i).scan(s).ring(ring).points(3,:),...
%                              50, 'b.');
                    scatter3(image_handle, data_split_with_ring(i).scan(s).ring(ring).points(1,:),...
                             data_split_with_ring(i).scan(s).ring(ring).points(2,:),...
                             data_split_with_ring(i).scan(s).ring(ring).points(3,:),...
                             50, 'r.');
%                         text(mean(data_split_with_ring(i).scan(s).ring(ring).points(1,:)), ...
%                              mean(data_split_with_ring(i).scan(s).ring(ring).points(2,:)), ...
%                              mean(data_split_with_ring(i).scan(s).ring(ring).points(3,:)), ...
%                              num2str(ring), 'Color','g','FontSize',10);
                end
            end
        end
    end
    set(get(image_handle,'parent'),'visible','on');% show the current axes
    xlabel(image_handle, 'x')
    ylabel(image_handle, 'y')
    zlabel(image_handle, 'z')
end
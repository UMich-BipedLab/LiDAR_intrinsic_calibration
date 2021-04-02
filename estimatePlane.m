% Calculate 'ground truth' points by projecting the angle onto the
% normal plane
%
% Assumption: we have the normal plane at this step in the form:
% plane_normal = [nx ny nz]

% example normal lies directly on the x axis
function plane = estimatePlane(datatype, cartesian_data, spherical_data, num_targets, num_beams, num_scans, usingMean, object_list)

        opt.corners.rpy_init = [45 2 3];
        opt.corners.T_init = [2, 0, 0];
        opt.corners.H_init = eye(4);
        opt.corners.method = "Constraint Customize"; %% will add more later on
        opt.corners.UseCentroid = 1;

        plane = cell(1,num_targets);
        
        for t = 1:num_targets
            if datatype == 1
                if ~usingMean
                    points_split_with_ring = accumulateNScans(cartesian_data(t).scan(:),num_beams, num_scans);
                else
                    [~,points_split_with_ring] = findMeanOfNScans(spherical_data(t).scan(:),num_beams, num_scans);
                end
                X= [];
                for i = 1: num_beams
                    if ~isempty(points_split_with_ring(i).points)
                        X = [X, points_split_with_ring(i).points];
                    end
                end
                [plane{t}, ~] = estimateNormal(opt.corners, X(1:3, :), 0.7);
            elseif datatype == 2 && exist('object_list','var')
                plane{t}.centroid =  [object_list(t).centroid; 1];
                plane{t}.normals =  object_list(t).normal;
                plane{t}.unit_normals = object_list(t).normal/(norm(object_list(t).normal));
            else
                warning("Wrong input for plane estimation");
            end
        end
end
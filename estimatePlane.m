% Calculate 'ground truth' points by projecting the angle onto the
% normal plane
%
% Assumption: we have the normal plane at this step in the form:
% plane_normal = [nx ny nz]

% example normal lies directly on the x axis
function plane = estimatePlane(datatype, data_split_with_ring, num_targets, num_beams, object_list)

        opt.corners.rpy_init = [45 2 3];
        opt.corners.T_init = [2, 0, 0];
        opt.corners.H_init = eye(4);
        opt.corners.method = "Constraint Customize"; %% will add more later on
        opt.corners.UseCentroid = 1;

        plane = cell(1,num_targets);
        
        for t = 1:num_targets
            if datatype == 1
                X = [];
                for j = 1: num_beams
                    X = [X, data_split_with_ring{t}(j).points];
                end
                [plane{t}, ~] = estimateNormal(opt.corners, X(1:3, :), 1.5);
            elseif datatype == 2
                plane{t}.centroid =  [object_list(t).centroid; 1];
                plane{t}.normals =  object_list(t).normal;
                plane{t}.unit_normals = object_list(t).normal/(norm(object_list(t).normal));
            else
                warning("Datatype not recognized in the estimatePlane function");
            end
        end
end
function [data_split_with_ring_cartesian, data_split_with_ring] = splitDataBasedOnRing(data, opts, num_targets)
    spherical_data = cell(1,num_targets);
    data_split_with_ring = cell(1, num_targets);
    data_split_with_ring_cartesian = cell(1, num_targets);

    disp("Parsing data...")
    for t = 1:num_targets
        if isfield(data,'payload_points')
            points = data(t).payload_points;
        elseif isfield(data,'points_mat')
            points = data(t).points_mat;
        end
        spherical_data{t} = Cartesian2Spherical(points);
        data_split_with_ring{t} = splitPointsBasedOnRing(spherical_data{t}, opts.num_beams, opts.datatype);
        data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(points, opts.num_beams, opts.datatype);
    end
    
    % disp("Find the mean of multiple scans...")
    % spherical_data = cell(1,num_targets);
    % split_spherical_data = cell(1,num_targets);
    % sorted_spherical_data = cell(1,num_targets);
    % data_split_with_ring = cell(1, num_targets);
    % data_split_with_ring_cartesian = cell(1, num_targets);
    % 
    % disp("Parsing data...")
    % for t = 1:num_targets
    %     min_azimuth = -pi*ones(1, opts.num_beams);
    %     min_num_points = realmax.*ones(1, opts.num_beams);
    %     for s = 1: opts.num_scans
    %         spherical_data{t}.scan(s).points = Cartesian2Spherical(data(t).point_cloud(s).payload_points);
    %         split_spherical_data{t}.scan(s).points = splitPointsBasedOnRing(spherical_data{t}.scan(s).points, opts.num_beams, opts.datatype);      
    %         for ring = 1:opts.num_beams
    %             if ~isempty(split_spherical_data{t}.scan(s).points(ring).points)
    %                 [~,I] = sort(split_spherical_data{t}.scan(s).points(ring).points(3,:)); % sort acoording to azimuth angle 
    %                 for j = 1:size(split_spherical_data{t}.scan(s).points(ring).points , 2)           
    %                     sorted_spherical_data{t}.scan(s).points(ring).points(:,j) =  split_spherical_data{t}.scan(s).points(ring).points(:,I(:,j));
    %                 end
    %                 %update the min azimuth angle for a specific ring
    %                 if(min_azimuth(ring) < sorted_spherical_data{t}.scan(s).points(ring).points(3,1))
    %                     min_azimuth(ring) = sorted_spherical_data{t}.scan(s).points(ring).points(3,1);
    %                 end
    %                 %update the min num of points in a specific ring
    %                 if min_num_points(ring) > j
    %                    min_num_points(ring) = j;
    %                 end 
    %             else
    %                 sorted_spherical_data{t}.scan(s).points(ring).points =[];
    %             end
    %         end         
    %     end
    %     data_split_with_ring{t} = findMeanOfNScans(sorted_spherical_data{t}, opts.num_scans, opts.num_beams, min_azimuth, min_num_points);
    %     for ring = 1:opts.num_beams
    %         data_split_with_ring_cartesian{t}(ring).points = Spherical2Cartesian(data_split_with_ring{t}(ring).points,"Basic");
    %     end
    % end
end
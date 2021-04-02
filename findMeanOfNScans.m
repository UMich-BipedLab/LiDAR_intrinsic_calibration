function [spherical_points, cartesian_points] = findMeanOfNScans(data, num_beams, num_scans)
%     disp("Find the mean of multiple scans...")
    sorted_spherical_data(num_scans) = struct();
    min_azimuth = -pi*ones(1, num_beams);
    min_num_points = realmax.*ones(1, num_beams);
    for scan = 1: num_scans
        for ring = 1:num_beams
            if ~isempty(data(scan).ring(ring).points)
%                 [~,I] = sort(data(scan).ring(ring).points(3,:),'ascend'); % sort acoording to azimuth angle 
%                 for j = 1:size(data(scan).ring(ring).points , 2)           
%                     sorted_spherical_data(scan).ring(ring).points(:,j) =  data(scan).ring(ring).points(:,I(:,j));
%                 end
                sorted_spherical_data(scan).ring(ring).points = sortrows(data(scan).ring(ring).points',3,'ascend')';
                %update the min azimuth angle for a specific ring
                if(min_azimuth(ring) < sorted_spherical_data(scan).ring(ring).points(3,1))
                    min_azimuth(ring) = sorted_spherical_data(scan).ring(ring).points(3,1);
                end
                %update the min num of points in a specific ring
                if min_num_points(ring) > size(data(scan).ring(ring).points , 2)
                   min_num_points(ring) = size(data(scan).ring(ring).points , 2);
                end 
            else
                sorted_spherical_data(scan).ring(ring).points =[];
            end
        end         
    end
    
    cartesian_points(num_beams) = struct();
    spherical_points = alignNScansOfPoints(sorted_spherical_data, num_scans, num_beams, min_azimuth, min_num_points);
    for ring = 1:num_beams    
        cartesian_points(ring).points = Spherical2Cartesian(spherical_points(ring).points);
    end
end
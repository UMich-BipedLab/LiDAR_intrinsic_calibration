function accumulated_data = accumulateNScans(data, num_beams, num_scans)
%     disp("Accumulating multiple scans of points");
    accumulated_data = struct('points',cell(1,num_beams));
    for i = 1:num_beams
        for scan = 1:num_scans
                accumulated_data(i).points = [accumulated_data(i).points, data(scan).ring(i).points];
        end
    end
end
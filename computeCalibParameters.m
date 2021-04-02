function final_param = computeCalibParameters(calib_param, num_beams, method)
    final_param = calib_param{1};
    for i= 2: length(calib_param)
        for ring = 1:num_beams
           if method == 1
               final_param(ring).H = calib_param{i}(ring).H * final_param(ring).H; 
               final_param(ring).Affine = calib_param{i}(ring).Affine * final_param(ring).Affine;
               final_param(ring).Scaling = calib_param{i}(ring).Scaling * final_param(ring).Scaling;
           elseif method ==2
               final_param(ring).D = calib_param{i}(ring).D + final_param(ring).D;
               final_param(ring).theta = calib_param{i}(ring).theta + final_param(ring).theta;
               final_param(ring).phi = calib_param{i}(ring).phi + final_param(ring).phi;
           elseif method ==3
           
           end
                   
        end       
    end
end
function [data]  = imposeNoise(data_split_with_ring, noise_type, num_beams, noise_level)         
    data(num_beams) = struct();
    noise_range = 0.02*noise_level;
    if(noise_type == 1)% induce 3 param noise model
%         nosie = getNoiseParameters(model);
        noise_d = genRandomNumber(-noise_range, noise_range, 1 ,32);
        noise_theta = genRandomNumber(deg2rad(-0.1), deg2rad(0.1), 1 ,32);
        noise_phi = genRandomNumber(deg2rad(-0.04), deg2rad(0.04), 1 ,32);
        for ring = 1: num_beams
            if (isempty(data_split_with_ring(ring).points))
                data(ring).points = [];
                data(ring).points_with_I = [];
            else
                data_split_with_ring(ring).points(1,:) = data_split_with_ring(ring).points(1,:) + noise_d(ring);
                data_split_with_ring(ring).points(2,:) = data_split_with_ring(ring).points(2,:) + noise_theta(ring);
                data_split_with_ring(ring).points(3,:) = data_split_with_ring(ring).points(3,:) + noise_phi(ring);
                data(ring).points = Spherical2Cartesian(data_split_with_ring(ring).points);
                data(ring).points_with_I = Spherical2Cartesian(data_split_with_ring(ring).points_with_I);
            end
        end
    end
    if(noise_type == 2)% induce 6 param noise model
        noise_d = genRandomNumber(-noise_range, noise_range, 1 ,32);
        noise_ds = genRandomNumber(0.99, 1.01, 2,32);
        noise_theta = genRandomNumber(deg2rad(-0.1), deg2rad(0.1), 3 ,32);
        noise_phi = genRandomNumber(deg2rad(-0.04), deg2rad(0.04), 4 ,32);
        noise_v = genRandomNumber(-0.01, 0.01, 5, 32);
        noise_h = genRandomNumber(-0.01, 0.01, 6, 32);
        for ring = 1: num_beams
            if (isempty(data_split_with_ring(ring).points))
                data(ring).points = [];
                data(ring).points_with_I = [];
            else
                data(ring).points = data_split_with_ring(ring).points;
                data(ring).points_with_I = data_split_with_ring(ring).points_with_I;
                D = data_split_with_ring(ring).points(1,:);
                theta = data_split_with_ring(ring).points(2,:);
                phi = data_split_with_ring(ring).points(3,:);
                data(ring).points(1,:) = (D + noise_d(ring)).* noise_ds(ring).*sin(theta+noise_theta(ring)).*cos(phi-noise_phi(ring))-noise_h(ring).*sin(phi-noise_phi(ring));
                data(ring).points(2,:) = (D + noise_d(ring)).* noise_ds(ring).*sin(theta+noise_theta(ring)).*sin(phi-noise_phi(ring))+noise_h(ring).*cos(phi-noise_phi(ring));
                data(ring).points(3,:) = (D + noise_d(ring)).* noise_ds(ring).*cos(theta+noise_theta(ring)) + noise_v(ring);
                data(ring).points_with_I(1:3,:) = data(ring).points(1:3,:);
            end
        end
    end
    if(noise_type == 3)% induce sim3 noise model
        H = eye(4);
        theta_x = genRandomNumber(deg2rad(-0.2*noise_range), deg2rad(0.2*noise_range), 7 ,32);
        theta_y = genRandomNumber(deg2rad(-0.2*noise_range), deg2rad(0.2*noise_range), 8 ,32);
        theta_z = genRandomNumber(deg2rad(-0.2*noise_range), deg2rad(0.2*noise_range), 9 ,32);
        T = [genRandomNumber(-noise_range,noise_range,10,32)';
            genRandomNumber(-noise_range,noise_range,11,32)';
            genRandomNumber(-noise_range,noise_range,12,32)'];
        S = genRandomNumber(1-0.2*noise_range, 1+0.2*noise_range,13,32);
        
        for ring = 1: num_beams
            if (isempty(data_split_with_ring(ring).points))
                data(ring).points = [];
                data(ring).points_with_I = [];
            else
                R = rotx(theta_x(ring)) * roty(theta_y(ring)) * rotz(theta_z(ring));
                H(1:3, 1:3) = R;
                H(1:3, 4) = T(:,ring);
                Scaling = [S(ring)    0    0    0
                           0    S(ring)    0    0
                           0    0    S(ring)    0
                           0    0    0    1];
                points = Spherical2Cartesian(data_split_with_ring(ring).points);
                data(ring).points = Scaling* H* points;
                data(ring).points_with_I = data_split_with_ring(ring).points_with_I;
                data(ring).points_with_I(1:3,:) = data(ring).points(1:3,:);
            end
        end
    end
end
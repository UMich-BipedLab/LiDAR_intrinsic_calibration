function Cartesianpoints = updateBL1(spherical_data, delta)
    Cartesianpoints = [];
    if ~isempty(spherical_data)
            Cartesianpoints(1,:) = (spherical_data(1,:)+ delta.D).*sin(spherical_data(2,:)+delta.theta).*cos(spherical_data(3,:)+delta.phi);
            Cartesianpoints(2,:) = (spherical_data(1,:)+ delta.D).*sin(spherical_data(2,:)+delta.theta).*sin(spherical_data(3,:)+delta.phi);
            Cartesianpoints(3,:) = (spherical_data(1,:)+ delta.D).*cos(spherical_data(2,:)+delta.theta);
            Cartesianpoints(4,:) = ones(1, size(spherical_data,2));
    end
end
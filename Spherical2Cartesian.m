function Cartesianpoints = Spherical2Cartesian(spherical_data)
    Cartesianpoints = [];
    if ~isempty(spherical_data)
        Cartesianpoints(1,:) = spherical_data(1,:).*sin(spherical_data(2,:)).*cos(spherical_data(3,:));
        Cartesianpoints(2,:) = spherical_data(1,:).*sin(spherical_data(2,:)).*sin(spherical_data(3,:));
        Cartesianpoints(3,:) = spherical_data(1,:).*cos(spherical_data(2,:));
        Cartesianpoints(4,:) = ones(1, size(spherical_data,2));
    end
end
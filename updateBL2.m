function Cartesianpoints = updateBL2(spherical_data, delta)
    Cartesianpoints = [];
    if ~isempty(spherical_data)
        dxy = (spherical_data(1,:)*delta.D_s + delta.D).*sin(delta.V_c);
        Cartesianpoints(1,:) = dxy .* cos(spherical_data(3,:)- delta.A_c)- delta.H_oc *sin(spherical_data(3,:)- delta.A_c);
        Cartesianpoints(2,:) = dxy .* sin(spherical_data(3,:)- delta.A_c)+ delta.H_oc *cos(spherical_data(3,:)- delta.A_c);
        Cartesianpoints(3,:) = (spherical_data(1,:)*delta.D_s+ delta.D).*cos(delta.V_c) + delta.V_oc;
        Cartesianpoints(4,:) = ones(1, size(spherical_data,2));
    end
end
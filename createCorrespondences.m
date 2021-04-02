function correspondences = createCorrespondences(data, planes, ring, num_targets)
    correspondences = [];
    for i = 1:num_targets
        plane = Plane(planes{i}.centroid(1:3), planes{i}.unit_normals);
        points = data(i).ring(ring).points;
        for j = 1: size(points,2)
            point = Point(points(1:3,j));
            correspondences = [correspondences, Point2Plane(point,plane)];
        end
    end
end
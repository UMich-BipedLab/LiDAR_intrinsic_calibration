function payload = getPayload(point_cloud, pc_iter)
    payload = [];
    points = point_cloud(pc_iter, :, :);
    points = (reshape(points, size(points, 2),[]))';
    points = removeZeros(points);
    payload = [points; ones(1, size(payload, 2))];
end
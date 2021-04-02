function [points,XYZIR] = getPointsfromStruct(pointStruct)
    
    len = length(pointStruct.Fields);
    for i =1 : len
        pointField = rosmessage('sensor_msgs/PointField');
        pointField.Name = pointStruct.Fields(i).Name;
        pointField.Offset = pointStruct.Fields(i).Offset;
        pointField.Datatype = pointStruct.Fields(i).Datatype;
        pointField.Count = pointStruct.Fields(i).Count;
        pointFields(i) = pointField;
    end
    pointMsg = rosmessage('sensor_msgs/PointCloud2');
    pointMsg.Height = pointStruct.Height;
    pointMsg.Width = pointStruct.Width;
    pointMsg.PointStep = pointStruct.PointStep;
    pointMsg.RowStep = pointStruct.RowStep;
    pointMsg.Data = pointStruct.Data;
    pointMsg.Fields = pointFields;
    %payload points with xyz coordinates
    xyz = double(readXYZ(pointMsg));
    points = [xyz'; ones(1,size(xyz,1))];
    
    %xyzir points
    fieldnames = readAllFieldNames(pointMsg);
    if any(strcmp(fieldnames,'intensity')) 
    	intensity = readField(pointMsg,'intensity');
    else
    	error("No intensity from Rosbag pointcloud2 messages");
    end
    
    if any(strcmp(fieldnames,'ring'))
    	ring = readField(pointMsg,'ring');
    else
    	error("No intensity from Rosbag pointcloud2 messages");
    end
    XYZIR = [xyz'; double(intensity'); double(ring')];
end
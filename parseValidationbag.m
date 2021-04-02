function data = parseValidationbag(path, ext, scene, pair_num)

    files_from_a_folder = dir(fullfile(path, ext));
    if exist('scene', 'var')
        if scene > length(files_from_a_folder)
            error("Wrong scene number: %i/%i, scene", scene ,length(files_from_a_folder))
        else
            start_scene = 1;
            num_scene = scene;
        end
    else 
        start_scene = 1;
        num_scene = length(files_from_a_folder);
    end
    
    
    for scene = start_scene:num_scene
        selected_file = convertCharsToStrings(path) + convertCharsToStrings(files_from_a_folder(scene).name);
        RawData = getData(selected_file);
        BagData(scene).meta = files_from_a_folder(scene);
        BagData(scene).bagfile = convertCharsToStrings(files_from_a_folder(scene).name);
        total_points = 0;
        if exist('pair_num', 'var')
            start_scan = 1;
            num_scan = pair_num;
        else
            start_scan = 1;
            num_scan = size(RawData, 1);
        end

        
        % prepare for parfor loop
        scans(num_scan).num_tag = [];
        scans(num_scan).lidar_target = [];
        
        for scan = start_scan : num_scan
%             scan
            scans(scan).num_tag = length(RawData{scan}.Detections);
            if scans(scan).num_tag == 0
                continue
            end

            for i =1:scans(scan).num_tag
                [scans(scan).lidar_target(i).payload_points,...
                 scans(scan).lidar_target(i).XYZIR_points ] = getPointsfromStruct(RawData{scan}.Detections(i).Points); % [x;y;z;1]
                total_points = total_points + size(scans(scan).lidar_target(i).payload_points,2);
            end
        end
        
        BagData(scene).scans = scans;
        BagData(scene).total_points = total_points;
        clear scans;
    end
    
    num_scene = size(BagData,2);
    num_targets = num_scene;

%     for t =1 :num_scene
%         num_targets = num_targets + BagData(t).scans(scan).num_tag;
%     end

%     data = struct('payload_points', cell(1,num_targets));% XYZIR
    data = struct('point_cloud', cell(1,num_targets), 'tag_size', cell(1,num_targets));% XYZIR 
%     counter = 1;
    for i = 1:num_scene
        for scan =1:pair_num
%             num_tag = BagData(i).scans(scan).num_tag;
%             for t = 1:num_tag
                data(i).scan(scan).payload_points = BagData(i).scans(scan).lidar_target(1).XYZIR_points;
%                 counter = counter +1;
%             end
        end
    end

end
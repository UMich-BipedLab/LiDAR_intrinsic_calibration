%{
* Copyright (C) 2013-2025, The Regents of The University of Michigan.
* All rights reserved.
* This software was developed in the Biped Lab (https://www.biped.solutions/) 
* under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
* be available under alternative licensing terms; contact the address above.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the Regents of The University of Michigan.
* 
* AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
* WEBSITE: https://www.brucerobot.com/
%}


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

    data = struct('payload_points', cell(1,num_targets));% XYZIR

%     counter = 1;
    for scan =1:pair_num
        for i = 1:num_scene
%             num_tag = BagData(i).scans(scan).num_tag;
                data(i).payload_points = [data(i).payload_points, BagData(i).scans(scan).lidar_target(1).XYZIR_points];
%                 counter = counter +1;
        end
    end

end

function files_from_a_folder = loadFilesFromAFolder(path, extension)
    files_from_a_folder = dir(fullfile(path, extension));
    for file = 1:size(files_from_a_folder, 1)
        name = convertCharsToStrings(files_from_a_folder(file).name);
        filepath = convertCharsToStrings(files_from_a_folder(file).folder) + "/";
        files_from_a_folder(file).file_name = filepath + name;
        files_from_a_folder(file).tag_size = identifyTagSizeFromAName(files_from_a_folder(file).name);
    end
end
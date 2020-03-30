function data = getData(path, name)
    if nargin == 1
        file = path;
    else
        file = path + name;
    end
    
    if ~isfile(file)
        error('Invalid path when loading dataset: path %s', file)
    else
        bag = rosbag(file);
        data = readMessages(bag,'Dataformat','struct');
    end   

end
function tag_size = identifyTagSizeFromAName(name)
    if contains(name, 'LargeTag')
        tag_size = 1.216;
    elseif contains(name, 'SmallTag', 'IgnoreCase',true)
        tag_size = 0.8051;
    else
        warning("Cannot identify the size of the tag from the file name: %s", name)
        tag_size = 0;
    end
end
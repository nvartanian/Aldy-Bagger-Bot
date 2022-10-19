function obj=rotate(obj, rotation)
    for i=1:length(obj.components)
        obj.components{i} = rotate(obj.components{i}, rotation);
    end
    obj.glyph = rotate(obj.glyph, rotation);
end

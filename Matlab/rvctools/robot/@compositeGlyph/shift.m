function obj=shift(obj, translation)
    for i=1:length(obj.components)
        obj.components{i} = shift(obj.components{i}, translation);
    end
    obj.glyph = shift(obj.glyph, translation);
end

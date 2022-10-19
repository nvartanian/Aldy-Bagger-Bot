function obj=transform(obj, trans)
    for i=1:length(obj.components)
        obj.components{i} = transform(obj.components{i}, trans);
    end
    obj.glyph = transform(obj.glyph, trans);
end

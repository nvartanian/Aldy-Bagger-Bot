%SUBSREF Accessable fields a rectangularPrism object

function v = subsref(obj, s)

    if s.type  ~= '.'
        error('only .field supported')
    end
    switch s.subs,
        case 'width',
            v = obj.width;
        case 'breadth',
            v = obj.breadth;
        case 'height',
            v = obj.height;
        otherwise,
            v = subsref(obj.glyph, s);
    end
end
    
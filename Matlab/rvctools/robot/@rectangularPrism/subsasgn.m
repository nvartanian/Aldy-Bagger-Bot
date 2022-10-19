%SUBSASGN Assignment methods on a rectangularPrism object

function obj = subsasgn(obj, s, v)

    if s(1).type  ~= '.'
        error('only .field supported')
    end
    switch s(1).subs,
        case 'width',
            obj.width = v;
            s(1).subs = 'vertices';
            obj.glyph = subsasgn(obj.glyph, s, vertices(obj));
        case 'height',
            obj.height = v;
            s(1).subs = 'vertices';
            obj.glyph = subsasgn(obj.glyph, s, vertices(obj));
        case 'breadth',
            obj.breadth = v;
            s(1).subs = 'vertices';
            obj.glyph = subsasgn(obj.glyph, s, vertices(obj));
        otherwise,
            obj.glyph = subsasgn(obj.glyph, s, v);
    end
end

function v = vertices(obj)
    v(:,1) = [0,0,0]';
    v(:,2) = [obj.width,0,0]';
    v(:,3) = [obj.width,obj.breadth,0]';
    v(:,4) = [0,obj.breadth,0]';
    v(:,5) = [0,0,obj.height]';
    v(:,6) = [obj.width,0,obj.height]';
    v(:,7) = [obj.width,obj.breadth,obj.height]';
    v(:,8) = [0,obj.breadth,obj.height]';
end

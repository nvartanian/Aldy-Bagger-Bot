%SUBSREF Accessable fields a rightCircularCylinder object
%
%   obj.handle = handle of patch object assigned by plot

function v = subsref(obj, s)

    if s.type  ~= '.'
        error('only .field supported')
    end
    switch s.subs,
        case 'handle',
            if isfield(obj.glyph, 'handle')
                v = obj.glyph.handle;
            else
                error('handle is only available after plot has been called');
            end
    case 'radius',
        v = obj.radius;
    case 'width',
        v = max(obj.vertex(1,:)) - min(obj.vertex(1,:));
    case 'breadth',
        v = max(obj.vertex(2,:)) - min(obj.vertex(2,:));
    case 'height',
        v = max(obj.vertex(3,:)) - min(obj.vertex(3,:));
    case 'vertices',
        v = obj.vertex';
    case 'faces',
        v = [obj.topFace nan(size(obj.topFace,1), 1);obj.cylinderFace;obj.bottomFace nan(size(obj.bottomFace,1), 1)];
    otherwise,
        error(['Unknown field ', s.subs])
    end
end

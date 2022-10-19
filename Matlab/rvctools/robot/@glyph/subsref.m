%SUBSREF Accessable fields a glyph object
%
%   obj.handle = handle of patch object assigned by plot

function v = subsref(obj, s)

    if s.type  ~= '.'
        error('only .field supported')
    end
    switch s.subs,
        case 'width',
            v = max(obj.vertex(1,:)) - min(obj.vertex(1,:));
        case 'breadth',
            v = max(obj.vertex(2,:)) - min(obj.vertex(2,:));
        case 'height',
            v = max(obj.vertex(3,:)) - min(obj.vertex(3,:));
        case 'axes',
            if isfield(obj.axes, 'length')
                v = obj.axes.length;
            else
                v = 0;
            end
        case 'handle',
            if isfield(obj.glyph, 'handle')
                v = obj.glyph.handle;
            else
                error('handle is only available after plot has been called');
            end
        case 'transform',
            v = eye(4);
            v(1:3,1:3) = obj.orientation;
            v(1:3,4) = obj.origin;
        case 'orientation',
            v = obj.orientation;
        case 'color',
            v = obj.color;
        case 'origin',
            v = obj.origin;
        case 'vertices',
            v = obj.vertex';
        case 'faces',
            v = obj.face;
        case 'centroid',
            if isfield(obj.ellipsoid, 'centroid')
               v = obj.ellipsoid.centroid;
            else
                v = [];
            end
        case 'radii',
            if isfield(obj.ellipsoid, 'radii')
                v = obj.ellipsoid.radii;
            else
                v = [];
            end
       case 'color',
            v = obj.color;
        otherwise,
            error(['Unknown field ', s.subs])
    end
end
    
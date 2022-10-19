function obj=rotate(obj, rotation)
    if ~all(size(rotation) == [3 3])
        error('requires a 3x3 matrix');
    end
    obj.vertex = rotation * obj.vertex;
    obj.origin = rotation * obj.origin;
    if isfield(obj.axes, 'endPoint')
        obj.axes.endPoint = rotation * obj.axes.endPoint;
    end
end

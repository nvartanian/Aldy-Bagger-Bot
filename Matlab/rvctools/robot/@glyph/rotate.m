function obj=rotate(obj, rotation)
    if ~all(size(rotation) == [3 3])
        error('requires a 3x3 matrix');
    end
    obj.vertex = rotation * obj.vertex;
    obj.origin = rotation * obj.origin;
    obj.orientation = rotation * obj.orientation;
    if isfield(obj.axes, 'endPoint')
        obj.axes.endPoint = rotation * obj.axes.endPoint;
    end
    if isfield(obj.ellipsoid, 'x') && length(obj.ellipsoid.x)
        pt = rotation * [obj.ellipsoid.x(:),obj.ellipsoid.y(:),obj.ellipsoid.z(:)]';
        obj.ellipsoid.x = reshape(pt(1,:), size(obj.ellipsoid.x));
        obj.ellipsoid.y = reshape(pt(2,:), size(obj.ellipsoid.y));
        obj.ellipsoid.z = reshape(pt(3,:), size(obj.ellipsoid.z));
    end
end

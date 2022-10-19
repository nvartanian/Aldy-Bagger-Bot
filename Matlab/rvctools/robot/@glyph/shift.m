function obj=shift(obj, translation)
    if all(size(translation) == [1 3])
        translation = translation';
    elseif all(size(translation) ~= [3 1])
        error('requires a 3x1 or 1x3 vector');
    end
    obj.vertex = obj.vertex + repmat(translation, 1, size(obj.vertex, 2));
    obj.origin = obj.origin + translation;
    if isfield(obj.axes, 'endPoint')
        obj.axes.endPoint = repmat(translation, 1, size(obj.axes.endPoint , 2));
    end
    if isfield(obj.ellipsoid, 'x') && length(obj.ellipsoid.x)
        pt = [obj.ellipsoid.x(:),obj.ellipsoid.y(:),obj.ellipsoid.z(:)]' + repmat(trans(1:3,4), 1, length(obj.ellipsoid.x(:)));
        obj.ellipsoid.x = reshape(pt(1,:), size(obj.ellipsoid.x));
        obj.ellipsoid.y = reshape(pt(2,:), size(obj.ellipsoid.y));
        obj.ellipsoid.z = reshape(pt(3,:), size(obj.ellipsoid.z));
    end
end

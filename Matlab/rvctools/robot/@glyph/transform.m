%TRANSFORM Rotate and translate fields a glyph object
function obj=transform(obj, tr)
    if all(size(tr) == [4 4]) || all(size(tr) == [3 4])
        obj.vertex = tr(1:3,1:3) * obj.vertex + repmat(tr(1:3,4), 1, size(obj.vertex, 2));
        obj.origin = tr(1:3,1:3) * obj.origin + tr(1:3,4);
    	obj.orientation = tr(1:3,1:3) * obj.orientation;
        if isfield(obj.axes, 'endPoint')
            obj.axes.endPoint = tr(1:3,1:3) * obj.axes.endPoint + repmat(tr(1:3,4), 1, size(obj.axes.endPoint , 2));
        end
    else
        error('requires a 3x4 or 4x4 matrix');
    end
    if isfield(obj.ellipsoid, 'x') && length(obj.ellipsoid.x)
        pt = tr(1:3,1:3) * [obj.ellipsoid.x(:),obj.ellipsoid.y(:),obj.ellipsoid.z(:)]' + repmat(tr(1:3,1:3) * obj.ellipsoid.centroid' + tr(1:3,4), 1, length(obj.ellipsoid.x(:)));
        obj.ellipsoid.x = reshape(pt(1,:), size(obj.ellipsoid.x));
        obj.ellipsoid.y = reshape(pt(2,:), size(obj.ellipsoid.y));
        obj.ellipsoid.z = reshape(pt(3,:), size(obj.ellipsoid.z));
    end
end

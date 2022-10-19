function obj=transform(obj, trans)
    if all(size(trans) == [4 4]) || all(size(trans) == [3 4])
        obj.vertex = trans(1:3,1:3) * obj.vertex + repmat(trans(1:3,4), 1, size(obj.vertex, 2));
        obj.origin = trans(1:3,1:3) * obj.origin + trans(1:3,4);
        if isfield(obj.axes, 'endPoint')
            obj.axes.endPoint = trans(1:3,1:3) * obj.axes.endPoint + repmat(trans(1:3,4), 1, size(obj.axes.endPoint, 2));;
        end
    else
        error('requires a 3x4 or 4x4 matrix');
    end
end

function obj=shift(obj, translation)
    if all(size(translation) == [3 1])
		obj.vertex = obj.vertex + repmat(translation, 1, size(obj.vertex, 2));
    elseif all(size(translation) == [1 3])
		obj.vertex = obj.vertex + repmat(translation', 1, size(obj.vertex, 2));
	else
        error('requires a 3x1 or 1x3 vector');
    end
end

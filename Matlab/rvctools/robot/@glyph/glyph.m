function obj=glyph(vertex, face, color)
    if size(vertex, 2) == 3 || size(vertex, 2) == 4
        obj.vertex = vertex(:,1:3)';
    elseif size(vertex, 1) == 3 || size(vertex, 1) == 4
        obj.vertex = vertex(1:3,:);
    else
        error('Param 1 must be 3xn');
    end
    if any(face < 1 | size(obj.vertex, 2) < face)
        error('Param 2 must be indicies into param 1');
    end
    if nargin < 3
        obj.color = [.9,.9,.9]; % white
    else
        obj.color = color;
    end
    obj.face = face; % m x n
    obj.options = [];
    % Define axes
    obj.axes = [];
    obj.origin = [0;0;0];
    obj.orientation = eye(3);
    obj.ellipsoid = [];
    % Add display support
    obj.glyph = [];
    obj = class(obj, 'glyph');
end

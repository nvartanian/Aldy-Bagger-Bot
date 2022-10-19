function obj=rectangularPrism(width, breadth, height, color)
    if nargin < 1, width = 1; end
    if nargin < 2, breadth = 1; end
    if nargin < 3, height = 1; end
    obj.width = width;
    obj.breadth = breadth;
    obj.height = height;
    if nargin < 4
        color = [0,1,0]; % blue
    end
    obj.options = [];
    % Build a prism with counter clockwise (CCW) orientation
    face(1,:) = [1,4,3,2]; % z == 0
    face(2,:) = [1,5,8,4]; % x == 0
    face(3,:) = [1,2,6,5]; % y == 0
    face(4,:) = [3,7,6,2]; % x == width
    face(5,:) = [4,8,7,3]; % y == breadth
    face(6,:) = [5,6,7,8]; % z == height
    % Add display support
    obj = class(obj, 'rectangularPrism', glyph(vertices, face, color));

    function v = vertices
        v(:,1) = [0,0,0]';
        v(:,2) = [width,0,0]';
        v(:,3) = [width,breadth,0]';
        v(:,4) = [0,breadth,0]';
        v(:,5) = [0,0,height]';
        v(:,6) = [width,0,height]';
        v(:,7) = [width,breadth,height]';
        v(:,8) = [0,breadth,height]';
    end

end

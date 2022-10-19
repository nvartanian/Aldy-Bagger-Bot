function obj=rightCircularCylinder(radius, height, cylinderFaces)
    if nargin < 1, radius = 1; end
    if nargin < 2, height = 1; end
    if nargin < 3, cylinderFaces = 12; end
    obj.radius = radius;
    obj.height = height;
    obj.color = [0,1,0]; % blue
    obj.options = [];
    % Build a right circular cylinder with counter clockwise (CCW) orientation
    x = radius*sin(0:2*pi/cylinderFaces:2*pi-pi/cylinderFaces);
    y = radius*cos(0:2*pi/cylinderFaces:2*pi-pi/cylinderFaces);
    pointsPerEnd = length(x);
    obj.vertex = [0,x,x,0; ...
                  0,y,y,0; ...
                  0,zeros(1,pointsPerEnd),ones(1,pointsPerEnd)*height,height];
    % Construct the indices of the face vertices
    bottomPoints = [2:pointsPerEnd+1;3:pointsPerEnd+2];
    bottomPoints(end,end) = 2;
    obj.bottomFace = [ones(pointsPerEnd,1),flipud(bottomPoints)'];
    topPoints = bottomPoints+pointsPerEnd;
    lastPoint = size(obj.vertex,2);
    obj.topFace = [lastPoint*ones(pointsPerEnd,1),topPoints'];
    obj.cylinderFace = [bottomPoints' flipud(topPoints)'];
    % Define support for axes
    obj.axes = [];
    obj.origin = [0;0;0];
    % Add display support
    obj.glyph = [];
    obj = class(obj, 'rightCircularCylinder');
   
end

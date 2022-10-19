%SUBSASGN Assignment methods on a rightCircularCylinder object
%
%   c.color = 1x3 matrix : RGB values
%   c.axes = length;
%   c.axesColor = ColorSpec

function r = subsasgn(r, s, v)

    if s(1).type  ~= '.'
        error('only .field supported')
    end
    switch s(1).subs,
        case 'color',
            if size(v) ~= [1 3]
                error('color must be [1x3]');
            end
            r.color = v;
        case 'axes',
            if isfield(v, 'length')
                r.axes = v;
            else
                r.axes.length = v;
            end
            for i=1:3
                r.axes.endPoint(:,i) = r.origin;
                r.axes.endPoint(i,i) = r.origin(i) + r.axes.length;
            end
            if ~isfield(r.axes, 'color')
                r.axes.color = [0;0;1]; % blue
            end
            if ~isfield(r.axes, 'label')
                r.axes.label = {'X', 'Y', 'Z'};
            end
        case 'axesColor',
            if size(v) ~= [1 3]
                error('color must be [1x3]');
            end
            r.axes.color = v;
        case 'radius',
            r.radius = v;
            r = setSize(r, r.radius, r.height);
        case 'height',
            r.height = v;
            r = setSize(r, r.radius, r.height);
        otherwise,
            error(['Unknown field ', s.subs])
    end
end

function obj=setSize(obj, radius, height)
    x = radius*sin(0:pi/10:2*pi-pi/20);
    y = radius*cos(0:pi/10:2*pi-pi/20);
    pointsPerEnd = length(x);
    obj.vertex = [0,x,x,0; ...
                  0,y,y,0; ...
                  0,zeros(1,pointsPerEnd),ones(1,pointsPerEnd)*height,height];
    % Define axes
    obj.axes(:,1) = [radius*0.5;0;0]; % x axis
    obj.axes(:,2) = [0;radius*0.5;0]; % y axis
    obj.axes(:,3) = [0;0;radius*0.5]; % z axis
end
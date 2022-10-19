%SUBSASGN Assignment methods on a glyph object
%
%   glyph.color = 1x3 matrix : RGB values
%   glyph.axes = length | struct { length, [color], [label] }
%   glyph.ellipsoid = 1 or 0 to switch display

function obj = subsasgn(obj, s, v)

    if s(1).type  ~= '.'
        error('only .field supported')
    end
    switch s(1).subs,
        case 'color',
            if size(v) ~= [1 3]
                error('color must be [1x3]');
            end
            obj.color = v;
         case 'vertices',
            if size(v, 1) == 3
                obj.vertex = v;
            elseif size(v, 2) == 3
                obj.vertex = v';
            else
                error('Value must be 3xn');
            end
       case 'axes',
            if isfield(v, 'length')
                obj.axes = v;
            else
                obj.axes.length = v;
            end
            for i=1:3
                obj.axes.endPoint(:,i) = obj.origin;
                obj.axes.endPoint(i,i) = obj.origin(i) + obj.axes.length;
            end
            if ~isfield(obj.axes, 'color')
                obj.axes.color = [0;0;1]; % blue
            end
            if ~isfield(obj.axes, 'label')
                obj.axes.label = {'X', 'Y', 'Z'};
            end
        case 'transform',
            rChange = v(1:3,1:3) * obj.orientation';
            oChange =  v(1:3,4) - rChange * obj.origin;
            obj = transform(obj, [rChange, oChange]);
        case 'ellipsoid',
            [obj.ellipsoid.centroid,obj.ellipsoid.radii] = findEllipsoid(obj.vertex');
            if v
                [obj.ellipsoid.x, obj.ellipsoid.y, obj.ellipsoid.z] = ellipsoid(0,0,0, obj.ellipsoid.radii(1), obj.ellipsoid.radii(2), obj.ellipsoid.radii(3));
            else
                obj.ellipsoid.x = []; obj.ellipsoid.y = []; obj.ellipsoid.z = [];
                if isfield(obj.ellipsoid, 'handle') && ~isempty(obj.ellipsoid.handle)
                    delete(obj.ellipsoid.handle);
                    obj.ellipsoid.handle = [];
                end
            end
        otherwise,
            error(['Unknown field ', s.subs])
    end
end

function [centroid,radii]=findEllipsoid(vertex)
    centroid = (max(vertex) + min(vertex)) / 2;
    % start with the axis aligned bounding box radii 
    radii = (max(vertex) - min(vertex)) / 2;
    alternate = 0;
    while find(((vertex(:,1) - centroid(1)).^2) / radii(1)^2 +...
                ((vertex(:,2) - centroid(2)).^2) / radii(2)^2 +...
                ((vertex(:,3) - centroid(3)).^2) / radii(3)^2 > 1)
        if alternate==0
            radii = radii * 1.01;
            alternate=1;
        else  
            % work out the average of a,b,c and then add 1% of this average to each one
            radii = radii + sum(radii) / 3 * 0.01;
            alternate=0;
        end
    end
end

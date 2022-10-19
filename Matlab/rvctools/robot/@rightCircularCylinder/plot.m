function obj=plot(obj)
    if isfield(obj.glyph, 'handle')
        set(obj.glyph.handle(1), 'Vertices', obj.vertex');
        set(obj.glyph.handle(2), 'Vertices', obj.vertex');
        if isfield(obj.glyph, 'axes')
            for i=1:3
                set(obj.glyph.axes(i), 'xdata', [obj.origin(1);obj.axes.endPoint(1,i)], ...
                    'ydata', [obj.origin(2);obj.axes.endPoint(2,i)], ...
                    'zdata', [obj.origin(3);obj.axes.endPoint(3,i)]);
                set(obj.glyph.axesLabel(i), 'Position', obj.axes.endPoint(1:3,i));
            end
        end
    else
        % if current figure has hold on, then draw object here
        % otherwise, create a new figure
        if isempty(get(0,'CurrentFigure'))
           axis([min(obj.vertex(1,:)) - obj.radius, max(obj.vertex(1,:)) + obj.radius, ... % x range
                min(obj.vertex(2,:)) - obj.radius, max(obj.vertex(2,:)) + obj.radius, ... % y range
                min(obj.vertex(3,:)) - obj.height, max(obj.vertex(3,:)) + obj.height, ... % z range
                min(obj.vertex(3,:)) - obj.height, max(obj.vertex(3,:)) + obj.height]); % color range
        end
        % Create the surfaces of the object
        obj.glyph.handle(1) = patch('Vertices', obj.vertex', 'Faces', [obj.topFace;obj.bottomFace], 'FaceVertexCData', obj.color, 'FaceColor', 'flat');
        obj.glyph.handle(2) = patch('Vertices', obj.vertex', 'Faces', obj.cylinderFace, 'FaceVertexCData', obj.color, 'FaceColor', 'flat');
        if isfield(obj.axes, 'endPoint')
            for i=1:3
                obj.glyph.axesLabel(i) = text(obj.axes.endPoint(1,i), obj.axes.endPoint(2,i), obj.axes.endPoint(3,i), obj.axes.label{i}, 'erasemode', 'xor');
                obj.glyph.axes(i) = line('xdata', [obj.origin(1);obj.axes.endPoint(1,i)], ...
                    'ydata', [obj.origin(2);obj.axes.endPoint(2,i)], ...
                    'zdata', [obj.origin(3);obj.axes.endPoint(3,i)], ...
                    'color', obj.axes.color, ...
                    'linestyle', '--', ...
                    'erasemode', 'xor');
            end
        end
    end
end
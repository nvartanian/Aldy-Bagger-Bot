%PLOT Display a glyph object
function obj=plot(obj)
    handleIsEmptyOrInvalid = false;
    try
        if isfield(obj.glyph, 'handle')
            set(obj.glyph.handle, 'Vertices', obj.vertex');
            if isfield(obj.glyph, 'axes')
                for i=1:3
                    set(obj.glyph.axes(i), 'xdata', [obj.origin(1);obj.axes.endPoint(1,i)], ...
                        'ydata', [obj.origin(2);obj.axes.endPoint(2,i)], ...
                        'zdata', [obj.origin(3);obj.axes.endPoint(3,i)]);
                    set(obj.glyph.axesLabel(i), 'Position', obj.origin(1:3));
                end
            end
        else
            handleIsEmptyOrInvalid = true;
        end
    catch  %#ok<CTCH>
        handleIsEmptyOrInvalid = true;
    end
    
    if handleIsEmptyOrInvalid
        % if current figure is empty, then set axes to fit this glyph
        if isempty(get(0,'CurrentFigure'))
            lower = min(obj.vertex');
            upper = max(obj.vertex');
            width = upper - lower;
            axis([lower(1) - width(1), upper(1) + width(1), ... % x range
                lower(2) - width(2), upper(2) + width(2), ... % y range
                lower(3) - width(3), upper(3) + width(3)]); % z range
        end
        % Create the surfaces of the object
        obj.glyph.handle = patch('Vertices', obj.vertex', 'Faces', obj.face, ...
            'FaceVertexCData', repmat(obj.color, size(obj.vertex,2), 1), ...
            'FaceColor', obj.color, 'FaceLighting', 'gouraud', ...
            'EdgeColor', 'none','EdgeLighting', 'none');
        if isfield(obj.axes, 'endPoint')
            for i=1:3
                obj.glyph.axesLabel(i) = text(obj.axes.endPoint(1,i), obj.axes.endPoint(2,i), obj.axes.endPoint(3,i), ...
                    obj.axes.label{i}, 'erasemode', 'xor');
                obj.glyph.axes(i) = line('xdata', [obj.origin(1);obj.axes.endPoint(1,i)], ...
                    'ydata', [obj.origin(2);obj.axes.endPoint(2,i)], ...
                    'zdata', [obj.origin(3);obj.axes.endPoint(3,i)], ...
                    'color', obj.axes.color, ...
                    'linestyle', '--', ...
                    'erasemode', 'xor');
            end
        end
    end
    if isfield(obj.ellipsoid, 'x') && length(obj.ellipsoid.x)
        if isfield(obj.ellipsoid, 'handle') && ~isempty(obj.ellipsoid.handle)
            set(obj.ellipsoid.handle, 'XData', obj.ellipsoid.x, 'YData', obj.ellipsoid.y, 'ZData', obj.ellipsoid.z);
        else
            obj.ellipsoid.handle = surface('XData', obj.ellipsoid.x, 'YData', obj.ellipsoid.y, 'ZData', obj.ellipsoid.z, 'FaceColor', 'None');
        end
    end
end
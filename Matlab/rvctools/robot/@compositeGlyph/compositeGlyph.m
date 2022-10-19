function obj=compositeGlyph(components, color)
    if ~isa(components, 'cell')
        error('requires a cell array');
    end
    if nargin < 2
        color = [.9,.9,.9]; % white
    end
    obj.components = components;
    obj.ellipsoid = [];
    vertex = [];
    SetVertices;
    face = [];
    SetFaces;
    MinimiseVertices;
    obj = class(obj, 'compositeGlyph', glyph(vertex, face, color));

    function SetVertices
        verticesRef.type = '.';
        verticesRef.subs = 'vertices';
        for i=1:length(components)
            vertex = [vertex;subsref(components{i}, verticesRef)];
        end
    end

    function SetFaces
        verticesRef.type = '.';
        verticesRef.subs = 'vertices';
        facesRef.type = '.';
        facesRef.subs = 'faces';
        nVertices = 0;
        face = [];
        for i=1:length(components)
            newFaces = subsref(components{i}, facesRef) + nVertices;
            colDelta = size(face, 2) - size(newFaces, 2);
            if colDelta < 0
                face = [face nan(size(face,1), -colDelta); newFaces];
            elseif 0 < colDelta
                face = [face; newFaces nan(size(face,1), colDelta)];
            else
                face = [face; newFaces];
            end
            nVertices = nVertices + size(subsref(components{i}, verticesRef), 1);
        end
    end

    function MinimiseVertices
        vertexCount = size(vertex,1);
        % Identify duplicate vertex entries
        equivalent = zeros(vertexCount,1);
        for i=2:vertexCount-1
            range = i:vertexCount;
            difference = vertex(range,:) - repmat(vertex(i-1,:), length(range), 1);
            isDuplicate = false(vertexCount,1);
            isDuplicate(range) = sum(difference.*difference,2) < eps;
            equivalent(equivalent == 0 & isDuplicate) = i - 1;
        end
        if any(equivalent)
            % Remove duplicate vertex entries
            % and change references in face to the first instance of a vertex
            vertexMap = zeros(vertexCount, 1);
            newVertex = [];
            for i=1:vertexCount
                if equivalent(i) == 0
                    newVertex = [newVertex; vertex(i,:)];
                    vertexMap(i) = size(newVertex,1);
                else
                    face(face == i) = equivalent(i);
                end
            end
            % Replace NaN 
            vertexMap(end+1) = NaN;
            face(isnan(face)) = length(vertexMap);
            % Renumber vertex references in face
            newFace = reshape(vertexMap(face(:)), size(face));
            % Install the new data
            vertex = newVertex;
            face = newFace;
        end
    end
    
end

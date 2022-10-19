function files=createTriFiles(obj, base)
% Create a SWIFT++ file that describes a right circular cylinder
    files = {[base, '.tri']};
    fid = fopen(files{1}, 'w');
    fprintf(fid, 'TRI\n\n%d\n\n%d\n\n', size(obj.vertex, 2), ...
        size(obj.bottomFace, 1) + 2*size(obj.cylinderFace, 1) + size(obj.bottomFace, 1));
    fprintf(fid, '%.5f %.5f %.5f\n', obj.vertex);
    fprintf(fid, '\n');
    fprintf(fid, ' %d %d %d\n', obj.bottomFace');
    writeFaceData(fid, obj.cylinderFace);
    fprintf(fid, ' %d %d %d\n', obj.topFace');
    fclose(fid);

end

function writeFaceData(fid, faceData)
    for i=1:size(faceData, 1)
        faceVertices = '';
        nFaceVertices = 0;
        for j=1:length(faceData(i,:))
            if ~isnan(faceData(i,j))
                if j == 3
                    faceVertices = [faceVertices sprintf(' %d\n', faceData(i,j)-1)];
                    nFaceVertices = nFaceVertices + 1;
                end
                faceVertices = [faceVertices sprintf(' %d', faceData(i,j)-1)];
                nFaceVertices = nFaceVertices + 1;
                if j == 4
                    faceVertices = [faceVertices sprintf(' %d', faceData(i,1)-1)];
                    nFaceVertices = nFaceVertices + 1;
                end
            end
        end
        if nFaceVertices ~= 6
            error('Each face must be a rectangle');
        end
        fprintf(fid, '%s\n', faceVertices);
    end
end
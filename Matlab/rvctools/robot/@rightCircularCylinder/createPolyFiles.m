function files=createPolyFiles(obj, base)
% Create a SWIFT++ file that describes a right circular cylinder
    files = {[base, '.poly']};
    fid = fopen(files{1}, 'w');
    fprintf(fid, 'POLY\n\n%d\n\n%d\n\n', size(obj.vertex, 2), ...
        size(obj.bottomFace, 1) + size(obj.cylinderFace, 1) + size(obj.bottomFace, 1));
    fprintf(fid, '%.5f %.5f %.5f\n', obj.vertex);
    fprintf(fid, '\n');
    writeFaceData(obj, fid, obj.bottomFace)
    writeFaceData(obj, fid, obj.cylinderFace)
    writeFaceData(obj, fid, obj.topFace)
    fclose(fid);
end

function writeFaceData(obj, fid, faceData)
    for i=1:size(faceData, 1)
        faceVertices = '';
        nFaceVertices = 0;
        for j=1:length(faceData(i,:))
            if ~isnan(faceData(i,j))
                faceVertices = [faceVertices sprintf(' %d', faceData(i,j)-1)];
                nFaceVertices = nFaceVertices + 1;
            end
        end
        fprintf(fid, '%d %s\n', nFaceVertices, faceVertices);
    end
end

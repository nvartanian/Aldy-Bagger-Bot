function files=createPolyFiles(obj, base)
% Create a SWIFT++ file that describes a prism
    files = {[base, '.poly']};
    fid = fopen(files{1}, 'w');
    fprintf(fid, 'POLY\n\n%d\n\n%d\n\n', size(obj.vertex, 2), size(obj.face, 1));
    fprintf(fid, '%.5f %.5f %.5f\n', obj.vertex);
    fprintf(fid, '\n');

    for i=1:size(obj.face, 1)
        faceVertices = '';
        nFaceVertices = 0;
        for j=1:length(obj.face(i,:))
            if ~isnan(obj.face(i,j))
                faceVertices = [faceVertices sprintf(' %d', obj.face(i,j)-1)];
                nFaceVertices = nFaceVertices + 1;
            end
        end
        fprintf(fid, '%d %s\n', nFaceVertices, faceVertices);
    end
    fclose(fid);

end

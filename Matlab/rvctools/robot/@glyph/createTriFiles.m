function files=createTriFiles(obj, base)
% Create a SWIFT++ file that describes a glyph
    if size(obj.face, 2) ~= 3
        error('Each face must be a triangle');
    end
    files = {[base, '.tri']};
    fid = fopen(files{1}, 'w');
    fprintf(fid, 'TRI\n\n%d\n\n%d\n\n', size(obj.vertex, 2), size(obj.face, 1));
    fprintf(fid, '%.5f %.5f %.5f\n', obj.vertex);
    fprintf(fid, '\n');

    fprintf(fid, ' %d %d %d\n', obj.face');
    fclose(fid);

end

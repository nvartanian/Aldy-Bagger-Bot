function writePolyFile(obj, file)
    for i=1:length(obj.components)
        writePolyFile(obj.components{i}, [file, '.', int2str(i)]);
    end
end

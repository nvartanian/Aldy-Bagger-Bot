function files = createTriFiles(obj, base)
    files = {};
    for i=1:length(obj.components)
        files = union(files, createTriFiles(obj.components{i}, [base, '_', int2str(i)]));
    end

end

function track = loadTrackCsv(filePath)
%LOADTRACKCSV Load track CSV with left, center and right lines in meters.

    filePath = string(filePath);

    if filePath == ""
        error("LTO:Track:EmptyPath", "Track file path is empty.");
    end

    if ~isfile(filePath)
        error("LTO:Track:FileNotFound", ...
              "Track file does not exist: %s", filePath);
    end

    data = readtable(filePath, "VariableNamingRule", "preserve");

    requiredColumns = { ...
        'x_left_m', ...
        'y_left_m', ...
        'x_center_m', ...
        'y_center_m', ...
        'x_right_m', ...
        'y_right_m' ...
    };

    for i = 1:numel(requiredColumns)
        columnName = requiredColumns{i};

        if ~ismember(columnName, data.Properties.VariableNames)
            error("LTO:Track:MissingColumn", ...
                  "Missing required track column: %s", columnName);
        end
    end

    track = struct();

    track.file_path = filePath;
    track.units.position = "m";
    track.source_format = "left_center_right_m";

    track.x_left = data.("x_left_m")(:);
    track.y_left = data.("y_left_m")(:);

    track.x_center = data.("x_center_m")(:);
    track.y_center = data.("y_center_m")(:);

    track.x_right = data.("x_right_m")(:);
    track.y_right = data.("y_right_m")(:);

    track = lto.track.computeTrackGeometry(track);
end
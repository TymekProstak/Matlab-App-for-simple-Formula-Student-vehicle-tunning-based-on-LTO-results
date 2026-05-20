function writeSolutionCsvPackage(solution, outputFolder, prefix)
%WRITESOLUTIONCSVPACKAGE Export solution into readable CSV files.

    if nargin < 3
        prefix = "nlp";
    end

    outputFolder = string(outputFolder);

    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end

    prefix = string(prefix);

    writeNodesCsv(solution, fullfile(outputFolder, prefix + "_nodes.csv"));
    writeControlsCsv(solution, fullfile(outputFolder, prefix + "_controls.csv"));
    writeNormalLoadsCsv(solution, fullfile(outputFolder, prefix + "_normal_loads.csv"));
    writeTireUsageCsv(solution, fullfile(outputFolder, prefix + "_tire_usage_x.csv"), "x");
    writeTireUsageCsv(solution, fullfile(outputFolder, prefix + "_tire_usage_y.csv"), "y");
    writeTireUsageCsv(solution, fullfile(outputFolder, prefix + "_tire_usage_total.csv"), "total");
    writeTrackProjectionCsv(solution, fullfile(outputFolder, prefix + "_track_projection.csv"));
    writeMetadataJson(solution, fullfile(outputFolder, prefix + "_metadata.json"));
end


function writeNodesCsv(solution, filePath)
%WRITENODESCSV Export node-based solution data.

    data = table();

    data.s_m = solution.s(:);

    if isfield(solution, "t")
        data.t_s = solution.t(:);
    end

    if isfield(solution, "global")
        if isfield(solution.global, "x")
            data.global_x_m = solution.global.x(:);
        end

        if isfield(solution.global, "y")
            data.global_y_m = solution.global.y(:);
        end

        if isfield(solution.global, "psi")
            data.global_psi_rad = solution.global.psi(:);
        end
    end

    if isfield(solution, "X") && isfield(solution, "state_names")
        stateNames = string(solution.state_names);

        for i = 1:numel(stateNames)
            columnName = matlab.lang.makeValidName("state_" + stateNames(i));
            data.(columnName) = solution.X(i, :).';
        end
    end

    writetable(data, filePath);
end


function writeControlsCsv(solution, filePath)
%WRITECONTROLSCSV Export interval-based controls.

    if ~isfield(solution, "U") || isempty(solution.U)
        return;
    end

    data = table();

    s = solution.s(:);

    data.s_start_m = s(1:end-1);
    data.s_end_m = s(2:end);
    data.s_mid_m = 0.5 * (s(1:end-1) + s(2:end));

    if isfield(solution, "control_names")
        controlNames = string(solution.control_names);

        for i = 1:numel(controlNames)
            columnName = matlab.lang.makeValidName("control_" + controlNames(i));
            data.(columnName) = solution.U(i, :).';
        end
    end

    writetable(data, filePath);
end


function writeNormalLoadsCsv(solution, filePath)
%WRITENORMALLOADSCSV Export per-wheel normal loads.

    if ~isfield(solution, "normal_loads")
        return;
    end

    data = table();

    data.s_m = solution.s(:);

    wheelNames = getWheelNames(solution);

    for i = 1:size(solution.normal_loads, 1)
        columnName = matlab.lang.makeValidName("Fz_" + string(wheelNames{i}) + "_N");
        data.(columnName) = solution.normal_loads(i, :).';
    end

    writetable(data, filePath);
end


function writeTireUsageCsv(solution, filePath, usageType)
%WRITETIREUSAGECSV Export selected tire usage component.

    if ~isfield(solution, "tire_usage")
        return;
    end

    if ~isfield(solution.tire_usage, usageType)
        return;
    end

    usage = solution.tire_usage.(usageType);

    data = table();

    data.s_m = solution.s(:);

    wheelNames = getWheelNames(solution);

    for i = 1:size(usage, 1)
        columnName = matlab.lang.makeValidName("usage_" + usageType + "_" + string(wheelNames{i}));
        data.(columnName) = usage(i, :).';
    end

    writetable(data, filePath);
end


function writeTrackProjectionCsv(solution, filePath)
%WRITETRACKPROJECTIONCSV Export projected track quantities.

    if ~isfield(solution, "track")
        return;
    end

    data = table();

    if isfield(solution.track, "s_ref")
        data.s_ref_m = solution.track.s_ref(:);
    else
        data.s_ref_m = solution.s(:);
    end

    if isfield(solution.track, "kappa")
        data.kappa_1pm = solution.track.kappa(:);
    end

    if isfield(solution.track, "width_left")
        data.width_left_m = solution.track.width_left(:);
    end

    if isfield(solution.track, "width_right")
        data.width_right_m = solution.track.width_right(:);
    end

    if isfield(solution.track, "width_total")
        data.width_total_m = solution.track.width_total(:);
    end

    writetable(data, filePath);
end


function writeMetadataJson(solution, filePath)
%WRITEMETADATAJSON Export solution metadata.

    metadata = struct();

    if isfield(solution, "type")
        metadata.type = solution.type;
    end

    if isfield(solution, "status")
        metadata.status = solution.status;
    end

    if isfield(solution, "message")
        metadata.message = solution.message;
    end

    if isfield(solution, "lap_time")
        metadata.lap_time_s = solution.lap_time;
    end

    if isfield(solution, "state_names")
        metadata.state_names = solution.state_names;
    end

    if isfield(solution, "control_names")
        metadata.control_names = solution.control_names;
    end

    if isfield(solution, "wheel_names")
        metadata.wheel_names = solution.wheel_names;
    end

    jsonText = jsonencode(metadata);

    fid = fopen(filePath, "w");

    if fid < 0
        error("LTO:Output:MetadataWriteFailed", ...
              "Cannot open metadata file: %s", filePath);
    end

    cleanupObj = onCleanup(@() fclose(fid));

    fprintf(fid, "%s", jsonText);
end


function wheelNames = getWheelNames(solution)
%GETWHEELNAMES Return wheel names.

    if isfield(solution, "wheel_names")
        wheelNames = solution.wheel_names;
    else
        wheelNames = {'FL', 'FR', 'RL', 'RR'};
    end
end
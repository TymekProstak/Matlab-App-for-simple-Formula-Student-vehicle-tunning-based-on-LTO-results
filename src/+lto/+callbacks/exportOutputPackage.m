function exportOutputPackage(app)
%EXPORTOUTPUTPACKAGE Export selected output package.

    outputRoot = string(app.OutputfolderpathEditField.Value);

    if outputRoot == ""
        error("LTO:Output:EmptyOutputFolder", ...
              "Output folder path is empty.");
    end

    if ~isfolder(outputRoot)
        mkdir(outputRoot);
    end

    mode = string(app.SettingsofoutputexportDropDown.Value);

    timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    packageFolder = fullfile(outputRoot, "lto_output_" + timestamp);

    mkdir(packageFolder);

    try
        lto.ui.setStatus(app, "Exporting package", "loading");
        lto.ui.appendLog(app, "Output package export started.", "INFO");
        lto.ui.appendLog(app, "Export mode: " + mode, "INFO");

        exportNlpSolution(app, packageFolder);

        if modeContains(mode, "config")
            exportCurrentConfig(app, packageFolder);
        end

        if modeContains(mode, "track")
            exportTrack(app, packageFolder);
        end

        if modeContains(mode, "initial guess")
            exportInitialGuess(app, packageFolder);
        end

        if modeContains(mode, "backward forward") || modeContains(mode, "bacward forward")
            exportBackwardForwardComputedNow(app, packageFolder);
        end

        if modeContains(mode, "plots")
            exportPlots(app, packageFolder);
        end

        lto.ui.setStatus(app, "Package exported", "ready");
        lto.ui.appendLog(app, ...
            "Output package exported to: " + string(packageFolder), ...
            "INFO");

    catch ME
        lto.ui.setStatus(app, "Package export failed", "error");
        lto.ui.appendLog(app, ...
            "Failed to export output package: " + ME.message, ...
            "ERROR");
        rethrow(ME);
    end
end


function exportNlpSolution(app, packageFolder)
%EXPORTNLPSOLUTION Export LastSolution as MAT and readable CSV folder.

    if isempty(app.LastSolution)
        error("LTO:Output:NoLastSolution", ...
              "No LastSolution available for output package.");
    end

    solution = app.LastSolution;

    save(fullfile(packageFolder, "nlp_solution.mat"), "solution");

    csvFolder = fullfile(packageFolder, "nlp_csv");
    mkdir(csvFolder);

    lto.export.writeSolutionCsvPackage(solution, csvFolder, "nlp");

    lto.ui.appendLog(app, "Exported NLP solution MAT.", "INFO");
    lto.ui.appendLog(app, "Exported NLP CSV folder.", "INFO");
end


function exportCurrentConfig(app, packageFolder)
%EXPORTCURRENTCONFIG Export current GUI config to JSON.

    cfg = lto.config.readConfigFromApp(app);
    app.Config = cfg;

    jsonText = jsonencode(cfg);

    filePath = fullfile(packageFolder, "config_current.json");

    fid = fopen(filePath, "w");

    if fid < 0
        error("LTO:Output:ConfigWriteFailed", ...
              "Cannot open config output file: %s", filePath);
    end

    cleanupObj = onCleanup(@() fclose(fid));

    fprintf(fid, "%s", jsonText);

    lto.ui.appendLog(app, "Exported current config JSON.", "INFO");
end


function exportTrack(app, packageFolder)
%EXPORTTRACK Export original track CSV.

    if isempty(app.Track)
        lto.ui.appendLog(app, "No Track available. Track export skipped.", "WARNING");
        return;
    end

    outPath = fullfile(packageFolder, "track_original.csv");

    if isfield(app.Track, "file_path")
        trackPath = resolveProjectPath(app, string(app.Track.file_path));

        if isfile(trackPath)
            copyfile(trackPath, outPath);
            lto.ui.appendLog(app, "Copied original track CSV.", "INFO");
            return;
        end
    end

    trackTable = table();

    trackTable.x_left_m = app.Track.x_left(:);
    trackTable.y_left_m = app.Track.y_left(:);

    trackTable.x_center_m = app.Track.x_center(:);
    trackTable.y_center_m = app.Track.y_center(:);

    trackTable.x_right_m = app.Track.x_right(:);
    trackTable.y_right_m = app.Track.y_right(:);

    writetable(trackTable, outPath);

    lto.ui.appendLog(app, "Exported reconstructed track CSV.", "INFO");
end


function exportInitialGuess(app, packageFolder)
%EXPORTINITIALGUESS Export app.InitialGuess to MAT.

    if isempty(app.InitialGuess)
        lto.ui.appendLog(app, ...
            "No InitialGuess available. Initial guess export skipped.", ...
            "WARNING");
        return;
    end

    initialGuess = app.InitialGuess;

    save(fullfile(packageFolder, "initial_guess.mat"), "initialGuess");

    lto.ui.appendLog(app, "Exported initial guess MAT.", "INFO");
end


function exportBackwardForwardComputedNow(app, packageFolder)
%EXPORTBACKWARDFORWARDCOMPUTEDNOW Compute BF result and export it.

    if isempty(app.Track)
        lto.ui.appendLog(app, ...
            "Track not loaded. Loading track from field path before BF export.", ...
            "INFO");

        lto.callbacks.loadTrackFromField(app);
    end

    if isempty(app.Track)
        lto.ui.appendLog(app, ...
            "No Track available. Backward Forward export skipped.", ...
            "WARNING");
        return;
    end

    cfg = lto.config.readConfigFromApp(app);
    app.Config = cfg;

    solverTrack = lto.track.prepareSolverTrack( ...
        app.Track, ...
        cfg, ...
        cfg.solver.solve_level ...
    );

    lto.ui.appendLog(app, "Computing Backward Forward for export...", "INFO");

    bfTimer = tic;

    backwardForward = lto.solver.runBackwardForwardSolver(solverTrack, cfg);

    bfTime = toc(bfTimer);

    save(fullfile(packageFolder, "backward_forward_solution.mat"), ...
        "backwardForward");

    lto.ui.appendLog(app, ...
        "Backward Forward exported. Computation time: " + string(bfTime) + " s.", ...
        "INFO");
end


function exportPlots(app, packageFolder)
%EXPORTPLOTS Export all plot views as PNG.

    if isempty(app.LastSolution)
        lto.ui.appendLog(app, ...
            "No LastSolution available. Plot export skipped.", ...
            "WARNING");
        return;
    end

    plotsFolder = fullfile(packageFolder, "plots_png");
    mkdir(plotsFolder);

    plotItems = string(app.PlottypeselectionDropDown.Items);
    oldPlotType = string(app.PlottypeselectionDropDown.Value);

    for i = 1:numel(plotItems)
        plotType = plotItems(i);

        pngName = getPlotFileName(plotType);
        pngPath = fullfile(plotsFolder, pngName);

        try
            app.PlottypeselectionDropDown.Value = char(plotType);

            lto.plot.plotSolutionSelected(app, app.LastSolution, plotType);

            exportgraphics(app.UIAxes, pngPath, "Resolution", 200);

            lto.ui.appendLog(app, "Exported plot: " + string(pngName), "INFO");

        catch ME
            lto.ui.appendLog(app, ...
                "Plot export skipped for '" + plotType + "': " + ME.message, ...
                "WARNING");
        end
    end

    try
        app.PlottypeselectionDropDown.Value = char(oldPlotType);
        lto.plot.plotSolutionSelected(app, app.LastSolution, oldPlotType);
    catch
    end
end


function pngName = getPlotFileName(plotType)
%GETPLOTFILENAME Return readable default PNG name.

    plotType = string(plotType);

    switch plotType
        case "Trajectory colored by speed"
            pngName = "trajectory_colored_by_speed.png";

        case "Speed profile"
            pngName = "speed_profile.png";

        case "GG plot"
            pngName = "gg_plot.png";

        case "Friction usage"
            pngName = "friction_usage_plot.png";

        case "Controls"
            pngName = "controls_plot.png";

        case "Command rates"
            pngName = "command_rates_plot.png";

        case "Torque / moment"
            pngName = "torque_moment_plot.png";

        case "Beta angle"
            pngName = "beta_angle_plot.png";

        case "Slip angles"
            pngName = "slip_angles_plot.png";

        case "ey / epsi"
            pngName = "ey_epsi_plot.png";

        otherwise
            safeName = matlab.lang.makeValidName(char(plotType));
            pngName = string(safeName) + ".png";
    end
end


function tf = modeContains(mode, phrase)
%MODECONTAINS Case-insensitive dropdown mode check.

    tf = contains(normalizeText(mode), normalizeText(phrase));
end


function txt = normalizeText(txt)
%NORMALIZETEXT Normalize text for matching.

    txt = lower(string(txt));
    txt = erase(txt, " ");
    txt = erase(txt, "-");
    txt = erase(txt, "_");
end


function fullPath = resolveProjectPath(app, filePath)
%RESOLVEPROJECTPATH Resolve absolute or project-relative path.

    filePath = string(filePath);

    if isfile(filePath)
        fullPath = filePath;
        return;
    end

    if isprop(app, "ProjectRoot") && string(app.ProjectRoot) ~= ""
        candidate = string(fullfile(app.ProjectRoot, filePath));

        if isfile(candidate)
            fullPath = candidate;
            return;
        end
    end

    fullPath = filePath;
end
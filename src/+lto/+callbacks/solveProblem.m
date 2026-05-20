function solveProblem(app)
%SOLVEPROBLEM Run selected backend solver and store result in app.LastSolution.

    try
        lto.ui.setStatus(app, "Solving", "loading");
        lto.ui.appendLog(app, "Solve started.", "INFO");

        % Top bar is treated as source of truth.
        lto.callbacks.syncModelFromTopBar(app);
        lto.callbacks.syncSolveLevelFromTopBar(app);

        cfg = lto.config.readConfigFromApp(app);
        app.Config = cfg;

        lto.ui.appendLog(app, ...
            "Periodic track flag: " + string(logical(cfg.solver.periodic_track)), ...
            "INFO");

        lto.ui.appendLog(app, ...
            "Backend model: " + string(cfg.solver.lto_mode) + ...
            ", solve level: " + string(cfg.solver.solve_level) + ...
            ", initial guess: " + string(cfg.solver.initial_guess_strategy), ...
            "INFO");

        if ~hasLoadedTrack(app)
            lto.ui.appendLog(app, ...
                "Track not loaded. Loading track from field path.", ...
                "INFO");
            lto.callbacks.loadTrackFromField(app);
        end

        if ~hasLoadedTrack(app)
            error("LTO:Solve:NoTrack", ...
                  "No valid track is loaded.");
        end

        lto.ui.appendLog(app, "Preparing solver track...", "INFO");

        solverTrack = lto.track.prepareSolverTrack( ...
            app.Track, ...
            cfg, ...
            cfg.solver.solve_level ...
        );

        lto.ui.appendLog(app, ...
            "Solver track prepared. N = " + string(solverTrack.N) + ...
            ", length = " + string(solverTrack.length) + " m.", ...
            "INFO");

        [initialGuess, backendToRun] = prepareInitialGuessForBackend(app, cfg, solverTrack);

        lto.ui.setStatus(app, "Running IPOPT", "loading");
        lto.ui.appendLog(app, backendToRun + " IPOPT solve started.", "INFO");
        drawnow;

        solveTimer = tic;

        solution = runSelectedBackendSolver( ...
            backendToRun, ...
            solverTrack, ...
            cfg, ...
            initialGuess ...
        );

        solveTime = toc(solveTimer);

        lto.ui.appendLog(app, ...
            backendToRun + " IPOPT solve finished in " + string(solveTime) + " s.", ...
            "INFO");

        solution.solve_time_s = solveTime;

        app.LastSolution = solution;

        updateSolveSummaryTable(app, solution, solveTime);

        if string(solution.status) == "success"
            lto.ui.setStatus(app, "Solve finished", "ready");
        else
            lto.ui.setStatus(app, "Solve finished with warning", "warning");
        end

        lto.ui.appendLog(app, ...
            "Solve finished. Status: " + string(solution.status) + ...
            ", lap time: " + string(solution.lap_time) + " s.", ...
            "INFO");

    catch ME
        lto.ui.setStatus(app, "Solve failed", "error");
        lto.ui.appendLog(app, "Solve failed: " + ME.message, "ERROR");

        rethrow(ME);
    end
end


function [initialGuess, backendToRun] = prepareInitialGuessForBackend(app, cfg, solverTrack)
%PREPAREINITIALGUESSFORBACKEND Prepare initial guess and staged pipeline.

    backendToRun = string(cfg.solver.lto_mode);
    strategy = string(cfg.solver.initial_guess_strategy);

    backendNorm = normalizeText(backendToRun);
    strategyNorm = normalizeText(strategy);

    initialGuess = [];

    % ============================================================
    % Direct solve
    % ============================================================

    if strategyNorm == "directsolve"
        lto.ui.appendLog(app, ...
            "Initial guess strategy: Direct solve.", ...
            "INFO");

        lto.ui.appendLog(app, ...
            "No external initial guess will be used.", ...
            "INFO");

        app.InitialGuess = [];
        return;
    end

    % ============================================================
    % From file
    % ============================================================

    if strategyNorm == "fromfile"
        lto.ui.appendLog(app, ...
            "Initial guess strategy: From file.", ...
            "INFO");

        initialGuess = loadInitialGuessFromPath(app);
        app.InitialGuess = initialGuess;
        return;
    end

    % ============================================================
    % Backward Forward only
    % ============================================================

    if strategyNorm == "backwardforward" || strategyNorm == "bacwardforward"
        lto.ui.appendLog(app, ...
            "Initial guess strategy: Backward Forward.", ...
            "INFO");

        initialGuess = runBackwardForwardForInitialGuess(app, cfg, solverTrack);
        app.InitialGuess = initialGuess;
        return;
    end

    % ============================================================
    % Auto staged pipeline
    % ============================================================

    if strategyNorm == "autostagedpipeline"

        if backendNorm == "pointmass"
            lto.ui.appendLog(app, ...
                "Auto staged pipeline for Point mass: BF -> Point mass NLP.", ...
                "INFO");

            initialGuess = runBackwardForwardForInitialGuess(app, cfg, solverTrack);
            app.InitialGuess = initialGuess;
            return;
        end

        if backendNorm == "dynamicbicycle"
            lto.ui.appendLog(app, ...
                "Auto staged pipeline for Dynamic bicycle: BF -> Point mass NLP -> Dynamic bicycle NLP.", ...
                "INFO");

            bfGuess = runBackwardForwardForInitialGuess(app, cfg, solverTrack);

            lto.ui.setStatus(app, "Running point mass seed", "loading");
            lto.ui.appendLog(app, "Point mass seed solve started.", "INFO");
            drawnow;

            pmTimer = tic;

            pointMassSeed = lto.solver.runPointMassSolver( ...
                solverTrack, ...
                cfg, ...
                bfGuess ...
            );

            pmTime = toc(pmTimer);

            lto.ui.appendLog(app, ...
                "Point mass seed solve finished in " + string(pmTime) + ...
                " s. Status: " + string(pointMassSeed.status) + ".", ...
                "INFO");

            initialGuess = pointMassSeed;
            app.InitialGuess = pointMassSeed;
            return;
        end

        error("LTO:Solve:AutoPipelineNotImplemented", ...
              "Auto staged pipeline not implemented for backend: %s", backendToRun);
    end

    error("LTO:Solve:UnknownInitialGuessStrategy", ...
          "Unknown initial guess strategy: %s", strategy);
end


function solution = runSelectedBackendSolver(backendModel, solverTrack, cfg, initialGuess)
%RUNSELECTEDBACKENDSOLVER Run selected backend solver.

    backendNorm = normalizeText(backendModel);

    if backendNorm == "pointmass"
        solution = lto.solver.runPointMassSolver( ...
            solverTrack, ...
            cfg, ...
            initialGuess ...
        );
        return;
    end

    if backendNorm == "dynamicbicycle"
        solution = lto.solver.runDynamicBicycleSolver( ...
            solverTrack, ...
            cfg, ...
            initialGuess ...
        );
        return;
    end

    error("LTO:Solve:BackendNotImplemented", ...
          "Backend model not implemented yet: %s", string(backendModel));
end


function bfGuess = runBackwardForwardForInitialGuess(app, cfg, solverTrack)
%RUNBACKWARDFORWARDFORINITIALGUESS Run BF and log timing.

    lto.ui.setStatus(app, "Running BF", "loading");
    lto.ui.appendLog(app, ...
        "Backward-forward initial guess started.", ...
        "INFO");

    drawnow;

    bfTimer = tic;

    bfGuess = lto.solver.runBackwardForwardSolver(solverTrack, cfg);

    bfTime = toc(bfTimer);

    lto.ui.appendLog(app, ...
        "Backward-forward initial guess finished in " + string(bfTime) + " s.", ...
        "INFO");
end


function initialGuess = loadInitialGuessFromPath(app)
%LOADINITIALGUESSFROMPATH Load one solution-like structure from MAT path field.

    filePath = string(app.InitialguesspathEditField.Value);

    if filePath == ""
        error("LTO:InitialGuess:EmptyPath", ...
              "Initial guess path is empty.");
    end

    if ~isfile(filePath)
        error("LTO:InitialGuess:FileNotFound", ...
              "Initial guess file does not exist: %s", filePath);
    end

    data = load(filePath);
    variableNames = fieldnames(data);

    if numel(variableNames) ~= 1
        error("LTO:InitialGuess:InvalidMatFile", ...
              "MAT file must contain exactly one solution-like structure.");
    end

    initialGuess = data.(variableNames{1});

    if ~isstruct(initialGuess)
        error("LTO:InitialGuess:InvalidStructure", ...
              "Loaded initial guess must be a structure.");
    end

    app.InitialguesspathEditField.Value = char(filePath);

    lto.ui.appendLog(app, ...
        "Initial guess loaded from: " + filePath, ...
        "INFO");
end


function ok = hasLoadedTrack(app)
%HASLOADEDTRACK Check if app.Track contains valid track data.

    ok = false;

    if isempty(app.Track)
        return;
    end

    if ~isstruct(app.Track)
        return;
    end

    ok = isfield(app.Track, "x_center") && ...
         isfield(app.Track, "y_center") && ...
         isfield(app.Track, "s") && ...
         isfield(app.Track, "kappa");
end


function updateSolveSummaryTable(app, solution, solveTime)
%UPDATESOLVESUMMARYTABLE Show basic solve summary in results table.

    try
        app.UITable.Data = {
            'Type',           char(string(solution.type));
            'Status',         char(string(solution.status));
            'Lap time [s]',   solution.lap_time;
            'Solve time [s]', solveTime
        };
    catch
        lto.ui.appendLog(app, "Could not update results table.", "WARNING");
    end
end


function out = normalizeText(txt)
%NORMALIZETEXT Normalize dropdown/config text for robust matching.

    out = lower(string(txt));
    out = strtrim(out);
    out = erase(out, " ");
    out = erase(out, "-");
    out = erase(out, "_");
end
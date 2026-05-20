function solveProblem(app)
%SOLVEPROBLEM Run selected backend solver and store result in app.LastSolution.

    try
        lto.ui.setStatus(app, "Solving", "loading");
        lto.ui.appendLog(app, "Solve started.", "INFO");

        % Top bar is treated as the source of truth before Solve.
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
            lto.ui.appendLog(app, "Track not loaded. Loading track from field path.", "INFO");
            lto.callbacks.loadTrackFromField(app);
        end

        if ~hasLoadedTrack(app)
            error("LTO:Solve:NoTrack", ...
                  "No valid track is loaded.");
        end

        backendModel = string(cfg.solver.lto_mode);

        if backendModel ~= "Point mass"
            error("LTO:Solve:BackendNotImplemented", ...
                  "Backend model not implemented yet: %s", backendModel);
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

        initialGuess = choosePointMassInitialGuess(app, cfg, solverTrack);

        lto.ui.setStatus(app, "Running IPOPT", "loading");
        lto.ui.appendLog(app, "Point mass IPOPT solve started.", "INFO");

        solveTimer = tic;

        solution = lto.solver.runPointMassSolver( ...
            solverTrack, ...
            cfg, ...
            initialGuess ...
        );

        solveTime = toc(solveTimer);

        lto.ui.appendLog(app, ...
            "Point mass IPOPT solve finished in " + string(solveTime) + " s.", ...
            "INFO");

        solution.solve_time_s = solveTime;

        app.LastSolution = solution;

        % % The last NLP result is also a valid solution-like seed for later runs.
        % if isfield(solution, "X") && isfield(solution, "U")
        %     app.InitialGuess = solution;
        % end

        updateSolveSummaryTable(app, solution, solveTime);

        if string(solution.status) == "success"
            lto.ui.setStatus(app, "Solve finished", "ready");
        else
            lto.ui.setStatus(app, "Solve finished with warning", "warning");
        end

        lto.ui.appendLog( ...
            app, ...
            "Solve finished. Status: " + string(solution.status) + ...
            ", lap time: " + string(solution.lap_time) + " s.", ...
            "INFO" ...
        );

    catch ME
        lto.ui.setStatus(app, "Solve failed", "error");
        lto.ui.appendLog(app, "Solve failed: " + ME.message, "ERROR");

        rethrow(ME);
    end
end


function initialGuess = choosePointMassInitialGuess(app, cfg, solverTrack)
%CHOOSEPOINTMASSINITIALGUESS Choose initial guess for Point mass NLP.

    strategy = string(cfg.solver.initial_guess_strategy);
    initialGuess = [];

    strategyNorm = lower(strategy);
    strategyNorm = erase(strategyNorm, " ");
    strategyNorm = erase(strategyNorm, "-");
    strategyNorm = erase(strategyNorm, "_");

    if strategyNorm == "directsolve"
        lto.ui.appendLog(app, "Initial guess strategy: Direct solve.", "INFO");
        lto.ui.appendLog(app, "No external initial guess will be used.", "INFO");
        return;
    end

    if strategyNorm == "autostagedpipeline"
        lto.ui.appendLog(app, ...
            "Initial guess strategy: Auto staged pipeline.", ...
            "INFO");

        lto.ui.setStatus(app, "Running BF", "loading");
        lto.ui.appendLog(app, "Backward-forward initial guess started.", "INFO");

        bfTimer = tic;
        initialGuess = lto.solver.runBackwardForwardSolver(solverTrack, cfg);
        bfTime = toc(bfTimer);

        app.InitialGuess = initialGuess;

        lto.ui.appendLog(app, ...
            "Backward-forward initial guess finished in " + string(bfTime) + " s.", ...
            "INFO");

        return;
    end

    % Handles both correct "Backward Forward" and current typo "Bacward Forward".
    if strategyNorm == "backwardforward" || strategyNorm == "bacwardforward"
        lto.ui.appendLog(app, ...
            "Initial guess strategy: Backward Forward.", ...
            "INFO");

        lto.ui.setStatus(app, "Running BF", "loading");
        lto.ui.appendLog(app, "Backward-forward initial guess started.", "INFO");

        bfTimer = tic;
        initialGuess = lto.solver.runBackwardForwardSolver(solverTrack, cfg);
        bfTime = toc(bfTimer);

        app.InitialGuess = initialGuess;

        lto.ui.appendLog(app, ...
            "Backward-forward initial guess finished in " + string(bfTime) + " s.", ...
            "INFO");

        return;
    end

    if strategyNorm == "fromfile"
        lto.ui.appendLog(app, ...
            "Initial guess strategy: From file.", ...
            "INFO");

        lto.ui.appendLog(app, "Initial guess file loading started.", "INFO");

        fileTimer = tic;
        initialGuess = loadInitialGuessFromPath(app);
        fileTime = toc(fileTimer);

        app.InitialGuess = initialGuess;

        lto.ui.appendLog(app, ...
            "Initial guess file loading finished in " + string(fileTime) + " s.", ...
            "INFO");

        return;
    end

    error("LTO:Solve:UnknownInitialGuessStrategy", ...
          "Unknown initial guess strategy: %s", strategy);
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
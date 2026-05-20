function loadInitialGuessFromFile(app)
%LOADINITIALGUESSFROMFILE Load one solution-like structure from MAT file.

    [fileName, folderPath] = uigetfile( ...
        "*.mat", ...
        "Select initial guess MAT file" ...
    );

    if isequal(fileName, 0)
        lto.ui.appendLog(app, "Initial guess loading cancelled.", "INFO");
        return;
    end

    filePath = string(fullfile(folderPath, fileName));

    try
        lto.ui.setStatus(app, "Loading initial guess", "loading");

        data = load(filePath);

        variableNames = fieldnames(data);

        if numel(variableNames) ~= 1
            error("LTO:InitialGuess:InvalidMatFile", ...
                  "MAT file must contain exactly one solution-like structure.");
        end

        loadedStruct = data.(variableNames{1});

        if ~isstruct(loadedStruct)
            error("LTO:InitialGuess:InvalidStructure", ...
                  "Loaded variable must be a structure.");
        end

        app.InitialGuess = loadedStruct;
        app.InitialguesspathEditField.Value = char(filePath);

        lto.ui.setStatus(app, "Initial guess loaded", "ready");
        lto.ui.appendLog(app, "Initial guess loaded from: " + filePath, "INFO");

    catch ME
        lto.ui.setStatus(app, "Initial guess error", "error");
        lto.ui.appendLog(app, "Failed to load initial guess: " + ME.message, "ERROR");
        rethrow(ME);
    end
end
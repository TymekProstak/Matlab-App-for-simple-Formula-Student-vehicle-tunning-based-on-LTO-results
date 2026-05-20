function saveInitialGuessToFile(app)
%SAVEINITIALGUESSTOFILE Save app.InitialGuess as one solution-like MAT structure.

    if isempty(app.InitialGuess)
        lto.ui.setStatus(app, "No initial guess", "warning");
        lto.ui.appendLog(app, "No initial guess to save.", "WARNING");
        return;
    end

    [fileName, folderPath] = uiputfile( ...
        "*.mat", ...
        "Save initial guess as" ...
    );

    if isequal(fileName, 0)
        lto.ui.appendLog(app, "Initial guess saving cancelled.", "INFO");
        return;
    end

    filePath = string(fullfile(folderPath, fileName));

    [~, ~, ext] = fileparts(filePath);

    if ext == ""
        filePath = filePath + ".mat";
    end

    try
        lto.ui.setStatus(app, "Saving initial guess", "loading");

        solution = app.InitialGuess;

        save(filePath, "solution");

        app.InitialguesspathEditField.Value = char(filePath);

        lto.ui.setStatus(app, "Initial guess saved", "ready");
        lto.ui.appendLog(app, "Initial guess saved to: " + filePath, "INFO");

    catch ME
        lto.ui.setStatus(app, "Initial guess save error", "error");
        lto.ui.appendLog(app, "Failed to save initial guess: " + ME.message, "ERROR");
        rethrow(ME);
    end
end
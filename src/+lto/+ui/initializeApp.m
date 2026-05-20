function initializeApp(app)
%INITIALIZEAPP Reset runtime state and clear startup views.

    app.Config = struct();
    app.Track = struct();
    app.InitialGuess = struct();
    app.LastSolution = struct();

    app.ProjectRoot = lto.ui.getProjectRoot();

    app.ConsoleoutputTextArea.Value = {};

    lto.ui.setStatus(app, "Ready", "ready");

    cla(app.UIAxes);

    app.UITable.Data = cell(0, numel(app.UITable.ColumnName));

    lto.ui.appendLog(app, "Application initialized.", "READY");
end
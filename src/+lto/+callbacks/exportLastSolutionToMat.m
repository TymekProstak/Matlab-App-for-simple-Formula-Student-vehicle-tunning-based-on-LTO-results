function exportLastSolutionToMat(app)
%EXPORTLASTSOLUTIONTOMAT Export app.LastSolution to selected MAT file.

    if isempty(app.LastSolution)
        lto.ui.setStatus(app, "No NLP solution", "warning");
        lto.ui.appendLog(app, "No LastSolution available to export.", "WARNING");
        return;
    end

    [fileName, folderPath] = uiputfile( ...
        "*.mat", ...
        "Export NLP solution to MAT", ...
        "nlp_solution.mat" ...
    );

    if isequal(fileName, 0)
        lto.ui.appendLog(app, "NLP MAT export cancelled.", "INFO");
        return;
    end

    filePath = string(fullfile(folderPath, fileName));

    [~, ~, ext] = fileparts(filePath);

    if ext == ""
        filePath = filePath + ".mat";
    end

    try
        solution = app.LastSolution;

        save(filePath, "solution");

        lto.ui.setStatus(app, "NLP MAT exported", "ready");
        lto.ui.appendLog(app, "NLP solution exported to MAT: " + filePath, "INFO");

    catch ME
        lto.ui.setStatus(app, "NLP MAT export failed", "error");
        lto.ui.appendLog(app, "Failed to export NLP MAT: " + ME.message, "ERROR");
        rethrow(ME);
    end
end
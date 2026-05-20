function exportLastSolutionToCsvFolder(app)
%EXPORTLASTSOLUTIONTOCSVFOLDER Export app.LastSolution to readable CSV folder.

    if isempty(app.LastSolution)
        lto.ui.setStatus(app, "No NLP solution", "warning");
        lto.ui.appendLog(app, "No LastSolution available to export.", "WARNING");
        return;
    end

    folderPath = uigetdir(pwd, "Select folder for NLP CSV export");

    if isequal(folderPath, 0)
        lto.ui.appendLog(app, "NLP CSV export cancelled.", "INFO");
        return;
    end

    try
        lto.export.writeSolutionCsvPackage(app.LastSolution, folderPath, "nlp");

        lto.ui.setStatus(app, "NLP CSV exported", "ready");
        lto.ui.appendLog(app, ...
            "NLP solution CSV package exported to: " + string(folderPath), ...
            "INFO");

    catch ME
        lto.ui.setStatus(app, "NLP CSV export failed", "error");
        lto.ui.appendLog(app, "Failed to export NLP CSV package: " + ME.message, "ERROR");
        rethrow(ME);
    end
end
function browseAndExportOutputPackage(app)
%BROWSEANDEXPORTOUTPUTPACKAGE Select root folder and export package.

    folderPath = uigetdir(pwd, "Select output folder for LTO package");

    if isequal(folderPath, 0)
        lto.ui.appendLog(app, "Output package export cancelled.", "INFO");
        return;
    end

    app.OutputfolderpathEditField.Value = char(folderPath);

    lto.callbacks.exportOutputPackage(app);
end
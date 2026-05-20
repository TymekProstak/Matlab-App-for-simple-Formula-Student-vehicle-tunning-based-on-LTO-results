function saveConfigToFile(app)
%SAVECONFIGTOFILE Save current GUI config through a save-file dialog.

    currentPath = string(app.ConfigurationfilepathEditField.Value);

    if currentPath == ""
        defaultFolder = fullfile(app.ProjectRoot, "config");
        defaultFile = "config.json";
    else
        [defaultFolder, defaultName, defaultExt] = fileparts(currentPath);
        defaultFile = defaultName + defaultExt;

        if defaultFolder == ""
            defaultFolder = fullfile(app.ProjectRoot, "config");
        end
    end

    if ~isfolder(defaultFolder)
        mkdir(defaultFolder);
    end

    defaultPath = fullfile(defaultFolder, defaultFile);

    [fileName, folderPath] = uiputfile( ...
        "*.json", ...
        "Save configuration as", ...
        defaultPath ...
    );

    if isequal(fileName, 0)
        lto.ui.appendLog(app, "Config saving cancelled.", "INFO");
        return;
    end

    configPath = string(fullfile(folderPath, fileName));

    [~, ~, ext] = fileparts(configPath);

    if ext == ""
        configPath = configPath + ".json";
    end

    lto.ui.setStatus(app, "Saving config", "loading");

    lto.config.saveConfigJson(app, configPath);

    app.ConfigurationfilepathEditField.Value = char(configPath);
    app.ConfignotloadedyetLabel.Text = "Config successfully saved";

    lto.ui.setStatus(app, "Config saved", "ready");
    lto.ui.appendLog(app, "Config successfully saved.", "INFO");
end
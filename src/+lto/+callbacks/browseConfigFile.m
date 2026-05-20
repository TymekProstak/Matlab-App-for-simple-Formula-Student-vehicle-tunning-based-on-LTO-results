function configPath = browseConfigFile(app)
%BROWSECONFIGFILE Select config file and write path to GUI field.

    configPath = "";

    [fileName, folderPath] = uigetfile( ...
        "*.json", ...
        "Select configuration file" ...
    );

    if isequal(fileName, 0)
        lto.ui.appendLog(app, "Config file selection cancelled.", "INFO");
        return;
    end

    configPath = string(fullfile(folderPath, fileName));

    app.ConfigurationfilepathEditField.Value = char(configPath);
    app.ConfignotloadedyetLabel.Text = "Config path selected";

    lto.ui.appendLog(app, "Selected config path: " + configPath, "INFO");
end
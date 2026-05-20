function cfg = loadConfigJson(app, filePath)
%LOADCONFIGJSON Load config JSON file and write it to the app GUI.

    if nargin < 2 || isempty(filePath)
        filePath = string(app.ConfigurationfilepathEditField.Value);
    else
        filePath = string(filePath);
    end

    if filePath == ""
        filePath = fullfile(app.ProjectRoot, "config", "default_config.json");
    end

    try
        if ~isfile(filePath)
            error( ...
                "LTO:Config:FileNotFound", ...
                "Config file does not exist: %s", ...
                filePath ...
            );
        end

        jsonText = fileread(filePath);
        cfg = jsondecode(jsonText);

        app.Config = cfg;
        app.ConfigurationfilepathEditField.Value = char(filePath);

        lto.config.writeConfigToApp(app, cfg);

        app.ConfignotloadedyetLabel.Text = "Config loaded";

        lto.ui.setStatus(app, "Config loaded", "ready");
        lto.ui.appendLog(app, "Config loaded from: " + filePath, "INFO");

    catch ME
        app.ConfignotloadedyetLabel.Text = "Config load failed";

        lto.ui.setStatus(app, "Config error", "error");
        lto.ui.appendLog(app, "Failed to load config: " + ME.message, "ERROR");

        rethrow(ME);
    end
end
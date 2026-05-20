function cfg = readDefaultConfig(app)
%READDEFAULTCONFIG Load default JSON config into app.Config.

    if isempty(app.ProjectRoot)
        app.ProjectRoot = lto.ui.getProjectRoot();
    end

    defaultConfigPath = fullfile( ...
        app.ProjectRoot, ...
        "config", ...
        "default_config.json" ...
    );

    try
        if ~isfile(defaultConfigPath)
            error( ...
                "LTO:Config:DefaultConfigMissing", ...
                "Default config file does not exist: %s", ...
                defaultConfigPath ...
            );
        end

        jsonText = fileread(defaultConfigPath);
        cfg = jsondecode(jsonText);

        app.Config = cfg;

        app.ConfigurationfilepathEditField.Value = char(defaultConfigPath);
        app.ConfignotloadedyetLabel.Text = "Default config loaded";

        lto.ui.setStatus(app, "Config loaded", "ready");
        lto.ui.appendLog(app, "Default config loaded from: " + string(defaultConfigPath), "INFO");

    catch ME
        app.Config = struct();

        app.ConfignotloadedyetLabel.Text = "Config load failed";

        lto.ui.setStatus(app, "Config error", "error");
        lto.ui.appendLog(app, "Failed to load default config: " + ME.message, "ERROR");

        rethrow(ME);
    end
end
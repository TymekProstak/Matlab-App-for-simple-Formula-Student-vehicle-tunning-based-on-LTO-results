function loadConfigFromField(app)
%LOADCONFIGFROMFIELD Load config from path stored in text field.

    configPath = string(app.ConfigurationfilepathEditField.Value);

    if configPath == ""
        lto.ui.appendLog(app, "Config path is empty.", "WARNING");
        lto.ui.setStatus(app, "Missing config path", "warning");
        return;
    end

    lto.ui.setStatus(app, "Loading config", "loading");

    cfg = lto.config.loadConfigJson(app, configPath);

    app.Config = cfg;

    lto.ui.setStatus(app, "Config loaded", "ready");
    lto.ui.appendLog(app, "Config loaded from field path.", "INFO");
end
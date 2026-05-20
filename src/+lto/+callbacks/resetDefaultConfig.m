function resetDefaultConfig(app)
%RESETDEFAULTCONFIG Load default config and write it to GUI.

    try
        lto.ui.setStatus(app, "Loading default config", "loading");
        lto.ui.appendLog(app, "Loading default config...", "INFO");

        cfg = lto.config.readDefaultConfig(app);

        lto.config.writeConfigToApp(app, cfg);

        app.Config = cfg;
        app.ConfignotloadedyetLabel.Text = "Default configuration loaded";

        lto.ui.setStatus(app, "Default config loaded", "ready");
        lto.ui.appendLog(app, "Default configuration loaded successfully.", "INFO");

    catch ME
        lto.ui.setStatus(app, "Default config error", "error");
        lto.ui.appendLog(app, "Failed to load default config: " + ME.message, "ERROR");

        rethrow(ME);
    end
end
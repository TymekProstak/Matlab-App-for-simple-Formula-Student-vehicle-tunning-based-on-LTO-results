function loadConfigWithBrowse(app)
%LOADCONFIGWITHBROWSE Select config file and load it immediately.

    configPath = lto.callbacks.browseConfigFile(app);

    if configPath == ""
        lto.ui.setStatus(app, "Ready", "ready");
        lto.ui.appendLog(app, "Config loading cancelled.", "INFO");
        return;
    end

    lto.callbacks.loadConfigFromField(app);
end
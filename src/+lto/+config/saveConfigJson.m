function saveConfigJson(app, filePath)
%SAVECONFIGJSON Save current GUI configuration to a JSON file.

    if nargin < 2 || isempty(filePath)
        filePath = string(app.ConfigurationfilepathEditField.Value);
    else
        filePath = string(filePath);
    end

    try
        % Read current GUI values first.
        cfg = lto.config.readConfigFromApp(app);

        % Store selected config path in cfg.
        app.Config = cfg;

        % Create output folder if needed.
        outputFolder = fileparts(filePath);

        if outputFolder ~= "" && ~isfolder(outputFolder)
            mkdir(outputFolder);
        end

        % Encode JSON.
        jsonText = encodeJsonPretty(cfg);

        % Save file.
        fileID = fopen(filePath, "w");

        if fileID < 0
            error("LTO:Config:FileOpenFailed", ...
                  "Could not open config file for writing: %s", filePath);
        end

        fprintf(fileID, "%s", jsonText);
        fclose(fileID);

        % Update GUI info.
        app.ConfigurationfilepathEditField.Value = char(filePath);
        app.ConfignotloadedyetLabel.Text = "Config saved";

        lto.ui.setStatus(app, "Config saved", "ready");
        lto.ui.appendLog(app, "Config saved to: " + filePath, "INFO");

    catch ME
        lto.ui.setStatus(app, "Config save error", "error");
        lto.ui.appendLog(app, "Failed to save config: " + ME.message, "ERROR");

        rethrow(ME);
    end
end

function jsonText = encodeJsonPretty(cfg)
%ENCODEJSONPRETTY Encode struct to readable JSON.

    try
        jsonText = jsonencode(cfg, "PrettyPrint", true);
    catch
        jsonText = jsonencode(cfg);
    end
end
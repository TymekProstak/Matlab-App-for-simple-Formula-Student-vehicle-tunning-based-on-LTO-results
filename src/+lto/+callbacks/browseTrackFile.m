function trackPath = browseTrackFile(app)
%BROWSETRACKFILE Select track CSV and write path to GUI field.

    trackPath = "";

    [fileName, folderPath] = uigetfile( ...
        "*.csv", ...
        "Select track CSV file" ...
    );

    if isequal(fileName, 0)
        lto.ui.appendLog(app, "Track file selection cancelled.", "INFO");
        return;
    end

    trackPath = string(fullfile(folderPath, fileName));

    app.TrackfilepathEditField.Value = char(trackPath);
    app.TracknotloadedyetLabel.Text = "Track path selected";

    lto.ui.appendLog(app, "Selected track path: " + trackPath, "INFO");
end
function track = loadTrackFromField(app)
%LOADTRACKFROMFIELD Load track CSV from path stored in GUI field.

    track = struct();

    trackPath = string(app.TrackfilepathEditField.Value);

    if trackPath == ""
        lto.ui.setStatus(app, "Missing track path", "warning");
        lto.ui.appendLog(app, "Track path is empty.", "WARNING");
        return;
    end

    try
        lto.ui.setStatus(app, "Loading track", "loading");

        track = lto.track.loadTrackCsv(trackPath);

        app.Track = track;
        app.TracknotloadedyetLabel.Text = "Track loaded successfully";

        lto.ui.setStatus(app, "Track loaded", "ready");
        lto.ui.appendLog(app, "Track loaded successfully.", "INFO");

    catch ME
        app.Track = struct();
        app.TracknotloadedyetLabel.Text = "Track load failed";

        lto.ui.setStatus(app, "Track error", "error");
        lto.ui.appendLog(app, "Failed to load track: " + ME.message, "ERROR");

        rethrow(ME);
    end
end
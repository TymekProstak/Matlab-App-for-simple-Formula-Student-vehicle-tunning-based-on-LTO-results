function previewTrack(app)
%PREVIEWTRACK Plot currently loaded track. Load from field if needed.

    if isempty(app.Track) || ~isfield(app.Track, "x_center")
        lto.ui.appendLog(app, "Track not loaded. Loading from field path.", "INFO");
        lto.callbacks.loadTrackFromField(app);
    end

    if isempty(app.Track) || ~isfield(app.Track, "x_center")
        lto.ui.appendLog(app, "No valid track available for preview.", "WARNING");
        return;
    end

    lto.plot.plotTrackPreview(app, app.Track);

    app.TracknotloadedyetLabel.Text = "Track preview generated";

    lto.ui.setStatus(app, "Track preview", "ready");
    lto.ui.appendLog(app, "Track preview generated.", "INFO");
end
function loadTrackWithBrowse(app)
%LOADTRACKWITHBROWSE Select track CSV and load it immediately.

    trackPath = lto.callbacks.browseTrackFile(app);

    if trackPath == ""
        lto.ui.setStatus(app, "Ready", "ready");
        return;
    end

    lto.callbacks.loadTrackFromField(app);
end
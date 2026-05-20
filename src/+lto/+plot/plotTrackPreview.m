function plotTrackPreview(app, track)
%PLOTTRACKPREVIEW Draw left, center and right track lines.
    lto.plot.clearAxes(app);
    hold(app.UIAxes, "on");
    
    plot(app.UIAxes, track.x_left, track.y_left, "--", ...
        "DisplayName", "Left boundary");

    plot(app.UIAxes, track.x_right, track.y_right, "--", ...
        "DisplayName", "Right boundary");

    plot(app.UIAxes, track.x_center, track.y_center, "-", ...
        "DisplayName", "Centerline");

    hold(app.UIAxes, "off");

    axis(app.UIAxes, "equal");
    grid(app.UIAxes, "on");
    box(app.UIAxes, "on");

    title(app.UIAxes, "Track preview");
    xlabel(app.UIAxes, "x [m]");
    ylabel(app.UIAxes, "y [m]");

    legend(app.UIAxes, "Location", "best");
end
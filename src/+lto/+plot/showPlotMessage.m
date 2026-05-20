function showPlotMessage(app, message)
%SHOWPLOTMESSAGE Display message on result axes.

    ax = app.UIAxes;

    cla(ax);
    axis(ax, [0 1 0 1]);
    grid(ax, "off");
    box(ax, "on");

    text(ax, 0.5, 0.5, string(message), ...
        "HorizontalAlignment", "center", ...
        "VerticalAlignment", "middle");

    title(ax, "Results");
end
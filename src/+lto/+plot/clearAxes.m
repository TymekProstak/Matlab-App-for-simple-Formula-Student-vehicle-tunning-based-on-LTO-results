function ax = clearAxes(app)
%CLEARAXES Hard reset result axes before drawing a new plot.
%
% This removes old yyaxis state, colorbars, legends and old plotted objects.

    oldAx = app.UIAxes;

    parent = oldAx.Parent;
    row = oldAx.Layout.Row;
    col = oldAx.Layout.Column;

    % Delete colorbars from the same figure.
    try
        fig = ancestor(oldAx, "figure");
        delete(findall(fig, "Type", "ColorBar"));
    catch
    end

    % Delete old axes completely. This is the cleanest way to reset yyaxis.
    try
        if isvalid(oldAx)
            delete(oldAx);
        end
    catch
    end

    % Create fresh axes in the same grid cell.
    app.UIAxes = uiaxes(parent);
    app.UIAxes.Layout.Row = row;
    app.UIAxes.Layout.Column = col;

    ax = app.UIAxes;

    hold(ax, "off");
    grid(ax, "on");
    box(ax, "on");

    title(ax, "");
    xlabel(ax, "");
    ylabel(ax, "");
end
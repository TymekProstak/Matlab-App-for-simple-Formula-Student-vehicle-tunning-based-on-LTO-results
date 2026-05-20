function plotTrajectoryColoredBySpeed(app, solution)
%PLOTTRAJECTORYCOLOREDBYSPEED Plot global trajectory colored by speed.
    ax = app.UIAxes;

    hold(ax, "on");

    % Plot loaded track boundaries and centerline.
    if isprop(app, "Track") && ~isempty(app.Track) && isstruct(app.Track)

        if isfield(app.Track, "x_left") && isfield(app.Track, "y_left")
            plot(ax, app.Track.x_left, app.Track.y_left, "--", ...
                "LineWidth", 1.0, ...
                "DisplayName", "Left boundary");
        end

        if isfield(app.Track, "x_right") && isfield(app.Track, "y_right")
            plot(ax, app.Track.x_right, app.Track.y_right, "--", ...
                "LineWidth", 1.0, ...
                "DisplayName", "Right boundary");
        end

        if isfield(app.Track, "x_center") && isfield(app.Track, "y_center")
            plot(ax, app.Track.x_center, app.Track.y_center, ":", ...
                "LineWidth", 1.0, ...
                "DisplayName", "Centerline");
        end
    end

    x = solution.global.x(:);
    y = solution.global.y(:);
    vx = lto.plot.getState(solution, "vx");

    surface(ax, ...
        [x x], ...
        [y y], ...
        zeros(numel(x), 2), ...
        [vx vx], ...
        "FaceColor", "none", ...
        "EdgeColor", "interp", ...
        "LineWidth", 2.0, ...
        "HandleVisibility", "off");

    % Dummy handle only for legend.
    plot(ax, nan, nan, "-", ...
        "LineWidth", 2.0, ...
        "DisplayName", "Optimized trajectory");

    hold(ax, "off");

    axis(ax, "equal");
    grid(ax, "on");
    box(ax, "on");

    title(ax, "Trajectory colored by speed");
    xlabel(ax, "x [m]");
    ylabel(ax, "y [m]");

    cb = colorbar(ax);
    cb.Label.String = "v_x [m/s]";

    legend(ax, "Location", "best");
end
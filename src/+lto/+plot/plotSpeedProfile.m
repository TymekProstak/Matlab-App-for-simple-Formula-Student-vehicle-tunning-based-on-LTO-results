function plotSpeedProfile(app, solution)
%PLOTSPEEDPROFILE Plot longitudinal speed over path coordinate s.

    ax = app.UIAxes;

    s = solution.s(:);
    vx = lto.plot.getState(solution, "vx");

    plot(ax, s, vx, "LineWidth", 1.6, ...
        "DisplayName", "v_x");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "Speed profile");
    xlabel(ax, "s [m]");
    ylabel(ax, "v_x [m/s]");

    legend(ax, "Location", "best");
end
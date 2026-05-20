function plotBetaAngle(app, solution)
%PLOTBETAANGLE Plot beta angle computed from vx and vy.

    ax = app.UIAxes;

    cla(ax);

    s = solution.s(:);

    vx = lto.plot.getState(solution, "vx");
    vy = lto.plot.getState(solution, "vy");

    beta = atan2(vy, max(vx, 1e-9));

    plot(ax, s, beta, "LineWidth", 1.5, ...
        "DisplayName", "\beta");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "Beta angle");
    xlabel(ax, "s [m]");
    ylabel(ax, "\beta [rad]");

    legend(ax, "Location", "best");
end
function plotSlipAngles(app, solution)
%PLOTSLIPANGLES Plot reconstructed four-wheel slip angles.

    ax = app.UIAxes;

    cla(ax);

    slip = lto.plot.computeSlipAngles4W(app, solution);

    s = solution.s(:);
    alpha = slip.alpha;

    hold(ax, "on");

    plot(ax, s, alpha(1, :), "LineWidth", 1.4, ...
        "DisplayName", "\alpha_{FL}");

    plot(ax, s, alpha(2, :), "LineWidth", 1.4, ...
        "DisplayName", "\alpha_{FR}");

    plot(ax, s, alpha(3, :), "LineWidth", 1.4, ...
        "DisplayName", "\alpha_{RL}");

    plot(ax, s, alpha(4, :), "LineWidth", 1.4, ...
        "DisplayName", "\alpha_{RR}");

    yline(ax, 0.0, "--", "DisplayName", "zero");

    hold(ax, "off");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "Slip angles");
    xlabel(ax, "s [m]");
    ylabel(ax, "slip angle [rad]");

    legend(ax, "Location", "best");
end
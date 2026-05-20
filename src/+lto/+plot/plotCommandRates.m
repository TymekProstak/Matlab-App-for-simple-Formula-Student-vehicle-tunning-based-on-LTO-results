function plotCommandRates(app, solution)
%PLOTCOMMANDRATES Plot command rates over path coordinate.

    ax = app.UIAxes;

    cla(ax);

    s = solution.s(:);
    sU = s(1:end-1);

    MCmdDot = lto.plot.getControl(solution, "M_cmd_dot");
    deltaCmdDot = lto.plot.getControl(solution, "delta_cmd_dot");

    yyaxis(ax, "left");
    plot(ax, sU, MCmdDot, "LineWidth", 1.5, ...
        "DisplayName", "\dot{M}_{cmd}");
    ylabel(ax, "\dot{M}_{cmd} [Nm/s]");

    yyaxis(ax, "right");
    plot(ax, sU, deltaCmdDot, "LineWidth", 1.5, ...
        "DisplayName", "\dot{\delta}_{cmd}");
    ylabel(ax, "\dot{\delta}_{cmd} [rad/s]");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "Command rates");
    xlabel(ax, "s [m]");

    legend(ax, "Location", "best");
end
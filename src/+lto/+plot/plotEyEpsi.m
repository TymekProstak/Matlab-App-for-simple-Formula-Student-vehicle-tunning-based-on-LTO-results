function plotEyEpsi(app, solution)
%PLOTEYEPSI Plot lateral and heading error.

    ax = app.UIAxes;

    cla(ax);

    s = solution.s(:);

    ey = lto.plot.getState(solution, "e_y");
    epsi = lto.plot.getState(solution, "e_psi");

    yyaxis(ax, "left");
    plot(ax, s, ey, "LineWidth", 1.5, ...
        "DisplayName", "e_y");
    ylabel(ax, "e_y [m]");

    yyaxis(ax, "right");
    plot(ax, s, epsi, "LineWidth", 1.5, ...
        "DisplayName", "e_\psi");
    ylabel(ax, "e_\psi [rad]");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "ey / epsi");
    xlabel(ax, "s [m]");

    legend(ax, "Location", "best");
end
function plotGG(app, solution)
%PLOTGG Plot longitudinal-lateral acceleration diagram.

    ax = app.UIAxes;

    cla(ax);

    axVal = lto.plot.getState(solution, "ax_prev");
    ayVal = lto.plot.getState(solution, "ay_prev");

    plot(ax, axVal, ayVal, "LineWidth", 1.5, ...
        "DisplayName", "trajectory");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "GG plot");
    xlabel(ax, "a_x [m/s^2]");
    ylabel(ax, "a_y [m/s^2]");

    legend(ax, "Location", "best");
end
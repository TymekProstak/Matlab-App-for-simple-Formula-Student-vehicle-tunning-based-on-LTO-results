function plotTorqueMoment(app, solution)
%PLOTTORQUEMOMENT Plot commanded and realized rear torque.

    ax = app.UIAxes;

    cla(ax);

    s = solution.s(:);

    MRear = lto.plot.getState(solution, "M_rear");
    MCmd = lto.plot.getState(solution, "M_cmd");

    hold(ax, "on");

    plot(ax, s, MRear, "LineWidth", 1.5, ...
        "DisplayName", "M_{rear}");

    plot(ax, s, MCmd, "--", "LineWidth", 1.5, ...
        "DisplayName", "M_{cmd}");

    hold(ax, "off");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "Torque / moment");
    xlabel(ax, "s [m]");
    ylabel(ax, "moment [Nm]");

    legend(ax, "Location", "best");
end
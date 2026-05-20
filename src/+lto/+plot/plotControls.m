function plotControls(app, solution)
%PLOTCONTROLS Plot torque and steering-related states over s.

    ax = app.UIAxes;

    s = solution.s(:);

    MRear = lto.plot.getState(solution, "M_rear");
    MCmd = lto.plot.getState(solution, "M_cmd");

    delta = lto.plot.getState(solution, "delta");
    deltaCmd = lto.plot.getState(solution, "delta_cmd");

    yyaxis(ax, "left");

    h1 = plot(ax, s, MRear, "-", ...
        "LineWidth", 1.5, ...
        "DisplayName", "M_{rear}");

    hold(ax, "on");

    h2 = plot(ax, s, MCmd, "--", ...
        "LineWidth", 1.5, ...
        "DisplayName", "M_{cmd}");

    ylabel(ax, "torque / moment [Nm]");

    yyaxis(ax, "right");

    h3 = plot(ax, s, delta, "-", ...
        "LineWidth", 1.5, ...
        "DisplayName", "\delta");

    h4 = plot(ax, s, deltaCmd, "--", ...
        "LineWidth", 1.5, ...
        "DisplayName", "\delta_{cmd}");

    ylabel(ax, "steering angle [rad]");

    hold(ax, "off");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "Controls");
    xlabel(ax, "s [m]");

    legend(ax, [h1 h2 h3 h4], ...
        "Location", "best");
end
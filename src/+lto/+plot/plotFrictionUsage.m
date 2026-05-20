function plotFrictionUsage(app, solution)
%PLOTFRICTIONUSAGE Plot normalized longitudinal and lateral tire usage.

    ax = app.UIAxes;

    if ~isfield(solution, "tire_usage") || ...
       ~isfield(solution.tire_usage, "x") || ...
       ~isfield(solution.tire_usage, "y")

        lto.plot.showPlotMessage(app, ...
            "Friction usage x/y is not available.");
        return;
    end

    s = solution.s(:);

    usageX = solution.tire_usage.x;
    usageY = solution.tire_usage.y;

    hold(ax, "on");

    % Longitudinal normalized usage.
    h1 = plot(ax, s, usageX(1, :), "-",  "LineWidth", 1.2, "DisplayName", "FL x");
    h2 = plot(ax, s, usageX(2, :), "-",  "LineWidth", 1.2, "DisplayName", "FR x");
    h3 = plot(ax, s, usageX(3, :), "-",  "LineWidth", 1.2, "DisplayName", "RL x");
    h4 = plot(ax, s, usageX(4, :), "-",  "LineWidth", 1.2, "DisplayName", "RR x");

    % Lateral normalized usage.
    h5 = plot(ax, s, usageY(1, :), "--", "LineWidth", 1.2, "DisplayName", "FL y");
    h6 = plot(ax, s, usageY(2, :), "--", "LineWidth", 1.2, "DisplayName", "FR y");
    h7 = plot(ax, s, usageY(3, :), "--", "LineWidth", 1.2, "DisplayName", "RL y");
    h8 = plot(ax, s, usageY(4, :), "--", "LineWidth", 1.2, "DisplayName", "RR y");

    h9 = yline(ax,  1.0, ":", "DisplayName", "+limit");
    h10 = yline(ax, -1.0, ":", "DisplayName", "-limit");

    hold(ax, "off");

    grid(ax, "on");
    box(ax, "on");

    title(ax, "Friction usage");
    xlabel(ax, "s [m]");
    ylabel(ax, "normalized tire usage [-]");

    legend(ax, [h1 h2 h3 h4 h5 h6 h7 h8 h9 h10], ...
        "Location", "best");
end
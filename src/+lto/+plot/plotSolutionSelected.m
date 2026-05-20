function plotSolutionSelected(app, solution, plotType)
%PLOTSOLUTIONSELECTED Dispatch selected solution plot.

    plotType = string(plotType);
    lto.plot.clearAxes(app);

    switch plotType
        case "Trajectory colored by speed"
            lto.plot.plotTrajectoryColoredBySpeed(app, solution);

        case "Speed profile"
            lto.plot.plotSpeedProfile(app, solution);

        case "GG plot"
            lto.plot.plotGG(app, solution);

        case "Friction usage"
            lto.plot.plotFrictionUsage(app, solution);

        case "Controls"
            lto.plot.plotControls(app, solution);

        case "Command rates"
            lto.plot.plotCommandRates(app, solution);

        case "Torque / moment"
            lto.plot.plotTorqueMoment(app, solution);

        case "Beta angle"
            lto.plot.plotBetaAngle(app, solution);

        case "Slip angles"
            lto.plot.plotSlipAngles(app, solution);

        case "ey / epsi"
            lto.plot.plotEyEpsi(app, solution);

        otherwise
            lto.plot.showPlotMessage(app, "Unknown plot type: " + plotType);
    end
end
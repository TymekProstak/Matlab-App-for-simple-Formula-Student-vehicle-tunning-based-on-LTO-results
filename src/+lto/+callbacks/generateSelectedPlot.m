function generateSelectedPlot(app)
%GENERATESELECTEDPLOT Plot selected result type.

    plotType = string(app.PlottypeselectionDropDown.Value);
    solType = string(app.PlotmodeselectionDropDown.Value);

    if solType == ""
        lto.ui.setStatus(app, "No solution type selected", "warning");
        lto.ui.appendLog(app, "No plot mode selected for plotting.", "WARNING");
        return;
    end

    if solType == "NLP solution"

        if isempty(app.LastSolution)
            lto.ui.setStatus(app, "No NLP solution", "warning");
            lto.ui.appendLog(app, "No LastSolution available for plotting.", "WARNING");
            return;
        end

        lto.plot.plotSolutionSelected(app, app.LastSolution, plotType);

        lto.ui.appendLog(app, ...
            "Generated plot: NLP solution / " + plotType, ...
            "INFO");

    elseif solType == "Initial guess"

        if isempty(app.InitialGuess)
            lto.ui.setStatus(app, "No initial guess", "warning");
            lto.ui.appendLog(app, "No InitialGuess available for plotting.", "WARNING");
            return;
        end

        lto.plot.plotSolutionSelected(app, app.InitialGuess, plotType);

        lto.ui.appendLog(app, ...
            "Generated plot: Initial guess / " + plotType, ...
            "INFO");

    else
        lto.ui.setStatus(app, "Unknown plot mode", "warning");
        lto.ui.appendLog(app, ...
            "Unknown plot mode selected: " + solType, ...
            "WARNING");
    end
end
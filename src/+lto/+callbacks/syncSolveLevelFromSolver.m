function syncSolveLevelFromSolver(app)
%SYNCSOLVELEVELFROMSOLVER Copy solver-tab solve level to top bar.

    value = app.SolverLevelDropDown.Value;
    lto.ui.setDropDownValue(app, app.SolveLevelDropDown, value);

    if ~isempty(app.Config)
        app.Config.solver.solve_level = string(value);
    end
end
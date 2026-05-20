function syncSolveLevelFromTopBar(app)
%SYNCSOLVELEVELFROMTOPBAR Copy top-bar solve level to solver tab.

    value = app.SolveLevelDropDown.Value;
    lto.ui.setDropDownValue(app, app.SolverLevelDropDown, value);

    if ~isempty(app.Config)
        app.Config.solver.solve_level = string(value);
    end
end
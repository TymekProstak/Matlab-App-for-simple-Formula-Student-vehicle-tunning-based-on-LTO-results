function syncModelFromSolver(app)
%SYNCMODELFROMSOLVER Copy solver-tab model choice to top bar.

    value = app.LTOModeDropDown.Value;
    lto.ui.setDropDownValue(app, app.VehicleModelDropDown, value);

    if ~isempty(app.Config)
        app.Config.solver.lto_mode = string(value);
    end
end
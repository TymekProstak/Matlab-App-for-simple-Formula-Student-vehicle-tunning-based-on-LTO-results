function syncModelFromTopBar(app)
%SYNCMODELFROMTOPBAR Copy top-bar model choice to solver tab.

    value = app.VehicleModelDropDown.Value;
    lto.ui.setDropDownValue(app, app.LTOModeDropDown, value);

    if ~isempty(app.Config)
        app.Config.solver.lto_mode = string(value);
    end
end
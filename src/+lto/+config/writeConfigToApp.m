function writeConfigToApp(app, cfg)
%WRITECONFIGTOAPP Write configuration structure values into GUI fields.

    if nargin < 2 || isempty(cfg)
        cfg = app.Config;
    end

    try
        % ============================================================
        % Vehicle
        % ============================================================

        app.MassEditField.Value = cfg.vehicle.m;

        app.WheelbaseEditField.Value = cfg.vehicle.L;
        app.LfEditField.Value = cfg.vehicle.lf;
        app.LrEditField.Value = cfg.vehicle.lr;
        app.EditField3.Value = cfg.vehicle.h_CG;   % CG height

        app.FrontTrackEditField.Value = cfg.vehicle.tf;
        app.RearTrackEditField.Value = cfg.vehicle.tr;

        app.YawInertiaEditField.Value = cfg.vehicle.Iz;
        app.RollInertiaEditField.Value = cfg.vehicle.Iphi;
        app.PitchInertiaEditField.Value = cfg.vehicle.Itheta;

        app.DragCoefficentEditField.Value = cfg.vehicle.aero.Cd;
        app.FrontDownforceCoefficientEditField.Value = cfg.vehicle.aero.Cl_front;
        app.RearDownforceCoefficeintEditField.Value = cfg.vehicle.aero.Cl_rear;
        app.RollingResistanceEditField.Value = cfg.vehicle.aero.Cr;

        % ============================================================
        % Tire
        % ============================================================

        app.NominalLoadEditField.Value = cfg.tire.Fz0;
        app.TireRadiusEditField.Value = cfg.tire.R;
        app.RelaxationLengthEditField.Value = cfg.tire.relaxation_length_lateral;

        app.BEditField.Value = cfg.tire.smf_lateral.B;
        app.CEditField.Value = cfg.tire.smf_lateral.C;
        app.DEditField.Value = cfg.tire.smf_lateral.D;

        app.pCy1_EditField.Value = cfg.tire.mf52_lateral.pCy1;
        app.pDy1_EditField.Value = cfg.tire.mf52_lateral.pDy1;
        app.pDy_2_EditField.Value = cfg.tire.mf52_lateral.pDy2;
        app.lambday_EditField.Value = cfg.tire.mf52_lateral.lambda_y;

        app.pEy1_EditField.Value = cfg.tire.mf52_lateral.pEy1;
        app.pEy2_EditField.Value = cfg.tire.mf52_lateral.pEy2;
        app.pEy3_EditField.Value = cfg.tire.mf52_lateral.pEy3;

        app.pKy1_EditField.Value = cfg.tire.mf52_lateral.pKy1;
        app.pKy2_EditField.Value = cfg.tire.mf52_lateral.pKy2;
        app.pKy4_EditField.Value = cfg.tire.mf52_lateral.pKy4;

        % Longitudinal panel is used as mu_x(Fz) limit, not full Fx model.
        app.LambdaxEditField.Value = cfg.tire.longitudinal_limit.lambda_x;
        app.pCx1EditField.Value = cfg.tire.longitudinal_limit.pCx1;
        app.pDx1EditField.Value = cfg.tire.longitudinal_limit.pDx1;
        app.pDx2EditField.Value = cfg.tire.longitudinal_limit.pDx2;

        % ============================================================
        % Suspension
        % ============================================================

        app.HRollFrontEditField.Value = cfg.suspension.h_RCf;
        app.HRollRearEditField.Value = cfg.suspension.h_RCr;

        app.WhellRateFrontEditField.Value = cfg.suspension.kw_f;
        app.WhellRateRearEditField.Value = cfg.suspension.kw_r;

        app.ARBFrontEditField.Value = cfg.suspension.K_ARB_f;
        app.ARBRearEditField.Value = cfg.suspension.K_ARB_r;

        app.SprungMassFrontEditField.Value = cfg.suspension.ms_f;
        app.SprungMassRearEditField.Value = cfg.suspension.ms_r;

        app.DampingRatioLateralEditField.Value = cfg.suspension.zeta_phi;
        app.DamingRatioLongitudalEditField.Value = cfg.suspension.zeta_theta;

        app.AntiSquatEditField.Value = cfg.suspension.anti_squat;
        app.AntiDiveEditField.Value = cfg.suspension.anti_dive;

        % ============================================================
        % Drivetrain
        % ============================================================

        app.MaxDriveTorqueEditField.Value = cfg.drivetrain.max_drive_torque;
        app.MaxBrakeTorqueEditField.Value = cfg.drivetrain.max_brake_torque;

        app.MaxDrivePowerEditField.Value = cfg.drivetrain.max_drive_power_kW;
        app.MaxBrakePowerEditField.Value = cfg.drivetrain.max_brake_power_kW;

        app.FirstOrderTimeConstantEditField.Value = cfg.drivetrain.first_order_time_constant;

        % ============================================================
        % Steering
        % ============================================================

        app.MaxSteerignAngleEditField.Value = cfg.steering.max_steering_angle_rad;
        app.NaturalFrequancyEditField.Value = cfg.steering.natural_frequency_radps;
        app.DampingRatioEditField.Value = cfg.steering.damping_ratio;

        % ============================================================
        % Solver
        % ============================================================

        setDropDownValue(app, app.VehicleModelDropDown, cfg.solver.lto_mode);
        setDropDownValue(app, app.LTOModeDropDown, cfg.solver.lto_mode);

        setDropDownValue(app, app.SolveLevelDropDown, cfg.solver.solve_level);
        setDropDownValue(app, app.SolverLevelDropDown, cfg.solver.solve_level);

        setDropDownValue(app, app.InitialguessstrategyDropDown, cfg.solver.initial_guess_strategy);

        app.DebugNumberPointsEditField.Value = cfg.solver.discretization.N_debug;
        app.PrewievNumberPointsEditField.Value = cfg.solver.discretization.N_preview;
        app.FinalNumberPointsEditField.Value = cfg.solver.discretization.N_final;

        setDropDownValue(app, app.IntegratorDropDown, cfg.solver.discretization.integrator);

        app.InitalSpeedEditField.Value = cfg.solver.backward_forward.initial_speed;
        app.MaximalSpeedEditField.Value = cfg.solver.backward_forward.max_speed;
        app.SafetyFactoEditField.Value = cfg.solver.backward_forward.safety_factor;

        app.TolDebugEditField.Value = cfg.solver.ipopt.tol_debug;
        app.TolPreviewEditField.Value = cfg.solver.ipopt.tol_preview;
        app.TolFinalEditField.Value = cfg.solver.ipopt.tol_final;

        app.MaxIterationsEditField.Value = cfg.solver.ipopt.max_iterations;
        app.PrintLevelEditField.Value = cfg.solver.ipopt.print_level;
        app.AcceptableTolEditField.Value = cfg.solver.ipopt.acceptable_tol;

        app.SubstepsofIntegrationEditField.Value = cfg.solver.integrator.substeps;
        
        if isfield(cfg.solver, "periodic_track")
            if logical(cfg.solver.periodic_track)
                app.IstrackclosedperiodicDropDown.Value = 'true';
            else
                app.IstrackclosedperiodicDropDown.Value = 'false';
            end
        else
            app.IstrackclosedperiodicDropDown.Value = 'true';
        end
        % ============================================================
        % Bounds
        % ============================================================

        setDropDownValue(app, app.TrackWidithDropDown, cfg.bounds.track_width_mode);

        app.MinimalTrackMarginEditField.Value = cfg.bounds.min_track_margin;

        app.MinimalSpeedBoundEditField.Value = cfg.bounds.vx_min;
        app.MaximalSpeedBoundEditField.Value = cfg.bounds.vx_max;
        app.BetaAngleEditField.Value = cfg.bounds.beta_max_rad;

        app.MaxTorqueCommandRateEditField.Value = cfg.bounds.max_normalized_torque_rate;
        app.MaxSteeringCommandRateEditField.Value = cfg.bounds.max_steering_rate_radps;

        % ============================================================
        % Cost
        % ============================================================

        app.SpeedCostEditField.Value = cfg.cost.speed;
        app.BetaCostEditField.Value = cfg.cost.beta;

        app.TorqueRateCostEditField.Value = cfg.cost.torque_rate;
        app.SteerRateCostEditField.Value = cfg.cost.steer_rate;

        app.TrackViolationSlackCostEditField.Value = cfg.cost.track_violation_slack;
        app.FrictionElipseSlackCostEditField.Value = cfg.cost.friction_ellipse_slack;
        app.PowerSlackCostEditField.Value = cfg.cost.power_slack;

        % ============================================================
        % Files
        % ============================================================

        app.TrackfilepathEditField.Value = char(cfg.files.track_path);
        app.ConfigurationfilepathEditField.Value = char(cfg.files.config_path);
        app.InitialguesspathEditField.Value = char(cfg.files.initial_guess_path);
        app.OutputfolderpathEditField.Value = char(cfg.files.output_folder);

        % ============================================================
        % Store config and update GUI state
        % ============================================================

        app.Config = cfg;

        app.ConfignotloadedyetLabel.Text = "Config applied to GUI";

        lto.ui.setStatus(app, "Config applied", "ready");
        lto.ui.appendLog(app, "Config values written to GUI.", "INFO");

    catch ME
        lto.ui.setStatus(app, "Config apply error", "error");
        lto.ui.appendLog(app, "Failed to write config to GUI: " + ME.message, "ERROR");
        rethrow(ME);
    end
end

function setDropDownValue(app, dropdown, value)
%SETDROPDOWNVALUE Set dropdown value if it exists in dropdown items.

    desired = string(value);
    items = string(dropdown.Items);

    for i = 1:numel(items)
        if items(i) == desired
            dropdown.Value = char(items(i));
            return;
        end
    end

    lto.ui.appendLog( ...
        app, ...
        "Dropdown value not found: " + desired + ". Keeping current value.", ...
        "WARNING" ...
    );
end


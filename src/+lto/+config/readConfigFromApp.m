function cfg = readConfigFromApp(app)
%READCONFIGFROMAPP Read current GUI values into a config structure.

    % Start from existing config if available, so fields not exposed in GUI
    % are not lost, for example constants or numerical safeguards.
    if isempty(app.Config)
        cfg = struct();
    else
        cfg = app.Config;
    end

    % ============================================================
    % Meta / constants / numerics
    % ============================================================

    if ~isfield(cfg, "meta")
        cfg.meta.version = "0.1";
        cfg.meta.description = "Formula Student LTO configuration";
    end

    if ~isfield(cfg, "constants")
        cfg.constants.g = 9.81;
    end

    if ~isfield(cfg, "numerics")
        cfg.numerics.Fz_min = 50.0;
        cfg.numerics.eps_vx = 0.1;
        cfg.numerics.eps_kappa = 1e-4;
    end

    % ============================================================
    % Vehicle
    % ============================================================

    cfg.vehicle.m = app.MassEditField.Value;

    cfg.vehicle.L = app.WheelbaseEditField.Value;
    cfg.vehicle.lf = app.LfEditField.Value;
    cfg.vehicle.lr = app.LrEditField.Value;
    cfg.vehicle.h_CG = app.EditField3.Value;

    cfg.vehicle.tf = app.FrontTrackEditField.Value;
    cfg.vehicle.tr = app.RearTrackEditField.Value;

    cfg.vehicle.Iz = app.YawInertiaEditField.Value;
    cfg.vehicle.Iphi = app.RollInertiaEditField.Value;
    cfg.vehicle.Itheta = app.PitchInertiaEditField.Value;

    cfg.vehicle.aero.Cd = app.DragCoefficentEditField.Value;
    cfg.vehicle.aero.Cl_front = app.FrontDownforceCoefficientEditField.Value;
    cfg.vehicle.aero.Cl_rear = app.RearDownforceCoefficeintEditField.Value;
    cfg.vehicle.aero.Cr = app.RollingResistanceEditField.Value;
    cfg.solver.periodic_track = readBooleanDropDown(app.IstrackclosedperiodicDropDown);

    % ============================================================
    % Tire
    % ============================================================

    cfg.tire.Fz0 = app.NominalLoadEditField.Value;
    cfg.tire.R = app.TireRadiusEditField.Value;
    cfg.tire.relaxation_length_lateral = app.RelaxationLengthEditField.Value;

    cfg.tire.smf_lateral.B = app.BEditField.Value;
    cfg.tire.smf_lateral.C = app.CEditField.Value;
    cfg.tire.smf_lateral.D = app.DEditField.Value;

    cfg.tire.mf52_lateral.pCy1 = app.pCy1_EditField.Value;
    cfg.tire.mf52_lateral.pDy1 = app.pDy1_EditField.Value;
    cfg.tire.mf52_lateral.pDy2 = app.pDy_2_EditField.Value;
    cfg.tire.mf52_lateral.lambda_y = app.lambday_EditField.Value;

    cfg.tire.mf52_lateral.pEy1 = app.pEy1_EditField.Value;
    cfg.tire.mf52_lateral.pEy2 = app.pEy2_EditField.Value;
    cfg.tire.mf52_lateral.pEy3 = app.pEy3_EditField.Value;

    cfg.tire.mf52_lateral.pKy1 = app.pKy1_EditField.Value;
    cfg.tire.mf52_lateral.pKy2 = app.pKy2_EditField.Value;
    cfg.tire.mf52_lateral.pKy4 = app.pKy4_EditField.Value;

    % Longitudinal tire panel is used only for mu_x(Fz) limit.
    cfg.tire.longitudinal_limit.lambda_x = app.LambdaxEditField.Value;
    cfg.tire.longitudinal_limit.pCx1 = app.pCx1EditField.Value;
    cfg.tire.longitudinal_limit.pDx1 = app.pDx1EditField.Value;
    cfg.tire.longitudinal_limit.pDx2 = app.pDx2EditField.Value;

    % ============================================================
    % Suspension
    % ============================================================

    cfg.suspension.h_RCf = app.HRollFrontEditField.Value;
    cfg.suspension.h_RCr = app.HRollRearEditField.Value;

    cfg.suspension.kw_f = app.WhellRateFrontEditField.Value;
    cfg.suspension.kw_r = app.WhellRateRearEditField.Value;

    cfg.suspension.K_ARB_f = app.ARBFrontEditField.Value;
    cfg.suspension.K_ARB_r = app.ARBRearEditField.Value;

    cfg.suspension.ms_f = app.SprungMassFrontEditField.Value;
    cfg.suspension.ms_r = app.SprungMassRearEditField.Value;

    cfg.suspension.zeta_phi = app.DampingRatioLateralEditField.Value;
    cfg.suspension.zeta_theta = app.DamingRatioLongitudalEditField.Value;

    cfg.suspension.anti_squat = app.AntiSquatEditField.Value;
    cfg.suspension.anti_dive = app.AntiDiveEditField.Value;

    % ============================================================
    % Drivetrain
    % ============================================================

    cfg.drivetrain.max_drive_torque = app.MaxDriveTorqueEditField.Value;
    cfg.drivetrain.max_brake_torque = app.MaxBrakeTorqueEditField.Value;

    cfg.drivetrain.max_drive_power_kW = app.MaxDrivePowerEditField.Value;
    cfg.drivetrain.max_brake_power_kW = app.MaxBrakePowerEditField.Value;

    cfg.drivetrain.first_order_time_constant = app.FirstOrderTimeConstantEditField.Value;

    % ============================================================
    % Steering
    % ============================================================

    cfg.steering.max_steering_angle_rad = app.MaxSteerignAngleEditField.Value;
    cfg.steering.natural_frequency_radps = app.NaturalFrequancyEditField.Value;
    cfg.steering.damping_ratio = app.DampingRatioEditField.Value;

    % ============================================================
    % Solver
    % ============================================================

    cfg.solver.lto_mode = string(app.LTOModeDropDown.Value);
    cfg.solver.solve_level = string(app.SolverLevelDropDown.Value);
    cfg.solver.initial_guess_strategy = string(app.InitialguessstrategyDropDown.Value);

    cfg.solver.discretization.N_debug = app.DebugNumberPointsEditField.Value;
    cfg.solver.discretization.N_preview = app.PrewievNumberPointsEditField.Value;
    cfg.solver.discretization.N_final = app.FinalNumberPointsEditField.Value;
    cfg.solver.discretization.integrator = string(app.IntegratorDropDown.Value);

    cfg.solver.backward_forward.initial_speed = app.InitalSpeedEditField.Value;
    cfg.solver.backward_forward.max_speed = app.MaximalSpeedEditField.Value;
    cfg.solver.backward_forward.safety_factor = app.SafetyFactoEditField.Value;

    cfg.solver.ipopt.tol_debug = app.TolDebugEditField.Value;
    cfg.solver.ipopt.tol_preview = app.TolPreviewEditField.Value;
    cfg.solver.ipopt.tol_final = app.TolFinalEditField.Value;
    cfg.solver.ipopt.max_iterations = app.MaxIterationsEditField.Value;
    cfg.solver.ipopt.print_level = app.PrintLevelEditField.Value;
    cfg.solver.ipopt.acceptable_tol = app.AcceptableTolEditField.Value;
    cfg.solver.integrator.substeps = app.SubstepsofIntegrationEditField.Value;

    % ============================================================
    % Bounds
    % ============================================================

    cfg.bounds.track_width_mode = string(app.TrackWidithDropDown.Value);
    cfg.bounds.min_track_margin = app.MinimalTrackMarginEditField.Value;

    cfg.bounds.vx_min = app.MinimalSpeedBoundEditField.Value;
    cfg.bounds.vx_max = app.MaximalSpeedBoundEditField.Value;
    cfg.bounds.beta_max_rad = app.BetaAngleEditField.Value;

    cfg.bounds.max_normalized_torque_rate = app.MaxTorqueCommandRateEditField.Value;
    cfg.bounds.max_steering_rate_radps = app.MaxSteeringCommandRateEditField.Value;

    % ============================================================
    % Cost
    % ============================================================

    cfg.cost.speed = app.SpeedCostEditField.Value;
    cfg.cost.beta = app.BetaCostEditField.Value;

    cfg.cost.torque_rate = app.TorqueRateCostEditField.Value;
    cfg.cost.steer_rate = app.SteerRateCostEditField.Value;

    cfg.cost.track_violation_slack = app.TrackViolationSlackCostEditField.Value;
    cfg.cost.friction_ellipse_slack = app.FrictionElipseSlackCostEditField.Value;
    cfg.cost.power_slack = app.PowerSlackCostEditField.Value;

  
    % Store latest config in app.
    app.Config = cfg;

    lto.ui.appendLog(app, "Config read from GUI.", "INFO");
end

function value = readBooleanDropDown(dropdown)
%READBOOLEANDROPDOWN Read true/false dropdown as scalar logical.

    txt = lower(strtrim(string(dropdown.Value)));

    if txt == "true"
        value = true;
    elseif txt == "false"
        value = false;
    else
        error("LTO:Config:InvalidBooleanDropDown", ...
              "Expected true or false, got: %s", txt);
    end
end
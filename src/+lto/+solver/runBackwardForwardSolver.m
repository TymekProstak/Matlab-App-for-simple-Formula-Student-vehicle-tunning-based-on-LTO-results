function solution = runBackwardForwardSolver(solverTrack, cfg)
%RUNBACKWARDFORWARDSOLVER Build a centerline backward-forward solution.

    % ============================================================
    % Basic data
    % ============================================================

    s = solverTrack.s(:);
    ds = solverTrack.ds(:);
    kappa = solverTrack.kappa(:);

    xGlobal = solverTrack.center.x(:);
    yGlobal = solverTrack.center.y(:);
    psiGlobal = solverTrack.psi(:);

    N = numel(s);

    m = cfg.vehicle.m;
    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    R = cfg.tire.R;
    g = cfg.constants.g;

    epsVx = cfg.numerics.eps_vx;
    epsKappa = cfg.numerics.eps_kappa;

    safety = cfg.solver.backward_forward.safety_factor;
    vxMax = cfg.solver.backward_forward.max_speed;
    vx0 = cfg.solver.backward_forward.initial_speed;

    % ============================================================
    % Lateral speed limit from curvature
    % ============================================================

    vxLimit = zeros(N, 1);

    for i = 1:N
        vxLimit(i) = computeCurvatureSpeedLimit( ...
            abs(kappa(i)), ...
            vxMax, ...
            cfg, ...
            safety, ...
            epsKappa ...
        );
    end

    vx = vxLimit;
    vx(1) = min(vx(1), vx0);

    % ============================================================
    % Forward pass: acceleration limit
    % ============================================================

    for i = 1:N-1
        [axDriveMax, ~] = computeLongitudinalAccelLimits(vx(i), cfg, safety);

        vxPossible = sqrt(vx(i)^2 + 2.0 * axDriveMax * ds(i));
        vx(i+1) = min(vx(i+1), vxPossible);
    end

    % ============================================================
    % Backward pass: braking limit
    % ============================================================

    for i = N-1:-1:1
        [~, axBrakeMax] = computeLongitudinalAccelLimits(vx(i+1), cfg, safety);

        vxPossible = sqrt(vx(i+1)^2 + 2.0 * axBrakeMax * ds(i));
        vx(i) = min(vx(i), vxPossible);
    end

    % ============================================================
    % Reconstruct time
    % ============================================================

    dt = zeros(N-1, 1);

    for i = 1:N-1
        vxAvg = 0.5 * (vx(i) + vx(i+1));
        dt(i) = ds(i) / max(vxAvg, epsVx);
    end

    t = [0.0; cumsum(dt)];

    % ============================================================
    % Accelerations from final vx profile
    % ============================================================

    ax = zeros(N, 1);

    for i = 1:N-1
        ax(i) = (vx(i+1)^2 - vx(i)^2) / max(2.0 * ds(i), 1e-9);
    end

    ax(end) = ax(end-1);

    ay = vx.^2 .* kappa;

    axPrev = [ax(1); ax(1:end-1)];
    ayPrev = [ay(1); ay(1:end-1)];

    % ============================================================
    % Normal loads and tire usage
    % ============================================================

    Fz = zeros(4, N);

    for i = 1:N
        Fz(:, i) = computeNormalLoads(vx(i), cfg);
    end

    tireUsage = computeTireUsage(vx, ax, ay, Fz, cfg);

    % ============================================================
    % Approximate steering and rear torque commands
    % ============================================================

    delta = computeSteeringFeedforward(vx, ay, kappa, Fz, cfg);
    deltaCmd = delta;

    deltaDot = gradientWithTime(delta, t);

    MRear = computeRearTorqueCommand(vx, ax, cfg);
    MCmd = MRear;

    MCmdDot = diff(MCmd) ./ max(dt, 1e-9);
    deltaCmdDot = diff(deltaCmd) ./ max(dt, 1e-9);

    U = [
        MCmdDot.';
        deltaCmdDot.'
    ];

    % ============================================================
    % NLP state vector placeholder for full dynamic-suspension format
    % ============================================================

    eY = zeros(N, 1);
    ePsi = zeros(N, 1);
    vy = zeros(N, 1);
    yawRate = vx .* kappa;

    X = [
        eY.';
        ePsi.';
        vx.';
        vy.';
        yawRate.';

        MRear.';
        MCmd.';

        delta.';
        deltaDot.';
        deltaCmd.';

        axPrev.';
        ayPrev.';

        Fz(1, :);
        Fz(2, :);
        Fz(3, :);
        Fz(4, :)
    ];

    % ============================================================
    % Output solution
    % ============================================================

    solution = struct();

    solution.type = "BackwardForward";
    solution.status = "success";
    solution.message = "Backward-forward centerline solution generated.";

    solution.s = s;
    solution.t = t;
    solution.lap_time = t(end);

    solution.X = X;
    solution.U = U;

    solution.state_names = { ...
        'e_y', ...
        'e_psi', ...
        'vx', ...
        'vy', ...
        'yaw_rate', ...
        'M_rear', ...
        'M_cmd', ...
        'delta', ...
        'delta_dot', ...
        'delta_cmd', ...
        'ax_prev', ...
        'ay_prev', ...
        'Fz_FL', ...
        'Fz_FR', ...
        'Fz_RL', ...
        'Fz_RR' ...
    };

    solution.control_names = { ...
        'M_cmd_dot', ...
        'delta_cmd_dot' ...
    };

    solution.global.x = xGlobal;
    solution.global.y = yGlobal;
    solution.global.psi = psiGlobal;

    solution.track.s_ref = s;
    solution.track.kappa = kappa;
    solution.track.width_left = solverTrack.width.left;
    solution.track.width_right = solverTrack.width.right;
    solution.track.width_total = solverTrack.width.total;

    solution.wheel_names = {'FL', 'FR', 'RL', 'RR'};
    solution.normal_loads = Fz;

    solution.tire_usage.x = tireUsage.x;
    solution.tire_usage.y = tireUsage.y;
    solution.tire_usage.total = tireUsage.total;
end


function vxLimit = computeCurvatureSpeedLimit(kappaAbs, vxMax, cfg, safety, epsKappa)
%COMPUTECURVATURESPEEDLIMIT Solve simple fixed-point speed limit.

    if kappaAbs < epsKappa
        vxLimit = vxMax;
        return;
    end

    vxLimit = vxMax;

    for iter = 1:6
        Fz = computeNormalLoads(vxLimit, cfg);
        muY = computeMuY(Fz, cfg);

        FyMaxTotal = sum(muY .* Fz);
        ayMax = safety * FyMaxTotal / cfg.vehicle.m;

        vxNew = sqrt(ayMax / max(kappaAbs, epsKappa));
        vxLimit = min(vxMax, vxNew);
    end
end


function [axDriveMax, axBrakeMax] = computeLongitudinalAccelLimits(vx, cfg, safety)
%COMPUTELONGITUDINALACCELLIMITS Compute rear-axle drive/brake acceleration limits.

    m = cfg.vehicle.m;
    R = cfg.tire.R;

    Fz = computeNormalLoads(vx, cfg);
    muX = computeMuX(Fz, cfg);

    rearFrictionLimit = muX(3) * Fz(3) + muX(4) * Fz(4);

    driveForceTorque = cfg.drivetrain.max_drive_torque / R;
    brakeForceTorque = cfg.drivetrain.max_brake_torque / R;

    drivePowerW = 1000.0 * cfg.drivetrain.max_drive_power_kW;
    brakePowerW = 1000.0 * cfg.drivetrain.max_brake_power_kW;

    driveForcePower = drivePowerW / max(vx, 0.5);
    brakeForcePower = brakePowerW / max(vx, 0.5);

    driveForceMax = min([rearFrictionLimit, driveForceTorque, driveForcePower]);
    brakeForceMax = min([rearFrictionLimit, brakeForceTorque, brakeForcePower]);

    axDriveMax = safety * driveForceMax / m;
    axBrakeMax = safety * brakeForceMax / m;
end


function Fz = computeNormalLoads(vx, cfg)
%COMPUTENORMALLOADS Static + aero normal loads, no load transfer.

    m = cfg.vehicle.m;
    g = cfg.constants.g;

    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    frontShare = lr / L;
    rearShare = lf / L;

    FzFrontStatic = m * g * frontShare;
    FzRearStatic = m * g * rearShare;

    FzFrontAero = max(0.0, cfg.vehicle.aero.Cl_front * vx^2);
    FzRearAero = max(0.0, cfg.vehicle.aero.Cl_rear * vx^2);

    FzFL = 0.5 * (FzFrontStatic + FzFrontAero);
    FzFR = 0.5 * (FzFrontStatic + FzFrontAero);

    FzRL = 0.5 * (FzRearStatic + FzRearAero);
    FzRR = 0.5 * (FzRearStatic + FzRearAero);

    Fz = [FzFL; FzFR; FzRL; FzRR];

    if isfield(cfg, "numerics") && isfield(cfg.numerics, "Fz_min")
        Fz = max(Fz, cfg.numerics.Fz_min);
    end
end


function muX = computeMuX(Fz, cfg)
%COMPUTEMUX Compute load-dependent longitudinal peak friction.

    Fz0 = cfg.tire.Fz0;

    lambdaX = cfg.tire.longitudinal_limit.lambda_x;
    pDx1 = cfg.tire.longitudinal_limit.pDx1;
    pDx2 = cfg.tire.longitudinal_limit.pDx2;

    dfz = (Fz - Fz0) ./ max(Fz0, 1e-9);

    muX = lambdaX .* (pDx1 + pDx2 .* dfz);
    muX = max(muX, 0.05);
end


function muY = computeMuY(Fz, cfg)
%COMPUTEMUY Compute load-dependent lateral peak friction.

    Fz0 = cfg.tire.Fz0;

    lambdaY = cfg.tire.mf52_lateral.lambda_y;
    pDy1 = cfg.tire.mf52_lateral.pDy1;
    pDy2 = cfg.tire.mf52_lateral.pDy2;

    dfz = (Fz - Fz0) ./ max(Fz0, 1e-9);

    muY = lambdaY .* (pDy1 + pDy2 .* dfz);
    muY = max(muY, 0.05);
end


function tireUsage = computeTireUsage(vx, ax, ay, Fz, cfg)
%COMPUTETIREUSAGE Estimate normalized x/y tire usage per wheel.

    N = numel(vx);

    m = cfg.vehicle.m;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    L = cfg.vehicle.L;

    FxWheel = zeros(4, N);
    FyWheel = zeros(4, N);

    for i = 1:N
        FxTotal = computeRequiredLongitudinalForce(vx(i), ax(i), cfg);

        % RWD: rear wheels only.
        FxWheel(3, i) = 0.5 * FxTotal;
        FxWheel(4, i) = 0.5 * FxTotal;

        % Simple steady-state lateral force split.
        FyFrontAxle = m * ay(i) * lr / L;
        FyRearAxle = m * ay(i) * lf / L;

        FyWheel(1, i) = 0.5 * FyFrontAxle;
        FyWheel(2, i) = 0.5 * FyFrontAxle;
        FyWheel(3, i) = 0.5 * FyRearAxle;
        FyWheel(4, i) = 0.5 * FyRearAxle;
    end

    usageX = zeros(4, N);
    usageY = zeros(4, N);

    for i = 1:N
        muX = computeMuX(Fz(:, i), cfg);
        muY = computeMuY(Fz(:, i), cfg);

        usageX(:, i) = FxWheel(:, i) ./ max(muX .* Fz(:, i), 1e-9);
        usageY(:, i) = FyWheel(:, i) ./ max(muY .* Fz(:, i), 1e-9);
    end

    tireUsage.x = usageX;
    tireUsage.y = usageY;
    tireUsage.total = sqrt(usageX.^2 + usageY.^2);
end


function delta = computeSteeringFeedforward(vx, ay, kappa, Fz, cfg)
%COMPUTESTEERINGFEEDFORWARD Kinematic steering with simple understeer term.

    N = numel(vx);

    m = cfg.vehicle.m;
    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    delta = zeros(N, 1);

    for i = 1:N
        Cf = estimateFrontCorneringStiffness(Fz(:, i), cfg);
        Cr = estimateRearCorneringStiffness(Fz(:, i), cfg);

        Kus = (m / L) * (lr / max(Cf, 1e-9) - lf / max(Cr, 1e-9));

        deltaKinematic = atan(L * kappa(i));
        deltaUndersteer = Kus * ay(i);

        delta(i) = deltaKinematic + deltaUndersteer;
    end

    deltaMax = cfg.steering.max_steering_angle_rad;
    delta = max(min(delta, deltaMax), -deltaMax);
end


function Cf = estimateFrontCorneringStiffness(Fz, cfg)
%ESTIMATEFRONTCORNERINGSTIFFNESS Rough MF-like front axle stiffness.

    KyFL = estimateTireCorneringStiffness(Fz(1), cfg);
    KyFR = estimateTireCorneringStiffness(Fz(2), cfg);

    Cf = KyFL + KyFR;
end


function Cr = estimateRearCorneringStiffness(Fz, cfg)
%ESTIMATEREARCORNERINGSTIFFNESS Rough MF-like rear axle stiffness.

    KyRL = estimateTireCorneringStiffness(Fz(3), cfg);
    KyRR = estimateTireCorneringStiffness(Fz(4), cfg);

    Cr = KyRL + KyRR;
end


function Ky = estimateTireCorneringStiffness(Fz, cfg)
%ESTIMATETIRECORNERINGSTIFFNESS Approximate lateral stiffness from MF-like fields.

    Fz0 = cfg.tire.Fz0;

    pKy1 = cfg.tire.mf52_lateral.pKy1;
    pKy2 = cfg.tire.mf52_lateral.pKy2;
    pCy1 = cfg.tire.mf52_lateral.pCy1;

    loadRatio = Fz / max(Fz0, 1e-9);

    Ky = (pKy1 / max(pCy1, 1e-9)) * Fz0 * sin(2.0 * atan(loadRatio / max(pKy2, 1e-9)));

    Ky = max(Ky, 1e-6);
end


function MRear = computeRearTorqueCommand(vx, ax, cfg)
%COMPUTEREARTORQUECOMMAND Compute rear torque needed for requested ax.

    N = numel(vx);
    R = cfg.tire.R;

    MRear = zeros(N, 1);

    for i = 1:N
        FxRequired = computeRequiredLongitudinalForce(vx(i), ax(i), cfg);
        MRear(i) = FxRequired * R;

        MRear(i) = clampRearTorqueByLimits(MRear(i), vx(i), cfg);
    end
end


function FxRequired = computeRequiredLongitudinalForce(vx, ax, cfg)
%COMPUTEREQUIREDLONGITUDINALFORCE Force needed at tires for requested ax.

    m = cfg.vehicle.m;
    g = cfg.constants.g;

    Fdrag = cfg.vehicle.aero.Cd * vx^2;
    Frr = cfg.vehicle.aero.Cr * m * g;

    FxRequired = m * ax + Fdrag + Frr;
end


function M = clampRearTorqueByLimits(M, vx, cfg)
%CLAMPREARTORQUEBYLIMITS Clamp torque by torque and power limits.

    R = cfg.tire.R;

    if M >= 0
        MLimitTorque = cfg.drivetrain.max_drive_torque;
        MLimitPower = 1000.0 * cfg.drivetrain.max_drive_power_kW * R / max(vx, 0.5);

        MLimit = min(MLimitTorque, MLimitPower);
        M = min(M, MLimit);

    else
        MLimitTorque = cfg.drivetrain.max_brake_torque;
        MLimitPower = 1000.0 * cfg.drivetrain.max_brake_power_kW * R / max(vx, 0.5);

        MLimit = min(MLimitTorque, MLimitPower);
        M = max(M, -MLimit);
    end
end


function yDot = gradientWithTime(y, t)
%GRADIENTWITHTIME Compute dy/dt with safe time vector.

    if numel(y) ~= numel(t)
        error("LTO:Gradient:SizeMismatch", ...
              "Signal and time vectors must have the same length.");
    end

    if numel(y) < 2
        yDot = zeros(size(y));
        return;
    end

    yDot = gradient(y, t);

    yDot(~isfinite(yDot)) = 0.0;
end
function solution = runPointMassSolver(solverTrack, cfg, initialGuess)
%RUNPOINTMASSSOLVER Solve free-line point-mass NLP using CasADi/IPOPT.
%
% Internal NLP states:
%   e_y, e_psi, vx, M_cmd, M_rear, t
%
% Internal NLP controls:
%   M_cmd_dot, kappa_vehicle
%
% Notes:
%   - kappa_vehicle is a pseudo-curvature decision variable.
%   - Steering angle/rate constraints are NOT imposed in this point-mass NLP.
%   - Steering-related solution fields are reconstructed only for seeding later models.

    if nargin < 3
        initialGuess = [];
    end

    import casadi.*

    method = string(cfg.solver.discretization.integrator);

    if method ~= "Euler" && method ~= "RK4"
        error("LTO:Solver:IntegratorNotImplemented", ...
              "Integrator not implemented: %s. Use Euler or RK4.", method);
    end

    substeps = getIntegratorSubsteps(cfg);

    N = solverTrack.N;
    dsVec = solverTrack.ds(:);

    kappaRef = solverTrack.kappa(:);
    widthLeft = solverTrack.width.left(:);
    widthRight = solverTrack.width.right(:);

    nx = 6;
    nu = 2;

    IDX_EY = 1;
    IDX_EPSI = 2;
    IDX_VX = 3;
    IDX_MCMD = 4;
    IDX_MREAR = 5;
    IDX_T = 6;

    IDX_MCMD_DOT = 1;
    IDX_KAPPA_VEH = 2;

    opti = Opti();

    X = opti.variable(nx, N);
    U = opti.variable(nu, N - 1);

    % ============================================================
    % Bounds
    % ============================================================

    vxMin = cfg.bounds.vx_min;
    vxMax = cfg.bounds.vx_max;

    MDriveMax = cfg.drivetrain.max_drive_torque;
    MBrakeMax = cfg.drivetrain.max_brake_torque;

    maxTorque = max(MDriveMax, MBrakeMax);
    MCmdDotMax = cfg.bounds.max_normalized_torque_rate * maxTorque;

    margin = cfg.bounds.min_track_margin;
    trackMode = string(cfg.bounds.track_width_mode);

    vsMin = 1.0;

    % Speed bounds.
    opti.subject_to(X(IDX_VX, :) >= vxMin);
    opti.subject_to(X(IDX_VX, :) <= vxMax);

    % Torque state bounds.
    opti.subject_to(X(IDX_MCMD, :) >= -MBrakeMax);
    opti.subject_to(X(IDX_MCMD, :) <=  MDriveMax);

    opti.subject_to(X(IDX_MREAR, :) >= -MBrakeMax);
    opti.subject_to(X(IDX_MREAR, :) <=  MDriveMax);

    % Time starts at zero and must be nonnegative.
    opti.subject_to(X(IDX_T, 1) == 0.0);
    opti.subject_to(X(IDX_T, :) >= 0.0);

    % Control bound only for torque command rate.
    opti.subject_to(U(IDX_MCMD_DOT, :) >= -MCmdDotMax);
    opti.subject_to(U(IDX_MCMD_DOT, :) <=  MCmdDotMax);

    % Track bounds for lateral offset.
    for k = 1:N
        [eyMin, eyMax] = getTrackBounds(widthLeft(k), widthRight(k), margin, trackMode);

        opti.subject_to(X(IDX_EY, k) >= eyMin);
        opti.subject_to(X(IDX_EY, k) <= eyMax);
    end

    % ============================================================
    % Dynamics constraints
    % ============================================================

    for k = 1:N-1
        xk = X(:, k);
        uk = U(:, k);

        ds = dsVec(k);
        kappaK = kappaRef(k);

        xNext = integrateSpatialStep(xk, uk, ds, kappaK, cfg, method, substeps);

        opti.subject_to(X(:, k+1) == xNext);
    end

    % ============================================================
    % Tire friction, power, Frenet and progress constraints
    % ============================================================

    for k = 1:N
        ey = X(IDX_EY, k);
        epsi = X(IDX_EPSI, k);
        vx = X(IDX_VX, k);
        MRear = X(IDX_MREAR, k);

        if k < N
            kappaVeh = U(IDX_KAPPA_VEH, k);
        else
            kappaVeh = U(IDX_KAPPA_VEH, N-1);
        end

        Fz = computeNormalLoadsSymbolic(vx, cfg);
        usage = computePointMassTireUsageSymbolic(vx, MRear, kappaVeh, Fz, cfg);

        % NLP uses the real friction ellipse limit. BF safety factor is NOT used here.
        opti.subject_to(usage <= 1.0);

        power = MRear * vx / cfg.tire.R;

        opti.subject_to(power <= 1000.0 * cfg.drivetrain.max_drive_power_kW);
        opti.subject_to(power >= -1000.0 * cfg.drivetrain.max_brake_power_kW);

        % Avoid Frenet singularity.
        denom = 1.0 - kappaRef(k) * ey;
        opti.subject_to(denom >= 0.2);

        % Keep heading error reasonable. This is geometric, not steering-related.
        opti.subject_to(epsi >= -cfg.bounds.beta_max_rad);
        opti.subject_to(epsi <=  cfg.bounds.beta_max_rad);

        % Progress speed along reference centerline.
        vs = computeProgressSpeedSymbolic(ey, epsi, vx, kappaRef(k));

        opti.subject_to(vs >= vsMin);
    end

    % ============================================================
    % Boundary conditions
    % ============================================================

    isPeriodic = getPeriodicFlag(cfg);

    if isPeriodic
        opti.subject_to(X(IDX_EY, 1) == X(IDX_EY, N));
        opti.subject_to(X(IDX_EPSI, 1) == X(IDX_EPSI, N));
        opti.subject_to(X(IDX_VX, 1) == X(IDX_VX, N));
        opti.subject_to(X(IDX_MCMD, 1) == X(IDX_MCMD, N));
        opti.subject_to(X(IDX_MREAR, 1) == X(IDX_MREAR, N));
    else
        opti.subject_to(X(IDX_EY, 1) == 0.0);
        opti.subject_to(X(IDX_EPSI, 1) == 0.0);
        opti.subject_to(X(IDX_VX, 1) == cfg.solver.backward_forward.initial_speed);
        opti.subject_to(X(IDX_MCMD, 1) == 0.0);
        opti.subject_to(X(IDX_MREAR, 1) == 0.0);
    end

    % ============================================================
    % Objective
    % ============================================================

    objective = X(IDX_T, N);

    qSpeed = cfg.cost.speed;
    qTorqueRate = cfg.cost.torque_rate;
    qCurvatureSmooth = cfg.cost.steer_rate;

    trackLength = max(solverTrack.length, 1e-9);
    progressReward = 0.0;

    for k = 1:N-1
        ey = X(IDX_EY, k);
        epsi = X(IDX_EPSI, k);
        vx = X(IDX_VX, k);

        vs = computeProgressSpeedSymbolic(ey, epsi, vx, kappaRef(k));

        progressReward = progressReward + vs * dsVec(k) / trackLength;

        objective = objective + qTorqueRate * U(IDX_MCMD_DOT, k)^2;
    end

    for k = 1:N-2
        dkappa = U(IDX_KAPPA_VEH, k+1) - U(IDX_KAPPA_VEH, k);
        objective = objective + qCurvatureSmooth * dkappa^2;
    end

    % Negative progress/speed reward.
    objective = objective - qSpeed * progressReward;

    opti.minimize(objective);

    % ============================================================
    % Initial guess
    % ============================================================

    init = buildPointMassInitialGuess(solverTrack, cfg, initialGuess);

    opti.set_initial(X, init.X);
    opti.set_initial(U, init.U);

    % ============================================================
    % Solver options
    % ============================================================

    opts = struct();
    opts.print_time = false;
    opts.ipopt.print_level = cfg.solver.ipopt.print_level;
    opts.ipopt.max_iter = cfg.solver.ipopt.max_iterations;
    opts.ipopt.tol = getIpoptTolerance(cfg);
    opts.ipopt.acceptable_tol = cfg.solver.ipopt.acceptable_tol;

    opti.solver('ipopt', opts);

    % ============================================================
    % Solve
    % ============================================================

    try
        sol = opti.solve();

        Xsol = full(sol.value(X));
        Usol = full(sol.value(U));

        solution = buildCommonSolutionFromPointMass( ...
            solverTrack, cfg, Xsol, Usol, "success", "Point-mass NLP solved." ...
        );

    catch ME
        Xdebug = full(opti.debug.value(X));
        Udebug = full(opti.debug.value(U));

        solution = buildCommonSolutionFromPointMass( ...
            solverTrack, cfg, Xdebug, Udebug, "failed", ME.message ...
        );
    end
end


function dXds = pointMassSpatialRhs(x, u, kappaRef, cfg)
%POINTMASSSPATIALRHS Spatial RHS for free-line point-mass model.

    IDX_EY = 1;
    IDX_EPSI = 2;
    IDX_VX = 3;
    IDX_MCMD = 4;
    IDX_MREAR = 5;
    IDX_T = 6;

    IDX_MCMD_DOT = 1;
    IDX_KAPPA_VEH = 2;

    ey = x(IDX_EY);
    epsi = x(IDX_EPSI);
    vx = x(IDX_VX);
    MCmd = x(IDX_MCMD);
    MRear = x(IDX_MREAR);

    MCmdDot = u(IDX_MCMD_DOT);
    kappaVeh = u(IDX_KAPPA_VEH);

    denom = 1.0 - kappaRef * ey;

    dt_ds = denom / (vx * cos(epsi));

    Fx = MRear / cfg.tire.R;

    Fdrag = cfg.vehicle.aero.Cd * vx^2;
    Frr = cfg.vehicle.aero.Cr * cfg.vehicle.m * cfg.constants.g;

    ax = (Fx - Fdrag - Frr) / cfg.vehicle.m;

    tau = cfg.drivetrain.first_order_time_constant;

    dey_dt = vx * sin(epsi);
    depsi_ds = denom * kappaVeh / cos(epsi) - kappaRef;

    dvx_dt = ax;
    dMcmd_dt = MCmdDot;
    dMrear_dt = (MCmd - MRear) / tau;

    dXds = [
        dey_dt * dt_ds;
        depsi_ds;
        dvx_dt * dt_ds;
        dMcmd_dt * dt_ds;
        dMrear_dt * dt_ds;
        dt_ds
    ];
end


function xNext = integrateSpatialStep(x, u, ds, kappaRef, cfg, method, substeps)
%INTEGRATESPATIALSTEP Integrate spatial dynamics over one NLP interval.

    h = ds / substeps;
    xNext = x;

    for j = 1:substeps
        if method == "Euler"
            k1 = pointMassSpatialRhs(xNext, u, kappaRef, cfg);
            xNext = xNext + h * k1;

        elseif method == "RK4"
            k1 = pointMassSpatialRhs(xNext, u, kappaRef, cfg);
            k2 = pointMassSpatialRhs(xNext + 0.5 * h * k1, u, kappaRef, cfg);
            k3 = pointMassSpatialRhs(xNext + 0.5 * h * k2, u, kappaRef, cfg);
            k4 = pointMassSpatialRhs(xNext + h * k3, u, kappaRef, cfg);

            xNext = xNext + h / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
        else
            error("LTO:Solver:IntegratorNotImplemented", ...
                  "Integrator not implemented: %s.", method);
        end
    end
end


function vs = computeProgressSpeedSymbolic(ey, epsi, vx, kappaRef)
%COMPUTEPROGRESSSPEEDSYMBOLIC Compute ds/dt along reference centerline.

    denom = 1.0 - kappaRef * ey;
    vs = vx * cos(epsi) / denom;
end


function Fz = computeNormalLoadsSymbolic(vx, cfg)
%COMPUTENORMALLOADSSYMBOLIC Static + aero normal loads.

    m = cfg.vehicle.m;
    g = cfg.constants.g;

    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    frontShare = lr / L;
    rearShare = lf / L;

    FzFrontStatic = m * g * frontShare;
    FzRearStatic = m * g * rearShare;

    FzFrontAero = cfg.vehicle.aero.Cl_front * vx^2;
    FzRearAero = cfg.vehicle.aero.Cl_rear * vx^2;

    FzFL = 0.5 * (FzFrontStatic + FzFrontAero);
    FzFR = 0.5 * (FzFrontStatic + FzFrontAero);

    FzRL = 0.5 * (FzRearStatic + FzRearAero);
    FzRR = 0.5 * (FzRearStatic + FzRearAero);

    Fz = [FzFL; FzFR; FzRL; FzRR];
end


function usage = computePointMassTireUsageSymbolic(vx, MRear, kappaVeh, Fz, cfg)
%COMPUTEPOINTMASSTIREUSAGESYMBOLIC Tire ellipse usage per wheel.

    m = cfg.vehicle.m;
    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    R = cfg.tire.R;

    FxTotal = MRear / R;
    ay = vx^2 * kappaVeh;

    Fx = [0.0; 0.0; 0.5 * FxTotal; 0.5 * FxTotal];

    FyFront = m * ay * lr / L;
    FyRear = m * ay * lf / L;

    Fy = [
        0.5 * FyFront;
        0.5 * FyFront;
        0.5 * FyRear;
        0.5 * FyRear
    ];

    muX = computeMuX(Fz, cfg);
    muY = computeMuY(Fz, cfg);

    usageX = Fx ./ (muX .* Fz);
    usageY = Fy ./ (muY .* Fz);

    usage = usageX.^2 + usageY.^2;
end


function muX = computeMuX(Fz, cfg)
%COMPUTEMUX Load-dependent longitudinal peak friction.

    Fz0 = cfg.tire.Fz0;

    lambdaX = cfg.tire.longitudinal_limit.lambda_x;
    pDx1 = cfg.tire.longitudinal_limit.pDx1;
    pDx2 = cfg.tire.longitudinal_limit.pDx2;

    dfz = (Fz - Fz0) ./ Fz0;

    muX = lambdaX .* (pDx1 + pDx2 .* dfz);
end


function muY = computeMuY(Fz, cfg)
%COMPUTEMUY Load-dependent lateral peak friction.

    Fz0 = cfg.tire.Fz0;

    lambdaY = cfg.tire.mf52_lateral.lambda_y;
    pDy1 = cfg.tire.mf52_lateral.pDy1;
    pDy2 = cfg.tire.mf52_lateral.pDy2;

    dfz = (Fz - Fz0) ./ Fz0;

    muY = lambdaY .* (pDy1 + pDy2 .* dfz);
end


function init = buildPointMassInitialGuess(solverTrack, cfg, initialGuess)
%BUILDPOINTMASSINITIALGUESS Build NLP initial guess.

    N = solverTrack.N;

    Xinit = zeros(6, N);
    Uinit = zeros(2, N-1);

    useProvidedGuess = ~isempty(initialGuess);

    if isfield(cfg.solver, "initial_guess_strategy")
        strategy = lower(string(cfg.solver.initial_guess_strategy));

        if contains(strategy, "direct")
            useProvidedGuess = false;
        end
    end

    if ~useProvidedGuess
        vx0 = cfg.solver.backward_forward.initial_speed;
        vxMax = cfg.solver.backward_forward.max_speed;

        vxGuess = min(vxMax, max(vx0, 5.0));

        Xinit(1, :) = 0.0;
        Xinit(2, :) = 0.0;
        Xinit(3, :) = vxGuess;
        Xinit(4, :) = 0.0;
        Xinit(5, :) = 0.0;
        Xinit(6, :) = solverTrack.s(:).' / max(vxGuess, cfg.numerics.eps_vx);

        Uinit(1, :) = 0.0;
        Uinit(2, :) = solverTrack.kappa(1:N-1).';

    else
        Xinit(1, :) = readSolutionState(initialGuess, 'e_y', N, 0.0);
        Xinit(2, :) = readSolutionState(initialGuess, 'e_psi', N, 0.0);
        Xinit(3, :) = readSolutionState(initialGuess, 'vx', N, cfg.solver.backward_forward.initial_speed);
        Xinit(4, :) = readSolutionState(initialGuess, 'M_cmd', N, 0.0);
        Xinit(5, :) = readSolutionState(initialGuess, 'M_rear', N, 0.0);

        if isfield(initialGuess, "t")
            Xinit(6, :) = interpolateVector(initialGuess.t(:), N).';
        else
            Xinit(6, :) = solverTrack.s(:).' / max(mean(Xinit(3, :)), cfg.numerics.eps_vx);
        end

        Uinit(1, :) = 0.0;
        Uinit(2, :) = solverTrack.kappa(1:N-1).';
    end

    init.X = Xinit;
    init.U = Uinit;
end


function row = readSolutionState(solution, stateName, N, defaultValue)
%READSOLUTIONSTATE Read named state from common solution.

    row = defaultValue * ones(1, N);

    if ~isfield(solution, "X") || ~isfield(solution, "state_names")
        return;
    end

    idx = find(strcmp(solution.state_names, stateName), 1);

    if isempty(idx)
        return;
    end

    raw = solution.X(idx, :).';
    row = interpolateVector(raw, N).';
end


function y = interpolateVector(x, N)
%INTERPOLATEVECTOR Resample vector to N points.

    x = x(:);

    if numel(x) == N
        y = x;
        return;
    end

    oldGrid = linspace(0.0, 1.0, numel(x));
    newGrid = linspace(0.0, 1.0, N);

    y = interp1(oldGrid, x, newGrid, "linear", "extrap").';
end


function solution = buildCommonSolutionFromPointMass(solverTrack, cfg, Xpm, Upm, status, message)
%BUILDCOMMONSOLUTIONFROMPOINTMASS Convert point-mass NLP result to common solution.

    N = solverTrack.N;

    eY = Xpm(1, :).';
    ePsi = Xpm(2, :).';
    vx = Xpm(3, :).';
    MCmd = Xpm(4, :).';
    MRear = Xpm(5, :).';
    t = Xpm(6, :).';

    MCmdDot = Upm(1, :).';
    kappaVeh = Upm(2, :).';

    kappaNode = [kappaVeh; kappaVeh(end)];

    vy = zeros(N, 1);
    yawRate = vx .* kappaNode;

    ax = zeros(N, 1);
    ay = vx.^2 .* kappaNode;

    for k = 1:N
        Fx = MRear(k) / cfg.tire.R;
        Fdrag = cfg.vehicle.aero.Cd * vx(k)^2;
        Frr = cfg.vehicle.aero.Cr * cfg.vehicle.m * cfg.constants.g;

        ax(k) = (Fx - Fdrag - Frr) / cfg.vehicle.m;
    end

    axPrev = [ax(1); ax(1:end-1)];
    ayPrev = [ay(1); ay(1:end-1)];

    Fz = zeros(4, N);
    tireUsageX = zeros(4, N);
    tireUsageY = zeros(4, N);

    for k = 1:N
        Fz(:, k) = computeNormalLoadsNumeric(vx(k), cfg);

        [usageX, usageY] = computePointMassUsageNumeric(vx(k), MRear(k), kappaNode(k), Fz(:, k), cfg);

        tireUsageX(:, k) = usageX;
        tireUsageY(:, k) = usageY;
    end

    % Steering reconstruction is only for common solution / seeding later models.
    delta = zeros(N, 1);

    for k = 1:N
        delta(k) = convertKappaToEquivalentSteeringNumeric(vx(k), kappaNode(k), Fz(:, k), cfg);
    end

    deltaCmd = delta;
    deltaDot = gradientWithTime(delta, t);
    deltaCmdDotNode = gradientWithTime(deltaCmd, t);
    deltaCmdDot = deltaCmdDotNode(1:N-1);

    Xcommon = [
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

    Ucommon = [
        MCmdDot.';
        deltaCmdDot.'
    ];

    solution = struct();

    solution.type = "PointMass";
    solution.status = string(status);
    solution.message = string(message);

    solution.s = solverTrack.s(:);
    solution.t = t;
    solution.lap_time = t(end);

    solution.X = Xcommon;
    solution.U = Ucommon;

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

    solution.global.x = solverTrack.center.x(:) - eY .* sin(solverTrack.psi(:));
    solution.global.y = solverTrack.center.y(:) + eY .* cos(solverTrack.psi(:));
    solution.global.psi = solverTrack.psi(:) + ePsi;

    solution.track.s_ref = solverTrack.s(:);
    solution.track.kappa = solverTrack.kappa(:);
    solution.track.width_left = solverTrack.width.left(:);
    solution.track.width_right = solverTrack.width.right(:);
    solution.track.width_total = solverTrack.width.total(:);

    solution.wheel_names = {'FL', 'FR', 'RL', 'RR'};
    solution.normal_loads = Fz;

    solution.tire_usage.x = tireUsageX;
    solution.tire_usage.y = tireUsageY;
    solution.tire_usage.total = sqrt(tireUsageX.^2 + tireUsageY.^2);
end


function Fz = computeNormalLoadsNumeric(vx, cfg)
%COMPUTENORMALLOADSNUMERIC Static + aero normal loads.

    m = cfg.vehicle.m;
    g = cfg.constants.g;

    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    frontShare = lr / L;
    rearShare = lf / L;

    FzFrontStatic = m * g * frontShare;
    FzRearStatic = m * g * rearShare;

    FzFrontAero = cfg.vehicle.aero.Cl_front * vx^2;
    FzRearAero = cfg.vehicle.aero.Cl_rear * vx^2;

    Fz = [
        0.5 * (FzFrontStatic + FzFrontAero);
        0.5 * (FzFrontStatic + FzFrontAero);
        0.5 * (FzRearStatic + FzRearAero);
        0.5 * (FzRearStatic + FzRearAero)
    ];
end


function [usageX, usageY] = computePointMassUsageNumeric(vx, MRear, kappaVeh, Fz, cfg)
%COMPUTEPOINTMASSUSAGENUMERIC Numeric x/y tire usage.

    m = cfg.vehicle.m;
    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    FxTotal = MRear / cfg.tire.R;
    ay = vx^2 * kappaVeh;

    Fx = [0.0; 0.0; 0.5 * FxTotal; 0.5 * FxTotal];

    FyFront = m * ay * lr / L;
    FyRear = m * ay * lf / L;

    Fy = [
        0.5 * FyFront;
        0.5 * FyFront;
        0.5 * FyRear;
        0.5 * FyRear
    ];

    muX = computeMuX(Fz, cfg);
    muY = computeMuY(Fz, cfg);

    usageX = Fx ./ max(muX .* Fz, 1e-9);
    usageY = Fy ./ max(muY .* Fz, 1e-9);
end


function delta = convertKappaToEquivalentSteeringNumeric(vx, kappaVeh, Fz, cfg)
%CONVERTKAPPATOEQUIVALENTSTEERINGNUMERIC Reconstruct approximate steering angle.

    L = cfg.vehicle.L;
    m = cfg.vehicle.m;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    Cf = estimateFrontCorneringStiffness(Fz, cfg);
    Cr = estimateRearCorneringStiffness(Fz, cfg);

    ay = vx^2 * kappaVeh;

    Kus = (m / L) * (lr / max(Cf, 1e-9) - lf / max(Cr, 1e-9));

    deltaKinematic = atan(L * kappaVeh);
    deltaUndersteer = Kus * ay;

    delta = deltaKinematic + deltaUndersteer;
end


function Cf = estimateFrontCorneringStiffness(Fz, cfg)
%ESTIMATEFRONTCORNERINGSTIFFNESS Estimate front axle cornering stiffness.

    KyFL = estimateTireCorneringStiffness(Fz(1), cfg);
    KyFR = estimateTireCorneringStiffness(Fz(2), cfg);

    Cf = KyFL + KyFR;
end


function Cr = estimateRearCorneringStiffness(Fz, cfg)
%ESTIMATEREARCORNERINGSTIFFNESS Estimate rear axle cornering stiffness.

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

    Ky = (pKy1 / max(pCy1, 1e-9)) * Fz0 * ...
         sin(2.0 * atan(loadRatio / max(pKy2, 1e-9)));

    Ky = max(Ky, 1e-6);
end


function [eyMin, eyMax] = getTrackBounds(widthLeft, widthRight, margin, trackMode)
%GETTRACKBOUNDS Return lateral offset bounds.

    if trackMode == "Full track"
        eyMin = -widthRight;
        eyMax = widthLeft;

    elseif trackMode == "Respecting margin"
        eyMin = -widthRight + margin;
        eyMax = widthLeft - margin;

    else
        error("LTO:Bounds:UnknownTrackMode", ...
              "Unknown track width mode: %s.", trackMode);
    end
end


function isPeriodic = getPeriodicFlag(cfg)
%GETPERIODICFLAG Read periodic track flag from config.

    isPeriodic = false;

    if isfield(cfg.solver, "periodic_track")
        isPeriodic = logical(cfg.solver.periodic_track);
    end
end


function substeps = getIntegratorSubsteps(cfg)
%GETINTEGRATORSUBSTEPS Read integrator substeps.

    substeps = 1;

    if isfield(cfg.solver, "integrator")
        if isfield(cfg.solver.integrator, "substeps")
            substeps = cfg.solver.integrator.substeps;
        end
    end

    substeps = round(substeps);

    if substeps < 1
        substeps = 1;
    end
end


function tol = getIpoptTolerance(cfg)
%GETIPOPTTOLERANCE Select IPOPT tolerance from solve level.

    solveLevel = lower(string(cfg.solver.solve_level));

    if solveLevel == "debug"
        tol = cfg.solver.ipopt.tol_debug;

    elseif solveLevel == "preview"
        tol = cfg.solver.ipopt.tol_preview;

    elseif solveLevel == "final"
        tol = cfg.solver.ipopt.tol_final;

    else
        tol = cfg.solver.ipopt.tol_preview;
    end
end


function yDot = gradientWithTime(y, t)
%GRADIENTWITHTIME Compute dy/dt safely.

    y = y(:);
    t = t(:);

    if numel(y) < 2
        yDot = zeros(size(y));
        return;
    end

    yDot = gradient(y, t);
    yDot(~isfinite(yDot)) = 0.0;
end
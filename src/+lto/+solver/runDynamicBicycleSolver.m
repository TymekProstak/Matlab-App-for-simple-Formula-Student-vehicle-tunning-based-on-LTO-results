function solution = runDynamicBicycleSolver(solverTrack, cfg, initialGuess)
%RUNDYNAMICBICYCLESOLVER Solve dynamic bicycle NLP using CasADi/IPOPT.
%
% States:
%   e_y, e_psi, vx, vy, r, M_cmd, M_rear, delta_cmd, delta, delta_dot, t
%
% Controls:
%   M_cmd_dot, delta_cmd_dot
%
% Model assumptions:
%   - rear-wheel-drive dynamic bicycle model,
%   - no dynamic mass transfer,
%   - normal loads = static distribution + aerodynamic downforce,
%   - lateral force from MF-like 5.2 tire model,
%   - friction ellipse imposed on axles, not individual wheels,
%   - first-order drivetrain,
%   - second-order steering actuator model.

    if nargin < 3
        initialGuess = [];
    end

    import casadi.*

    method = char(string(cfg.solver.discretization.integrator));

    if ~strcmp(method, 'Euler') && ~strcmp(method, 'RK4')
        error('LTO:Solver:IntegratorNotImplemented', ...
              'Integrator not implemented: %s. Use Euler or RK4.', method);
    end

    substeps = getIntegratorSubsteps(cfg);

    N = solverTrack.N;
    dsVec = solverTrack.ds(:);

    kappaRef = solverTrack.kappa(:);
    widthLeft = solverTrack.width.left(:);
    widthRight = solverTrack.width.right(:);

    nx = 11;
    nu = 2;

    IDX_EY = 1;
    IDX_EPSI = 2;
    IDX_VX = 3;
    IDX_VY = 4;
    IDX_R = 5;
    IDX_MCMD = 6;
    IDX_MREAR = 7;
    IDX_DELTA_CMD = 8;
    IDX_DELTA = 9;
    IDX_DELTA_DOT = 10;
    IDX_T = 11;

    IDX_MCMD_DOT = 1;
    IDX_DELTA_CMD_DOT = 2;

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

    deltaMax = getMaxSteeringAngle(cfg);
    deltaCmdDotMax = getMaxSteeringCommandRate(cfg);

    margin = cfg.bounds.min_track_margin;
    trackMode = char(string(cfg.bounds.track_width_mode));

    vsMin = 1.0;

    opti.subject_to(X(IDX_VX, :) >= vxMin);
    opti.subject_to(X(IDX_VX, :) <= vxMax);

    opti.subject_to(X(IDX_MCMD, :) >= -MBrakeMax);
    opti.subject_to(X(IDX_MCMD, :) <=  MDriveMax);

    opti.subject_to(X(IDX_MREAR, :) >= -MBrakeMax);
    opti.subject_to(X(IDX_MREAR, :) <=  MDriveMax);

    opti.subject_to(X(IDX_DELTA_CMD, :) >= -deltaMax);
    opti.subject_to(X(IDX_DELTA_CMD, :) <=  deltaMax);

    opti.subject_to(X(IDX_DELTA, :) >= -deltaMax);
    opti.subject_to(X(IDX_DELTA, :) <=  deltaMax);

    opti.subject_to(X(IDX_DELTA_DOT, :) >= -deltaCmdDotMax);
    opti.subject_to(X(IDX_DELTA_DOT, :) <=  deltaCmdDotMax);

    opti.subject_to(X(IDX_T, 1) == 0.0);
    opti.subject_to(X(IDX_T, :) >= 0.0);

    opti.subject_to(U(IDX_MCMD_DOT, :) >= -MCmdDotMax);
    opti.subject_to(U(IDX_MCMD_DOT, :) <=  MCmdDotMax);

    opti.subject_to(U(IDX_DELTA_CMD_DOT, :) >= -deltaCmdDotMax);
    opti.subject_to(U(IDX_DELTA_CMD_DOT, :) <=  deltaCmdDotMax);

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

        xNext = integrateSpatialStep( ...
            xk, ...
            uk, ...
            dsVec(k), ...
            kappaRef(k), ...
            cfg, ...
            method, ...
            substeps ...
        );

        opti.subject_to(X(:, k+1) == xNext);
    end

    % ============================================================
    % Tire, power, Frenet and progress constraints
    % ============================================================

    for k = 1:N
        ey = X(IDX_EY, k);
        epsi = X(IDX_EPSI, k);
        vx = X(IDX_VX, k);
        vy = X(IDX_VY, k);
        r = X(IDX_R, k);
        MRear = X(IDX_MREAR, k);
        delta = X(IDX_DELTA, k);

        [FzF, FzR] = computeAxleNormalLoadsSymbolic(vx, cfg);

        [alphaF, alphaR] = computeSlipAnglesSymbolic(vx, vy, r, delta, cfg);

        FyF = computeAxleLateralForceSymbolic(alphaF, FzF, cfg);
        FyR = computeAxleLateralForceSymbolic(alphaR, FzR, cfg);

        FxR = MRear / cfg.tire.R;

        [usageF, usageR] = computeAxleFrictionUsageSymbolic( ...
            FxR, ...
            FyF, ...
            FyR, ...
            FzF, ...
            FzR, ...
            cfg ...
        );

        opti.subject_to(usageF <= 1.0);
        opti.subject_to(usageR <= 1.0);

        power = MRear * vx / cfg.tire.R;

        opti.subject_to(power <= 1000.0 * cfg.drivetrain.max_drive_power_kW);
        opti.subject_to(power >= -1000.0 * cfg.drivetrain.max_brake_power_kW);

        % Avoid Frenet singularity.
        % If kappaRef(k) == 0, then denom = 1 is constant and the constraint
        % is automatically satisfied. CasADi cannot accept constant subject_to().
        denom = 1.0 - kappaRef(k) * ey;
        
        if abs(kappaRef(k)) > 1e-12
            opti.subject_to(denom >= 0.2);
        end

        vs = computeProgressSpeedSymbolic(ey, epsi, vx, vy, kappaRef(k));
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
        opti.subject_to(X(IDX_VY, 1) == X(IDX_VY, N));
        opti.subject_to(X(IDX_R, 1) == X(IDX_R, N));
        opti.subject_to(X(IDX_MCMD, 1) == X(IDX_MCMD, N));
        opti.subject_to(X(IDX_MREAR, 1) == X(IDX_MREAR, N));
        opti.subject_to(X(IDX_DELTA_CMD, 1) == X(IDX_DELTA_CMD, N));
        opti.subject_to(X(IDX_DELTA, 1) == X(IDX_DELTA, N));
        opti.subject_to(X(IDX_DELTA_DOT, 1) == X(IDX_DELTA_DOT, N));

        if N > 2
            opti.subject_to(U(IDX_MCMD_DOT, 1) == U(IDX_MCMD_DOT, N-1));
            opti.subject_to(U(IDX_DELTA_CMD_DOT, 1) == U(IDX_DELTA_CMD_DOT, N-1));
        end
    else
        opti.subject_to(X(IDX_EY, 1) == 0.0);
        opti.subject_to(X(IDX_EPSI, 1) == 0.0);
        opti.subject_to(X(IDX_VX, 1) == cfg.solver.backward_forward.initial_speed);
        opti.subject_to(X(IDX_VY, 1) == 0.0);
        opti.subject_to(X(IDX_R, 1) == 0.0);
        opti.subject_to(X(IDX_MCMD, 1) == 0.0);
        opti.subject_to(X(IDX_MREAR, 1) == 0.0);
        opti.subject_to(X(IDX_DELTA_CMD, 1) == 0.0);
        opti.subject_to(X(IDX_DELTA, 1) == 0.0);
        opti.subject_to(X(IDX_DELTA_DOT, 1) == 0.0);
    end

    % ============================================================
    % Objective
    % ============================================================

    objective = X(IDX_T, N);

    qTorqueRate = cfg.cost.torque_rate;
    qSteerRate = cfg.cost.steer_rate;
    qBeta = cfg.cost.beta;

    for k = 1:N-1
        vx = X(IDX_VX, k);
        vy = X(IDX_VY, k);
        delta = X(IDX_DELTA, k);

        betaDyn = atan(vy / (vx + cfg.numerics.eps_vx));
        betaKin = atan((cfg.vehicle.lr / cfg.vehicle.L) * tan(delta));

        objective = objective + qTorqueRate * U(IDX_MCMD_DOT, k)^2;
        objective = objective + qSteerRate * U(IDX_DELTA_CMD_DOT, k)^2;
        objective = objective + qBeta * (betaDyn - betaKin)^2 * dsVec(k);
    end

    opti.minimize(objective);

    % ============================================================
    % Initial guess
    % ============================================================

    init = buildDynamicBicycleInitialGuess(solverTrack, cfg, initialGuess);

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

    if isfield(cfg.solver.ipopt, 'max_cpu_time')
        opts.ipopt.max_cpu_time = cfg.solver.ipopt.max_cpu_time;
    end

    opti.solver('ipopt', opts);

    % ============================================================
    % Solve
    % ============================================================

    try
        sol = opti.solve();

        Xsol = full(sol.value(X));
        Usol = full(sol.value(U));

        solution = buildCommonSolutionFromDynamicBicycle( ...
            solverTrack, cfg, Xsol, Usol, 'success', 'Dynamic bicycle NLP solved.' ...
        );

    catch ME
        Xdebug = full(opti.debug.value(X));
        Udebug = full(opti.debug.value(U));

        solution = buildCommonSolutionFromDynamicBicycle( ...
            solverTrack, cfg, Xdebug, Udebug, 'failed', ME.message ...
        );
    end
end


function dXds = dynamicBicycleSpatialRhs(x, u, kappaRef, cfg)
%DYNAMICBICYCLESPATIALRHS Spatial RHS for dynamic bicycle model.

    IDX_EY = 1;
    IDX_EPSI = 2;
    IDX_VX = 3;
    IDX_VY = 4;
    IDX_R = 5;
    IDX_MCMD = 6;
    IDX_MREAR = 7;
    IDX_DELTA_CMD = 8;
    IDX_DELTA = 9;
    IDX_DELTA_DOT = 10;

    IDX_MCMD_DOT = 1;
    IDX_DELTA_CMD_DOT = 2;

    ey = x(IDX_EY);
    epsi = x(IDX_EPSI);
    vx = x(IDX_VX);
    vy = x(IDX_VY);
    r = x(IDX_R);
    MCmd = x(IDX_MCMD);
    MRear = x(IDX_MREAR);
    deltaCmd = x(IDX_DELTA_CMD);
    delta = x(IDX_DELTA);
    deltaDot = x(IDX_DELTA_DOT);

    MCmdDot = u(IDX_MCMD_DOT);
    deltaCmdDot = u(IDX_DELTA_CMD_DOT);

    m = cfg.vehicle.m;
    Iz = getYawInertia(cfg);
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    denom = 1.0 - kappaRef * ey;

    vs = (vx * cos(epsi) - vy * sin(epsi)) / denom;
    vsSafe = vs + cfg.numerics.eps_vx;

    [FzF, FzR] = computeAxleNormalLoadsSymbolic(vx, cfg);

    [alphaF, alphaR] = computeSlipAnglesSymbolic(vx, vy, r, delta, cfg);

    FyF = computeAxleLateralForceSymbolic(alphaF, FzF, cfg);
    FyR = computeAxleLateralForceSymbolic(alphaR, FzR, cfg);

    FxR = MRear / cfg.tire.R;

    Fdrag = cfg.vehicle.aero.Cd * vx^2;
    Frr = cfg.vehicle.aero.Cr * cfg.vehicle.m * cfg.constants.g;

    vxDot = (FxR - FyF * sin(delta) - Fdrag - Frr) / m + vy * r;
    vyDot = (FyF * cos(delta) + FyR) / m - vx * r;
    rDot = (lf * FyF * cos(delta) - lr * FyR) / Iz;

    tauDrive = max(cfg.drivetrain.first_order_time_constant, 1e-6);

    MCmdDotDt = MCmdDot;
    MRearDot = (MCmd - MRear) / tauDrive;

    wn = getSteeringNaturalFrequency(cfg);
    zeta = getSteeringDampingRatio(cfg);

    deltaCmdDotDt = deltaCmdDot;
    deltaDotDt = deltaDot;
    deltaDDot = wn^2 * (deltaCmd - delta) - 2.0 * zeta * wn * deltaDot;

    eyDot = vx * sin(epsi) + vy * cos(epsi);
    epsiDot = r - kappaRef * vs;

    dXds = [
        eyDot / vsSafe;
        epsiDot / vsSafe;
        vxDot / vsSafe;
        vyDot / vsSafe;
        rDot / vsSafe;
        MCmdDotDt / vsSafe;
        MRearDot / vsSafe;
        deltaCmdDotDt / vsSafe;
        deltaDotDt / vsSafe;
        deltaDDot / vsSafe;
        1.0 / vsSafe
    ];
end


function xNext = integrateSpatialStep(x, u, ds, kappaRef, cfg, method, substeps)
%INTEGRATESPATIALSTEP Integrate one spatial interval.

    h = ds / substeps;
    xNext = x;

    for j = 1:substeps
        if strcmp(method, 'Euler')
            k1 = dynamicBicycleSpatialRhs(xNext, u, kappaRef, cfg);
            xNext = xNext + h * k1;

        elseif strcmp(method, 'RK4')
            k1 = dynamicBicycleSpatialRhs(xNext, u, kappaRef, cfg);
            k2 = dynamicBicycleSpatialRhs(xNext + 0.5 * h * k1, u, kappaRef, cfg);
            k3 = dynamicBicycleSpatialRhs(xNext + 0.5 * h * k2, u, kappaRef, cfg);
            k4 = dynamicBicycleSpatialRhs(xNext + h * k3, u, kappaRef, cfg);

            xNext = xNext + h / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
        else
            error('LTO:Solver:IntegratorNotImplemented', ...
                  'Integrator not implemented: %s.', method);
        end
    end
end


function vs = computeProgressSpeedSymbolic(ey, epsi, vx, vy, kappaRef)
%COMPUTEPROGRESSSPEEDSYMBOLIC Compute ds/dt.

    denom = 1.0 - kappaRef * ey;
    vs = (vx * cos(epsi) - vy * sin(epsi)) / denom;
end


function [FzF, FzR] = computeAxleNormalLoadsSymbolic(vx, cfg)
%COMPUTEAXLENORMALLOADSSYMBOLIC Static + aero axle loads.

    m = cfg.vehicle.m;
    g = cfg.constants.g;

    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    frontShare = lr / L;
    rearShare = lf / L;

    FzF = m * g * frontShare + cfg.vehicle.aero.Cl_front * vx^2;
    FzR = m * g * rearShare + cfg.vehicle.aero.Cl_rear * vx^2;
end


function [alphaF, alphaR] = computeSlipAnglesSymbolic(vx, vy, r, delta, cfg)
%COMPUTESLIPANGLESSYMBOLIC Front and rear slip angles.

    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    vxSafe = vx + cfg.numerics.eps_vx;

    alphaF = delta - atan((vy + lf * r) / vxSafe);
    alphaR =       - atan((vy - lr * r) / vxSafe);
end


function Fy = computeAxleLateralForceSymbolic(alpha, FzAxle, cfg)
%COMPUTEAXLELATERALFORCESYMBOLIC MF-like axle lateral force.

    Fz0 = cfg.tire.Fz0;

    C = cfg.tire.mf52_lateral.pCy1;
    E0 = cfg.tire.mf52_lateral.pEy1;

    E1 = 0.0;
    E2 = 0.0;

    if isfield(cfg.tire.mf52_lateral, 'pEy2')
        E1 = cfg.tire.mf52_lateral.pEy2;
    end

    if isfield(cfg.tire.mf52_lateral, 'pEy3')
        E2 = cfg.tire.mf52_lateral.pEy3;
    end

    FzWheel = 0.5 * FzAxle;
    dfz = (FzWheel - Fz0) / Fz0;

    muY = computeMuYSymbolic(FzWheel, cfg);

    DWheel = muY * FzWheel;
    Daxle = 2.0 * DWheel;

    KyWheel = estimateTireCorneringStiffnessSymbolic(FzWheel, cfg);
    KyAxle = 2.0 * KyWheel;

    B = KyAxle / (C * Daxle + 1e-9);
    E = E0 + E1 * dfz + E2 * dfz^2;

    Fy = Daxle * sin(C * atan(B * alpha - E * (B * alpha - atan(B * alpha))));
end


function [usageF, usageR] = computeAxleFrictionUsageSymbolic(FxR, FyF, FyR, FzF, FzR, cfg)
%COMPUTEAXLEFRICTIONUSAGESYMBOLIC Axle-level friction ellipse.

    FzFWheel = 0.5 * FzF;
    FzRWheel = 0.5 * FzR;

    muXF = computeMuXSymbolic(FzFWheel, cfg);
    muXR = computeMuXSymbolic(FzRWheel, cfg);

    muYF = computeMuYSymbolic(FzFWheel, cfg);
    muYR = computeMuYSymbolic(FzRWheel, cfg);

    FxF = 0.0;

    usageF = (FxF / (muXF * FzF + 1e-9))^2 + ...
             (FyF / (muYF * FzF + 1e-9))^2;

    usageR = (FxR / (muXR * FzR + 1e-9))^2 + ...
             (FyR / (muYR * FzR + 1e-9))^2;
end


function muX = computeMuXSymbolic(FzWheel, cfg)
%COMPUTEMUXSYMBOLIC Load-dependent longitudinal peak.

    Fz0 = cfg.tire.Fz0;

    lambdaX = cfg.tire.longitudinal_limit.lambda_x;
    pDx1 = cfg.tire.longitudinal_limit.pDx1;
    pDx2 = cfg.tire.longitudinal_limit.pDx2;

    dfz = (FzWheel - Fz0) / Fz0;

    muX = lambdaX * (pDx1 + pDx2 * dfz);
end


function muY = computeMuYSymbolic(FzWheel, cfg)
%COMPUTEMUYSYMBOLIC Load-dependent lateral peak.

    Fz0 = cfg.tire.Fz0;

    lambdaY = cfg.tire.mf52_lateral.lambda_y;
    pDy1 = cfg.tire.mf52_lateral.pDy1;
    pDy2 = cfg.tire.mf52_lateral.pDy2;

    dfz = (FzWheel - Fz0) / Fz0;

    muY = lambdaY * (pDy1 + pDy2 * dfz);
end


function Ky = estimateTireCorneringStiffnessSymbolic(FzWheel, cfg)
%ESTIMATETIRECORNERINGSTIFFNESSSYMBOLIC Approximate stiffness.

    Fz0 = cfg.tire.Fz0;

    pKy1 = cfg.tire.mf52_lateral.pKy1;
    pKy2 = cfg.tire.mf52_lateral.pKy2;
    pCy1 = cfg.tire.mf52_lateral.pCy1;

    loadRatio = FzWheel / Fz0;

    Ky = (pKy1 / (pCy1 + 1e-9)) * Fz0 * ...
         sin(2.0 * atan(loadRatio / (pKy2 + 1e-9)));
end


function init = buildDynamicBicycleInitialGuess(solverTrack, cfg, initialGuess)
%BUILDDYNAMICBICYCLEINITIALGUESS Build initial guess.

    N = solverTrack.N;

    Xinit = zeros(11, N);
    Uinit = zeros(2, N - 1);

    deltaMax = getMaxSteeringAngle(cfg);
    deltaCmdDotMax = getMaxSteeringCommandRate(cfg);

    MDriveMax = cfg.drivetrain.max_drive_torque;
    MBrakeMax = cfg.drivetrain.max_brake_torque;

    maxTorque = max(MDriveMax, MBrakeMax);
    MCmdDotMax = cfg.bounds.max_normalized_torque_rate * maxTorque;

    useProvidedGuess = ~isempty(initialGuess);

    if isfield(cfg.solver, 'initial_guess_strategy')
        strategy = lower(char(string(cfg.solver.initial_guess_strategy)));

        if contains(strategy, 'direct')
            useProvidedGuess = false;
        end
    end

    if ~useProvidedGuess
        vx0 = cfg.solver.backward_forward.initial_speed;
        vxMax = cfg.solver.backward_forward.max_speed;

        vxGuess = min(vxMax, max(vx0, 5.0));

        deltaGuess = atan(cfg.vehicle.L * solverTrack.kappa(:));
        deltaGuess = clampNumeric(deltaGuess, -deltaMax, deltaMax);

        betaKin = atan((cfg.vehicle.lr / cfg.vehicle.L) * tan(deltaGuess));
        vyGuess = vxGuess .* tan(betaKin);

        rGuess = vxGuess .* solverTrack.kappa(:);

        Xinit(1, :) = 0.0;
        Xinit(2, :) = 0.0;
        Xinit(3, :) = vxGuess;
        Xinit(4, :) = vyGuess(:).';
        Xinit(5, :) = rGuess(:).';
        Xinit(6, :) = 0.0;
        Xinit(7, :) = 0.0;
        Xinit(8, :) = deltaGuess(:).';
        Xinit(9, :) = deltaGuess(:).';
        Xinit(10, :) = 0.0;
        Xinit(11, :) = solverTrack.s(:).' / max(vxGuess, cfg.numerics.eps_vx);

        Uinit(1, :) = 0.0;
        Uinit(2, :) = 0.0;
    else
        eY = readSolutionState(initialGuess, 'e_y', N, 0.0);
        ePsi = readSolutionState(initialGuess, 'e_psi', N, 0.0);
        vx = readSolutionState(initialGuess, 'vx', N, cfg.solver.backward_forward.initial_speed);

        delta = readSolutionState(initialGuess, 'delta', N, NaN);

        if any(~isfinite(delta))
            delta = atan(cfg.vehicle.L * solverTrack.kappa(:)).';
        end

        delta = clampNumeric(delta, -deltaMax, deltaMax);

        deltaCmd = readSolutionState(initialGuess, 'delta_cmd', N, NaN);

        if any(~isfinite(deltaCmd))
            deltaCmd = delta;
        end

        deltaCmd = clampNumeric(deltaCmd, -deltaMax, deltaMax);

        deltaDot = readSolutionState(initialGuess, 'delta_dot', N, 0.0);
        deltaDot = clampNumeric(deltaDot, -deltaCmdDotMax, deltaCmdDotMax);

        MRear = readSolutionState(initialGuess, 'M_rear', N, 0.0);
        MCmd = readSolutionState(initialGuess, 'M_cmd', N, MRear);

        MRear = clampNumeric(MRear, -MBrakeMax, MDriveMax);
        MCmd = clampNumeric(MCmd, -MBrakeMax, MDriveMax);

        vy = readSolutionState(initialGuess, 'vy', N, NaN);

        if any(~isfinite(vy))
            betaKin = atan((cfg.vehicle.lr / cfg.vehicle.L) * tan(delta));
            vy = vx .* tan(betaKin);
        end

        r = readSolutionState(initialGuess, 'yaw_rate', N, NaN);

        if any(~isfinite(r))
            r = vx .* solverTrack.kappa(:).';
        end

        if isfield(initialGuess, 't')
            t = interpolateVector(initialGuess.t(:), N).';
        else
            t = solverTrack.s(:).' / max(mean(vx), cfg.numerics.eps_vx);
        end

        Xinit(1, :) = eY;
        Xinit(2, :) = ePsi;
        Xinit(3, :) = vx;
        Xinit(4, :) = vy;
        Xinit(5, :) = r;
        Xinit(6, :) = MCmd;
        Xinit(7, :) = MRear;
        Xinit(8, :) = deltaCmd;
        Xinit(9, :) = delta;
        Xinit(10, :) = deltaDot;
        Xinit(11, :) = t;

        MCmdDot = readSolutionControl(initialGuess, 'M_cmd_dot', N - 1, 0.0);
        deltaCmdDot = readSolutionControl(initialGuess, 'delta_cmd_dot', N - 1, NaN);

        if any(~isfinite(deltaCmdDot))
            deltaCmdDotNode = gradientWithTime(deltaCmd(:), t(:));
            deltaCmdDot = deltaCmdDotNode(1:N-1).';
        end

        MCmdDot = clampNumeric(MCmdDot, -MCmdDotMax, MCmdDotMax);
        deltaCmdDot = clampNumeric(deltaCmdDot, -deltaCmdDotMax, deltaCmdDotMax);

        Uinit(1, :) = MCmdDot;
        Uinit(2, :) = deltaCmdDot;
    end

    init.X = Xinit;
    init.U = Uinit;
end


function row = readSolutionState(solution, stateName, N, defaultValue)
%READSOLUTIONSTATE Read named state from common solution.

    if isscalar(defaultValue)
        row = defaultValue * ones(1, N);
    else
        row = interpolateVector(defaultValue(:), N).';
    end

    if ~isfield(solution, 'X') || ~isfield(solution, 'state_names')
        return;
    end

    idx = find(strcmp(solution.state_names, stateName), 1);

    if isempty(idx)
        return;
    end

    raw = solution.X(idx, :).';
    row = interpolateVector(raw, N).';
end


function row = readSolutionControl(solution, controlName, Nu, defaultValue)
%READSOLUTIONCONTROL Read named control from common solution.

    if isscalar(defaultValue)
        row = defaultValue * ones(1, Nu);
    else
        row = interpolateVector(defaultValue(:), Nu).';
    end

    if ~isfield(solution, 'U') || ~isfield(solution, 'control_names')
        return;
    end

    idx = find(strcmp(solution.control_names, controlName), 1);

    if isempty(idx)
        return;
    end

    raw = solution.U(idx, :).';
    row = interpolateVector(raw, Nu).';
end


function y = interpolateVector(x, N)
%INTERPOLATEVECTOR Resample vector.

    x = x(:);

    if numel(x) == N
        y = x;
        return;
    end

    oldGrid = linspace(0.0, 1.0, numel(x));
    newGrid = linspace(0.0, 1.0, N);

    y = interp1(oldGrid, x, newGrid, 'linear', 'extrap');
    y = y(:);
end


function solution = buildCommonSolutionFromDynamicBicycle(solverTrack, cfg, Xdb, Udb, status, message)
%BUILDCOMMONSOLUTIONFROMDYNAMICBICYCLE Convert result to common solution.

    N = solverTrack.N;

    eY = Xdb(1, :).';
    ePsi = Xdb(2, :).';
    vx = Xdb(3, :).';
    vy = Xdb(4, :).';
    yawRate = Xdb(5, :).';
    MCmd = Xdb(6, :).';
    MRear = Xdb(7, :).';
    deltaCmd = Xdb(8, :).';
    delta = Xdb(9, :).';
    deltaDot = Xdb(10, :).';
    t = Xdb(11, :).';

    MCmdDot = Udb(1, :).';
    deltaCmdDot = Udb(2, :).';

    ax = zeros(N, 1);
    ay = zeros(N, 1);

    Fz = zeros(4, N);
    tireUsageX = zeros(4, N);
    tireUsageY = zeros(4, N);

    alphaF = zeros(N, 1);
    alphaR = zeros(N, 1);

    for k = 1:N
        [FzF, FzR] = computeAxleNormalLoadsNumeric(vx(k), cfg);

        Fz(:, k) = [
            0.5 * FzF;
            0.5 * FzF;
            0.5 * FzR;
            0.5 * FzR
        ];

        [alphaF(k), alphaR(k)] = computeSlipAnglesNumeric(vx(k), vy(k), yawRate(k), delta(k), cfg);

        FyF = computeAxleLateralForceNumeric(alphaF(k), FzF, cfg);
        FyR = computeAxleLateralForceNumeric(alphaR(k), FzR, cfg);

        FxR = MRear(k) / cfg.tire.R;

        Fdrag = cfg.vehicle.aero.Cd * vx(k)^2;
        Frr = cfg.vehicle.aero.Cr * cfg.vehicle.m * cfg.constants.g;

        ax(k) = (FxR - FyF * sin(delta(k)) - Fdrag - Frr) / cfg.vehicle.m + vy(k) * yawRate(k);
        ay(k) = (FyF * cos(delta(k)) + FyR) / cfg.vehicle.m - vx(k) * yawRate(k);

        [usageX, usageY] = computeWheelUsageFromAxleForcesNumeric(FxR, FyF, FyR, Fz(:, k), cfg);

        tireUsageX(:, k) = usageX;
        tireUsageY(:, k) = usageY;
    end

    axPrev = [ax(1); ax(1:end-1)];
    ayPrev = [ay(1); ay(1:end-1)];

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

    solution.type = 'DynamicBicycle';
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

    solution.tire.alpha_front = alphaF;
    solution.tire.alpha_rear = alphaR;
end


function [FzF, FzR] = computeAxleNormalLoadsNumeric(vx, cfg)
%COMPUTEAXLENORMALLOADSNUMERIC Static + aero axle loads.

    m = cfg.vehicle.m;
    g = cfg.constants.g;

    L = cfg.vehicle.L;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    frontShare = lr / L;
    rearShare = lf / L;

    FzF = m * g * frontShare + cfg.vehicle.aero.Cl_front * vx^2;
    FzR = m * g * rearShare + cfg.vehicle.aero.Cl_rear * vx^2;
end


function [alphaF, alphaR] = computeSlipAnglesNumeric(vx, vy, r, delta, cfg)
%COMPUTESLIPANGLESNUMERIC Slip angles.

    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;

    vxSafe = vx + cfg.numerics.eps_vx;

    alphaF = delta - atan((vy + lf * r) / vxSafe);
    alphaR =       - atan((vy - lr * r) / vxSafe);
end


function Fy = computeAxleLateralForceNumeric(alpha, FzAxle, cfg)
%COMPUTEAXLELATERALFORCENUMERIC MF-like axle lateral force.

    Fz0 = cfg.tire.Fz0;

    C = cfg.tire.mf52_lateral.pCy1;
    E0 = cfg.tire.mf52_lateral.pEy1;

    E1 = 0.0;
    E2 = 0.0;

    if isfield(cfg.tire.mf52_lateral, 'pEy2')
        E1 = cfg.tire.mf52_lateral.pEy2;
    end

    if isfield(cfg.tire.mf52_lateral, 'pEy3')
        E2 = cfg.tire.mf52_lateral.pEy3;
    end

    FzWheel = 0.5 * FzAxle;
    dfz = (FzWheel - Fz0) / Fz0;

    muY = computeMuYNumeric(FzWheel, cfg);

    DWheel = muY * FzWheel;
    Daxle = 2.0 * DWheel;

    KyWheel = estimateTireCorneringStiffnessNumeric(FzWheel, cfg);
    KyAxle = 2.0 * KyWheel;

    B = KyAxle / max(C * Daxle, 1e-9);
    E = E0 + E1 * dfz + E2 * dfz^2;

    Fy = Daxle * sin(C * atan(B * alpha - E * (B * alpha - atan(B * alpha))));
end


function [usageX, usageY] = computeWheelUsageFromAxleForcesNumeric(FxR, FyF, FyR, Fz, cfg)
%COMPUTEWHEELUSAGEFROMAXLEFORCESNUMERIC Wheel usage for plots/output only.

    Fx = [
        0.0;
        0.0;
        0.5 * FxR;
        0.5 * FxR
    ];

    Fy = [
        0.5 * FyF;
        0.5 * FyF;
        0.5 * FyR;
        0.5 * FyR
    ];

    muX = computeMuXNumeric(Fz, cfg);
    muY = computeMuYNumeric(Fz, cfg);

    usageX = Fx ./ max(muX .* Fz, 1e-9);
    usageY = Fy ./ max(muY .* Fz, 1e-9);
end


function muX = computeMuXNumeric(FzWheel, cfg)
%COMPUTEMUXNUMERIC Longitudinal peak.

    Fz0 = cfg.tire.Fz0;

    lambdaX = cfg.tire.longitudinal_limit.lambda_x;
    pDx1 = cfg.tire.longitudinal_limit.pDx1;
    pDx2 = cfg.tire.longitudinal_limit.pDx2;

    dfz = (FzWheel - Fz0) ./ Fz0;

    muX = lambdaX .* (pDx1 + pDx2 .* dfz);
end


function muY = computeMuYNumeric(FzWheel, cfg)
%COMPUTEMUYNUMERIC Lateral peak.

    Fz0 = cfg.tire.Fz0;

    lambdaY = cfg.tire.mf52_lateral.lambda_y;
    pDy1 = cfg.tire.mf52_lateral.pDy1;
    pDy2 = cfg.tire.mf52_lateral.pDy2;

    dfz = (FzWheel - Fz0) ./ Fz0;

    muY = lambdaY .* (pDy1 + pDy2 .* dfz);
end


function Ky = estimateTireCorneringStiffnessNumeric(FzWheel, cfg)
%ESTIMATETIRECORNERINGSTIFFNESSNUMERIC Approximate stiffness.

    Fz0 = cfg.tire.Fz0;

    pKy1 = cfg.tire.mf52_lateral.pKy1;
    pKy2 = cfg.tire.mf52_lateral.pKy2;
    pCy1 = cfg.tire.mf52_lateral.pCy1;

    loadRatio = FzWheel / max(Fz0, 1e-9);

    Ky = (pKy1 / max(pCy1, 1e-9)) * Fz0 * ...
         sin(2.0 * atan(loadRatio / max(pKy2, 1e-9)));

    Ky = max(Ky, 1e-6);
end


function [eyMin, eyMax] = getTrackBounds(widthLeft, widthRight, margin, trackMode)
%GETTRACKBOUNDS Return lateral bounds.

    if strcmp(trackMode, 'Full track')
        eyMin = -widthRight;
        eyMax = widthLeft;

    elseif strcmp(trackMode, 'Respecting margin')
        eyMin = -widthRight + margin;
        eyMax = widthLeft - margin;

    else
        error('LTO:Bounds:UnknownTrackMode', ...
              'Unknown track width mode: %s.', trackMode);
    end
end


function isPeriodic = getPeriodicFlag(cfg)
%GETPERIODICFLAG Read periodic flag.

    isPeriodic = false;

    if isfield(cfg.solver, 'periodic_track')
        isPeriodic = logical(cfg.solver.periodic_track);
    end
end


function substeps = getIntegratorSubsteps(cfg)
%GETINTEGRATORSUBSTEPS Read substeps.

    substeps = 1;

    if isfield(cfg.solver, 'integrator')
        if isfield(cfg.solver.integrator, 'substeps')
            substeps = cfg.solver.integrator.substeps;
        end
    end

    if isfield(cfg.solver, 'discretization')
        if isfield(cfg.solver.discretization, 'substeps')
            substeps = cfg.solver.discretization.substeps;
        end
    end

    substeps = round(substeps);

    if substeps < 1
        substeps = 1;
    end
end


function tol = getIpoptTolerance(cfg)
%GETIPOPTTOLERANCE Select tolerance.

    solveLevel = lower(char(string(cfg.solver.solve_level)));

    if strcmp(solveLevel, 'debug')
        tol = cfg.solver.ipopt.tol_debug;

    elseif strcmp(solveLevel, 'preview')
        tol = cfg.solver.ipopt.tol_preview;

    elseif strcmp(solveLevel, 'final')
        tol = cfg.solver.ipopt.tol_final;

    else
        tol = cfg.solver.ipopt.tol_preview;
    end
end


function yDot = gradientWithTime(y, t)
%GRADIENTWITHTIME Safe gradient.

    y = y(:);
    t = t(:);

    if numel(y) < 2
        yDot = zeros(size(y));
        return;
    end

    yDot = gradient(y, t);
    yDot(~isfinite(yDot)) = 0.0;
end


function y = clampNumeric(y, lo, hi)
%CLAMPNUMERIC Clamp numeric values.

    y = min(max(y, lo), hi);
end


function Iz = getYawInertia(cfg)
%GETYAWINERTIA Read yaw inertia.

    if isfield(cfg.vehicle, 'Iz')
        Iz = cfg.vehicle.Iz;
        return;
    end

    if isfield(cfg.vehicle, 'inertia')
        if isfield(cfg.vehicle.inertia, 'Iz')
            Iz = cfg.vehicle.inertia.Iz;
            return;
        end

        if isfield(cfg.vehicle.inertia, 'yaw')
            Iz = cfg.vehicle.inertia.yaw;
            return;
        end
    end

    Iz = 85.0;
end


function deltaMax = getMaxSteeringAngle(cfg)
%GETMAXSTEERINGANGLE Read steering angle limit.

    if isfield(cfg, 'steering')
        if isfield(cfg.steering, 'max_angle')
            deltaMax = cfg.steering.max_angle;
            return;
        end

        if isfield(cfg.steering, 'max_steering_angle')
            deltaMax = cfg.steering.max_steering_angle;
            return;
        end
    end

    deltaMax = 0.4;
end


function deltaCmdDotMax = getMaxSteeringCommandRate(cfg)
%GETMAXSTEERINGCOMMANDRATE Read steering command rate limit.

    if isfield(cfg.bounds, 'max_steering_command_rate')
        deltaCmdDotMax = cfg.bounds.max_steering_command_rate;
        return;
    end

    if isfield(cfg.bounds, 'max_steering_rate')
        deltaCmdDotMax = cfg.bounds.max_steering_rate;
        return;
    end

    deltaCmdDotMax = 5.0;
end


function wn = getSteeringNaturalFrequency(cfg)
%GETSTEERINGNATURALFREQUENCY Read steering natural frequency.

    if isfield(cfg, 'steering')
        if isfield(cfg.steering, 'natural_frequency')
            wn = cfg.steering.natural_frequency;
            return;
        end

        if isfield(cfg.steering, 'natural_frequency_radps')
            wn = cfg.steering.natural_frequency_radps;
            return;
        end

        if isfield(cfg.steering, 'wn')
            wn = cfg.steering.wn;
            return;
        end
    end

    wn = 15.0;
end


function zeta = getSteeringDampingRatio(cfg)
%GETSTEERINGDAMPINGRATIO Read steering damping ratio.

    if isfield(cfg, 'steering')
        if isfield(cfg.steering, 'damping_ratio')
            zeta = cfg.steering.damping_ratio;
            return;
        end

        if isfield(cfg.steering, 'zeta')
            zeta = cfg.steering.zeta;
            return;
        end
    end

    zeta = 1.0;
end

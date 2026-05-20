function solverTrack = prepareSolverTrack(track, cfg, solveLevel)
%PREPARESOLVERTRACK Resample raw track to solver spatial grid.
    if isfield(cfg.solver, "periodic_track")
        if logical(cfg.solver.periodic_track)
            track = lto.track.closeTrackIfNeeded(track);
        end
    end
    if nargin < 3 || strlength(string(solveLevel)) == 0
        solveLevel = cfg.solver.solve_level;
    end

    N = getPointCount(cfg, solveLevel);

    sRaw = track.s(:);

    if numel(sRaw) < 3
        error("LTO:Track:TooFewPoints", ...
              "Track must contain at least 3 points.");
    end

    % Remove duplicated s values.
    [sRaw, uniqueIdx] = unique(sRaw, "stable");

    xCenterRaw = track.x_center(uniqueIdx);
    yCenterRaw = track.y_center(uniqueIdx);

    xLeftRaw = track.x_left(uniqueIdx);
    yLeftRaw = track.y_left(uniqueIdx);

    xRightRaw = track.x_right(uniqueIdx);
    yRightRaw = track.y_right(uniqueIdx);

    % Curvature comes from original track geometry, not from interpolated x/y.
    kappaRaw = track.kappa(uniqueIdx);

    % Solver grid in path coordinate.
    sSolver = linspace(sRaw(1), sRaw(end), N).';

    % Smooth original curvature before interpolation.
    smoothWindow = 7;
    kappaRawSmooth = movmean(kappaRaw, smoothWindow);

    % Linear interpolation to solver grid.
    xCenter = interp1(sRaw, xCenterRaw, sSolver, "linear");
    yCenter = interp1(sRaw, yCenterRaw, sSolver, "linear");

    xLeft = interp1(sRaw, xLeftRaw, sSolver, "linear");
    yLeft = interp1(sRaw, yLeftRaw, sSolver, "linear");

    xRight = interp1(sRaw, xRightRaw, sSolver, "linear");
    yRight = interp1(sRaw, yRightRaw, sSolver, "linear");

    % Raw and smoothed curvature on solver grid.
    kappaRawSolver = interp1(sRaw, kappaRaw, sSolver, "linear");
    kappaSolver = interp1(sRaw, kappaRawSmooth, sSolver, "linear");

    % Solver step sizes.
    ds = diff(sSolver);

    % Centerline heading from resampled centerline.
    dx_ds = gradient(xCenter, sSolver);
    dy_ds = gradient(yCenter, sSolver);

    psi = atan2(dy_ds, dx_ds);

    % Tangent and normal vectors.
    tangentX = cos(psi);
    tangentY = sin(psi);

    normalX = -sin(psi);
    normalY =  cos(psi);

    % Widths measured from centerline to boundaries.
    widthLeft = sqrt((xLeft - xCenter).^2 + (yLeft - yCenter).^2);
    widthRight = sqrt((xRight - xCenter).^2 + (yRight - yCenter).^2);

    % Output structure for solvers.
    solverTrack = struct();

    solverTrack.N = N;
    solverTrack.s = sSolver;
    solverTrack.ds = ds;
    solverTrack.length = sSolver(end) - sSolver(1);

    solverTrack.center.x = xCenter;
    solverTrack.center.y = yCenter;

    solverTrack.left.x = xLeft;
    solverTrack.left.y = yLeft;

    solverTrack.right.x = xRight;
    solverTrack.right.y = yRight;

    solverTrack.kappa_raw = kappaRawSolver;
    solverTrack.kappa = kappaSolver;

    solverTrack.psi = psi;

    solverTrack.tangent.x = tangentX;
    solverTrack.tangent.y = tangentY;

    solverTrack.normal.x = normalX;
    solverTrack.normal.y = normalY;

    solverTrack.width.left = widthLeft;
    solverTrack.width.right = widthRight;
    solverTrack.width.total = widthLeft + widthRight;

    if isfield(track, "file_path")
        solverTrack.source.file_path = track.file_path;
    else
        solverTrack.source.file_path = "";
    end

    solverTrack.source.raw_points = numel(track.s);
    solverTrack.source.solve_level = string(solveLevel);
    solverTrack.source.smoothing_window = smoothWindow;
end


function N = getPointCount(cfg, solveLevel)
%GETPOINTCOUNT Return number of solver grid points.

    solveLevel = lower(string(solveLevel));

    if solveLevel == "debug"
        N = cfg.solver.discretization.N_debug;

    elseif solveLevel == "preview"
        N = cfg.solver.discretization.N_preview;

    elseif solveLevel == "final"
        N = cfg.solver.discretization.N_final;

    else
        error("LTO:Solver:UnknownSolveLevel", ...
              "Unknown solve level: %s", solveLevel);
    end

    N = round(N);

    if N < 3
        error("LTO:Solver:TooFewPoints", ...
              "Solver track must have at least 3 points.");
    end
end



function track = computeTrackGeometry(track)
%COMPUTETRACKGEOMETRY Compute arc length, curvature and track width.

    x = track.x_center(:);
    y = track.y_center(:);

    if numel(x) < 3
        error("LTO:Track:TooFewPoints", ...
              "Track must contain at least 3 centerline points.");
    end

    dx = diff(x);
    dy = diff(y);

    ds = sqrt(dx.^2 + dy.^2);

    if any(ds <= 0)
        error("LTO:Track:DuplicatePoints", ...
              "Track contains duplicate or zero-distance centerline points.");
    end

    s = [0; cumsum(ds)];

    track.s = s;
    track.ds = ds;
    track.length = s(end);
    track.n_points = numel(s);

    dx_ds = gradient(x, s);
    dy_ds = gradient(y, s);

    d2x_ds2 = gradient(dx_ds, s);
    d2y_ds2 = gradient(dy_ds, s);

    numerator = dx_ds .* d2y_ds2 - dy_ds .* d2x_ds2;
    denominator = (dx_ds.^2 + dy_ds.^2).^(3/2);

    track.kappa = numerator ./ max(denominator, 1e-9);

    track.width = sqrt( ...
        (track.x_left(:) - track.x_right(:)).^2 + ...
        (track.y_left(:) - track.y_right(:)).^2 ...
    );
end
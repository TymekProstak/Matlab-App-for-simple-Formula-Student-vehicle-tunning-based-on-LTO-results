function track = closeTrackIfNeeded(track)
%CLOSETRACKIFNEEDED Append first track point at the end if track is not closed.

    dx = track.x_center(end) - track.x_center(1);
    dy = track.y_center(end) - track.y_center(1);

    closureError = hypot(dx, dy);

    if closureError < 1e-9
        return;
    end

    track.x_left(end+1, 1) = track.x_left(1);
    track.y_left(end+1, 1) = track.y_left(1);

    track.x_center(end+1, 1) = track.x_center(1);
    track.y_center(end+1, 1) = track.y_center(1);

    track.x_right(end+1, 1) = track.x_right(1);
    track.y_right(end+1, 1) = track.y_right(1);

    track = lto.track.computeTrackGeometry(track);
end
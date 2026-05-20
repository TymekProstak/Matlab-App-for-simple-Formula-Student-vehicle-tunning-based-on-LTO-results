function slip = computeSlipAngles4W(app, solution)
%COMPUTESLIPANGLES4W Compute four-wheel slip angles from solution states.
%
% Wheel order:
%   1 = FL
%   2 = FR
%   3 = RL
%   4 = RR
%
% Convention:
%   alpha = atan2(v_lateral_in_wheel_frame, v_longitudinal_in_wheel_frame)

    cfg = app.Config;

    vx = lto.plot.getState(solution, "vx");
    vy = lto.plot.getState(solution, "vy");
    yawRate = lto.plot.getState(solution, "yaw_rate");
    delta = lto.plot.getState(solution, "delta");

    N = numel(vx);

    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    tf = cfg.vehicle.tf;
    tr = cfg.vehicle.tr;

    % Wheel positions in vehicle body frame:
    % x forward, y left.
    xWheel = [
         lf;
         lf;
        -lr;
        -lr
    ];

    yWheel = [
         tf / 2.0;
        -tf / 2.0;
         tr / 2.0;
        -tr / 2.0
    ];

    alpha = zeros(4, N);
    vLong = zeros(4, N);
    vLat = zeros(4, N);

    for k = 1:N
        for i = 1:4

            % Velocity of wheel center in body frame.
            % v_wheel = v_CG + omega_z x r_wheel
            vxWheel = vx(k) - yawRate(k) * yWheel(i);
            vyWheel = vy(k) + yawRate(k) * xWheel(i);

            % Front wheels steer by delta, rear wheels do not steer.
            if i <= 2
                deltaWheel = delta(k);
            else
                deltaWheel = 0.0;
            end

            % Unit vector along wheel heading.
            eLong = [
                cos(deltaWheel);
                sin(deltaWheel)
            ];

            % Unit vector lateral to wheel heading.
            eLat = [
                -sin(deltaWheel);
                 cos(deltaWheel)
            ];

            vWheel = [
                vxWheel;
                vyWheel
            ];

            vLong(i, k) = dot(vWheel, eLong);
            vLat(i, k) = dot(vWheel, eLat);

            alpha(i, k) = atan2(vLat(i, k), max(vLong(i, k), 1e-9));
        end
    end

    slip.wheel_names = {'FL', 'FR', 'RL', 'RR'};
    slip.alpha = alpha;
    slip.v_long = vLong;
    slip.v_lat = vLat;
end
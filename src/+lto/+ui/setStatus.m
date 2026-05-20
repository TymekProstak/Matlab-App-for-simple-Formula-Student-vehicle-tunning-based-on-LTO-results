function setStatus(app, statusText, statusType)
%SETSTATUS Update top-bar status text and lamp color.

    if nargin < 3 || isempty(statusType)
        statusType = "ready";
    end

    statusText = string(statusText);
    statusType = lower(string(statusType));

    switch statusType
        case {"ready", "ok", "success"}
            lampColor = [0.2, 0.8, 0.2];   % green

        case {"busy", "running", "loading", "solving"}
            lampColor = [0.2, 0.45, 1.0];  % blue

        case {"warning", "warn"}
            lampColor = [1.0, 0.75, 0.0];  % yellow

        case {"error", "fail", "failed"}
            lampColor = [0.9, 0.1, 0.1];   % red

        otherwise
            lampColor = [0.6, 0.6, 0.6];   % gray
    end

    app.StatusLabel.Text = char(statusText);
    app.StatusLamp.Color = lampColor;

    drawnow limitrate;
end
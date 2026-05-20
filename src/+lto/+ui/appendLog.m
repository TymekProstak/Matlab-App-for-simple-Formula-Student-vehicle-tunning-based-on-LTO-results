function appendLog(app, message, level)
%APPENDLOG Append one formatted message to the GUI console.

    if nargin < 3 || isempty(level)
        level = "INFO";
    end

    maxLines = 50;

    message = string(message);
    level = upper(string(level));

    timeStamp = string(datetime("now", "Format", "HH:mm:ss"));
    newLine = "[" + timeStamp + "] [" + level + "] " + message;

    oldValue = app.ConsoleoutputTextArea.Value;

    if isempty(oldValue)
        logLines = strings(0, 1);
    elseif iscell(oldValue)
        logLines = string(oldValue(:));
    elseif isstring(oldValue)
        logLines = oldValue(:);
    elseif ischar(oldValue)
        logLines = string(oldValue);
    else
        logLines = strings(0, 1);
    end

    logLines = [logLines; newLine];

    % Keep only the newest lines 
    if numel(logLines) > maxLines
        logLines = logLines(end-maxLines+1:end);
    end

    app.ConsoleoutputTextArea.Value = cellstr(logLines);

    drawnow limitrate;

    % Scrool to bootom of log console if given Matlab version support this.
    try
        scroll(app.ConsoleoutputTextArea, "bottom");
    catch
    end
end
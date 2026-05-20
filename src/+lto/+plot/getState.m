function value = getState(solution, stateName)
%GETSTATE Return state row by name.

    idx = find(strcmp(solution.state_names, stateName), 1);

    if isempty(idx)
        error("LTO:Plot:MissingState", ...
              "State not found in solution: %s", stateName);
    end

    value = solution.X(idx, :).';
end
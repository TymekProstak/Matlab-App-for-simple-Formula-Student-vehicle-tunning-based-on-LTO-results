function value = getControl(solution, controlName)
%GETCONTROL Return control row by name.

    idx = find(strcmp(solution.control_names, controlName), 1);

    if isempty(idx)
        error("LTO:Plot:MissingControl", ...
              "Control not found in solution: %s", controlName);
    end

    value = solution.U(idx, :).';
end
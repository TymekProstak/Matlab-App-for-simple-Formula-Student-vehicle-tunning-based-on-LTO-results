function setDropDownValue(app, dropdown, value)
%SETDROPDOWNVALUE Set dropdown value if matching item exists.

    desired = string(value);
    items = string(dropdown.Items);

    idx = find(items == desired, 1);

    if ~isempty(idx)
        dropdown.Value = char(items(idx));
        return;
    end

    desiredNorm = normalizeText(desired);

    for i = 1:numel(items)
        if normalizeText(items(i)) == desiredNorm
            dropdown.Value = char(items(i));
            return;
        end
    end

    lto.ui.appendLog( ...
        app, ...
        "Dropdown value not found: " + desired + ". Keeping current value.", ...
        "WARNING" ...
    );
end


function out = normalizeText(txt)
%NORMALIZETEXT Normalize dropdown text.

    out = lower(string(txt));
    out = strtrim(out);
    out = erase(out, " ");
    out = erase(out, "-");
    out = erase(out, "_");
end
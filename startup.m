% ============================================================
% Project startup
% ============================================================

projectRoot = fileparts(mfilename("fullpath"));

srcPath = fullfile(projectRoot, "src");
addpath(srcPath);

casadiRoot = fullfile(projectRoot, "external", "casadi");

if isfolder(casadiRoot)
    mexFiles = dir(fullfile(casadiRoot, "**", "casadiMEX*"));

    if ~isempty(mexFiles)
        casadiPath = mexFiles(1).folder;
        addpath(casadiPath);

        fprintf("[LTO APP] CasADi path added:\n");
        fprintf("          %s\n", casadiPath);
    else
        fprintf("[LTO APP] CasADi folder exists, but casadiMEX was not found.\n");
        fprintf("[LTO APP] Run: install_casadi(""auto"")\n");
    end
else
    fprintf("[LTO APP] CasADi is not installed yet.\n");
    fprintf("[LTO APP] Run: install_casadi(""auto"")\n");
end

rehash;

fprintf("[LTO APP] Project path initialized:\n");
fprintf("          %s\n", projectRoot);
fprintf("[LTO APP] Source path added:\n");
fprintf("          %s\n", srcPath);
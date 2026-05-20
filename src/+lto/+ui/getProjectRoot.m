function projectRoot = getProjectRoot()
%GETPROJECTROOT Return project root based on this package file location.

    thisFile = mfilename("fullpath");

    uiFolder = fileparts(thisFile);      % src/+lto/+ui
    ltoFolder = fileparts(uiFolder);     % src/+lto
    srcFolder = fileparts(ltoFolder);    % src
    projectRoot = fileparts(srcFolder);  % project root
end
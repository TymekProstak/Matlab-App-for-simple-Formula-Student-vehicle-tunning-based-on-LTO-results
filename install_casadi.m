function install_casadi(osFlag)
%INSTALL_CASADI Install latest compatible CasADi MATLAB package.
%
% Usage:
%   install_casadi("auto")
%   install_casadi("windows")
%   install_casadi("linux")
%   install_casadi("mac-intel")
%   install_casadi("mac-arm")
%
% This file should be placed directly in the project root, next to startup.m.

    if nargin < 1 || strlength(string(osFlag)) == 0
        osFlag = "auto";
    end

    osFlag = lower(string(osFlag));

    projectRoot = fileparts(mfilename("fullpath"));
    externalDir = fullfile(projectRoot, "external");
    casadiRoot = fullfile(externalDir, "casadi");

    if ~isfolder(externalDir)
        mkdir(externalDir);
    end

    if ~isfolder(casadiRoot)
        mkdir(casadiRoot);
    end

    osFlag = resolveOsFlag(osFlag);

    fprintf("[LTO APP] Project root:\n");
    fprintf("          %s\n", projectRoot);
    fprintf("[LTO APP] Requested CasADi platform: %s\n", osFlag);

    releaseInfo = readLatestCasadiRelease();
    tagName = string(releaseInfo.tag_name);

    fprintf("[LTO APP] Latest CasADi release: %s\n", tagName);

    asset = selectMatlabAsset(releaseInfo.assets, osFlag);

    fprintf("[LTO APP] Selected asset:\n");
    fprintf("          %s\n", asset.name);

    installDir = fullfile(casadiRoot, char(tagName));

    if ~isfolder(installDir)
        mkdir(installDir);
    end

    archivePath = fullfile(installDir, asset.name);

    if ~isfile(archivePath)
        fprintf("[LTO APP] Downloading CasADi archive...\n");
        fprintf("          %s\n", asset.browser_download_url);

        websave(archivePath, asset.browser_download_url);
    else
        fprintf("[LTO APP] Archive already exists. Download skipped.\n");
    end

    unpackDir = fullfile(installDir, "unpacked");

    if ~isfolder(unpackDir)
        mkdir(unpackDir);
    end

    unpackCasadiArchive(archivePath, unpackDir);

    casadiPath = findCasadiMatlabFolder(unpackDir);

    addpath(casadiPath);
    rehash;

    fprintf("[LTO APP] CasADi path added:\n");
    fprintf("          %s\n", casadiPath);

    testCasadi();

    fprintf("[LTO APP] CasADi installation completed successfully.\n");
end


function osFlag = resolveOsFlag(osFlag)
%RESOLVEOSFLAG Convert auto flag into concrete platform flag.

    if osFlag ~= "auto"
        validateOsFlag(osFlag);
        validateMacArmCompatibility(osFlag);
        return;
    end

    arch = string(computer("arch"));

    if ispc
        osFlag = "windows";

    elseif isunix && ~ismac
        osFlag = "linux";

    elseif ismac
        if arch == "maca64"
            if isMatlabReleaseAtLeast(2023, "b")
                osFlag = "mac-arm";
            else
                osFlag = "mac-intel";
                warning("Apple Silicon package needs MATLAB R2023b or newer. Falling back to mac-intel/Rosetta package.");
            end
        else
            osFlag = "mac-intel";
        end

    else
        error("LTO:Install:UnsupportedOS", ...
              "Unsupported operating system.");
    end
end


function validateOsFlag(osFlag)
%VALIDATEOSFLAG Check allowed OS flags.

    allowed = ["windows", "linux", "mac-intel", "mac-arm"];

    if ~any(osFlag == allowed)
        error("LTO:Install:InvalidOSFlag", ...
              "Invalid osFlag: %s. Use auto/windows/linux/mac-intel/mac-arm.", ...
              osFlag);
    end
end


function validateMacArmCompatibility(osFlag)
%VALIDATEMACARMCOMPATIBILITY Check MATLAB release for Apple Silicon package.

    if osFlag == "mac-arm"
        if ~isMatlabReleaseAtLeast(2023, "b")
            error("LTO:Install:MacArmMatlabTooOld", ...
                  "mac-arm CasADi package requires MATLAB R2023b or newer.");
        end
    end
end


function tf = isMatlabReleaseAtLeast(yearMin, halfMin)
%ISMATLABRELEASEATLEAST Return true if MATLAB release is at least Ryyyy[a/b].

    rel = string(version("-release"));   % Example: "2024b"

    if strlength(rel) < 5
        tf = false;
        return;
    end

    yearNow = str2double(extractBefore(rel, 5));
    halfNow = lower(extractAfter(rel, 4));

    if isnan(yearNow)
        tf = false;
        return;
    end

    halfNowValue = releaseHalfToNumber(halfNow);
    halfMinValue = releaseHalfToNumber(lower(string(halfMin)));

    if yearNow > yearMin
        tf = true;
    elseif yearNow < yearMin
        tf = false;
    else
        tf = halfNowValue >= halfMinValue;
    end
end


function value = releaseHalfToNumber(halfName)
%RELEASEHALFTONUMBER Convert release half a/b into numeric order.

    if halfName == "a"
        value = 1;
    elseif halfName == "b"
        value = 2;
    else
        value = 0;
    end
end


function releaseInfo = readLatestCasadiRelease()
%READLATESTCASADIRELEASE Read latest CasADi GitHub release metadata.

    apiUrl = "https://api.github.com/repos/casadi/casadi/releases/latest";

    options = weboptions( ...
        "Timeout", 60, ...
        "ContentType", "json" ...
    );

    releaseInfo = webread(apiUrl, options);
end


function asset = selectMatlabAsset(assets, osFlag)
%SELECTMATLABASSET Select MATLAB asset matching selected platform.

    selectedIndex = [];

    for i = 1:numel(assets)
        assetName = lower(string(assets(i).name));

        if ~contains(assetName, "matlab")
            continue;
        end

        if matchesOs(assetName, osFlag)
            selectedIndex = i;
            break;
        end
    end

    if isempty(selectedIndex)
        error("LTO:Install:AssetNotFound", ...
              "No MATLAB CasADi release asset found for platform: %s.", ...
              osFlag);
    end

    asset = assets(selectedIndex);
end


function ok = matchesOs(assetName, osFlag)
%MATCHESOS Check whether asset name matches selected platform.

    ok = false;

    if osFlag == "windows"
        ok = contains(assetName, "windows") || contains(assetName, "win64");

    elseif osFlag == "linux"
        ok = contains(assetName, "linux");

    elseif osFlag == "mac-intel"
        ok = contains(assetName, "osx64") || ...
             contains(assetName, "mac64") || ...
             contains(assetName, "maci64") || ...
             (contains(assetName, "mac") && ~contains(assetName, "arm") && ~contains(assetName, "maca64"));

    elseif osFlag == "mac-arm"
        ok = contains(assetName, "arm64") || ...
             contains(assetName, "maca64") || ...
             contains(assetName, "apple");
    end
end


function unpackCasadiArchive(archivePath, unpackDir)
%UNPACKCASADIARCHIVE Unpack CasADi archive.

    archivePath = string(archivePath);

    fprintf("[LTO APP] Unpacking archive...\n");

    if endsWith(archivePath, ".zip")
        unzip(archivePath, unpackDir);

    elseif endsWith(archivePath, ".tar.gz") || endsWith(archivePath, ".tgz")
        untar(archivePath, unpackDir);

    elseif endsWith(archivePath, ".tar")
        untar(archivePath, unpackDir);

    else
        error("LTO:Install:UnknownArchiveType", ...
              "Unsupported archive type: %s", archivePath);
    end
end


function casadiPath = findCasadiMatlabFolder(unpackDir)
%FINDCASADIMATLABFOLDER Find folder containing CasADi MATLAB files.

    mexFiles = dir(fullfile(unpackDir, "**", "casadiMEX*"));

    if isempty(mexFiles)
        error("LTO:Install:CasadiMexNotFound", ...
              "Could not find casadiMEX file after unpacking.");
    end

    casadiPath = mexFiles(1).folder;
end


function testCasadi()
%TESTCASADI Check if CasADi can be imported and used.

    import casadi.*

    x = MX.sym('x');
    y = jacobian(sin(x), x);

    fprintf("[LTO APP] CasADi test expression:\n");
    disp(y);
end
function p = prop_paths()
%PROP_PATHS  lib = 脚本和 CFD 原文；results = 中间产物

    p.lib = fileparts(mfilename('fullpath'));
    p.root = fileparts(p.lib);
    p.results = fullfile(p.root, 'results');
    if ~isfolder(p.results)
        mkdir(p.results);
    end
end

function main()
%MAIN  举升 + 倾转：从 CFD 原文建表并装进 base 工作区
%
%   cd 到本目录后：
%     main
%     open_system('hover_prop_table')   % 或 tilt_prop_table

    root = fileparts(mfilename('fullpath'));
    addpath(root);
    addpath(fullfile(root, 'lib'));

    fprintf('=== hover: CFD -> table ===\n');
    read_hover_cfd;
    nondim_hover_cfd;
    build_hover_2d_table('method', 'linear');

    fprintf('=== tilt: CFD -> table ===\n');
    read_cfd_data;
    nondim_cfd_data;
    build_cfd_3d_table('method', 'linear');

    fprintf('=== load both tables into base ===\n');
    load_prop_tables;
    fprintf('done: cfd2d_* and cfd3d_* are in base. Open an slx and Run.\n');
end

function load_prop_tables()
%LOAD_PROP_TABLES  举升 2D + 倾转 3D 都装进 base（互不覆盖）

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    hover_2d_to_workspace;
    cfd_3d_to_workspace;
end

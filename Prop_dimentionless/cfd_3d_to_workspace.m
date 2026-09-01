function cfd_3d_to_workspace()
%CFD_3D_TO_WORKSPACE  把三维倾转桨表拆到 base，供 Simulink n-D Lookup
%
% 断点：cfd3d_ja, cfd3d_jl, cfd3d_pitch
% 表：cfd3d_P2_CEF_X ... cfd3d_P2_CEM_Z，cfd3d_P6_* 同名
% 常数：cfd3d_D, cfd3d_rho, cfd3d_kT, cfd3d_kQ, cfd3d_aInf, cfd3d_cantDeg

    here = fileparts(mfilename('fullpath'));
    f = fullfile(here, 'CFD_DATA_3D.mat');
    if ~exist(f, 'file')
        error('cfd_3d_to_workspace:NoTable', '先运行 build_cfd_3d_table。');
    end
    S = load(f, 'cfd3d');
    L = S.cfd3d;
    assignin('base', 'cfd3d', L);
    assignin('base', 'cfd3d_ja', L.ja);
    assignin('base', 'cfd3d_jl', L.jl);
    assignin('base', 'cfd3d_pitch', L.pitch);
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    for k = 1:6
        assignin('base', ['cfd3d_P2_' coeff{k}], L.prop2.(coeff{k}));
        assignin('base', ['cfd3d_P6_' coeff{k}], L.prop6.(coeff{k}));
    end
    assignin('base', 'cfd3d_P2_valid', double(L.prop2.valid));
    assignin('base', 'cfd3d_P6_valid', double(L.prop6.valid));
    assignin('base', 'cfd3d_D', L.D);
    assignin('base', 'cfd3d_rho', L.rho);
    assignin('base', 'cfd3d_kT', L.kT);
    assignin('base', 'cfd3d_kQ', L.kQ);
    assignin('base', 'cfd3d_aInf', L.aInf);
    assignin('base', 'cfd3d_cantDeg', L.cantDeg);
    fprintf('assigned cfd3d_* to base  Ja=%d  Jl=%d  pitch=%d  kT=%.4f  kQ=%.4f\n', ...
        numel(L.ja), numel(L.jl), numel(L.pitch), L.kT, L.kQ);
end

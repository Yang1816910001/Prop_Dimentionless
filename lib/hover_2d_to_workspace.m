function hover_2d_to_workspace()
%HOVER_2D_TO_WORKSPACE  把二维举升桨表拆到 base，供 Simulink n-D Lookup
%
% 断点：cfd2d_ja, cfd2d_jl
% 表：cfd2d_P1_CEF_X ... cfd2d_P1_CEM_Z，cfd2d_P5_* 同名
% 常数：cfd2d_D, cfd2d_rho, cfd2d_kT, cfd2d_kQ, cfd2d_aInf, cfd2d_cantDeg

    io = prop_paths();
    f = fullfile(io.results, 'CFD_HOVER_2D.mat');
    if ~exist(f, 'file')
        error('hover_2d_to_workspace:NoTable', '先运行 build_hover_2d_table。');
    end
    S = load(f, 'cfd2d');
    L = S.cfd2d;
    assignin('base', 'cfd2d', L);
    assignin('base', 'cfd2d_ja', L.ja);
    assignin('base', 'cfd2d_jl', L.jl);
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    for k = 1:6
        assignin('base', ['cfd2d_P1_' coeff{k}], L.prop1.(coeff{k}));
        assignin('base', ['cfd2d_P5_' coeff{k}], L.prop5.(coeff{k}));
    end
    assignin('base', 'cfd2d_P1_valid', double(L.prop1.valid));
    assignin('base', 'cfd2d_P5_valid', double(L.prop5.valid));
    assignin('base', 'cfd2d_D', L.D);
    assignin('base', 'cfd2d_rho', L.rho);
    assignin('base', 'cfd2d_kT', L.kT);
    assignin('base', 'cfd2d_kQ', L.kQ);
    assignin('base', 'cfd2d_aInf', L.aInf);
    assignin('base', 'cfd2d_cantDeg', L.cantDeg);
    fprintf('assigned cfd2d_* to base  Ja=%d  Jl=%d  kT=%.4f  kQ=%.4f\n', ...
        numel(L.ja), numel(L.jl), L.kT, L.kQ);
end

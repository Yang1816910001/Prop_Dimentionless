function T = nondim_cfd_data(varargin)
%NONDIM_CFD_DATA  将 CFD 力/矩换成螺旋桨系数，并计算轴向/侧向进距比
%
% T = nondim_cfd_data
% T = nondim_cfd_data('D', 3.0, 'rho', 1.225)
%
% 力/矩（每台桨用自己的 n）：
%   C_ef = F / (rho * n^2 * D^4)
%   C_em = M / (rho * n^2 * D^5)
%   n = RPM/60  (rps)
%
% 进距比：见 engine_inflow.m（含攻角 α、侧滑 β）
%   表内 α=β=0，此处按 0 调用；运行时把真实 α、β 传进去
%   Ja = Vx_eng / (n D),  Jl = hypot(Vy,Vz)_eng / (n D)
%   α=β=0 时 Ja = V cos(tilt)/(nD), Jl = V |sin(tilt)|/(nD)
%
% 马赫：悬停点拟合 C = C0 (1+k M_tip^2)，建表存 C_inc = C_cfd / (1+k M^2)
%   M_tip = pi n D / a_inf
%   实时：C = cfd_apply_mach(C_inc, n, D, k, a_inf)
%
% 读取 CFD_DATA_clean.txt，写出 CFD_DATA_nondim.txt / .mat

    p = inputParser;
    addParameter(p, 'dataFile', '', @(s) ischar(s) || isstring(s));
    addParameter(p, 'D', 3.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'rho', 1.225, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'cantDeg', 7, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'aInf', 340.294, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});

    io = prop_paths();
    dataFile = char(p.Results.dataFile);
    if isempty(dataFile)
        dataFile = fullfile(io.results, 'CFD_DATA_clean.txt');
    end
    D = p.Results.D;
    rho = p.Results.rho;
    cantDeg = p.Results.cantDeg;
    aInf = p.Results.aInf;

    opts = detectImportOptions(dataFile, 'FileType', 'text');
    if any(strcmp(properties(opts), 'VariableNamingRule'))
        opts.VariableNamingRule = 'preserve';
    end
    T = readtable(dataFile, opts);

    if any(strcmp(T.Properties.VariableNames, 'HOVER PROP RPM'))
        T.('HOVER PROP RPM') = [];
    end

    V = T.('VELOCITY MPS');
    tiltDeg = T.('TILT ANGLE DEG');
    n2 = T.('PROP 2 RPM') / 60;
    n6 = T.('PROP 6 RPM') / 60;

    % 2/6 同为左侧，外倾符号相同。建表仍用 α=β=0；函数本身支持非零攻角/侧滑
    a0 = zeros(size(V));
    b0 = zeros(size(V));
    [~, ~, inflow] = engine_inflow(V, a0, b0, tiltDeg, cantDeg, n2, D);
    Va = inflow.Va;
    Vlat = inflow.Vlat;
    T.PROP2_JA = Va ./ (n2 * D);
    T.PROP2_JL = Vlat ./ (n2 * D);
    T.PROP6_JA = Va ./ (n6 * D);
    T.PROP6_JL = Vlat ./ (n6 * D);

    qF2 = rho .* n2.^2 .* D^4;
    qM2 = rho .* n2.^2 .* D^5;
    qF6 = rho .* n6.^2 .* D^4;
    qM6 = rho .* n6.^2 .* D^5;

    T.PROP2_CEF_X = T.PROP2_FX ./ qF2;
    T.PROP2_CEF_Y = T.PROP2_FY ./ qF2;
    T.PROP2_CEF_Z = T.PROP2_FZ ./ qF2;
    T.PROP2_CEM_X = T.PROP2_MX ./ qM2;
    T.PROP2_CEM_Y = T.PROP2_MY ./ qM2;
    T.PROP2_CEM_Z = T.PROP2_MZ ./ qM2;
    T.PROP6_CEF_X = T.PROP6_FX ./ qF6;
    T.PROP6_CEF_Y = T.PROP6_FY ./ qF6;
    T.PROP6_CEF_Z = T.PROP6_FZ ./ qF6;
    T.PROP6_CEM_X = T.PROP6_MX ./ qM6;
    T.PROP6_CEM_Y = T.PROP6_MY ./ qM6;
    T.PROP6_CEM_Z = T.PROP6_MZ ./ qM6;

    M2_2 = (pi * n2 * D / aInf).^2;
    M2_6 = (pi * n6 * D / aInf).^2;
    hover = (V == 0);
    kT = fit_mach_k(T.PROP2_CEF_X(hover), M2_2(hover));
    kQ = fit_mach_k(T.PROP2_CEM_X(hover), M2_2(hover));
    ctBefore = T.PROP2_CEF_X(hover);
    cqBefore = T.PROP2_CEM_X(hover);

    cef2 = {'PROP2_CEF_X','PROP2_CEF_Y','PROP2_CEF_Z'};
    cem2 = {'PROP2_CEM_X','PROP2_CEM_Y','PROP2_CEM_Z'};
    cef6 = {'PROP6_CEF_X','PROP6_CEF_Y','PROP6_CEF_Z'};
    cem6 = {'PROP6_CEM_X','PROP6_CEM_Y','PROP6_CEM_Z'};
    for i = 1:3
        T.(cef2{i}) = T.(cef2{i}) ./ (1 + kT * M2_2);
        T.(cem2{i}) = T.(cem2{i}) ./ (1 + kQ * M2_2);
        T.(cef6{i}) = T.(cef6{i}) ./ (1 + kT * M2_6);
        T.(cem6{i}) = T.(cem6{i}) ./ (1 + kQ * M2_6);
    end

    T.PROP2_FX = []; T.PROP2_FY = []; T.PROP2_FZ = [];
    T.PROP2_MX = []; T.PROP2_MY = []; T.PROP2_MZ = [];
    T.PROP6_FX = []; T.PROP6_FY = []; T.PROP6_FZ = [];
    T.PROP6_MX = []; T.PROP6_MY = []; T.PROP6_MZ = [];

    T = movevars(T, {'PROP2_JA','PROP2_JL','PROP6_JA','PROP6_JL'}, ...
        'After', 'PROP 6 RPM');

    fprintf('rho = %.4f kg/m^3,  D = %.3f m,  cant = %.1f deg,  a = %.1f m/s\n', ...
        rho, D, cantDeg, aInf);
    dJa = max(abs(Va - V(:) .* cosd(tiltDeg(:))));
    dJl = max(abs(Vlat - abs(V(:) .* sind(tiltDeg(:)))));
    fprintf('alpha=beta=0 check: max|Va-V cosθ|=%.2e, max|Vlat-V sinθ|=%.2e\n', dJa, dJl);
    fprintf('%d 行 × %d 列\n', height(T), width(T));

    if any(hover)
        ct = T.PROP2_CEF_X(hover);
        cq = T.PROP2_CEM_X(hover);
        span0 = (max(ctBefore) - min(ctBefore)) / mean(ctBefore);
        span1 = (max(ct) - min(ct)) / mean(ct);
        fprintf('Mach strip: k_T = %.4f,  k_Q = %.4f  (hover PROP2)\n', kT, kQ);
        fprintf('  C_ef,x hover span/mean: %.2f%% -> %.2f%%\n', 100*span0, 100*span1);
        fprintf('  C_inc C_ef,x: min=%.4f  max=%.4f  mean=%.4f\n', min(ct), max(ct), mean(ct));
        fprintf('  C_inc C_em,x: min=%.5f  max=%.5f  (was %.5f .. %.5f)\n', ...
            min(cq), max(cq), min(cqBefore), max(cqBefore));
    end

    outTxt = fullfile(io.results, 'CFD_DATA_nondim.txt');
    outMat = fullfile(io.results, 'CFD_DATA_nondim.mat');
    writetable(T, outTxt, 'Delimiter', '\t');
    save(outMat, 'T', 'rho', 'D', 'cantDeg', 'aInf', 'kT', 'kQ');
    fprintf('已写入:\n  %s\n  %s\n', outTxt, outMat);

    % 查表维：桨距 + 各桨 (Ja, Jl)；去掉 V / 倾转 / RPM
    Tlut = T;
    Tlut.('VELOCITY MPS') = [];
    Tlut.('TILT ANGLE DEG') = [];
    Tlut.('PROP 2 RPM') = [];
    Tlut.('PROP 6 RPM') = [];
    Tlut = movevars(Tlut, {'PROP2_JA','PROP2_JL','PROP6_JA','PROP6_JL'}, ...
        'Before', 'ROTOR PITCH ANGLE DEG');
    lutTxt = fullfile(io.results, 'CFD_DATA_lookup.txt');
    lutMat = fullfile(io.results, 'CFD_DATA_lookup.mat');
    writetable(Tlut, lutTxt, 'Delimiter', '\t');
    save(lutMat, 'Tlut', 'rho', 'D', 'cantDeg', 'aInf', 'kT', 'kQ');
    fprintf('查表表 %d 行 × %d 列:\n  %s\n  %s\n', height(Tlut), width(Tlut), lutTxt, lutMat);
end

function k = fit_mach_k(C, M2)
% C = C0 (1 + k M^2)  →  C = C0 + (C0 k) M^2
    C = C(:);
    M2 = M2(:);
    p = [ones(size(C)), M2] \ C;
    k = p(2) / p(1);
end

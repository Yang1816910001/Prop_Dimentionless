function T = nondim_hover_cfd(varargin)
%NONDIM_HOVER_CFD  举升桨 CFD 无量纲 + 进距比 + 剥马赫
%
% T = nondim_hover_cfd
% T = nondim_hover_cfd('D', 3.0, 'rho', 1.225, 'cantDeg', 7)
%
% 读 CFD_HOVER_clean.txt。PROP1 / PROP5 各用自己的 n。
% 倾转固定 90°，入流走 lift_inflow（含表内真实 α、β）。
% 悬停 V=0 四点拟合 C = C0 (1+k M_tip^2)，表内存 C_inc。

    p = inputParser;
    addParameter(p, 'dataFile', '', @(s) ischar(s) || isstring(s));
    addParameter(p, 'D', 3.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'rho', 1.225, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'cantDeg', 7, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'aInf', 340.294, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});

    here = fileparts(mfilename('fullpath'));
    dataFile = char(p.Results.dataFile);
    if isempty(dataFile)
        dataFile = fullfile(here, 'CFD_HOVER_clean.txt');
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

    V = T.('VELOCITY MPS');
    aDeg = T.('ALPHA DEG');
    bDeg = T.('BETA DEG');
    n1 = T.('PROP 1 RPM') / 60;
    n5 = T.('PROP 5 RPM') / 60;

    [~, ~, in1] = lift_inflow(V, aDeg, bDeg, n1, D, cantDeg);
    [~, ~, in5] = lift_inflow(V, aDeg, bDeg, n5, D, cantDeg);
    T.PROP1_JA = in1.Va ./ (in1.n * D);
    T.PROP1_JL = in1.Vlat ./ (in1.n * D);
    T.PROP5_JA = in5.Va ./ (in5.n * D);
    T.PROP5_JL = in5.Vlat ./ (in5.n * D);

    qF1 = rho .* n1.^2 .* D^4;
    qM1 = rho .* n1.^2 .* D^5;
    qF5 = rho .* n5.^2 .* D^4;
    qM5 = rho .* n5.^2 .* D^5;

    T.PROP1_CEF_X = T.PROP1_FX ./ qF1;
    T.PROP1_CEF_Y = T.PROP1_FY ./ qF1;
    T.PROP1_CEF_Z = T.PROP1_FZ ./ qF1;
    T.PROP1_CEM_X = T.PROP1_MX ./ qM1;
    T.PROP1_CEM_Y = T.PROP1_MY ./ qM1;
    T.PROP1_CEM_Z = T.PROP1_MZ ./ qM1;
    T.PROP5_CEF_X = T.PROP5_FX ./ qF5;
    T.PROP5_CEF_Y = T.PROP5_FY ./ qF5;
    T.PROP5_CEF_Z = T.PROP5_FZ ./ qF5;
    T.PROP5_CEM_X = T.PROP5_MX ./ qM5;
    T.PROP5_CEM_Y = T.PROP5_MY ./ qM5;
    T.PROP5_CEM_Z = T.PROP5_MZ ./ qM5;

    M2_1 = (pi * n1 * D / aInf).^2;
    M2_5 = (pi * n5 * D / aInf).^2;
    hover = (V == 0);
    kT = fit_mach_k(T.PROP1_CEF_X(hover), M2_1(hover));
    kQ = fit_mach_k(T.PROP1_CEM_X(hover), M2_1(hover));
    ctBefore = T.PROP1_CEF_X(hover);
    cqBefore = T.PROP1_CEM_X(hover);

    cef1 = {'PROP1_CEF_X','PROP1_CEF_Y','PROP1_CEF_Z'};
    cem1 = {'PROP1_CEM_X','PROP1_CEM_Y','PROP1_CEM_Z'};
    cef5 = {'PROP5_CEF_X','PROP5_CEF_Y','PROP5_CEF_Z'};
    cem5 = {'PROP5_CEM_X','PROP5_CEM_Y','PROP5_CEM_Z'};
    for i = 1:3
        T.(cef1{i}) = T.(cef1{i}) ./ (1 + kT * M2_1);
        T.(cem1{i}) = T.(cem1{i}) ./ (1 + kQ * M2_1);
        T.(cef5{i}) = T.(cef5{i}) ./ (1 + kT * M2_5);
        T.(cem5{i}) = T.(cem5{i}) ./ (1 + kQ * M2_5);
    end

    T.PROP1_FX = []; T.PROP1_FY = []; T.PROP1_FZ = [];
    T.PROP1_MX = []; T.PROP1_MY = []; T.PROP1_MZ = [];
    T.PROP5_FX = []; T.PROP5_FY = []; T.PROP5_FZ = [];
    T.PROP5_MX = []; T.PROP5_MY = []; T.PROP5_MZ = [];

    T = movevars(T, {'PROP1_JA','PROP1_JL','PROP5_JA','PROP5_JL'}, ...
        'After', 'PROP 5 RPM');

    fprintf('rho = %.4f kg/m^3,  D = %.3f m,  cant = %.1f deg,  a = %.1f m/s\n', ...
        rho, D, cantDeg, aInf);
    a0 = (abs(aDeg) < 1e-9) & (abs(bDeg) < 1e-9);
    dJa = max(abs(T.PROP1_JA(a0)));
    dJl = max(abs(T.PROP1_JL(a0) - V(a0) ./ (n1(a0) * D)));
    fprintf('alpha=beta=0 check: max|Ja|=%.2e, max|Jl-V/(nD)|=%.2e\n', dJa, dJl);
    fprintf('%d 行 × %d 列\n', height(T), width(T));
    fprintf('Ja range PROP1 [%+.4f, %+.4f]  Jl [%.4f, %.4f]\n', ...
        min(T.PROP1_JA), max(T.PROP1_JA), min(T.PROP1_JL), max(T.PROP1_JL));

    if any(hover)
        ct = T.PROP1_CEF_X(hover);
        cq = T.PROP1_CEM_X(hover);
        span0 = (max(ctBefore) - min(ctBefore)) / mean(ctBefore);
        span1 = (max(ct) - min(ct)) / mean(ct);
        fprintf('Mach strip: k_T = %.4f,  k_Q = %.4f  (hover PROP1)\n', kT, kQ);
        fprintf('  C_ef,x hover span/mean: %.2f%% -> %.2f%%\n', 100*span0, 100*span1);
        fprintf('  C_inc C_ef,x: min=%.4f  max=%.4f  mean=%.4f\n', min(ct), max(ct), mean(ct));
        fprintf('  C_inc C_em,x: min=%.5f  max=%.5f  (was %.5f .. %.5f)\n', ...
            min(cq), max(cq), min(cqBefore), max(cqBefore));
    end

    outTxt = fullfile(here, 'CFD_HOVER_nondim.txt');
    outMat = fullfile(here, 'CFD_HOVER_nondim.mat');
    writetable(T, outTxt, 'Delimiter', '\t');
    save(outMat, 'T', 'rho', 'D', 'cantDeg', 'aInf', 'kT', 'kQ');
    fprintf('已写入:\n  %s\n  %s\n', outTxt, outMat);

    Tlut = T;
    Tlut.('VELOCITY MPS') = [];
    Tlut.('ALPHA DEG') = [];
    Tlut.('BETA DEG') = [];
    Tlut.('PROP 1 RPM') = [];
    Tlut.('PROP 5 RPM') = [];
    lutTxt = fullfile(here, 'CFD_HOVER_lookup.txt');
    lutMat = fullfile(here, 'CFD_HOVER_lookup.mat');
    writetable(Tlut, lutTxt, 'Delimiter', '\t');
    save(lutMat, 'Tlut', 'rho', 'D', 'cantDeg', 'aInf', 'kT', 'kQ');
    fprintf('查表表 %d 行 × %d 列:\n  %s\n  %s\n', height(Tlut), width(Tlut), lutTxt, lutMat);
end

function k = fit_mach_k(C, M2)
    C = C(:);
    M2 = M2(:);
    p = [ones(size(C)), M2] \ C;
    k = p(2) / p(1);
end

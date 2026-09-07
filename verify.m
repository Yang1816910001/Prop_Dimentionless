function verify()
%VERIFY  核对插值表 + 两份 slx 能否还原 CFD 力
%
%   cd 到 Prop_dimentionless 后：
%     main      % 若表还没建
%     verify

    root = fileparts(mfilename('fullpath'));
    addpath(root);
    addpath(fullfile(root, 'lib'));
    load_prop_tables;

    fprintf('\n======== MATLAB 表回放 vs CFD ========\n');
    verify_hover_cfd(root);
    verify_tilt_cfd(root);

    fprintf('\n======== Simulink 测试常数 vs MATLAB ========\n');
    verify_slx(root, 'hover_prop_table', 'hover');
    verify_slx(root, 'tilt_prop_table', 'tilt');
end

function verify_hover_cfd(root)
    Tc = load(fullfile(root, 'results', 'CFD_HOVER_clean.mat')).T;
    Nd = load(fullfile(root, 'results', 'CFD_HOVER_nondim.mat'));
    L = evalin('base', 'cfd2d');
    V = Tc.('VELOCITY MPS');
    aDeg = Tc.('ALPHA DEG');
    bDeg = Tc.('BETA DEG');
    rpm1 = Tc.('PROP 1 RPM');
    rpm5 = Tc.('PROP 5 RPM');
    [F1, ~, p1] = reconstruct_hover(V, aDeg, bDeg, Nd.cantDeg, rpm1, 1, L);
    [F5, ~, p5] = reconstruct_hover(V, aDeg, bDeg, Nd.cantDeg, rpm5, 5, L);
    Fc1 = [Tc.PROP1_FX, Tc.PROP1_FY, Tc.PROP1_FZ];
    Fc5 = [Tc.PROP5_FX, Tc.PROP5_FY, Tc.PROP5_FZ];
    a0 = abs(aDeg) < 1e-9 & abs(bDeg) < 1e-9;
    [~, ~, in1] = lift_inflow(V, aDeg, bDeg, rpm1/60, Nd.D, Nd.cantDeg);
    fprintf('\n--- hover 表 ---\n');
    fprintf('α=β=0  max|Ja|=%.3e  max|Vlat-V|=%.3e\n', ...
        max(abs(in1.Va(a0))), max(abs(in1.Vlat(a0) - V(a0))));
    print_fx('PROP1', F1(:,1), Fc1(:,1), p1.inHull, V, aDeg);
    print_fx('PROP5', F5(:,1), Fc5(:,1), p5.inHull, V, aDeg);
end

function verify_tilt_cfd(root)
    Tc = load(fullfile(root, 'results', 'CFD_DATA_clean.mat')).T;
    Nd = load(fullfile(root, 'results', 'CFD_DATA_nondim.mat'));
    L = evalin('base', 'cfd3d');
    V = Tc.('VELOCITY MPS');
    tilt = Tc.('TILT ANGLE DEG');
    pitch = Tc.('ROTOR PITCH ANGLE DEG');
    rpm2 = Tc.('PROP 2 RPM');
    rpm6 = Tc.('PROP 6 RPM');
    a0 = zeros(size(V));
    b0 = zeros(size(V));
    [F2, ~, p2] = reconstruct_tilt(V, a0, b0, tilt, Nd.cantDeg, rpm2, pitch, 2, L);
    [F6, ~, p6] = reconstruct_tilt(V, a0, b0, tilt, Nd.cantDeg, rpm6, pitch, 6, L);
    Fc2 = [Tc.PROP2_FX, Tc.PROP2_FY, Tc.PROP2_FZ];
    Fc6 = [Tc.PROP6_FX, Tc.PROP6_FY, Tc.PROP6_FZ];
    [~, ~, P] = engine_inflow(V, a0, b0, tilt, Nd.cantDeg, rpm2/60, Nd.D);
    fprintf('\n--- tilt 表 ---\n');
    fprintf('α=β=0  max|Va-V cosθ|=%.3e  max|Vlat-V|sinθ||=%.3e\n', ...
        max(abs(P.Va - V .* cosd(tilt))), max(abs(P.Vlat - abs(V .* sind(tilt)))));
    print_fx('PROP2', F2(:,1), Fc2(:,1), p2.inHull, V, tilt);
    print_fx('PROP6', F6(:,1), Fc6(:,1), p6.inHull, V, tilt);
end

function print_fx(name, F, Fc, inH, p1, p2)
    rel = abs(F - Fc) ./ max(abs(Fc), 1);
    fprintf('%s FX  in-hull n=%d  relRMS=%.2f%%  relMax=%.2f%%\n', ...
        name, nnz(inH), 100*rms(rel(inH)), 100*max(rel(inH)));
    fprintf('         all     n=%d  relRMS=%.2f%%  relMax=%.2f%%\n', ...
        numel(F), 100*rms(rel), 100*max(rel));
    [~, iw] = max(rel);
    fprintf('         worst i=%d  p1=%.3g p2=%.3g  FX_lut=%.3g  FX_cfd=%.3g\n', ...
        iw, p1(iw), p2(iw), F(iw), Fc(iw));
end

function [F, M, pack] = reconstruct_hover(V, a, b, cant, rpm, prop, L)
    n = rpm(:) / 60;
    [Ja, Jl, inflow] = lift_inflow(V, a, b, n, L.D, cant);
    P = L.(sprintf('prop%d', prop));
    [Cef, Cem, inH] = lookup2(L.ja, L.jl, P, Ja, Jl);
    F = scale_force(Cef, inflow.n, L, 'F');
    M = scale_force(Cem, inflow.n, L, 'M');
    pack = struct('Ja', Ja, 'Jl', Jl, 'inHull', inH);
end

function [F, M, pack] = reconstruct_tilt(V, a, b, tilt, cant, rpm, pitch, prop, L)
    n = rpm(:) / 60;
    [Ja, Jl, inflow] = engine_inflow(V, a, b, tilt, cant, n, L.D);
    P = L.(sprintf('prop%d', prop));
    [Cef, Cem, inH] = lookup3(L.ja, L.jl, L.pitch, P, Ja, Jl, pitch(:));
    F = scale_force(Cef, inflow.n, L, 'F');
    M = scale_force(Cem, inflow.n, L, 'M');
    pack = struct('Ja', Ja, 'Jl', Jl, 'inHull', inH);
end

function [Cef, Cem, inH] = lookup2(ja, jl, P, Ja, Jl)
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    C = zeros(numel(Ja), 6);
    for k = 1:6
        G = griddedInterpolant({ja, jl}, P.(coeff{k}), 'linear', 'nearest');
        C(:, k) = G(Ja, Jl);
    end
    Cef = C(:, 1:3);
    Cem = C(:, 4:6);
    Gv = griddedInterpolant({ja, jl}, double(P.valid), 'nearest', 'nearest');
    inBox = Ja >= min(ja) & Ja <= max(ja) & Jl >= min(jl) & Jl <= max(jl);
    inH = inBox & Gv(Ja, Jl) > 0.5;
end

function [Cef, Cem, inH] = lookup3(ja, jl, pitch, P, Ja, Jl, th)
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    C = zeros(numel(Ja), 6);
    for k = 1:6
        G = griddedInterpolant({ja, jl, pitch}, P.(coeff{k}), 'linear', 'nearest');
        C(:, k) = G(Ja, Jl, th);
    end
    Cef = C(:, 1:3);
    Cem = C(:, 4:6);
    Gv = griddedInterpolant({ja, jl, pitch}, double(P.valid), 'nearest', 'nearest');
    inBox = Ja >= min(ja) & Ja <= max(ja) & Jl >= min(jl) & Jl <= max(jl) ...
        & th >= min(pitch) & th <= max(pitch);
    inH = inBox & Gv(Ja, Jl, th) > 0.5;
end

function FM = scale_force(Cinc, n, L, kind)
    Mtip2 = (pi .* n .* L.D ./ L.aInf).^2;
    if kind == 'F'
        C = Cinc .* (1 + L.kT .* Mtip2);
        FM = C .* (L.rho .* n.^2 .* L.D^4);
    else
        C = Cinc .* (1 + L.kQ .* Mtip2);
        FM = C .* (L.rho .* n.^2 .* L.D^5);
    end
end

function verify_slx(root, mdl, kind)
    slx = fullfile(root, [mdl '.slx']);
    if ~isfile(slx)
        fprintf('%s  missing, skip\n', mdl);
        return
    end
    load_system(mdl);
    cleaner = onCleanup(@() bdclose(mdl));
    rpm = str2double(get_param([mdl '/Constant2'], 'Value'));
    cant = str2double(get_param([mdl '/Constant3'], 'Value'));
    alpha = str2double(get_param([mdl '/Constant5'], 'Value'));
    beta = str2double(get_param([mdl '/Constant6'], 'Value'));
    V = str2double(get_param([mdl '/Constant8'], 'Value'));
    if strcmp(kind, 'hover')
        L = evalin('base', 'cfd2d');
        tilt = str2double(get_param([mdl '/Constant4'], 'Value'));
        pitch = NaN;
        [Fml, Mml] = reconstruct_hover(V, alpha, beta, cant, rpm, 1, L);
        [F5, M5] = reconstruct_hover(V, alpha, beta, cant, rpm, 5, L);
        tag = sprintf('hover Const V=%.3g a=%.3g rpm=%.0f tilt=%.0f', V, alpha, rpm, tilt);
    else
        L = evalin('base', 'cfd3d');
        tilt = str2double(get_param([mdl '/Constant4'], 'Value'));
        pitch = str2double(get_param([mdl '/Constant7'], 'Value'));
        [Fml, Mml] = reconstruct_tilt(V, alpha, beta, tilt, cant, rpm, pitch, 2, L);
        [F5, M5] = reconstruct_tilt(V, alpha, beta, tilt, cant, rpm, pitch, 6, L);
        tag = sprintf('tilt Const V=%.3g tilt=%.0f rpm=%.0f pitch=%.1f', V, tilt, rpm, pitch);
    end
    out = sim(mdl, 'StopTime', '0.05', 'SrcWorkspace', 'base', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
    y = out.yout;
    names = {'FX_2','MX_2','FY_2','FZ_2','MY_2','MZ_2', ...
             'FX_6','MX_6','FY_6','FZ_6','MY_6','MZ_6'};
    ml = [Fml(1,:), Mml(1,:), F5(1,:), M5(1,:)];
    % Outports: FX MX FY FZ MY MZ then engine 6
    ml = [Fml(1,1), Mml(1,1), Fml(1,2), Fml(1,3), Mml(1,2), Mml(1,3), ...
          F5(1,1),  M5(1,1),  F5(1,2),  F5(1,3),  M5(1,2),  M5(1,3)];
    fprintf('\n--- %s ---\n%s\n', mdl, tag);
    ok = true;
    for i = 1:numel(names)
        sl = y.getElement(i).Values.Data(end);
        den = max(abs(ml(i)), 1);
        rel = abs(sl - ml(i)) / den;
        flag = '';
        if rel > 0.02
            flag = '  **';
            ok = false;
        end
        fprintf('  %-5s  slx=%12.4f  ml=%12.4f  d=%.2f%%%s\n', ...
            names{i}, sl, ml(i), 100*rel, flag);
    end
    if ok
        fprintf('  Simulink matches MATLAB within 2%% at this constant case.\n');
    else
        fprintf('  mismatch >2%% (often DCM/Inport still on Constants, not the same inflow as reconstruct).\n');
    end
end

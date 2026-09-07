function cfd3d = build_cfd_3d_table(varargin)
%BUILD_CFD_3D_TABLE  把散点 C_inc(Ja,Jl,桨距) 铺成规则三维表
%
% cfd3d = build_cfd_3d_table
% cfd3d = build_cfd_3d_table('method', 'smooth')      % 每个桨距一张二维薄板（默认）
% cfd3d = build_cfd_3d_table('method', 'smooth3')     % 三维薄板（Ja,Jl,θ 一起）
% cfd3d = build_cfd_3d_table('method', 'spline')      % 过点双调和（griddata v4，二维切片）
% cfd3d = build_cfd_3d_table('method', 'linear')      % 凸包内线性（按切片）
% cfd3d = build_cfd_3d_table('nJa', 31, 'nJl', 22, 'smoothLam', 3e-3)
%
% 网格：Ja = linspace(0, maxJa, nJa)，Jl = linspace(0, maxJl, nJl)
% 桨距断点 = CFD 出现过的 6 个值，不是均匀的。
% method='smooth'：每个桨距在 (Ja,Jl) 上做二维薄板 φ=r²log r。
% 凸包外仍用最近邻填满矩形（Simulink Clip 只钳矩形）。

    p = inputParser;
    addParameter(p, 'nJa', 31, @(x) isnumeric(x) && isscalar(x) && x >= 5);
    addParameter(p, 'nJl', 22, @(x) isnumeric(x) && isscalar(x) && x >= 5);
    addParameter(p, 'jaMax', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
    addParameter(p, 'jlMax', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
    addParameter(p, 'method', 'linear', @(s) any(strcmpi(char(s), ...
        {'linear', 'spline', 'smooth', 'smooth2', 'smooth3'})));
    addParameter(p, 'smoothLam', 3e-3, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    parse(p, varargin{:});
    method = lower(char(p.Results.method));
    lam = p.Results.smoothLam;

    io = prop_paths();
    [T, meta] = load_lookup(io.results);

    pitchAll = T.('ROTOR PITCH ANGLE DEG');
    pitch_bp = unique(round(pitchAll, 1));
    nP = numel(pitch_bp);

    ja2 = T.PROP2_JA;
    jl2 = T.PROP2_JL;
    ja6 = T.PROP6_JA;
    jl6 = T.PROP6_JL;
    jaMax = p.Results.jaMax;
    jlMax = p.Results.jlMax;
    if isempty(jaMax)
        jaMax = max([ja2; ja6]);
    end
    if isempty(jlMax)
        jlMax = max([jl2; jl6]);
    end
    ja_bp = linspace(0, jaMax, p.Results.nJa).';
    jl_bp = linspace(0, jlMax, p.Results.nJl).';
    nJa = numel(ja_bp);
    nJl = numel(jl_bp);

    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    prop2 = empty_prop(nJa, nJl, nP, coeff);
    prop6 = empty_prop(nJa, nJl, nP, coeff);

    if strcmp(method, 'smooth3')
        fprintf('3D thin-plate (phi=r)  lamRel=%g  unique pts PROP2/6 after group:\n', lam);
        for k = 1:6
            [g, v, nU2] = fill_volume(ja2, jl2, pitchAll, T.(['PROP2_' coeff{k}]), ...
                ja_bp, jl_bp, pitch_bp, lam);
            prop2.(coeff{k}) = g;
            if k == 1
                prop2.valid = v;
                fprintf('  PROP2 unique (Ja,Jl,θ)=%d  3D-hull %.1f%% of grid\n', ...
                    nU2, 100*mean(v, 'all'));
            end
            [g, v, nU6] = fill_volume(ja6, jl6, pitchAll, T.(['PROP6_' coeff{k}]), ...
                ja_bp, jl_bp, pitch_bp, lam);
            prop6.(coeff{k}) = g;
            if k == 1
                prop6.valid = v;
                fprintf('  PROP6 unique (Ja,Jl,θ)=%d  3D-hull %.1f%% of grid\n', ...
                    nU6, 100*mean(v, 'all'));
            end
        end
        for ip = 1:nP
            fprintf('  pitch %5.1f deg: PROP2 hull %4.1f%%  PROP6 hull %4.1f%%\n', ...
                pitch_bp(ip), 100*mean(prop2.valid(:,:,ip), 'all'), ...
                100*mean(prop6.valid(:,:,ip), 'all'));
        end
    else
        sliceMethod = method;
        if any(strcmp(method, {'smooth', 'smooth2'}))
            sliceMethod = 'smooth';
        end
        for ip = 1:nP
            m = abs(pitchAll - pitch_bp(ip)) < 0.05;
            for k = 1:6
                [g, v] = fill_slice(ja2(m), jl2(m), T.(['PROP2_' coeff{k}])(m), ...
                    ja_bp, jl_bp, sliceMethod, lam);
                prop2.(coeff{k})(:, :, ip) = g;
                if k == 1
                    prop2.valid(:, :, ip) = v;
                end
                [g, v] = fill_slice(ja6(m), jl6(m), T.(['PROP6_' coeff{k}])(m), ...
                    ja_bp, jl_bp, sliceMethod, lam);
                prop6.(coeff{k})(:, :, ip) = g;
                if k == 1
                    prop6.valid(:, :, ip) = v;
                end
            end
            fprintf('pitch %5.1f deg: PROP2 hull %4.1f%%  PROP6 hull %4.1f%%  (n=%d)\n', ...
                pitch_bp(ip), 100*mean(prop2.valid(:,:,ip), 'all'), ...
                100*mean(prop6.valid(:,:,ip), 'all'), nnz(m));
        end
    end

    cfd3d = meta;
    cfd3d.ja = ja_bp;
    cfd3d.jl = jl_bp;
    cfd3d.pitch = pitch_bp;
    cfd3d.prop2 = prop2;
    cfd3d.prop6 = prop6;
    cfd3d.fitMethod = method;
    cfd3d.smoothLam = lam;
    cfd3d.dims = {'Ja', 'Jl', 'rotor_pitch_deg'};
    if strcmp(method, 'smooth3')
        fitNote = sprintf(['3D Duchon thin-plate phi=r on (Ja,Jl,pitch), ', ...
            'axes normalized, smoothLam=%g. Hull=3D Delaunay; outside=nearest. '], lam);
    else
        fitNote = sprintf(['Slice fit method=%s (smoothLam=%g). ', ...
            'Hull=per-pitch 2D; outside=nearest. '], method, lam);
    end
    cfd3d.note = sprintf([ ...
        'Grid: Ja linspace(0,max,%d), Jl linspace(0,max,%d), pitch=CFD values. ', ...
        '%sSimulink n-D Lookup: bp1/2/3=cfd3d.ja/jl/pitch, Table=cfd3d.prop2.CEF_X.'], ...
        nJa, nJl, fitNote);

    err = table_rms(cfd3d, T, 2);
    fprintf('method=%s  lam=%g  table [%d %d %d]\n', method, lam, nJa, nJl, nP);
    fprintf('resample RMS PROP2 C_ef,x at CFD points: %.4g  max|d|=%.4g\n', err.rms, err.max);
    err6 = table_rms(cfd3d, T, 6);
    fprintf('resample RMS PROP6 C_ef,x at CFD points: %.4g  max|d|=%.4g\n', err6.rms, err6.max);
    cfd3d.resample_err_prop2_cef_x = err;
    cfd3d.resample_err_prop6_cef_x = err6;

    outMat = fullfile(io.results, 'CFD_DATA_3D.mat');
    save(outMat, 'cfd3d');
    fprintf('wrote %s  table size [%d %d %d]\n', outMat, nJa, nJl, nP);
    cfd_3d_to_workspace;
end

function P = empty_prop(nJa, nJl, nP, coeff)
    P = struct();
    for k = 1:numel(coeff)
        P.(coeff{k}) = zeros(nJa, nJl, nP);
    end
    P.valid = false(nJa, nJl, nP);
end

function [G, valid, nU] = fill_volume(ja, jl, th, c, ja_bp, jl_bp, th_bp, lam)
    ja = ja(:); jl = jl(:); th = th(:); c = c(:);
    [grp, jaU, jlU, thU] = findgroups(round(ja, 6), round(jl, 6), round(th, 1));
    cU = splitapply(@mean, c, grp);
    nU = numel(cU);
    [JA, JL, TH] = ndgrid(ja_bp, jl_bp, th_bp);
    G = nan(size(JA));
    valid = inhull3(jaU, jlU, thU, JA, JL, TH);
    if nU >= 5
        G = tps3_eval(jaU, jlU, thU, cU, JA, JL, TH, lam);
        G(~valid) = NaN;
    end
    Fnear = scatteredInterpolant(jaU, jlU, thU, cU, 'nearest', 'nearest');
    miss = ~isfinite(G);
    if any(miss(:))
        G(miss) = Fnear(JA(miss), JL(miss), TH(miss));
    end
end

function [G, valid] = fill_slice(ja, jl, c, ja_bp, jl_bp, method, lam)
    ja = ja(:); jl = jl(:); c = c(:);
    [grp, jaU, jlU] = findgroups(round(ja, 6), round(jl, 6));
    cU = splitapply(@mean, c, grp);
    [JA, JL] = ndgrid(ja_bp, jl_bp);
    G = nan(size(JA));
    valid = false(size(JA));
    nU = numel(cU);
    span = [jaU - mean(jaU), jlU - mean(jlU)];
    if nU >= 3 && rank(span) >= 2
        try
            kh = convhull(jaU, jlU);
            valid = inpolygon(JA, JL, jaU(kh), jlU(kh));
            switch method
                case 'smooth'
                    G = tps_eval(jaU, jlU, cU, JA, JL, lam);
                case 'spline'
                    G = griddata(jaU, jlU, cU, JA, JL, 'v4');
                otherwise
                    Flin = scatteredInterpolant(jaU, jlU, cU, 'linear', 'none');
                    G = Flin(JA, JL);
            end
            G(~valid) = NaN;
        catch
        end
    end
    Fnear = scatteredInterpolant(jaU, jlU, cU, 'nearest', 'nearest');
    miss = ~isfinite(G);
    if any(miss(:))
        G(miss) = Fnear(JA(miss), JL(miss));
    end
end

function valid = inhull3(x, y, t, xi, yi, ti)
% 查询点是否在样本的三维凸包内（坐标先按样本范围归一，避免桨距度数压过 J）。
    pts = [x(:), y(:), t(:)];
    q = [xi(:), yi(:), ti(:)];
    lo = min(pts, [], 1);
    sc = max(max(pts, [], 1) - lo, eps);
    pn = (pts - lo) ./ sc;
    qn = (q - lo) ./ sc;
    pn = uniquetol(pn, 1e-10, 'ByRows', true);
    valid = false(size(xi));
    if size(pn, 1) < 4 || rank(pn - mean(pn, 1), 1e-8) < 3
        return
    end
    try
        TR = delaunayTriangulation(pn);
        loc = pointLocation(TR, qn);
        valid = reshape(~isnan(loc), size(xi));
    catch
    end
end

function zi = tps3_eval(x, y, t, z, xi, yi, ti, lamRel)
% 3D Duchon 薄板（m=2,d=3）：phi = r。多项式 1,x,y,t。三轴各自归一。
% lamRel 相对核中位数；0 为过点。
    x = x(:); y = y(:); t = t(:); z = z(:);
    n = numel(z);
    lo = [min(x), min(y), min(t)];
    sc = [max(x) - lo(1), max(y) - lo(2), max(t) - lo(3)];
    sc = max(sc, eps);
    xn = (x - lo(1)) / sc(1);
    yn = (y - lo(2)) / sc(2);
    tn = (t - lo(3)) / sc(3);
    dx = xn - xn.';
    dy = yn - yn.';
    dt = tn - tn.';
    K = sqrt(dx.^2 + dy.^2 + dt.^2);
    pos = K(K > 0);
    scaleK = median(pos);
    if ~(isfinite(scaleK) && scaleK > 0)
        scaleK = 1;
    end
    lam = lamRel * scaleK;
    P = [ones(n, 1), xn, yn, tn];
    A = [K + lam * eye(n), P; P.', zeros(4)];
    sol = A \ [z; zeros(4, 1)];
    w = sol(1:n);
    a = sol(n+1:end);
    xin = (xi(:) - lo(1)) / sc(1);
    yin = (yi(:) - lo(2)) / sc(2);
    tin = (ti(:) - lo(3)) / sc(3);
    R = sqrt((xin - xn.').^2 + (yin - yn.').^2 + (tin - tn.').^2);
    zi = [ones(numel(xin), 1), xin, yin, tin] * a + R * w;
    zi = reshape(zi, size(xi));
end

function zi = tps_eval(x, y, z, xi, yi, lamRel)
% 2D 薄板片条：phi = r^2 log(r)，岭回归平滑。lamRel 相对核中位数。
    x = x(:); y = y(:); z = z(:);
    n = numel(z);
    xmin = min(x); ymin = min(y);
    sx = max(max(x) - xmin, eps);
    sy = max(max(y) - ymin, eps);
    xn = (x - xmin) / sx;
    yn = (y - ymin) / sy;
    dx = xn - xn.';
    dy = yn - yn.';
    r2 = dx.^2 + dy.^2;
    K = zeros(n);
    nz = r2 > 0;
    K(nz) = 0.5 * r2(nz) .* log(r2(nz));
    pos = K(K ~= 0);
    scaleK = median(abs(pos));
    if ~(isfinite(scaleK) && scaleK > 0)
        scaleK = 1;
    end
    lam = lamRel * scaleK;
    P = [ones(n, 1), xn, yn];
    A = [K + lam * eye(n), P; P.', zeros(3)];
    sol = A \ [z; 0; 0; 0];
    w = sol(1:n);
    a = sol(n+1:end);
    xin = (xi(:) - xmin) / sx;
    yin = (yi(:) - ymin) / sy;
    zi = a(1) + a(2) * xin + a(3) * yin;
    for j = 1:n
        rr = (xin - xn(j)).^2 + (yin - yn(j)).^2;
        phi = zeros(size(rr));
        m = rr > 0;
        phi(m) = 0.5 * rr(m) .* log(rr(m));
        zi = zi + w(j) * phi;
    end
    zi = reshape(zi, size(xi));
end

function err = table_rms(cfd3d, T, prop)
    F = griddedInterpolant({cfd3d.ja, cfd3d.jl, cfd3d.pitch}, ...
        cfd3d.(sprintf('prop%d', prop)).CEF_X, 'linear', 'nearest');
    ja = T.(sprintf('PROP%d_JA', prop));
    jl = T.(sprintf('PROP%d_JL', prop));
    th = T.('ROTOR PITCH ANGLE DEG');
    c0 = T.(sprintf('PROP%d_CEF_X', prop));
    cq = F(ja, jl, th);
    d = cq - c0;
    err = struct('rms', rms(d), 'max', max(abs(d)));
end

function [T, meta] = load_lookup(here)
    matFile = fullfile(here, 'CFD_DATA_lookup.mat');
    txtFile = fullfile(here, 'CFD_DATA_lookup.txt');
    meta = struct('D', 3.0, 'rho', 1.225, 'cantDeg', 7, ...
        'aInf', 340.294, 'kT', 0.170, 'kQ', 0.218);
    if exist(matFile, 'file')
        S = load(matFile);
        T = S.Tlut;
        for f = {'D','rho','cantDeg','aInf','kT','kQ'}
            if isfield(S, f{1})
                meta.(f{1}) = S.(f{1});
            end
        end
    else
        opts = detectImportOptions(txtFile, 'FileType', 'text');
        if any(strcmp(properties(opts), 'VariableNamingRule'))
            opts.VariableNamingRule = 'preserve';
        end
        T = readtable(txtFile, opts);
        warning('CFD_DATA_lookup.mat missing; constants from defaults.');
    end
end

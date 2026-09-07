function cfd2d = build_hover_2d_table(varargin)
%BUILD_HOVER_2D_TABLE  举升桨散点 C_inc(Ja,Jl) 铺成规则二维表
%
% cfd2d = build_hover_2d_table
% cfd2d = build_hover_2d_table('method', 'linear')
%
% 无桨距维。凸包内线性，包外最近邻（Simulink Clip 只钳矩形）。
% Ja 可正可负（攻角会进轴向），网格按保留样本 min/max。
% 默认丢掉 Jl>3：V=40/100rpm、V=50/100rpm、V=50/300rpm。

    p = inputParser;
    addParameter(p, 'nJa', 31, @(x) isnumeric(x) && isscalar(x) && x >= 5);
    addParameter(p, 'nJl', 22, @(x) isnumeric(x) && isscalar(x) && x >= 5);
    addParameter(p, 'jlMax', 3, @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
    addParameter(p, 'method', 'linear', @(s) any(strcmpi(char(s), ...
        {'linear', 'spline', 'smooth'})));
    addParameter(p, 'smoothLam', 3e-3, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    parse(p, varargin{:});
    method = lower(char(p.Results.method));
    lam = p.Results.smoothLam;
    jlMax = p.Results.jlMax;

    io = prop_paths();
    [T, meta] = load_lookup(io.results);
    Tkeep = drop_high_jl(T, io.results, jlMax);

    ja1 = Tkeep.PROP1_JA;
    jl1 = Tkeep.PROP1_JL;
    ja5 = Tkeep.PROP5_JA;
    jl5 = Tkeep.PROP5_JL;
    ja_bp = linspace(min([ja1; ja5]), max([ja1; ja5]), p.Results.nJa).';
    jl_bp = linspace(0, max([jl1; jl5]), p.Results.nJl).';
    nJa = numel(ja_bp);
    nJl = numel(jl_bp);

    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    prop1 = empty_prop(nJa, nJl, coeff);
    prop5 = empty_prop(nJa, nJl, coeff);

    for k = 1:6
        [g, v] = fill_slice(ja1, jl1, Tkeep.(['PROP1_' coeff{k}]), ...
            ja_bp, jl_bp, method, lam);
        prop1.(coeff{k}) = g;
        if k == 1
            prop1.valid = v;
        end
        [g, v] = fill_slice(ja5, jl5, Tkeep.(['PROP5_' coeff{k}]), ...
            ja_bp, jl_bp, method, lam);
        prop5.(coeff{k}) = g;
        if k == 1
            prop5.valid = v;
        end
    end
    fprintf('PROP1 hull %.1f%%  unique (Ja,Jl) after group\n', 100*mean(prop1.valid, 'all'));
    fprintf('PROP5 hull %.1f%%\n', 100*mean(prop5.valid, 'all'));

    cfd2d = meta;
    cfd2d.ja = ja_bp;
    cfd2d.jl = jl_bp;
    cfd2d.prop1 = prop1;
    cfd2d.prop5 = prop5;
    cfd2d.fitMethod = method;
    cfd2d.smoothLam = lam;
    cfd2d.jlMax = max(jl_bp);
    cfd2d.dims = {'Ja', 'Jl'};
    cfd2d.note = sprintf([ ...
        'Lift-prop 2D table, tilt=90, cant=%.1f deg. method=%s. ', ...
        'Dropped Jl>%.3g. Hull=2D Delaunay; outside=nearest.'], ...
        meta.cantDeg, method, jlMax);

    err1 = table_rms(cfd2d, Tkeep, 1);
    err5 = table_rms(cfd2d, Tkeep, 5);
    fprintf('method=%s  table [%d %d]\n', method, nJa, nJl);
    fprintf('resample RMS PROP1 C_ef,x: %.4g  max|d|=%.4g\n', err1.rms, err1.max);
    fprintf('resample RMS PROP5 C_ef,x: %.4g  max|d|=%.4g\n', err5.rms, err5.max);
    cfd2d.resample_err_prop1_cef_x = err1;
    cfd2d.resample_err_prop5_cef_x = err5;

    outMat = fullfile(io.results, 'CFD_HOVER_2D.mat');
    save(outMat, 'cfd2d');
    fprintf('wrote %s  table size [%d %d]\n', outMat, nJa, nJl);
    hover_2d_to_workspace;
end

function T = drop_high_jl(T, here, jlMax)
    if isempty(jlMax)
        return
    end
    drop = T.PROP1_JL > jlMax | T.PROP5_JL > jlMax;
    if ~any(drop)
        return
    end
    ndFile = fullfile(here, 'CFD_HOVER_nondim.mat');
    fprintf('drop Jl>%.3g  (%d rows):\n', jlMax, nnz(drop));
    if exist(ndFile, 'file')
        Nd = load(ndFile);
        Tn = Nd.T;
        V = Tn.('VELOCITY MPS');
        aDeg = Tn.('ALPHA DEG');
        rpm1 = Tn.('PROP 1 RPM');
        rpm5 = Tn.('PROP 5 RPM');
        idx = find(drop);
        for k = 1:numel(idx)
            i = idx(k);
            fprintf('  i=%d  V=%.0f m/s  alpha=%.0f deg  RPM1/5=%.0f/%.0f  Jl1=%.3f  Jl5=%.3f\n', ...
                i, V(i), aDeg(i), rpm1(i), rpm5(i), T.PROP1_JL(i), T.PROP5_JL(i));
        end
    else
        idx = find(drop);
        for k = 1:numel(idx)
            i = idx(k);
            fprintf('  i=%d  Ja1=%.3f  Jl1=%.3f  Ja5=%.3f  Jl5=%.3f\n', ...
                i, T.PROP1_JA(i), T.PROP1_JL(i), T.PROP5_JA(i), T.PROP5_JL(i));
        end
    end
    T(drop, :) = [];
end

function P = empty_prop(nJa, nJl, coeff)
    P = struct();
    for k = 1:numel(coeff)
        P.(coeff{k}) = zeros(nJa, nJl);
    end
    P.valid = false(nJa, nJl);
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

function zi = tps_eval(x, y, z, xi, yi, lamRel)
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

function err = table_rms(cfd2d, T, prop)
    F = griddedInterpolant({cfd2d.ja, cfd2d.jl}, ...
        cfd2d.(sprintf('prop%d', prop)).CEF_X, 'linear', 'nearest');
    ja = T.(sprintf('PROP%d_JA', prop));
    jl = T.(sprintf('PROP%d_JL', prop));
    c0 = T.(sprintf('PROP%d_CEF_X', prop));
    d = F(ja, jl) - c0;
    err = struct('rms', rms(d), 'max', max(abs(d)));
end

function [T, meta] = load_lookup(here)
    matFile = fullfile(here, 'CFD_HOVER_lookup.mat');
    meta = struct('D', 3.0, 'rho', 1.225, 'cantDeg', 7, ...
        'aInf', 340.294, 'kT', 0.170, 'kQ', 0.218);
    S = load(matFile);
    T = S.Tlut;
    for f = {'D','rho','cantDeg','aInf','kT','kQ'}
        if isfield(S, f{1})
            meta.(f{1}) = S.(f{1});
        end
    end
end

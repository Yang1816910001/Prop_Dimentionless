function [Ja, Jl, pack] = engine_inflow(V, alphaDeg, betaDeg, tiltDeg, cantDeg, n, D)
%ENGINE_INFLOW  体轴来流（含攻角/侧滑）→ 发动机系进距比 Ja, Jl
%
% [Ja, Jl] = engine_inflow(V, alphaDeg, betaDeg, tiltDeg, cantDeg, n, D)
% [Ja, Jl, pack] = engine_inflow(...)
%
% 体轴：x 前、y 右、z 上。V 是空速标量 (m/s)，α、β、倾转、外倾为度。
% n 为 rps（RPM/60），D 为桨径 (m)。
%
%   V_body = V * [cosα cosβ;  sinβ;  -sinα cosβ]
%   C_be   = Rx(Γ) * Ry(-θ_t)     % 短舱刚体：先绕 −y 倾转，再绕 +x 外倾
%   V_eng  = C_be.' * V_body
%   Ja     = Vx_eng / (n D)
%   Jl     = hypot(Vy_eng, Vz_eng) / (n D)
%
% α=β=0 时退化为 Ja = V cosθ/(nD)，Jl = V |sinθ|/(nD)，与外倾无关。
% 当前 CFD 表没有 α、β 样本；函数仍按上式算，查表时 Jl 只保留盘面速度模。
%
% pack 还给出发动机系迎角/侧滑（度）：α_e = atan2(w_e,u_e)，β_e = asin(v_e/|V|)。

    %#codegen

    V = V(:);
    N = numel(V);
    alphaDeg = expand_col(alphaDeg, N);
    betaDeg  = expand_col(betaDeg, N);
    tiltDeg  = expand_col(tiltDeg, N);
    cantDeg  = expand_col(cantDeg, N);
    n        = expand_col(n, N);
    D        = expand_col(D, N);

    ca = cosd(alphaDeg);
    sa = sind(alphaDeg);
    cb = cosd(betaDeg);
    sb = sind(betaDeg);
    ub = V .* ca .* cb;
    vb = V .* sb;
    wb = -V .* sa .* cb;

    Va   = zeros(N, 1);
    Vlat = zeros(N, 1);
    ue   = zeros(N, 1);
    ve   = zeros(N, 1);
    we   = zeros(N, 1);
    Cbe  = zeros(3, 3, N);
    nSafe = max(n, 1e-4);

    for i = 1:N
        Ci = engine_cbe(tiltDeg(i), cantDeg(i));
        Vi = Ci * [ub(i); vb(i); wb(i)];
        ue(i) = Vi(1);
        ve(i) = Vi(2);
        we(i) = Vi(3);
        Va(i) = Vi(1);
        Vlat(i) = hypot(Vi(2), Vi(3));
        Cbe(:, :, i) = Ci;
    end

    nD = nSafe .* D;
    Ja = Va ./ nD;
    Jl = Vlat ./ nD;

    spd = sqrt(ue.^2 + ve.^2 + we.^2);
    aEng = zeros(N, 1);
    bEng = zeros(N, 1);
    moving = spd > 1e-6;
    aEng(moving) = atan2d(we(moving), ue(moving));
    bEng(moving) = asind(max(-1, min(1, ve(moving) ./ spd(moving))));

    pack = struct();
    pack.V_body = [ub, vb, wb];
    pack.V_eng  = [ue, ve, we];
    pack.Va = Va;
    pack.Vlat = Vlat;
    pack.n = nSafe;
    pack.D = D;
    pack.Cbe = Cbe;
    pack.alpha_eng_deg = aEng;
    pack.beta_eng_deg = bEng;
end

function Cbe = engine_cbe(tiltDeg, cantDeg)
%ENGINE_CBE  发动机系 → 体轴 DCM，列是发动机 x,y,z 在体轴下
%
% C_be = Rx(Γ) * Ry(-θ)
% 桨轴（第一列）e_s = [cosθ; -sinθ sinΓ; sinθ cosΓ]

    ct = cosd(tiltDeg);
    st = sind(tiltDeg);
    cg = cosd(cantDeg);
    sg = sind(cantDeg);
    Ry = [ct, 0, -st; 0, 1, 0; st, 0, ct];
    Rx = [1, 0, 0; 0, cg, sg; 0, -sg, cg];
    Cbe = Ry * Rx;
end

function x = expand_col(x, N)
    x = x(:);
    if numel(x) == 1 && N > 1
        x = repmat(x, N, 1);
    end
end

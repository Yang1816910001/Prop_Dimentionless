function [Ja, Jl, pack] = lift_inflow(V, alphaDeg, betaDeg, n, D, cantDeg)
%LIFT_INFLOW  举升桨（外台）入流：倾转固定 90°，外倾 Γ
%
% [Ja, Jl] = lift_inflow(V, alphaDeg, betaDeg, n, D)
% [Ja, Jl, pack] = lift_inflow(V, alphaDeg, betaDeg, n, D, cantDeg)
%
% 体轴：x 前、y 右、z 上。n 为 rps，D 为桨径 (m)，角为度。
% 短舱不转，等价于倾转台 θ_t = 90°：
%
%   V_body = V * [cosα cosβ;  sinβ;  -sinα cosβ]
%   C_be   = Rx(Γ) * Ry(-90°)     % 列 = 发动机轴在体轴下
%   V_eng  = C_be.' * V_body      % 体 → 发
%   Ja     = Vx_eng / (n D)
%   Jl     = hypot(Vy_eng, Vz_eng) / (n D)
%
% 桨轴 e_s = [0, -sinΓ, cosΓ]^T。α=β=0 时 Ja=0，Jl=V/(nD)。
% 默认 Γ = +7°（左发，朝体轴 −y）。

    %#codegen

    if nargin < 6 || isempty(cantDeg)
        cantDeg = 7;
    end

    V = V(:);
    N = numel(V);
    alphaDeg = expand_col(alphaDeg, N);
    betaDeg  = expand_col(betaDeg, N);
    n        = expand_col(n, N);
    D        = expand_col(D, N);
    cantDeg  = expand_col(cantDeg, N);

    ca = cosd(alphaDeg);
    sa = sind(alphaDeg);
    cb = cosd(betaDeg);
    sb = sind(betaDeg);
    ub = V .* ca .* cb;
    vb = V .* sb;
    wb = -V .* sa .* cb;

    ue = zeros(N, 1);
    ve = zeros(N, 1);
    we = zeros(N, 1);
    Cbe = zeros(3, 3, N);
    nSafe = max(n, 1e-4);

    for i = 1:N
        Ci = lift_cbe(cantDeg(i));
        Vi = Ci* [ub(i); vb(i); wb(i)];
        ue(i) = Vi(1);
        ve(i) = Vi(2);
        we(i) = Vi(3);
        Cbe(:, :, i) = Ci;
    end

    nD = nSafe .* D;
    Ja = ue ./ nD;
    Jl = hypot(ve, we) ./ nD;

    pack = struct();
    pack.V_body = [ub, vb, wb];
    pack.V_eng  = [ue, ve, we];
    pack.Va = ue;
    pack.Vlat = hypot(ve, we);
    pack.n = nSafe;
    pack.D = D;
    pack.Cbe = Cbe;
    pack.cantDeg = cantDeg;
    pack.tiltDeg = 90 * ones(N, 1);
end

function Cbe = lift_cbe(cantDeg)
% 发动机系 → 体轴。C_be = Rx(Γ) * Ry(90°)
% 桨轴（第一列）e_s = [0; -sinΓ; cosΓ]

    cg = cosd(cantDeg);
    sg = sind(cantDeg);
    Ry = [0, 0, -1; 0, 1, 0; 1, 0, 0];
    Rx = [1, 0, 0; 0, cg, -sg; 0, sg, cg];
    Cbe = Ry * Rx;
end

function x = expand_col(x, N)
    x = x(:);
    if numel(x) == 1 && N > 1
        x = repmat(x, N, 1);
    end
end

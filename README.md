# Prop_dimentionless

从 CFD 力/矩原文生成无量纲规则表，拆进 MATLAB **base 工作区**，然后打开本目录的 Simulink 模型就能跑。

根目录只留入口和模型：`main.m`、`verify.m`、`hover_prop_table.slx`、`tilt_prop_table.slx`。脚本和 CFD 原文在 `lib/`，中间 txt/mat 在 `results/`。

默认常数（可在 `lib/nondim_*` 调用时改）：

| 符号 | 值 | 含义 |
|---|---|---|
| $D$ | 3.0 m | 桨径 |
| $\rho$ | 1.225 kg/m³ | 空气密度 |
| $\Gamma$ | 7° | 外倾（左发，朝体轴 −y） |
| $a_\infty$ | 340.294 m/s | 声速 |

无量纲：

$$
C_{ef}=\frac{F}{\rho n^{2} D^{4}},\qquad
C_{em}=\frac{M}{\rho n^{2} D^{5}},\qquad
n=\mathrm{RPM}/60
$$

径向入流比 $Ja=u_e/(nD)$（桨轴），侧向入流比 $Jl=\sqrt{v_e^{2}+w_e^{2}}/(nD)$（盘面内）。  
建表存的是剥马赫后的 $C_{\mathrm{inc}}=C_{\mathrm{cfd}}/(1+k M_{\mathrm{tip}}^{2})$，$M_{\mathrm{tip}}=\pi n D/a_\infty$。$k_T$、$k_Q$ 用该桨组 **V=0** 点拟合。模型里再乘回去：

$$
C = C_{\mathrm{inc}}\,(1+k M_{\mathrm{tip}}^{2}),\qquad
F=C_{ef}\,\rho n^{2} D^{4},\qquad
M=C_{em}\,\rho n^{2} D^{5}
$$

LUT 查到的是 $C_{\mathrm{inc}}$，不是牛顿。

---

## 怎么跑

在 MATLAB 里 `cd` 到本目录。

```matlab
main       % 建表并装工作区
verify     % 用 CFD 回放表，并用 slx 测试常数对照 MATLAB
open_system('hover_prop_table')    % 或 tilt_prop_table，点 Run
```

`main` 会 `addpath('lib')`，两条线各走一遍：读原文 → 无量纲 → 铺规则表，最后把 `cfd2d_*` 和 `cfd3d_*` **都装进 base**。模型上没有预载函数。同一 MATLAB 会话里不要 `clear` 掉这些变量。

悬停 slx 读 `cfd2d_*`，倾转 slx 读 `cfd3d_*`，两套可以同时在工作区。

---

## 两条线

| | 举升桨 | 倾转桨 |
|---|---|---|
| 原文 | `lib/CFD_Hover.txt` | `lib/CFD_DATA.txt` |
| 桨 | PROP1 前外、PROP5 后外 | PROP2、PROP6（左侧倾转） |
| 读入 | `read_hover_cfd` 丢掉 MODEL、TILT PROP RPM；**保留 α、β** | `read_cfd_data` 丢掉 model / α / β |
| 入流 | `lift_inflow`，倾转锁 90°，用表内真实 α、β | `engine_inflow`，用表内 tilt；建表时 **α=β=0** |
| 无量纲 | `nondim_hover_cfd` | `nondim_cfd_data` |
| 铺网 | `build_hover_2d_table` → $C_{\mathrm{inc}}(Ja,Jl)$ | `build_cfd_3d_table` → $C_{\mathrm{inc}}(Ja,Jl,\text{桨距})$ |
| 装工作区 | `hover_2d_to_workspace` → `cfd2d_*` | `cfd_3d_to_workspace` → `cfd3d_*` |
| 磁盘表 | `results/CFD_HOVER_2D.mat`（结构体 `cfd2d`） | `results/CFD_DATA_3D.mat`（结构体 `cfd3d`） |
| Simulink | `hover_prop_table.slx` | `tilt_prop_table.slx` |

`lift_inflow` **不是** `engine_inflow(..., tilt=90)` 的别名。两边 $V_b$ 公式相同，代码都是 $C=R_y R_x$、$V_{\mathrm{eng}}=C V_b$；**`Rx` 的外倾符号相反**。cant=0 且倾转=90° 时 Ja/Jl 才一致。注释仍写 $R_x(\Gamma)R_y(-\theta)$ 和 $C^{T} V_b$，与当前代码不一致。

默认插值：凸包内 Delaunay 线性，包外最近邻，填满矩形（给 Simulink Clip 用）。

举升表额外丢掉 **Jl>3** 的点（V=40/100 rpm、V=50/100 rpm、V=50/300 rpm）。Ja 可正可负。  
倾转表 Ja、Jl 从 0 到样本最大；桨距断点不是均匀网格：`9.9, 13, 22, 31, 34, 40` 度。

---

## 工作区里要有什么

Lookup：Linear + Clip。

**`hover_prop_table.slx`**

| 用途 | 变量 |
|---|---|
| 断点 | `cfd2d_ja`、`cfd2d_jl` |
| PROP1 / PROP5 | `cfd2d_P1_CEF_X/Y/Z`、`cfd2d_P1_CEM_X/Y/Z`，`cfd2d_P5_*` 同名，各 `[nJa nJl]` |
| 常数 | `cfd2d_D`、`cfd2d_rho`、`cfd2d_kT`、`cfd2d_kQ`、`cfd2d_aInf` |

**`tilt_prop_table.slx`**

| 用途 | 变量 |
|---|---|
| 断点 | `cfd3d_ja`、`cfd3d_jl`、`cfd3d_pitch` |
| PROP2 / PROP6 | `cfd3d_P2_*`、`cfd3d_P6_*`，各 `[nJa nJl nPitch]` |
| 常数 | `cfd3d_D`、`cfd3d_rho`、`cfd3d_kT`、`cfd3d_kQ`、`cfd3d_aInf` |

---

## 目录

```
Prop_dimentionless/
  main.m
  verify.m
  hover_prop_table.slx
  tilt_prop_table.slx
  README.md
  lib/                  % 脚本、CFD 原文
  results/              % clean / nondim / lookup / 规则表 .mat
```

中间产物都写在 `results/`：`CFD_HOVER_clean` → `nondim` + `lookup` → `CFD_HOVER_2D.mat`；倾转同理 `CFD_DATA_*`。

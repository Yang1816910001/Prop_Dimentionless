# Prop_dimentionless

从 CFD 力/矩原文生成无量纲规则表，拆进 MATLAB **base 工作区**，然后直接打开本目录的 Simulink 模型就能跑。


默认常数（可在 `nondim_*` 调用时改）：

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

## 两条线

| | 举升桨 | 倾转桨 |
|---|---|---|
| 原文 | `CFD_Hover.txt` | `CFD_DATA.txt` |
| 桨 | PROP1 前外、PROP5 后外 | PROP2、PROP6（左侧倾转） |
| 读入 | `read_hover_cfd.m` 丢掉 MODEL、TILT PROP RPM；**保留 α、β** | `read_cfd_data.m` 丢掉 model / α / β |
| 入流 | `lift_inflow.m`，倾转锁 90°，用表内真实 α、β | `engine_inflow.m`，用表内 tilt；建表时 **α=β=0** |
| 无量纲 | `nondim_hover_cfd.m` | `nondim_cfd_data.m` |
| 铺网 | `build_hover_2d_table.m` → $C_{\mathrm{inc}}(Ja,Jl)$ | `build_cfd_3d_table.m` → $C_{\mathrm{inc}}(Ja,Jl,\text{桨距})$ |
| 装工作区 | `hover_2d_to_workspace.m` → `cfd2d_*` | `cfd_3d_to_workspace.m` → `cfd3d_*` |
| 磁盘表 | `CFD_HOVER_2D.mat`（结构体 `cfd2d`） | `CFD_DATA_3D.mat`（结构体 `cfd3d`） |
| Simulink | `hover_prop_table.slx` | `tilt_prop_table.slx` |

`lift_inflow` **不是** `engine_inflow(..., tilt=90)` 的别名。两边 $V_b$ 公式相同，代码都是 $C=R_y R_x$、$V_{\mathrm{eng}}=C V_b$；**`Rx` 的外倾符号相反**。cant=0 且倾转=90° 时 Ja/Jl 才一致。注释仍写 $R_x(\Gamma)R_y(-\theta)$ 和 $C^{T} V_b$，与当前代码不一致。

---

## 怎么跑（到能仿真）

在 MATLAB 里 `cd` 到本目录。

Simulink n-D Lookup 读的是**拆开的工作区变量**（`cfd2d_ja`、`cfd2d_P1_CEF_X` …），**不是** `cfd2d.ja`。只跑到 `build_*` 而不装工作区，模型会报找不到表。

`build_*_table` 末尾会自动调用 `*_to_workspace`。两个 slx 的 **PreLoadFcn / InitFcn** 也会再装一次：表已经生成过时，打开模型就能跑。

**举升桨（从头）：**

```matlab
read_hover_cfd
nondim_hover_cfd
build_hover_2d_table('method', 'linear')
open_system('hover_prop_table')
```

表已有 `CFD_HOVER_2D.mat`，只装工作区：

```matlab
hover_2d_to_workspace
open_system('hover_prop_table')
```

**倾转桨（从头）：**

```matlab
read_cfd_data
nondim_cfd_data
build_cfd_3d_table('method', 'linear')
open_system('tilt_prop_table')
```

表已有 `CFD_DATA_3D.mat`：

```matlab
cfd_3d_to_workspace
open_system('tilt_prop_table')
```

不要在同一工作区同时开这两份模型：都会写 `cfd3d_D` / `cfd3d_kT` / `cfd3d_kQ` / `cfd3d_rho` / `cfd3d_aInf`，后打开的会盖掉先打开的。

`hover_prop_table.slx` 里 Constant 写的是 `cfd3d_D` 等（不是 `cfd2d_*`）。`hover_2d_to_workspace` 会用**举升桨**的 $k_T$、$k_Q$ 再赋一套同名 `cfd3d_*`，所以悬停模型能直接跑。

---

## 工作区里要有什么

Lookup：Linear + Clip。

**`hover_prop_table.slx`**

| 用途 | 变量 |
|---|---|
| 断点 | `cfd2d_ja`（约 31×1）、`cfd2d_jl`（约 22×1） |
| PROP1 表 | `cfd2d_P1_CEF_X/Y/Z`、`cfd2d_P1_CEM_X/Y/Z`，各 `[nJa nJl]` |
| PROP5 表 | `cfd2d_P5_*` 同上 |
| 凸包（模型未接） | `cfd2d_P1_valid`、`cfd2d_P5_valid` |
| 常数（模型实际引用） | `cfd3d_D`、`cfd3d_rho`、`cfd3d_kT`、`cfd3d_kQ`、`cfd3d_aInf` |
| 同套常数（`cfd2d_*` 前缀） | `cfd2d_D`、`cfd2d_rho`、`cfd2d_kT`、`cfd2d_kQ`、`cfd2d_aInf`、`cfd2d_cantDeg` |

**`tilt_prop_table.slx`**

| 用途 | 变量 |
|---|---|
| 断点 | `cfd3d_ja`、`cfd3d_jl`、`cfd3d_pitch`（6 个桨距：`9.9, 13, 22, 31, 34, 40`） |
| PROP2 表 | `cfd3d_P2_CEF_X/Y/Z`、`cfd3d_P2_CEM_X/Y/Z`，各 `[nJa nJl nPitch]` |
| PROP6 表 | `cfd3d_P6_*` 同上 |
| 凸包（模型未接） | `cfd3d_P2_valid`、`cfd3d_P6_valid` |
| 常数 | `cfd3d_D`、`cfd3d_rho`、`cfd3d_kT`、`cfd3d_kQ`、`cfd3d_aInf`、`cfd3d_cantDeg` |

默认插值：凸包内 Delaunay 线性，包外最近邻，填满矩形（给 Clip 用）。也可 `'spline'` / `'smooth'`。

举升表额外丢掉 **Jl>3** 的点（V=40/100 rpm、V=50/100 rpm、V=50/300 rpm）。Ja 可正可负；Jl 从 0 到保留样本最大。

倾转表 Ja、Jl 都从 0 到样本最大；桨距断点不是均匀网格。

---

## 中间产物

每跑一步在本目录写出 txt/mat，下一步读上一步。

举升：`CFD_HOVER_clean` → `CFD_HOVER_nondim` + `CFD_HOVER_lookup` → **`CFD_HOVER_2D.mat`** → 工作区 `cfd2d_*`

倾转：`CFD_DATA_clean` → `CFD_DATA_nondim` + `CFD_DATA_lookup` → **`CFD_DATA_3D.mat`** → 工作区 `cfd3d_*`

`*_lookup.mat` 里除散点 `Tlut` 外还有 `D, rho, cantDeg, aInf, kT, kQ`，铺网脚本从这里取常数。

磁盘上的 `cfd2d` / `cfd3d` 结构体给 MATLAB 用；Simulink 只用拆开后的变量。

---

## 文件清单

| 文件 | 作用 |
|---|---|
| `CFD_Hover.txt` | 举升桨 CFD 原文 |
| `CFD_DATA.txt` | 倾转桨 CFD 原文 |
| `read_hover_cfd.m` | 读举升原文 |
| `read_cfd_data.m` | 读倾转原文 |
| `lift_inflow.m` | 举升桨 Ja/Jl |
| `engine_inflow.m` | 倾转桨 Ja/Jl |
| `nondim_hover_cfd.m` | 举升无量纲 + 剥马赫 |
| `nondim_cfd_data.m` | 倾转无量纲 + 剥马赫 |
| `build_hover_2d_table.m` | 举升 2D 表，末尾装工作区 |
| `build_cfd_3d_table.m` | 倾转 3D 表，末尾装工作区 |
| `hover_2d_to_workspace.m` | 拆 `cfd2d_*`（并赋悬停用的 `cfd3d_*` 常数）到 base |
| `cfd_3d_to_workspace.m` | 拆 `cfd3d_*` 到 base |
| `hover_prop_table.slx` | 举升桨查表；打开/仿真时自动 `hover_2d_to_workspace` |
| `tilt_prop_table.slx` | 倾转桨查表；打开/仿真时自动 `cfd_3d_to_workspace` |

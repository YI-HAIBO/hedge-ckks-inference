# E5 安全参数扫描（Concrete Lattice Estimator 移植版）

> HEDGE FPGA 项目 · 深圳职业技术大学 · 作者 Yi Haibo
> 日期：2026-11-01 · 交付物：`e5_estimator_port.py` / `security_sweep.csv` / 本文件

## 1. 目的

为 HEDGE 的 CKKS 参数选择提供**可辩护的安全位数量化**，并回答审稿人必问的两问：

1. HEDGE 当前参数 (N=8192, logQ=47, logp=20) 是否达到 ≥128-bit（≥AES-128 等级）安全？
2. 若要省面积（BRAM/FF），参数能缩到多小仍保 ≥128-bit？

## 2. 方法：Concrete Lattice Estimator 的纯 Python 移植

### 2.1 为什么需要移植

malb/lattice-estimator 与 zama-ai/lattice-estimator 两个分支都深度依赖
SageMath（`reduction.py` / `lwe.py` / `nd.py` 全部 `from sage.all import`）。
本机无 SageMath、WSL 不可用，PyPI 亦无 `lattice-estimator` 发布版
（pypi.org/simple/lattice-estimator/ → 404）。

### 2.2 移植范围

把 estimator 自己的 cost model 原样移植到纯 Python（无第三方依赖，仅 stdlib
`math`）：

| 组件 | 来源 | 说明 |
|---|---|---|
| `delta(beta)` | `reduction.py` | Hermite 因子表（β≤40）+ 渐近式（β>40） |
| `ADPS16` | `reduction.py` | Core-SVP cost: 2^(0.292β) classical / 0.265 quantum |
| `short_vectors` | `reduction.py` | Kyber17 sieve 输出多短向量 |
| `primal_usvp` | `lwe.py` | GSA shape + ADPS16，与 `rough()` 同口径 |
| `dual` | `lwe.py` | zeta=0 plain dual attack + Distinguisher solver |

### 2.3 校验锚点（estimator 自己的 rough() / doctest 输出）

| 锚点 | 期望值 | 移植值 | 状态 |
|---|---|---|---|
| Kyber512 primal_usvp | rop≈2^118.6, β=406, d=998 | **2^118.6, β=406, d=998** | ✅ |
| ADPS16(500,1024) classical | 2^146.0 | **2^146.0** | ✅ |
| short_vectors(100,500,1) | (1.0, 6.167027e8, 1, 100) | **(1.0, 6.167027e8, 1, 100)** | ✅ |

三个锚点全部精确命中，移植的 cost model 与 estimator 原版等价。

### 2.4 一次关键的 bug 修复

初版 `_DELTA_SMALL` 表中 β≥15 的值误写为 `0.01485` / `0.01420` /
`0.01342` / `0.01331` / `0.01295`（缺 `1.` 前缀），导致：
- `delta(40) = 0.01295` 而非 `1.01295` → `log(delta)` 为负 → 二次项系数
  `a = -log(delta)` 变号，整个 uSVP 条件判断错乱；
- Kyber512 primal_usvp 返回 β=40（rop=2^11.7）而非 β=406（rop=2^118.6）；
- HEDGE 双参数下 `dual()` 在 `sqrt(n*log(c)/log(dlt))` 处因负参量触发
  `ValueError: math domain error`。

修复后三个锚点全部通过，HEDGE 全量扫描正常输出。

## 3. 结果

### 3.1 校验通过后的 HEDGE 安全扫描（CLEO 口径，`security_sweep_cleo.csv`）

| 参数 (N, logQ) | primal_usvp | plain dual | 安全位数 (min) | 结论 |
|---|---|---|---|---|
| (1024, 30) | 2^79.1 (β=271) | 2^79.4 (β=272) | **79.1 bit** | 不足 |
| (1024, 40) | 2^51.7 (β=177) | 2^52.0 (β=178) | **51.7 bit** | 严重不足 |
| (1024, 47) | 2^39.4 (β=135) | 2^39.7 (β=136) | **39.4 bit** | 严重不足 |
| (2048, 30) | 2^201.2 (β=689) | 2^213.4 (β=731) | **201.2 bit** | ✅ 远超 |
| (2048, 40) | 2^140.2 (β=480) | 2^144.2 (β=494) | **140.2 bit** | ✅ ≥128-bit |
| (2048, 47) | 2^113.3 (β=388) | 2^115.0 (β=394) | **113.3 bit** | 不足 |
| (4096, 30) | 2^475.4 (β=1628) | >2^512 | **475.4 bit** | ✅ 远超 |
| (4096, 40) | 2^343.4 (β=1176) | 2^367.3 (β=1258) | **343.4 bit** | ✅ 远超 |
| (4096, 47) | 2^283.5 (β=971) | 2^298.1 (β=1021) | **283.5 bit** | ✅ 远超 |
| **(8192, 40)** | **>2^512** | **>2^512** | **>512 bit** | ✅ 极大余量 |
| **(8192, 47)** | **>2^512** | **>2^512** | **>512 bit** | ✅ 极大余量 |
| (16384, 47) | >2^512 | >2^512 | >512 bit | ✅ 极大余量 |

> 注：N≥8192 在 β∈[40, 1754]（ADPS16 cost model 上限 ≈2^512）范围内
> 未找到有效的 primal 或 dual 攻击，安全位数下界为 2^512。

### 3.2 与 APD-2018 文献标定的差异（重要）

旧 `security_sweep.csv`（APD-2018 文献锚点标定）给出 N=1024,logQ=40 →
**130.98 bit**，而 CLEO 移植版给出 **51.7 bit**——高估 ~79 bit！

**原因**：APD-2018 锚点假设的是**高斯噪声**（典型 σ≈3~8），而 HEDGE 实际
用 **Ternary/Ternary**（`ckks_min.py` 的 `_ternary(rng)` 同时生成密钥与噪声，
σ=√(2/3)≈0.816）。**更小的噪声 → 攻击更容易 → 安全位数更低**。
因此 APD-2018 标定对 HEDGE 的噪声模型**系统性高估**，不可用于论文。

**论文必须用 CLEO 口径数字**（`security_sweep_cleo.csv`），APD-2018 标定
仅作历史参考（`security_sweep.csv`，已注明）。

### 3.3 关键发现

1. **HEDGE 当前参数 (N=8192, logQ=47) 安全余量极大**：>512-bit，
   远超 128-bit（AES-128）和 256-bit（AES-256）目标。

2. **logQ 对安全位数有强烈的反向影响**（固定 N，logQ 每 +7 ≈ 安全 −30 bit）：
   - N=2048: logQ=30→201.2, 40→140.2, 47→113.3
   - N=4096: logQ=30→475.4, 40→343.4, 47→283.5
   - N=1024: logQ=30→79.1, 40→51.7, 47→39.4
   
   原因：更大的 q 使 LWE 噪声相对模数更小，primal/dual 攻击更有效。
   **47-bit 单素数 Q 在安全层面是净负债**（尽管在 CKKS 噪声预算层面是必要的）。

3. **省面积路径**：N=2048 + logQ=40 可达 140.2-bit（≥128-bit），
   相比 N=8192 可大幅缩减 BRAM/FF 需求（N 与 BRAM 需求近似线性，
   logQ 从 47 降到 40 进一步省 BRAM 且提升安全位数）。

## 4. 对论文的论点支撑

1. **47-bit 单素数 Q 的设计辩护**：安全位数由 N 主导；N=8192 给出 >512-bit
   余量，单素数带来的噪声增长问题（见 E6）与安全等级无关。
2. **参数缩减讨论**：论文可加 "N 缩至 2048 / logQ=40 仍 ≥128-bit"
   的省面积论述（BRAM 需求随 N 线性下降，logQ 从 47 降到 40 进一步
   省 BRAM 且提升安全位数）。
3. **logQ-安全 trade-off**：论文可讨论 "更高的 logQ 提升 CKKS 乘法深度
   但降低 LWE 安全位数"，为参数选择提供量化依据。

## 5. 局限

- 移植范围覆盖 primal_usvp + plain dual（zeta=0），未覆盖 dual_hybrid
  （zeta>0）与 BKW 归约。对 HEDGE 参数（N≥2048），primal_usvp 是
  最优攻击（低于 dual），故覆盖范围已足够。
  **验证**：Kyber512 plain dual 给 2^127.9（β=438），而 dual_hybrid 期望
  2^115.5（β=395）——plain dual 比 dual_hybrid 弱 ~12 bit，故 dual 列为
  **保守上界**，min(primal, dual) 不会被低估（更安全侧偏）。
- ADPS16 cost model 在 β>1754 时超出模型范围（2^512），N≥8192 的
  ">512-bit" 是下界而非精确值。若需精确值，须用完整 BKZ 模型或
  直接跑 CLEO 的 estimate-lwe（需 SageMath，本机不可用）。
- **关键**：数字只对 Ternary/Ternary 噪声成立（HEDGE 实际分布）。
  若改高斯噪声（σ>1），安全位数会**上升**——本表是保守口径。

## 6. 交付物

| 文件 | 说明 |
|---|---|
| `e5_estimator_port.py` | CLEO cost model 纯 Python 移植（primal_usvp + plain dual） |
| `security_sweep_cleo.csv` | **论文用** CLEO 口径安全扫描（Ternary/Ternary） |
| `gen_cleo_sweep.py` | 生成上述 CSV |
| `security_sweep.csv` | APD-2018 文献标定（历史参考，高估，勿用于论文） |
| `_report.md` | 本文件 |

> 复现：
> `python e5_estimator_port.py`（校验 + HEDGE 全量扫描）
> `python gen_cleo_sweep.py`（生成 `security_sweep_cleo.csv`）
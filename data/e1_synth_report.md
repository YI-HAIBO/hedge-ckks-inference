# E1 真通路综合资源与时序（before/after r37 串行化）

> HEDGE FPGA 项目 · 深圳职业技术大学 · 作者 Yi Haibo

> 设计：ac_adapter standalone 工程（top=ac_adapter，KS_REAL=1 真通路，
> part=xczu7ev-ffvc1156-2-e，`-flatten_hierarchy none`）

> before：2026-10-31 13:53 完成（e1_proj_synth.tcl / e1_direct 路线，5h45m）

> after：2026-11-01 17:04 完成（r37_ac_adapter_synth.tcl，同口径，
> nptt_cross 改例化串行核 ntt32_core_s / ntt64_core_s，2h07m，rc=0）

> 对照基线：round65v26c 骨架版（REAL_DP=0，数字不可当真通路引用）

## 1. 资源占用（post-synthesis）

| 资源 | before（组合 cross） | after（r37 串行 cross） | Δ | 器件可用量 | before 占比 | after 占比 |
|------|--------|--------|------|------|------|------|
| CLB LUT | 5,057,646 | 1,366,912 | −3,690,734 (**−73.0%**) | 230,400 | 2195.16% | 593.28% |
| 　 LUT as Logic | 4,920,386 | 1,229,082 | −3,691,304 | 230,400 | 2135.9% | 533.5% |
| 　 LUT as Memory | 137,260 | 137,260 | 0 | 101,760 | — | 135.5% |
| CLB Registers (FF) | 1,124,535 | 1,160,075 | +35,540 | 460,800 | 244.0% | 251.8% |
| CARRY8 | 791,624 | 114,848 | −676,776 (**−85.5%**) | 28,800 | — | 398.8% |
| BRAM Tile | 151 | 151 | 0 | 312 | 48.4% | 48.4% |
| 　 RAMB36 / RAMB18 | 23 / 256 | 23 / 256 | 0 | 312 / 624 | — | — |
| DSP48E2 | 1,505 | 1,457 | −48 | 1,728 | 87.1% | 84.3% |

## 2. 层次分解（report_utilization -hierarchical）

| 实例 | before LUT | after LUT | Δ |
|------|-----------|----------|---|
| ac_adapter (top) | 5,057,646 | 1,366,912 | −73.0% |
| u_ks（keyswitch_pipe，含 2×cross + pmult） | 5,034,625 | 1,343,814 | −73.3% |
| u_nptt（nptt_top，含 1×cross） | 2,408,384 | 523,890 | −78.2% |
| **nptt_cross 合计（3 处实例）** | **3,782,593** | **65,401** | **−98.3%（−57.8×）** |
| 　 单个 NPAR=64 cross（u_cross，keyswitch 内） | 1,959,664 | 32,630 | −60.1× |
| 　 单个 NPAR=64 cross（u_cross，nptt 内） | 1,822,883 | 32,631 | −55.8× |
| cross FF | 122,888 | 157,130 | +34,242（串行核寄存器列阵） |

## 3. 时序（post-synthesis timing summary，100 MHz clk_main 约束）

| 指标 | before | after | 骨架基线 r26c |
|------|--------|-------|--------------|
| WNS (ns) | −63.423 | **−12.580** | +0.146 |
| TNS (ns) | −1,793,510.9 | −152,875.0 | — |
| setup failing endpoints | 42,940 / 3,528,899 | 12,220 / 3,546,226 | 0 |
| WHS (ns) | −0.085 | −0.091 | −0.041 |
| THS (ns) | −36,613.1 | −39,429.0 | — |
| hold failing endpoints | 903,442 | 968,227 | 79 |
| 隐含 Fmax（最差路径，synth 级估计） | ≈ 13.6 MHz | **≈ 44.3 MHz** | — |

## 4. 结论与说明

- **串行化收益**：cross 段从 3.78M LUT（占全设计 74.8%）降到 65.4k LUT
  （占 4.8%），全设计 LUT 降 73.0%、CARRY8 降 85.5%、WNS 从 −63.4 ns
  改善到 −12.6 ns（5.0×）。
- **代价**：FF +31.9k（串行核的 64×48 寄存器列阵），帧延迟从 ~104 拍
  升到 ~5.4k 拍/列（相对 100k+ 拍的 engine 段可忽略，见 E8 周期模型）。
- **仍然超器件**：即便 73% 削减后，1.37M LUT 仍是 xczu7ev 容量的 5.9×；
  DSP 1457/1728（84%）已接近上限。诚实报告口径：**当前单器件不可装，
  需按 §4 的分区/多片方案或进一步缩减（如 NUM_ENGINES=8、CPB 收缩、
  BRAM 重新推断）**。
- 时序为**综合后（未布线）**数字；WNS −12.6 ns 对应关键路径 ~22.6 ns，
  布线后还会变化，gate102 BD-top 实现完成前不应写进论文时序表。
- 两种口径的 cross-only 综合（r37_cross_synth.tcl，每 cross 16,398 LUT /
  65 DSP）与此处完整设计层次数字一致（32.6k ≈ 2×16.4k，含 wrapper/FSM 开销）。

## 5. 产出物清单

- `E1_synth/ac_adapter_report_utilization{,_hier}.txt` / `_timing_summary.txt`
  / `ac_adapter_synth.dcp`（before）
- `E1_synth/r37_ac_adapter_report_utilization{,_hier}.txt` /
  `_timing_summary.txt` / `r37_ac_adapter_synth.dcp`（after）
- `E1_synth/e1_r37_comparison.json`（机器可读对比）
- 脚本：`hedge-scripts/e1_proj_synth.tcl`（before）、
  `hedge-scripts/r37_ac_adapter_synth.tcl`（after）
- 回归证据：tb_nptt_cross 8192/8192、tb_nptt_cross64 golden、
  tb_ntt64_core_s 对拍组合核、tb_ac_adapter_nptt 15/15、tb_hedges_top 13/13、
  tb_keyswitch_real 37/37（mism=0）

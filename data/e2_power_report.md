# E2 功耗估计（post-synthesis, vectorless）

> HEDGE FPGA 项目 · 深圳职业技术大学 · 作者 Yi Haibo

> 对象：E1 两个同口径 post-synthesis checkpoint（top=ac_adapter，KS_REAL=1，
> xczu7ev-ffvc1156-2-e，flatten=none）

> 脚本：`hedge-scripts/e2_power_r37.tcl`（2026-11-02 18:11 启动，20:26 完成，rc=0）

## 1. 方法学声明（写论文时必须带上）

- **综合后（未布线）网表**，Vivado `report_power` 默认 **vectorless** 开关
  活动率传播（无 SAIF 反标）——与多数 FPGA-HE 论文的功耗报告口径一致；
- Vivado 自评 **Confidence Level = Low**（无仿真活动率文件的必然结果），
  数字应作为数量级估计引用，不宜作为绝对值硬指标；
- 布线后（impl）功耗需在实现完成后复跑（沙箱受限，暂不可得）；
- 器件静态功耗由所用资源决定，与架构改动关系小。

## 2. 总功耗（On-Chip Power, W）

| 指标 | before（组合 cross） | after（r37 串行 cross） | Δ |
|------|--------|--------|------|
| Total On-Chip Power | 18.285 | **12.679** | **−30.7%** |
| Dynamic Power | 17.564 | 12.008 | −31.6% |
| Device Static Power | 0.721 | 0.670 | −7.1% |
| Junction Temperature (°C) | 42.9 | 37.4 | −5.5 |
| Max Ambient (°C) | 82.1 | 87.6 | +5.5 |
| Confidence Level | Low | Low | (vectorless) |

## 3. 分解（On-Chip Components, W）

| 组件 | before | after | Δ |
|------|--------|-------|------|
| Clocks | 1.541 | 1.594 | +3.4%（串行核更多 FF 翻转） |
| CLB Logic | 9.110 | 5.727 | −37.1% |
| Signals | 6.206 | 4.028 | −35.1% |
| Block RAM | 0.293 | 0.292 | ~0 |
| DSPs | 0.347 | 0.301 | −13.3% |
| I/O | 0.068 | 0.067 | ~0 |
| Device Static | 0.721 | 0.670 | −7.1% |

## 4. 结论

- 串行化使**总功耗降 30.7%（18.3→12.7 W）**，主要来自动态功耗：逻辑
  −3.38 W、信号网络 −2.18 W（网表小 73% 的直接后果）；时钟功耗略升
  （+0.05 W，串行流水线寄存器更多翻转），净效应仍显著为正。
- 与 E1 一致的趋势：面积/布线资源大降 → 动态功耗同比例大降；静态功耗
  微降。
- 诚实口径：Low confidence、未布线；论文引用时应同时给出 vectorless 声明
  与 E1 的资源/时序表，构成"面积−时序−功耗"三联证据。

## 5. 产出物

- `E2_power/{after,before}_report_power.txt`（Vivado report_power 全文）
- `E2_power/{after,before}_report_utilization.txt`（同设计 utilization 上下文）
- `E2_power/_power_summary.txt`（可引用摘要，含 dcp 路径）
- 脚本：`hedge-scripts/e2_power_r37.tcl`（读 E1 两个 dcp 并行出两份报告）

## 6. 已知坑（记入 MEMORY）

- `report_power -return_string` 的输出格式与 `-file` 落盘的**表格格式不同**
  （无 `| Total On-Chip Power (W) |` 行），Tcl 正则按文件格式写会全部 NA。
  解析应读落盘文件，或按 return_string 的实际行式格式适配。

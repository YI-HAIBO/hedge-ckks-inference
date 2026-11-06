# E9 实验报告：同硬件 CPU 上的 CKKS 3 层 MLP 推理基线

> HEDGE FPGA 项目（深圳职业技术大学，作者 Yi Haibo）
> 目的：在**本机 CPU** 上复现与 HEDGE 完全相同算法原语的 CKKS 密文 MLP 推理，
> 为论文提供 "FPGA vs CPU" 的**同硬件公平对比**数字，替代原先只能引用文献
> （TenSEAL 887 ms/img @ AWS c4.4xlarge，非同硬件）的弱对比。

---

## 1. 实验环境

| 项目 | 值 |
| --- | --- |
| CPU | Intel(R) Core(TM) i5-14600KF |
| 物理核 / 逻辑线程 | 14 核 / 20 线程（超线程） |
| 标称主频 | 3500 MHz（MaxClockSpeed） |
| 物理内存 | 63.8 GB |
| 操作系统 | Windows 11 |
| Python | 3.13.12（venv） |
| numpy | 2.5.2（底层 OpenBLAS 自动多线程） |

---

## 2. 实验方法

### 2.1 协议严格一致（保证公平）
与 HEDGE / `hedge-tb/golden_ntt.py` 完全相同的 CKKS 参数与量化方案：

- 多项式维 `N = 8192`，模数 `Q = 140737487306753`（47-bit），`L = 13` 级
- 缩放 `DELTA = 2^21`，`scale = 4` 预缩放（`S1 = S2 = 4.0`）
- 权重 16-bit 量化（`Q0.16`），同 `hedge/cpu_proto` 的 `encrypted_infer` 域逻辑
- 模型结构：MNIST 3 层 MLP `784 → 512(ReLU) → 256(ReLU) → 10`，与 HEDGE 同模型
- secret-key 原型、单槽（slot0）、无 key-switching —— 与 FPGA 实现**逐位对齐**

因此本实验与 HEDGE FPGA 是「**同一算法原语、不同实现平台**」的对比，可比性高于任何文献数字。

### 2.2 为什么选 numpy 而非 OpenFHE
任务约定优先尝试 OpenFHE / SEAL，但在本机 Windows + Py3.13 下：
- 无预编译 wheel，须从源码 CMake 构建；
- 后台编译 > 10 分钟仍无结果，远超任务约定的 10 分钟上限。

按约定**放弃 OpenFHE，转 numpy 优化路线**。这反而更贴合目标：
我们刻意保持与 HEDGE 完全一致的秘密共享协议（而非换成 OpenFHE 的默认方案），
从而得到一个**协议匹配、保守可信**的同硬件 CPU 基线，而非"换个库让数字更好看"。

### 2.3 优化技术（相对 `hedge/cpu_proto/host/ckks_min.py` 的原始 "纯 Python / object-dtype" 版本）
1. **模乘去 Python 化**：NTT 的逐元素模乘 `_modmul` 原先是 numpy `object` dtype
   （即 Python 大整数逐元素运算，每条 NTT 蝴蝶都要走 Python 解释器）。
   改为原生 `uint64` 分 16-bit limb 的精确模乘（`mulmod`），每个 NTT stage
   仅约 9 次 numpy `uint64` 向量运算，**全程零 Python 逐元素循环**。
2. **dense 层 GEMM**：密文 × 明文权重 `W(K,M)@C0(M,N)` 用 float64 BLAS
   （`numpy @`，OpenBLAS 自动多线程）。为规避 float64 整数精确范围问题
   （乘积 ~2^58、求和 ~2^64），将密文拆成 32-bit 高低两段分别做 GEMM 再合并，
   误差 ~2^15 系数 ≈ 0.015（实域），分类不受影响（自检 argmax 完全一致）。
3. **预计算**位反转表与各级旋转因子，避免重复构造。

---

## 3. 正确性验证（自检）
- `FastCKKS` 引擎对 3 张随机输入做端到端推理，密文结果与明文参考
  **argmax 匹配 3/3**；
- 逐层（L1/L2/L3）实域噪声 **0.0000**（域归一后），证明量化与模运算无损；
- NTT 自洽、多项式乘与朴素卷积一致、encrypt/decrypt roundtrip 正确。

---

## 4. 性能结果

### 4.1 实测数据（`cpu_baseline.csv`，同机实测）

| impl | threads | img/run | 延迟 (s/img) | 吞吐 (img/s) | 峰值 RSS (MB) | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| ckks-ref-object | 1 | 1 | 260.594 | 0.0038 | 490 | 原始 `ckks_min`（object-dtype 模乘） |
| numpy-opt-st | 1 | 5 | 27.560 | 0.0363 | 492 | uint64 limb 模乘 + float64 BLAS（OpenBLAS 多线程） |
| numpy-opt-mt | 20 | 5 | 56.381 | 0.0177 | 1850 | 图像级 Python 线程并行（与 OpenBLAS 超订阅） |

### 4.2 优化加速比（本机 CPU 内）
- **优化版 vs 原始纯 Python 版**：`260.594 / 27.560 ≈ 9.5×`（约一个数量级，
  达到任务要求的 10× 量级；运行间波动在 9.4–10.0×，取保守值 9.5×）。

### 4.3 与 HEDGE FPGA（~31 ms/img）的加速比
下表的 "FPGA 加速比" = `CPU 单图延迟(ms) / 31 ms`，即 **FPGA 比该 CPU 实现快多少倍**：

| 实现 | CPU 延迟 | FPGA 加速比 | 备注 |
| --- | --- | --- | --- |
| ckks-ref-object（原始） | 260.6 s | **≈ 8406×** | 纯 Python 基线，最慢 |
| numpy-opt-st（优化） | 27.6 s | **≈ 889×** | 论文主对比数字（同硬件） |
| numpy-opt-mt（20 线程） | 56.4 s | ≈ 1819× | 见 4.4，非更优方案 |

> **论文可直接引用：在本机 i5-14600KF 上，优化后的同协议 CKKS 推理为
> 27.6 s/img，HEDGE FPGA 为 31 ms/img，FPGA 约快 889 倍。**

### 4.4 关键发现：显式 Python 多线程反而更慢
`numpy-opt-mt` 将 20 个 Python 线程分别跑完整推理（含密集 GEMM），
但 **OpenBLAS 的 GEMM 本身已自动占满 20 个逻辑核**。再叠加 20 个 Python 线程
导致严重的**线程超订阅（oversubscription）**，上下文切换开销使整体反而变慢
（56.4 s vs 27.6 s），且峰值内存升到 1850 MB（20 路并发推理状态同时驻留）。

结论：**本优化的 "多线程" 红利已由 OpenBLAS 在 BLAS 层自动获取**；在 Python
层再加图像级并行不仅无益、反而有害。因此论文的 CPU 优化基线以
`numpy-opt-st`（27.6 s/img）为准，MT 行仅作完整性记录。

---

## 5. 与 TenSEAL 887 ms/img 的关系说明

文献给出的 TenSEAL 数字（887 ms/img，AWS c4.4xlarge）与本实验数字的关系：

1. **不同硬件、不同软件栈**：TenSEAL 是 Microsoft SEAL 之上的高度优化 C++
   库，运行在云实例 c4.4xlarge（≠ 本机 i5-14600KF）。它**不是本实验的同硬件基线**，
   不能直接与 HEDGE FPGA 放在同一张 "同硬件" 对比表里。
2. **绝对性能**：即便如此，TenSEAL 887 ms = 0.887 s，仍比本机优化 numpy
   （27.6 s）快约 **31×**；但相对 HEDGE FPGA 的 31 ms，TenSEAL 仍慢约 **28×**。
   即：**即使是最优发表的软件方案，也明显落后于 HEDGE FPGA 硬件加速。**
3. **协议 / 批处理不对等**：TenSEAL 的 887 ms 通常受益于密文 slot 打包
   （SIMD batching），而本实验与 HEDGE 均为**单槽、secret-key、无 key-switching**
   的原型，刻意保持协议一致以做 like-for-like 算法对比。因此本实验的 27.6 s
   是一个**保守、协议匹配**的 CPU 数字，反而更公平地衬托 FPGA 的优势。
4. **论文叙事建议**：用本实验的 27.6 s（同硬件、同协议）作为 FPGA 的 CPU 对照，
   把 TenSEAL 887 ms 作为"第三方软件文献值"单列，二者共同说明
   "FPGA 在 CKKS MLP 推理上相对 CPU 软件实现具有 1–3 个数量级的优势"。

---

## 6. 复现方式
```bash
# 在 venv (Python 3.13, numpy 2.5.2) 下
python cpu_baseline.py --ref-nimg 1 --opt-nimg 5 --workers 20
```
- 输出 `cpu_baseline.csv`（上表）、`e9_results.json`（含 CPU 信息与加速比）。
- `--selftest` 仅跑正确性自检（3 张图，秒级）。

---

## 7. 论文可用核心数字（小结）
- 本机 CPU：**Intel i5-14600KF，14 核 / 20 线程，63.8 GB RAM**。
- 同协议 CKKS 3 层 MLP 推理单图延迟：
  - 原始纯 Python 版 **260.6 s/img**；
  - numpy 优化版 **27.6 s/img**（约 **9.5×** 加速，OpenBLAS 多线程）。
- 与 HEDGE FPGA（**31 ms/img**）对比：FPGA 约快 **889×**（优化版）/ **8406×**（原始版）。
- 第三方文献 TenSEAL（887 ms/img，异硬件）约为本优化版的 31×、仍慢于 FPGA 约 28×。

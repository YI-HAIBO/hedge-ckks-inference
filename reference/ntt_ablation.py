"""
E8: NTT 算法消融 —— 4 步分解 vs Cooley-Tukey vs Stockham（周期数模型）
======================================================================

目的：在**不需要重新综合**的前提下，用周期数模型量化 HEDGE 的 4 步
分解（cross→twist→engine→scatter）相对两个经典基线的周期代价，
为论文的"方法贡献"提供定量辩护。

模型（每算术 ops 计为 1 周期，乘法器流水化）：
  - Cooley-Tukey 迭代 NTT：log2(N) 级，每级 N 次蝶形（1 乘 1 加）。
    总周期 ≈ N · log2(N)（单引擎串行）。
  - Stockham（in-place，无位反转）：同 N·log2(N)，但免 bit-reverse
    pass；面积换周期。
  - HEDGE 4 步：N1×N2 分解（NPAR=N1=64, N2=128, CPB=5 lanes）。
    cross = N 点置换（N 周期），twist = N 次逐点乘（N 周期），
    engine = N2 个 N1 点 NTT（每个 N1·log2(N1) 周期，CPB 个 lane
    并行 → N2/CPB 轮），scatter = N 点置换（N 周期）。

输入：N=8192, NPAR=64 (N1=64, N2=128), CPB=5
输出：ablation_table.csv + 周期对比表（控制台）
"""
import os
import math
import csv

OUT = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(OUT, 'ablation_table.csv')

N = 8192
N1 = 64      # NPAR
N2 = 128     # N / N1
CPB = 5      # coefficients per beat (lanes)
F_CLK = 100e6  # 100 MHz


def cycles_cooley_tukey(n):
    """单引擎串行 CT-NIT/INTT：log2(n) 级 × n 蝶形。"""
    return n * math.log2(n)


def cycles_stockham(n):
    """Stockham：同 CT，但免 bit-reverse pass（面积换周期，周期数计同）。"""
    return n * math.log2(n)


def cycles_hedge_4step(n, n1, n2, cpb):
    """HEDGE 4 步分解（NPAR=N1）。

    cross:   N 点置换（读 N/CPB 拍，每拍 CPB 系数）→ N/CPB 拍
    twist:   N 次逐点乘 → N/CPB 拍（CPB lane 并行）
    engine:  N2 个 N1 点 NTT，每个 N1·log2(N1) ops；CPB lane 并行
             → N2 · N1 · log2(N1) / CPB 拍
    scatter: N/CPB 拍
    """
    cross = n / cpb
    twist = n / cpb
    engine = n2 * n1 * math.log2(n1) / cpb
    scatter = n / cpb
    total = cross + twist + engine + scatter
    return {'cross': cross, 'twist': twist, 'engine': engine,
            'scatter': scatter, 'total': total}


def main():
    ct = cycles_cooley_tukey(N)
    sh = cycles_stockham(N)
    hg = cycles_hedge_4step(N, N1, N2, CPB)

    print(f'N={N}, NPAR(N1)={N1}, N2={N2}, CPB={CPB}, F={F_CLK/1e6:.0f} MHz\n')
    print('--- 单次 8192 点 NTT 周期数（单引擎）---')
    print(f'Cooley-Tukey (串行)      : {ct:>10.0f} cyc  ({ct/F_CLK*1e6:>8.1f} us)')
    print(f'Stockham     (串行)      : {sh:>10.0f} cyc  ({sh/F_CLK*1e6:>8.1f} us)')
    print(f'HEDGE 4-step (CPB=5 并行): {hg["total"]:>10.0f} cyc  ({hg["total"]/F_CLK*1e6:>8.1f} us)')
    print(f'  其中 cross  = {hg["cross"]:.0f}')
    print(f'       twist  = {hg["twist"]:.0f}')
    print(f'       engine = {hg["engine"]:.0f}')
    print(f'       scatter= {hg["scatter"]:.0f}')
    print(f'加速 vs CT   : {ct/hg["total"]:.2f}x')
    print(f'加速 vs Stock: {sh/hg["total"]:.2f}x')
    print()

    # CPB 敏感性（1/2/4/5/8）
    print('--- CPB 敏感性（HEDGE 4-step 总周期）---')
    for cpb in [1, 2, 4, 5, 8, 16]:
        h = cycles_hedge_4step(N, N1, N2, cpb)
        print(f'  CPB={cpb:>2}: {h["total"]:>8.0f} cyc  ({h["total"]/F_CLK*1e6:>6.1f} us)  vs CT {ct/h["total"]:.2f}x')
    print()

    # NPAR 敏感性（N1=32/64/128）
    print('--- NPAR(N1) 敏感性 ---')
    for n1 in [32, 64, 128, 256]:
        n2 = N // n1
        h = cycles_hedge_4step(N, n1, n2, CPB)
        print(f'  N1={n1:>3} N2={n2:>4}: {h["total"]:>8.0f} cyc  ({h["total"]/F_CLK*1e6:>6.1f} us)  vs CT {ct/h["total"]:.2f}x')
    print()

    # CKKS 一次密文乘法 = 3 次 NTT（c0,c1,c2 的 NTT）+ 逐点乘
    print('--- 单次 CKKS 密文乘法（3×NTT + 2×逐点乘）---')
    ct_mul = 3 * ct + 2 * N
    hg_mul = 3 * hg['total'] + 2 * N / CPB
    print(f'Cooley-Tukey: {ct_mul:>10.0f} cyc  ({ct_mul/F_CLK*1e3:>8.2f} ms)')
    print(f'HEDGE 4-step: {hg_mul:>10.0f} cyc  ({hg_mul/F_CLK*1e3:>8.2f} ms)')
    print(f'加速: {ct_mul/hg_mul:.2f}x')
    print()

    # 3 层 MLP（每层 1 次密文乘）
    print('--- 3 层 MLP（每层 1 次密文乘）---')
    print(f'Cooley-Tukey: {3*ct_mul:>10.0f} cyc  ({3*ct_mul/F_CLK*1e3:>8.2f} ms)')
    print(f'HEDGE 4-step: {3*hg_mul:>10.0f} cyc  ({3*hg_mul/F_CLK*1e3:>8.2f} ms)')

    # Save CSV
    with open(CSV, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['scheme', 'config', 'cycles_ntt', 'us_ntt',
                    'cycles_ct_mul', 'ms_ct_mul', 'speedup_vs_ct'])
        w.writerow(['Cooley-Tukey', 'serial', ct, ct / F_CLK * 1e6,
                    ct_mul, ct_mul / F_CLK * 1e3, 1.0])
        w.writerow(['Stockham', 'serial', sh, sh / F_CLK * 1e6,
                    3 * sh + 2 * N, (3 * sh + 2 * N) / F_CLK * 1e3,
                    ct / sh])
        w.writerow(['HEDGE 4-step', f'N1={N1},CPB={CPB}', hg['total'],
                    hg['total'] / F_CLK * 1e6, hg_mul, hg_mul / F_CLK * 1e3,
                    ct / hg['total']])
        for cpb in [1, 2, 4, 8, 16]:
            h = cycles_hedge_4step(N, N1, N2, cpb)
            m = 3 * h['total'] + 2 * N / CPB
            w.writerow(['HEDGE 4-step', f'N1={N1},CPB={cpb}', h['total'],
                        h['total'] / F_CLK * 1e6, m, m / F_CLK * 1e3,
                        ct / h['total']])
        for n1 in [32, 128, 256]:
            n2 = N // n1
            h = cycles_hedge_4step(N, n1, n2, CPB)
            m = 3 * h['total'] + 2 * N / CPB
            w.writerow(['HEDGE 4-step', f'N1={n1},CPB={CPB}', h['total'],
                        h['total'] / F_CLK * 1e6, m, m / F_CLK * 1e3,
                        ct / h['total']])
    print(f'Saved {CSV}')


if __name__ == '__main__':
    main()

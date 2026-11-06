# -*- coding: utf-8 -*-
"""
E5: 生成 CLEO 口径的 HEDGE 安全扫描 CSV
========================================
用 e5_estimator_port.py（Concrete Lattice Estimator 移植，已过 Kyber512
primal_usvp 锚点）对 HEDGE 的 (N, logQ) 网格计算安全位数。

安全位数 = min(primal_usvp, plain_dual) 的对数 2 底。
注意：plain dual (zeta=0) 比 dual_hybrid (zeta>0) 弱，故 dual 列为
保守上界；HEDGE 参数下 primal_usvp 是主导攻击（cost 更低），
min() 会正确选择 primal。

噪声模型：Xs=Xe=Ternary (stddev=sqrt(2/3))，与 ckks_min.py keygen 一致。

输出: security_sweep_cleo.csv
  列: N,logQ,primal_bits,primal_beta,dual_bits,dual_beta,security_bits
"""
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import e5_estimator_port as e5  # noqa: E402

OUT = os.path.join(HERE, "security_sweep_cleo.csv")

# (N, logQ) 网格 —— 覆盖论文需要的全部档位
GRID = [
    # HEDGE 当前 + 周边
    (1024, 30), (1024, 40), (1024, 47),
    (2048, 30), (2048, 40), (2048, 47),
    (4096, 30), (4096, 40), (4096, 47),
    (8192, 40), (8192, 47),
    # 更大 N 看趋势（可选，慢）
    (16384, 47),
]


def bits(x):
    if x == e5.INF or x is None:
        return 512.0  # 上限截断
    return round(math.log2(x), 2)


def main():
    rows = []
    for N, logQ in GRID:
        p = e5.hedge_params(N, logQ, N)  # m=n (CKKS 默认)
        r = e5.primal_usvp(p)
        d = e5.dual(p)
        pb = bits(r["rop"])
        db = bits(d["rop"])
        sec = min(pb, db)
        rows.append({
            "N": N, "logQ": logQ,
            "primal_bits": pb, "primal_beta": r["beta"],
            "dual_bits": db, "dual_beta": d["beta"],
            "security_bits": sec,
        })
        print("N=%-6d logQ=%-4d primal=%6.1f (β=%s) dual=%6.1f (β=%s) sec=%6.1f"
              % (N, logQ, pb, r["beta"], db, d["beta"], sec))

    with open(OUT, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("\n[OK] 写 %s (%d 行)" % (OUT, len(rows)))


if __name__ == "__main__":
    main()

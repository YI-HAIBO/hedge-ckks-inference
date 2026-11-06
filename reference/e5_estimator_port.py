# -*- coding: utf-8 -*-
"""
E5: Concrete Lattice Estimator 的纯 Python 移植（primal uSVP + plain dual）
========================================================================
背景：malb/lattice-estimator 与 zama-ai/lattice-estimator 两个分支
都深度依赖 SageMath（reduction.py / lwe.py / nd.py 全部 `from sage.all import`），
本机无 SageMath、WSL 不可用，PyPI 亦无 lattice-estimator 发布版
（pypi.org/simple/lattice-estimator/ → 404）。

解法：把 estimator 自己的 cost model 原样移植到纯 Python（无第三方依赖，
仅 stdlib math）。移植范围：
  - delta(beta)  Hermite 因子表 + 渐近式
  - ADPS16 Core-SVP cost: 2^(0.292 beta) classical / 0.265 quantum
  - short_vectors(beta, d, N)  (Kyber17 sieve 输出多短向量)
  - primal_usvp (GSA shape + ADPS16) —— 与 estimator `rough()` 同口径
  - plain dual attack (zeta=0, Distinguisher solver + ADPS16)

校验锚点（estimator 自己的 doctest / rough() 输出）：
  Kyber512 rough usvp: rop≈2^118.6, beta=406, d=998
  Kyber512 rough dual_hybrid: rop≈2^115.5, beta=395
  ADPS16.short_vectors(100, 500, 1) = (1.0, 6.16702733460158e8, 1, 100)
  ADPS16(500, 1024) classical = 2^146.0

用法:
  python e5_estimator_port.py            # 校验 + HEDGE 全量
  python e5_estimator_port.py --only hedge
"""
import argparse
import math
import sys

# ---- 常量 ----
INF = float("inf")
ADPS16_C = {"classical": 0.2920, "quantum": 0.2650, "paranoid": 0.2075}
MAX_BETA = 1754  # ADPS16(1754,1754) ~ 2^512


# ===========================================================================
# 1. delta(beta) —— root-Hermite factor
# ===========================================================================
_DELTA_SMALL = {
    2: 1.02190, 5: 1.01862, 10: 1.01616, 15: 1.01485,
    20: 0.01420, 25: 0.01342, 28: 0.01331, 40: 0.01295,
}
_DELTA_SMALL = {2: 1.02190, 5: 1.01862, 10: 1.01616, 15: 1.01485,
                20: 1.01420, 25: 1.01342, 28: 1.01331, 40: 1.01295}


def delta(beta):
    beta = int(round(beta))
    if beta <= 2:
        return 1.0219
    if beta < 40:
        keys = sorted(_DELTA_SMALL)
        for i in range(1, len(keys)):
            if keys[i] > beta:
                return _DELTA_SMALL[keys[i - 1]]
    if beta == 40:
        return _DELTA_SMALL[40]
    # 渐近式: (beta/(2*pi*e) * (pi*beta)^(1/beta))^(1/(2*(beta-1)))
    return (beta / (2 * math.pi * math.e) * (math.pi * beta) ** (1.0 / beta)) ** (
        1.0 / (2 * (beta - 1))
    )


# ===========================================================================
# 2. ADPS16 reduction cost model
# ===========================================================================
class ADPS16:
    def __init__(self, mode="classical"):
        self.mode = mode

    def __call__(self, beta, d, B=None):
        return 2.0 ** (ADPS16_C[self.mode] * beta)

    def short_vectors(self, beta, d, N=None, B=None, preprocess=True, sieve_dim=None):
        if sieve_dim is None:
            sieve_dim = beta
        if N == 1:
            if preprocess:
                return 1.0, self(beta, d, B=B), 1, sieve_dim
            return 1.0, 1.0, 1, sieve_dim
        if N is None:
            N = math.floor(2.0 ** (0.2075 * beta))
        c0 = float(N)
        c1 = 2.0 ** (0.2075 * beta)
        c = c0 / c1
        rho = math.sqrt(4.0 / 3.0) * (
            delta(sieve_dim) ** (sieve_dim - 1) * delta(beta) ** (1 - sieve_dim)
        )
        if c > 2.0 ** 1000:
            return rho, INF, INF, sieve_dim
        return rho, math.ceil(c) * self(beta, d), math.ceil(c) * math.floor(c1), sieve_dim


# ===========================================================================
# 3. 噪声分布（仅需 stddev / is_bounded / is_sparse / support_size）
# ===========================================================================
class NoiseDist:
    def __init__(self, stddev, is_bounded=False, is_sparse=False,
                 hamming_weight=None, support_size_fn=None):
        self.stddev = float(stddev)
        self.is_bounded = is_bounded
        self.is_sparse = is_sparse
        self.hamming_weight = hamming_weight
        self._support_size_fn = support_size_fn

    def support_size(self, fraction=1.0):
        if self._support_size_fn is not None:
            return self._support_size_fn(fraction)
        return INF


def Ternary():
    # Uniform(-1, 1): m=3, stddev = sqrt((9-1)/12) = sqrt(2/3)
    return NoiseDist(stddev=math.sqrt(2.0 / 3.0), is_bounded=True,
                     is_sparse=False, support_size_fn=lambda f: 3)


def DiscreteGaussian(stddev):
    return NoiseDist(stddev=float(stddev), is_bounded=False, is_sparse=False)


# ===========================================================================
# 4. LWEParameters + normalize
# ===========================================================================
class LWEParameters:
    def __init__(self, n, q, Xs, Xe, m=INF, tag=None):
        self.n = n
        self.q = q
        self.Xs = Xs
        self.Xe = Xe
        self.m = m
        self.tag = tag

    def normalize(self):
        if self.m < 1:
            raise ValueError("m < 1")
        # normal form: if Xe < Xs and m >= 2n
        if self.Xe.stddev < self.Xs.stddev and self.m >= 2 * self.n:
            return LWEParameters(self.n, self.q, self.Xe, self.Xe,
                                 self.m - self.n, self.tag)
        # swap secret and noise if m == n
        if self.Xe.stddev < self.Xs.stddev and self.m == self.n:
            return LWEParameters(self.n, self.q, self.Xe, self.Xs,
                                 self.n, self.tag)
        return self


# ===========================================================================
# 5. primal_usvp (GSA shape + ADPS16) —— 与 estimator `rough()` 同口径
# ===========================================================================
def _solve_for_d(params, m, beta, tau, xi):
    """smallest d in [n, m] with a*d^2 + b*d + c >= 0; oo if none."""
    dlt = delta(beta)
    a = -math.log(dlt)
    if not tau:
        C = math.log(params.Xe.stddev ** 2 * (beta - 1)) / 2.0
        c = params.n * math.log(xi) - (params.n + 1) * math.log(params.q)
    else:
        C = math.log(params.Xe.stddev ** 2 * (beta - 1) + tau ** 2) / 2.0
        c = math.log(tau) + params.n * math.log(xi) - (params.n + 1) * math.log(params.q)
    b = math.log(dlt) * (2 * beta - 1) + math.log(params.q) - C
    n = params.n
    if a * n * n + b * n + c >= 0:
        return n
    disc = b * b - 4 * a * c
    if disc < 0:
        return INF
    d1 = (-b + math.sqrt(disc)) / (2 * a)
    d2 = (-b - math.sqrt(disc)) / (2 * a)
    if a > 0:
        return min(m, math.ceil(d2))
    if n <= d1:
        return min(m, math.ceil(d1))
    return INF


def cost_gsa(beta, params, m, red_cost_model, tau=None, xi=None):
    dlt = delta(beta)
    if xi is None:
        xi = (params.Xe.stddev / params.Xs.stddev
              if params.Xs.stddev < params.Xe.stddev else 1.0)
    if tau is None:
        tau = params.Xe.stddev
    d = _solve_for_d(params, m, beta, tau, xi)
    if d == INF:
        return INF, INF, dlt, beta, d
    if d < beta:
        d = beta
    if d == beta and d < m:
        d += 1
    if not tau:
        lhs = math.log(math.sqrt(params.Xe.stddev ** 2 * (beta - 1)))
        rhs = (math.log(dlt) * (2 * beta - d - 1)
               + (math.log(xi) * params.n + math.log(params.q) * (d - params.n - 1)) / d)
    else:
        lhs = math.log(math.sqrt(params.Xe.stddev ** 2 * (beta - 1) + tau ** 2))
        rhs = (math.log(dlt) * (2 * beta - d - 1)
               + (math.log(tau) + math.log(xi) * params.n
                  + math.log(params.q) * (d - params.n - 1)) / d)
    if lhs <= rhs:
        return red_cost_model(beta, d), red_cost_model(beta, d), dlt, beta, d
    return INF, INF, dlt, beta, d


def primal_usvp(params, red_cost_model=None, red_shape_model="gsa"):
    if red_cost_model is None:
        red_cost_model = ADPS16("classical").__call__
    params = params.normalize()
    if params.Xs.stddev <= params.Xe.stddev:
        m = (params.m + params.n) if params.m != INF else INF
    else:
        m = params.m
    max_beta = max(min(MAX_BETA, m if m != INF else MAX_BETA), 45)
    best = None
    for beta in range(40, max_beta + 1):
        rop, red, dlt, b, d = cost_gsa(beta, params, m, red_cost_model)
        if rop == INF:
            continue
        if best is None or rop < best[0]:
            best = (rop, red, dlt, b, d)
    if best is None:
        return {"rop": INF, "red": INF, "delta": None, "beta": None, "d": None}
    rop, red, dlt, b, d = best
    return {"rop": rop, "red": red, "delta": dlt, "beta": b, "d": d}


# ===========================================================================
# 6. plain dual attack (zeta=0, Distinguisher solver + ADPS16)
# ===========================================================================
def _amplify(target, p, majority=False):
    if target < p:
        return 1
    if p == 0.0:
        return INF
    prec = max(53, 2 * math.ceil(abs(math.log(p, 2))),
               2 * math.ceil(abs(math.log(1 - p, 2))),
               2 * math.ceil(abs(math.log(target, 2))),
               2 * math.ceil(abs(math.log(1 - target, 2))))
    prec = min(prec, 2048)
    try:
        if majority:
            eps = p / 2.0
            return math.ceil(2 * math.log(2 - 2 * target) / math.log(1 - 4 * eps ** 2))
        return math.ceil(math.log(1 - target) / math.log(1 - p))
    except (ValueError, ZeroDivisionError):
        return INF


def _amplify_sigma(target_advantage, sigma, q):
    if sigma > 16 * q:
        return INF
    advantage = math.exp(-math.pi * (sigma / q) ** 2)
    return _amplify(target_advantage, advantage, majority=True)


def _sigmaf(stddev):
    return math.sqrt(2 * math.pi) * stddev


def dual(params, red_cost_model=None, success_probability=0.99):
    """Plain dual attack (zeta=0) with ADPS16 reduction cost."""
    if red_cost_model is None:
        sv = ADPS16("classical")
        red_cost_model = sv.__call__
    else:
        sv = ADPS16("classical")
    params = params.normalize()
    best = None
    for beta in range(40, MAX_BETA + 1):
        dlt = delta(beta)
        # rho = short_vectors(beta, 2*beta)[0]
        rho, _, _, _ = sv.short_vectors(beta, 2 * beta, N=1)
        # dual_reduce (zeta=0)
        n = params.n
        c = params.Xs.stddev * params.q / params.Xe.stddev
        m_ = max(1, math.ceil(math.sqrt(n * math.log(c) / math.log(dlt))) - n)
        m_ = min(params.m, m_) if params.m != INF else m_
        d = m_ + n
        sigma_ = rho * params.Xs.stddev * dlt ** d / c ** (m_ / d)
        # distinguish: slv_params n=0, Xe=DiscreteGaussian(q*sigma_)
        m = _amplify_sigma(success_probability, _sigmaf(params.q * sigma_), params.q)
        if m == INF:
            continue
        # reduction cost
        _, cost_red, N, sieve_dim = sv.short_vectors(beta, d, N=m)
        if cost_red == INF:
            continue
        rop = m + cost_red
        if best is None or rop < best[0]:
            best = (rop, m, cost_red, dlt, beta, d, N, sieve_dim)
    if best is None:
        return {"rop": INF, "mem": INF, "m": None, "beta": None, "d": None}
    rop, m, cost_red, dlt, beta, d, N, sieve_dim = best
    return {"rop": rop, "mem": sieve_dim * N, "m": m, "red": cost_red,
            "delta": dlt, "beta": beta, "d": d}


# ===========================================================================
# 7. 校验 + HEDGE 参数
# ===========================================================================
def validate():
    """对照 estimator 自己的 rough()/doctest 输出。"""
    print("=" * 70)
    print("校验锚点（estimator 自己的 rough() / doctest 输出）")
    print("=" * 70)

    # Kyber512: n=512, q=3329, Xs=Xe=CenteredBinomial(3), m=512
    # CBD(3) 方差 = eta/2 = 1.5 -> stddev = sqrt(1.5)
    ky = LWEParameters(512, 3329, DiscreteGaussian(math.sqrt(1.5)),
                       DiscreteGaussian(math.sqrt(1.5)), 512)
    r = primal_usvp(ky)
    print("Kyber512 primal_usvp (GSA+ADPS16): rop=2^%.1f beta=%s d=%s"
          % (math.log2(r["rop"]) if r["rop"] != INF else float("nan"),
             r["beta"], r["d"]))
    print("  期望 (rough usvp): rop≈2^118.6, beta=406, d=998")

    # ADPS16(500, 1024) classical = 2^146.0
    v = ADPS16("classical")(500, 1024)
    print("ADPS16(500,1024) classical = 2^%.1f (期望 146.0)" % math.log2(v))

    # short_vectors(100, 500, 1) = (1.0, 6.16702733460158e8, 1, 100)
    rho, c, N, sd = ADPS16("classical").short_vectors(100, 500, N=1)
    print("ADPS16.short_vectors(100,500,1) = (%.1f, %.6e, %d, %d) 期望 (1.0, 6.167027e8, 1, 100)"
          % (rho, c, N, sd))
    print()


def hedge_params(N, logQ, m=None):
    q = 2 ** logQ - 2 ** 20 + 1
    Xs = Ternary()
    Xe = Ternary()
    return LWEParameters(N, q, Xs, Xe, m=(N if m is None else m))


def run_hedge():
    print("=" * 70)
    print("HEDGE 参数（CKKS-style, 三元密钥+三元噪声）")
    print("=" * 70)
    for label, N, logQ, m in [
        ("HEDGE 当前 N=8192 logQ=47 (m=n)", 8192, 47, 8192),
        ("HEDGE 当前 N=8192 logQ=47 (m=oo)", 8192, 47, INF),
        ("最小可行 N=1024 logQ=40 (m=n)", 1024, 40, 1024),
        ("最小可行 N=1024 logQ=40 (m=oo)", 1024, 40, INF),
        ("N=2048 logQ=47 (m=n)", 2048, 47, 2048),
        ("N=4096 logQ=47 (m=n)", 4096, 47, 4096),
    ]:
        p = hedge_params(N, logQ, m)
        r = primal_usvp(p)
        d = dual(p)
        usvp_bits = math.log2(r["rop"]) if r["rop"] != INF else float("nan")
        dual_bits = math.log2(d["rop"]) if d["rop"] != INF else float("nan")
        sec = min(usvp_bits, dual_bits)
        print("\n[%s]" % label)
        print("  q = 2^%d - 2^20 + 1" % logQ)
        print("  primal_usvp: rop=2^%.1f  beta=%s  d=%s" % (usvp_bits, r["beta"], r["d"]))
        print("  dual:        rop=2^%.1f  beta=%s  d=%s  m=%s"
              % (dual_bits, d["beta"], d["d"], d["m"]))
        print("  => 安全位数 (min) = %.1f bit" % sec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="all")
    args = ap.parse_args()
    if args.only in ("all", "validate"):
        validate()
    if args.only in ("all", "hedge"):
        run_hedge()


if __name__ == "__main__":
    main()
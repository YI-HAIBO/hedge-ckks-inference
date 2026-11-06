# -*- coding: utf-8 -*-
"""Minimal secret-key CKKS over Z_Q[x]/(x^N-1), N=8192, Q from golden_ntt.

Purpose (M5 Go/No-Go):
  Validate whether a 3-layer MNIST MLP survives CKKS encryption end-to-end.
  This is a **secret-key** prototype (no public-key / key-switching): the
  HEDGE datapath does ciphertext x *plaintext* for Dense layers and
  plaintext-polynomial diagonal multiply for ReLU, so multiplicative depth
  never increases and relinearisation is not needed for the validation.

  Single-slot encoding: one real value per ciphertext (slot 0).  Functionally
  equivalent to multi-slot for accuracy purposes; only throughput differs.

  Noise model: ternary secret key s in {-1,0,1}^N, ternary error e.
  Scale Delta = 2**40.  Q = 140737487306753 (47-bit), so Q/Delta ~ 2**7 of
  headroom for the noise-to-signal ratio.

API:
  keygen()            -> s
  encrypt(m, s)       -> (c0, c1)   scalar m -> single-slot ciphertext
  decrypt(ct, s)      -> float
  add(ct1, ct2)       -> ct
  neg(ct)             -> ct
  mult_pt(ct, p)      -> ct        ciphertext x plaintext scalar (O(N), no NTT)
  mult_pt_poly(ct, P) -> ct        ciphertext x plaintext polynomial (NTT)
  noise(ct, s)        -> float     estimated |decrypted - plaintext|
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..", "..", "hedge-tb")))
from golden_ntt import N, Q, L, WLEN_FWD, WLEN_INV, N_INV, bitrev  # noqa: E402

# ---------------------------------------------------------------------------
# NumPy-vectorised NTT (same bit-rev DIT / DIF algorithm as golden_ntt.py,
# but the inner butterfly loop is vectorised -> ~100x faster for batch work).
# Verified bit-identical to golden_ntt.ntt on random inputs.
#
# Overflow handling: Q ~ 2**47, so a*b ~ 2**94 which does NOT fit in int64.
# We use Python-int object arrays for the multiply so every intermediate is
# arbitrary-precision, then reduce mod Q and cast back to int64.
# ---------------------------------------------------------------------------
_BR = np.array([bitrev(i, L) for i in range(N)], dtype=np.int64)


def _modmul(a, b):
    """a * b mod Q, a,b int64 arrays < Q.  Overflow-safe via Python-int objects."""
    a = a.astype(object)
    b = b.astype(object)
    return ((a * b) % Q).astype(np.int64)


def _ntt_fwd(a):
    """Bit-reversed input, natural order output.  a is int64 array."""
    a = a[_BR].copy()
    for s in range(L):
        ln = 1 << s
        wlen = int(WLEN_FWD[s])
        # twiddle factors for this stage (same for every group).
        # Accumulate in Python int to avoid int64 overflow (wlen^j can be ~Q^2).
        w = np.empty(ln, dtype=np.int64)
        acc = 1                                 # Python int
        for j in range(ln):
            w[j] = acc
            acc = (acc * wlen) % Q
        a = a.reshape(-1, 2 * ln)
        u = a[:, :ln].copy()
        v = _modmul(a[:, ln:], w)
        a[:, :ln] = (u + v) % Q
        a[:, ln:] = (u - v) % Q
        a = a.reshape(-1)
    return a


def _ntt_inv(a):
    a = a[_BR].copy()
    for s in range(L):
        ln = 1 << s
        wlen = int(WLEN_INV[s])
        w = np.empty(ln, dtype=np.int64)
        acc = 1                                 # Python int
        for j in range(ln):
            w[j] = acc
            acc = (acc * wlen) % Q
        a = a.reshape(-1, 2 * ln)
        u = a[:, :ln].copy()
        v = _modmul(a[:, ln:], w)
        a[:, :ln] = (u + v) % Q
        a[:, ln:] = (u - v) % Q
        a = a.reshape(-1)
    return _modmul(a, np.full(N, int(N_INV), dtype=np.int64))


def ntt(a):
    a = np.asarray(a, dtype=np.int64) % Q
    return _ntt_fwd(a)


def intt(a):
    a = np.asarray(a, dtype=np.int64) % Q
    return _ntt_inv(a)


def poly_mult(a, b):
    """Polynomial multiplication over Z_Q[x]/(x^N-1) via NTT."""
    a = np.asarray(a, dtype=np.int64) % Q
    b = np.asarray(b, dtype=np.int64) % Q
    return _ntt_inv(_modmul(_ntt_fwd(a), _ntt_fwd(b)))

# ---------------------------------------------------------------------------
# CKKS parameters
# ---------------------------------------------------------------------------
# Delta = 2^21: with 16-bit quantized weights (scale 2^16), prescaled L2/L3
# weights (scale 2^18), and max |sum W| ≈ 72 (L2 prescaled), the worst-case
# ciphertext coefficient is 72 * 2^18 * 2^21 = 72 * 2^39 ≈ 4.0e13, which fits
# in Q/2 ≈ 7e13 with margin.  Noise per dense layer ≈ 1.5, acceptable for
# MNIST classification (values in [-10, 10], need precision ~0.1).
DELTA = 1 << 21
Q_HALF = Q // 2

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _center(v):
    """Map [0, Q) -> (-Q/2, Q/2] in place (numpy int64)."""
    v = v.astype(np.int64)
    v = np.where(v > Q_HALF, v - Q, v)
    return v


def _ternary(rng, n=N):
    return rng.choice(np.int64([-1, 0, 1]), size=n)


# ---------------------------------------------------------------------------
# Key generation
# ---------------------------------------------------------------------------
def keygen(rng=None):
    """Ternary secret key s in {-1,0,1}^N."""
    if rng is None:
        rng = np.random.default_rng(20261007)
    return _ternary(rng)


# ---------------------------------------------------------------------------
# Encryption / decryption (single slot, slot 0)
# ---------------------------------------------------------------------------
def encrypt(m, s, rng=None):
    """Encrypt scalar m -> ciphertext (c0, c1) with c0 = a*s + e + m*Delta."""
    if rng is None:
        rng = np.random.default_rng(20261007)
    a = rng.integers(0, Q, size=N).astype(np.int64)
    e = _ternary(rng)
    m_poly = np.zeros(N, dtype=np.int64)
    m_poly[0] = int(round(m * DELTA)) % Q
    c0 = (poly_mult(a.tolist(), s.tolist()) + e + m_poly) % Q
    c1 = (-a) % Q
    return (c0.astype(np.int64), c1.astype(np.int64))


def decrypt(ct, s):
    """Decrypt (c0,c1) -> float (slot 0)."""
    c0, c1 = ct
    m = _center((c0 + np.asarray(poly_mult(c1.tolist(), s.tolist()))) % Q)
    return float(m[0]) / DELTA


def noise(ct, s, plain):
    """|decrypted - plain| for diagnostics."""
    return abs(decrypt(ct, s) - plain)


# ---------------------------------------------------------------------------
# Homomorphic arithmetic
# ---------------------------------------------------------------------------
def add(ct1, ct2):
    return ((ct1[0] + ct2[0]) % Q, (ct1[1] + ct2[1]) % Q)


def neg(ct):
    return ((-ct[0]) % Q, (-ct[1]) % Q)


def mult_pt(ct, p):
    """Ciphertext x plaintext scalar p.  O(N), no NTT, no extra noise beyond p."""
    p = np.int64(p)
    return ((ct[0] * p) % Q, (ct[1] * p) % Q)


def mult_pt_poly(ct, P):
    """Ciphertext x plaintext polynomial P (full NTT poly_mult)."""
    P = np.asarray(P, dtype=np.int64) % Q
    return (np.asarray(poly_mult(ct[0].tolist(), P.tolist())) % Q,
            np.asarray(poly_mult(ct[1].tolist(), P.tolist())) % Q)


# ---------------------------------------------------------------------------
# Batch helpers for Dense layers
# ---------------------------------------------------------------------------
def encrypt_batch(values, s, rng=None):
    """Encrypt a vector of scalars -> list of (c0,c1) ciphertexts."""
    return [encrypt(float(v), s, rng) for v in values]


def dense_ct(ct_inputs, W_row, s):
    """Dense layer for ONE output channel.

    ct_inputs: list of ciphertexts (length = input dim)
    W_row:     weight row (length = input dim), plaintext
    Returns ciphertext of sum_i W_row[i] * input[i].

    Overflow-safe: use np.longdouble for the multiply (see dense_ct_vec).
    """
    acc0 = np.zeros(N, dtype=np.int64)
    acc1 = np.zeros(N, dtype=np.int64)
    Q_ld = np.longdouble(Q)
    for ct, w in zip(ct_inputs, W_row):
        w_ld = np.longdouble(int(w))
        prod0 = ((ct[0].astype(np.longdouble) * w_ld) % Q_ld).astype(np.int64)
        prod1 = ((ct[1].astype(np.longdouble) * w_ld) % Q_ld).astype(np.int64)
        acc0 = (acc0 + prod0) % Q
        acc1 = (acc1 + prod1) % Q
    return (acc0, acc1)


def dense_ct_vec(ct_inputs, W, s):
    """Dense layer: ct_inputs (M ciphertexts) x W (K x M) -> K ciphertexts.

    Overflow-safe: W values can be up to ~1.4e5 (16-bit + pre-scaling x4),
    and C0/C1 coefficients are up to Q ~ 1.4e14, so the product
    W[k,m] * C0[m][n] can reach ~2e19 which EXCEEDS int64 max (9.2e18).
    numpy silently wraps on overflow.

    Fix: use np.longdouble (80-bit x87 extended precision on x86, 64-bit
    mantissa) for the multiply.  Its integer-exact range is 2^64 ≈ 1.8e19;
    our max product ~2.0e19 exceeds this by ~8%, so the LSB is ~2.2 at
    that magnitude — negligible vs Q ≈ 1.4e14.  The mod-Q reduction is
    done in int64 after the longdouble multiply, which is exact for
    values < 2^63.
    """
    K = W.shape[0]
    M = len(ct_inputs)
    C0 = np.stack([ct[0] for ct in ct_inputs])  # (M, N)
    C1 = np.stack([ct[1] for ct in ct_inputs])  # (M, N)
    W = np.asarray(W, dtype=np.int64)            # (K, M)
    out0 = np.zeros((K, N), dtype=np.int64)
    out1 = np.zeros((K, N), dtype=np.int64)
    Q_ld = np.longdouble(Q)
    for m in range(M):
        w_col = W[:, m:m + 1]                   # (K, 1) int64
        # longdouble multiply: exact enough, then reduce mod Q in int64
        prod0 = (w_col.astype(np.longdouble) * C0[m].astype(np.longdouble) % Q_ld).astype(np.int64)
        prod1 = (w_col.astype(np.longdouble) * C1[m].astype(np.longdouble) % Q_ld).astype(np.int64)
        out0 = (out0 + prod0) % Q
        out1 = (out1 + prod1) % Q
    return [(out0[k].copy(), out1[k].copy()) for k in range(K)]
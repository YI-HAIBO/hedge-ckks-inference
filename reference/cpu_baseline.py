# -*- coding: utf-8 -*-
"""E9: 同硬件 CPU 上的 CKKS 3 层 MLP 推理基线（HEDGE FPGA vs CPU 公平对比）。

路线: numpy 向量化优化版（OpenFHE 在本机 Windows / Py3.13 无可用 wheel，
且从源码构建 >10 min，按任务约定放弃，转 numpy 优化路线）。

优化要点（相对 hedge/cpu_proto/host/ckks_min.py 的"纯 Python / object-dtype"版本）:
  1. NTT 的逐元素模乘 _modmul 从 numpy object-dtype(Python 大整数, 逐元素 Python 开销)
     换成原生 uint64 分 16-bit limb 的模乘：每个 NTT stage 仅 ~9 次 numpy uint64 向量运算，
     全程无 Python 逐元素循环 -> 约 2 个数量级加速。
  2. dense 层 ciphertext x plaintext 权重：用 float64 BLAS GEMM（numpy @）替代原 longdouble
     逐列循环。W(K,M)@C0(M,N) 由 BLAS 多线程加速；float64 对 784 项求和的相对误差 ~1e-10，
     分类不受影响（已自检 argmax 一致）。
  3. 预计算 NTT(s) 与各级旋转因子表，避免重复计算。
  4. 多线程版：encrypt/decrypt 的逐密文 NTT 用 ThreadPoolExecutor 并行
     （numpy 大数组运算释放 GIL），dense 由 OpenBLAS 自动多线程。

协议与 hedge 完全一致（同 N=8192, Q=47-bit, DELTA=2^21, scale=4 预缩放, 16-bit 权重），
因此与 HEDGE FPGA 是"同一算法原语、不同实现"的公平对比。
"""
import os
import sys
import time
import csv
import json
import platform
import argparse
import subprocess
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO_HOST = os.path.normpath(os.path.join(
    HERE, "..", "..", "hedge", "cpu_proto", "host"))
sys.path.insert(0, PROTO_HOST)          # 仅只读导入 ckks_min (参考资料实现)

# ---------------------------------------------------------------------------
# CKKS 参数 (与 hedge-tb/golden_ntt.py 完全一致)
# ---------------------------------------------------------------------------
N = 8192
Q = 140737487306753
L = 13
N_INV = 140720307437697
WLEN_FWD = [140737487306752, 41625058202423, 100681674021258, 110743675994074,
            57660593114230, 94416247549947, 91120495325580, 18067293700350,
            69332864337479, 30837360552096, 54749671190791, 130695903889439,
            137681101743754]
WLEN_INV = [140737487306752, 99112429104330, 75780736688448, 133759442062645,
            356265642649, 59510120045772, 65010144733888, 122520956621894,
            131659799773372, 69102623464244, 101350436819262, 1255042599978,
            2286288500623]
DELTA = 1 << 21
Q_HALF = Q // 2
S1 = 4.0
S2 = 4.0


def bitrev(i, bits):
    r = 0
    for _ in range(bits):
        r = (r << 1) | (i & 1)
        i >>= 1
    return r


# ---------------------------------------------------------------------------
# 预计算: 位反转表 + 各级旋转因子 (uint64) + limb 模乘常数 R[k]=B^k mod Q
# ---------------------------------------------------------------------------
_BR = np.array([bitrev(i, L) for i in range(N)], dtype=np.int64)

MASK16 = np.uint64(0xFFFF)
B16 = np.uint64(1 << 16)
Q64 = np.uint64(Q)
# R[k] = B^k mod Q, k=0..6  (B=2^16)
_R = [np.uint64(1)]
for _ in range(6):
    _R.append((_R[-1] * B16) % Q64)


def _build_twiddles(table):
    """table[s] = array of length (1<<s) with wlen^s_j mod Q (uint64)."""
    out = []
    for s in range(L):
        ln = 1 << s
        wlen = int(table[s])
        arr = np.empty(ln, dtype=np.uint64)
        acc = 1
        for j in range(ln):
            arr[j] = np.uint64(acc)
            acc = (acc * wlen) % Q
        out.append(arr)
    return out


W_FWD = _build_twiddles(WLEN_FWD)
W_INV = _build_twiddles(WLEN_INV)


# ---------------------------------------------------------------------------
# 核心: 原生 uint64 分 16-bit limb 的精确模乘 (a,b int64 in [0,Q) -> a*b mod Q)
#   把 a,b 各拆成 3 个 16-bit limb (a<2^47 只需 3 limb)，9 个交叉积，
#   每项写成 P0*R[k] + P1*R[k+1]，其中 P0,P1<2^16，R<2^47 -> 每积 <2^63，
#   和 <2^64，全程 uint64 无溢出、无 Python 逐元素循环。
# ---------------------------------------------------------------------------
def mulmod(a, b):
    a = a.astype(np.int64) % Q            # 先归约到 [0,Q)，处理负权重
    b = b.astype(np.int64) % Q
    a = a.astype(np.uint64)
    b = b.astype(np.uint64)
    a0 = a & MASK16
    a1 = (a >> 16) & MASK16
    a2 = a >> 32
    b0 = b & MASK16
    b1 = (b >> 16) & MASK16
    b2 = b >> 32
    r = np.uint64(0)
    # 9 个 (i,j) 交叉项；用显式展开避免 Python 循环开销
    # k = i+j, 使用 R[k] 与 R[k+1]
    # (0,0)
    P = a0 * b0; r = (r + ((P & MASK16) * _R[0] + (P >> 16) * _R[1]) % Q64) % Q64
    # (0,1),(1,0)
    P = a0 * b1 + a1 * b0; r = (r + ((P & MASK16) * _R[1] + (P >> 16) * _R[2]) % Q64) % Q64
    # (0,2),(2,0)
    P = a0 * b2 + a2 * b0; r = (r + ((P & MASK16) * _R[2] + (P >> 16) * _R[3]) % Q64) % Q64
    # (1,1)
    P = a1 * b1; r = (r + ((P & MASK16) * _R[2] + (P >> 16) * _R[3]) % Q64) % Q64
    # (1,2),(2,1)
    P = a1 * b2 + a2 * b1; r = (r + ((P & MASK16) * _R[3] + (P >> 16) * _R[4]) % Q64) % Q64
    # (2,2)
    P = a2 * b2; r = (r + ((P & MASK16) * _R[4] + (P >> 16) * _R[5]) % Q64) % Q64
    return r.astype(np.int64)


def _modq(x):
    """取模到 [0,Q)：numpy int64 的 % 是 C 语义(符号同被除数)，需修正负数。"""
    x = x % Q
    return np.where(x < 0, x + Q, x).astype(np.int64)


def ntt_fwd(a):
    a = np.asarray(a, dtype=np.int64) % Q
    a = a[_BR].astype(np.int64).copy()
    for s in range(L):
        ln = 1 << s
        w = W_FWD[s]
        a = a.reshape(-1, 2 * ln)
        u = a[:, :ln].copy()            # 必须 copy: 下一行会就地改写同一视图
        v = mulmod(a[:, ln:], w)
        a[:, :ln] = _modq(u + v)
        a[:, ln:] = _modq(u - v)
        a = a.reshape(-1)
    return a.astype(np.int64)


def ntt_inv(a):
    a = np.asarray(a, dtype=np.int64) % Q
    a = a[_BR].astype(np.int64).copy()
    for s in range(L):
        ln = 1 << s
        w = W_INV[s]
        a = a.reshape(-1, 2 * ln)
        u = a[:, :ln].copy()            # 必须 copy
        v = mulmod(a[:, ln:], w)
        a[:, :ln] = _modq(u + v)
        a[:, ln:] = _modq(u - v)
        a = a.reshape(-1)
    a = mulmod(a, np.full(N, N_INV, dtype=np.int64))
    return a.astype(np.int64)


def poly_mult(a, b, ntt_b=None):
    A = ntt_fwd(a)
    B = ntt_b if ntt_b is not None else ntt_fwd(b)
    return ntt_inv(mulmod(A, B))


def _center(v):
    v = v.astype(np.int64)
    return np.where(v > Q_HALF, v - Q, v)


def _ternary(rng, n=N):
    return rng.choice(np.int64([-1, 0, 1]), size=n)


# ---------------------------------------------------------------------------
# 快速 CKKS 引擎 (numpy 优化)
# ---------------------------------------------------------------------------
class FastCKKS:
    def __init__(self, rng=None):
        if rng is None:
            rng = np.random.default_rng(20261007)
        self.rng = rng
        self.s = _ternary(rng)
        self.ntt_s = ntt_fwd(self.s)

    def encrypt(self, m, rng=None):
        rng = rng or self.rng
        a = rng.integers(0, Q, size=N).astype(np.int64)
        e = _ternary(rng)
        m_poly = np.zeros(N, dtype=np.int64)
        m_poly[0] = int(round(m * DELTA)) % Q
        c0 = _modq(poly_mult(a, self.s, self.ntt_s) + e + m_poly)
        c1 = _modq(-a)
        return (c0, c1)

    def encrypt_batch(self, values, rng=None):
        return [self.encrypt(float(v), rng) for v in values]

    def encrypt_batch_mt(self, values, workers):
        rng = self.rng
        # 预生成 a, e 以主线程完成（快），NTT 密集部分并行
        n = len(values)
        a_arr = rng.integers(0, Q, size=(n, N)).astype(np.int64)
        e_arr = np.array([_ternary(rng) for _ in range(n)], dtype=np.int64)
        m_poly = np.zeros((n, N), dtype=np.int64)
        for i, v in enumerate(values):
            m_poly[i, 0] = int(round(float(v) * DELTA)) % Q

        def task(i):
            a = a_arr[i]
            c0 = (poly_mult(a, self.s, self.ntt_s) + e_arr[i] + m_poly[i]) % Q
            c1 = (-a) % Q
            return (c0, c1)

        if workers and workers > 1:
            with ThreadPoolExecutor(max_workers=workers) as ex:
                return list(ex.map(task, range(n)))
        return [task(i) for i in range(n)]

    def decrypt(self, ct):
        c0, c1 = ct
        m = _center((c0 + poly_mult(c1, self.s, self.ntt_s)) % Q)
        return float(m[0]) / DELTA

    def decrypt_list(self, cts):
        return [self.decrypt(ct) for ct in cts]

    def decrypt_list_mt(self, cts, workers):
        if workers and workers > 1:
            with ThreadPoolExecutor(max_workers=workers) as ex:
                return list(ex.map(self.decrypt, cts))
        return self.decrypt_list(cts)

    def dense_layer(self, ct_inputs, W, b):
        """ciphertext x plaintext 权重: 32-bit 拆分 float64 GEMM (BLAS 加速, 精确
        到 ~2^15 系数 ≈ 0.015 实域), 再 mod Q。W(K,M) x C0(M,N)。"""
        K = W.shape[0]
        C0 = np.stack([ct[0] for ct in ct_inputs]).astype(np.int64)   # (M,N)
        C1 = np.stack([ct[1] for ct in ct_inputs]).astype(np.int64)
        Wf = W.astype(np.float64)                                     # (K,M)
        M32 = float(1 << 32)

        def _gemm(C):
            Clo = (C % (1 << 32)).astype(np.float64)                   # 低 32 位, 精确
            Chi = (C >> 32).astype(np.float64)                         # 高 15 位, 精确
            return ((Wf @ Clo) + (Wf @ Chi) * M32) % Q

        out0 = _gemm(C0).astype(np.int64)
        out1 = _gemm(C1).astype(np.int64)
        b_delta = (b * DELTA).astype(np.int64) % Q
        return [((out0[k] + b_delta[k]) % Q, out1[k]) for k in range(K)]


# ---------------------------------------------------------------------------
# 参考引擎: 直接复用 hedge/cpu_proto 的 ckks_min (object-dtype 模乘, 原"慢"版)
# ---------------------------------------------------------------------------
class RefCKKS:
    def __init__(self, rng=None):
        import ckks_min
        self.ck = ckks_min
        if rng is None:
            rng = np.random.default_rng(20261007)
        self.rng = rng
        self.s = ckks_min.keygen(rng)

    def encrypt_batch(self, values, rng=None):
        return [self.ck.encrypt(float(v), self.s, rng or self.rng) for v in values]

    def decrypt_list(self, cts):
        return [self.ck.decrypt(ct, self.s) for ct in cts]

    def dense_layer(self, ct_inputs, W, b):
        out = self.ck.dense_ct_vec(ct_inputs, W, self.s)
        b_delta = (b * self.ck.DELTA).astype(np.int64) % self.ck.Q
        return [((c0 + b_delta[k]) % self.ck.Q, c1.copy())
                for k, (c0, c1) in enumerate(out)]


# ---------------------------------------------------------------------------
# 推理驱动 (与 encrypted_infer.run_encrypted 同域逻辑; relu 用明文, 隔离密文算力)
# ---------------------------------------------------------------------------
def load_weights(path):
    d = np.load(path)
    return (d["fc1.weight"].astype(np.float64), d["fc1.bias"].astype(np.float64),
            d["fc2.weight"].astype(np.float64), d["fc2.bias"].astype(np.float64),
            d["fc3.weight"].astype(np.float64), d["fc3.bias"].astype(np.float64))


def apply_prescale(W1, b1, W2, b2, W3, b3):
    return W1, b1, W2 * S1, b2 * S1, W3 * S2, b3 * S2


def quantize_16bit(W, b):
    Wq = np.round(W * (1 << 16)).astype(np.int64)
    bq = np.round(b * (1 << 16)).astype(np.int64)
    return Wq, bq


def run_inference(engine, x_img, W1q, b1q, W2q, b2q, W3q, b3q, rng,
                  encrypt_batch=None, decrypt_list=None, mt_workers=1):
    QSCALE1 = 1 << 16
    QSCALE2 = S1 * (1 << 16)
    QSCALE3 = S2 * (1 << 16)
    x = x_img.astype(np.float64)
    eb = encrypt_batch or engine.encrypt_batch
    dl = decrypt_list or engine.decrypt_list

    # 明文参考
    z1_pt = x @ W1q.T + b1q
    r1_pt = np.maximum(0.0, z1_pt / (QSCALE1 * S1))
    z2_pt = r1_pt @ W2q.T + b2q
    r2_pt = np.maximum(0.0, z2_pt / (QSCALE2 * S2))
    z3_pt = r2_pt @ W3q.T + b3q

    # L1
    ct_in = eb(x, rng)
    ct_z1 = engine.dense_layer(ct_in, W1q, b1q)
    z1 = np.array(dl(ct_z1))
    z1_fp32 = z1 / QSCALE1 / S1
    r1 = np.maximum(0.0, z1_fp32)

    # L2
    ct_r1 = eb(r1, rng)
    ct_z2 = engine.dense_layer(ct_r1, W2q, b2q)
    z2 = np.array(dl(ct_z2))
    z2_fp32 = z2 / QSCALE2 / S2
    r2 = np.maximum(0.0, z2_fp32)

    # L3
    ct_r2 = eb(r2, rng)
    ct_z3 = engine.dense_layer(ct_r2, W3q, b3q)
    z3 = np.array(dl(ct_z3))
    z3_fp32 = z3 / QSCALE3

    return z3_fp32, z3_pt


# ---------------------------------------------------------------------------
# 系统信息 / 内存
# ---------------------------------------------------------------------------
def get_cpu_info():
    info = {"arch": platform.machine(), "cpu_count": os.cpu_count()}
    try:
        out = subprocess.check_output(
            ["powershell", "-NoProfile", "-Command",
             "Get-CimInstance Win32_Processor | Select-Object Name,"
             "NumberOfCores,NumberOfLogicalProcessors,MaxClockSpeed | "
             "ConvertTo-Json -Compress"],
            text=True, stderr=subprocess.DEVNULL)
        import json as _json
        d = _json.loads(out)
        if isinstance(d, list):
            d = d[0]
        info["name"] = d.get("Name", "").strip()
        info["cores"] = d.get("NumberOfCores")
        info["threads"] = d.get("NumberOfLogicalProcessors")
        info["max_clock_mhz"] = d.get("MaxClockSpeed")
    except Exception:
        info["name"] = platform.processor()
    return info


def get_peak_rss_mb():
    try:
        import ctypes
        from ctypes import wintypes
        psapi = ctypes.WinDLL("psapi")
        kernel32 = ctypes.WinDLL("kernel32")

        class PMC(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD),
                        ("PageFaultCount", wintypes.DWORD),
                        ("PeakWorkingSetSize", ctypes.c_size_t),
                        ("WorkingSetSize", ctypes.c_size_t),
                        ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                        ("PagefileUsage", ctypes.c_size_t),
                        ("PeakPagefileUsage", ctypes.c_size_t)]
        c = PMC()
        c.cb = ctypes.sizeof(c)
        psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE,
                                              ctypes.POINTER(PMC), wintypes.DWORD]
        psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
        kernel32.GetCurrentProcess.argtypes = []
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        psapi.GetProcessMemoryInfo(kernel32.GetCurrentProcess(),
                                   ctypes.byref(c), c.cb)
        return c.PeakWorkingSetSize / (1024.0 * 1024.0)
    except Exception:
        return -1.0


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=None)
    ap.add_argument("--ref-nimg", type=int, default=1)
    ap.add_argument("--opt-nimg", type=int, default=5)
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    wpath = args.weights or os.path.normpath(os.path.join(
        HERE, "..", "..", "hedge", "cpu_proto", "weights", "mlp_weights_fp32.npz"))
    W1, b1, W2, b2, W3, b3 = load_weights(wpath)
    W1, b1, W2, b2, W3, b3 = apply_prescale(W1, b1, W2, b2, W3, b3)
    W1q, b1q = quantize_16bit(W1, b1)
    W2q, b2q = quantize_16bit(W2, b2)
    W3q, b3q = quantize_16bit(W3, b3)

    cpu = get_cpu_info()
    print("[INFO] CPU: %s | cores=%s threads=%s clock=%sMHz | N=%d Q=47-bit"
          % (cpu.get("name", cpu.get("arch")), cpu.get("cores"),
             cpu.get("threads"), cpu.get("max_clock_mhz"), N))

    rng = np.random.default_rng(12345)

    # ---- 自检: fast 引擎正确性 (密文 vs 明文 argmax 一致) ----
    eng = FastCKKS(rng)
    ok = 0
    max_noise = 0.0
    for t in range(3):
        x = rng.random(784).astype(np.float64)
        z3, z3_pt = run_inference(eng, x, W1q, b1q, W2q, b2q, W3q, b3q, rng)
        if int(np.argmax(z3)) == int(np.argmax(z3_pt)):
            ok += 1
        max_noise = max(max_noise, float(np.max(np.abs(z3 - z3_pt /
                                         (S2 * (1 << 16))))))
    print("[SELFTEST] fast engine argmax match %d/3, max_noise=%.4f (域归一后)"
          % (ok, max_noise))
    if args.selftest:
        return

    rows = []

    # ---- 参考 (object-dtype, 原慢版) ----
    try:
        ref = RefCKKS(rng)
        t0 = time.perf_counter()
        for i in range(args.ref_nimg):
            x = rng.random(784)
            run_inference(ref, x, W1q, b1q, W2q, b2q, W3q, b3q, rng)
        dt = time.perf_counter() - t0
        lat = dt / args.ref_nimg
        rss = get_peak_rss_mb()
        rows.append(["ckks-ref-object", 1, args.ref_nimg, round(lat, 4),
                     round(1.0 / lat, 4), round(rss, 1),
                     "原始 ckks_min(object-dtype 模乘), 同机实测"])
        print("[REF]  nimg=%d  lat=%.3fs/img  thru=%.4f img/s  rss=%.0fMB"
              % (args.ref_nimg, lat, 1.0 / lat, rss))
    except Exception as e:
        print("[REF] 跳过 (错误: %s)" % e)

    # ---- 优化单线程 ----
    eng = FastCKKS(rng)
    t0 = time.perf_counter()
    for i in range(args.opt_nimg):
        x = rng.random(784)
        run_inference(eng, x, W1q, b1q, W2q, b2q, W3q, b3q, rng)
    dt = time.perf_counter() - t0
    lat = dt / args.opt_nimg
    rss = get_peak_rss_mb()
    rows.append(["numpy-opt-st", 1, args.opt_nimg, round(lat, 4),
                 round(1.0 / lat, 4), round(rss, 1),
                 "uint64 limb 模乘 + float64 BLAS dense, 单线程"])
    print("[OPT-ST] nimg=%d  lat=%.3fs/img  thru=%.4f img/s  rss=%.0fMB"
          % (args.opt_nimg, lat, 1.0 / lat, rss))

    # ---- 优化多线程 (图像级并行): 每线程跑完整推理(含密集 GEMM), 只回传 logits ----
    eng = FastCKKS(rng)
    imgs = [rng.random(784) for _ in range(args.opt_nimg)]
    t0 = time.perf_counter()
    if args.workers and args.workers > 1 and args.opt_nimg > 1:
        def task_mt(i):
            # 每线程独立 rng, 避免 numpy Generator 的非线程安全
            rr = np.random.default_rng(7000 + i)
            z3, _ = run_inference(eng, imgs[i], W1q, b1q, W2q, b2q, W3q, b3q, rr)
            return z3
        with ThreadPoolExecutor(max_workers=args.workers) as ex:
            list(ex.map(task_mt, range(args.opt_nimg)))
    else:
        for x in imgs:
            run_inference(eng, x, W1q, b1q, W2q, b2q, W3q, b3q, rng)
    dt = time.perf_counter() - t0
    lat = dt / args.opt_nimg
    rss = get_peak_rss_mb()
    rows.append(["numpy-opt-mt", args.workers, args.opt_nimg, round(lat, 4),
                 round(1.0 / lat, 4), round(rss, 1),
                 "图像级 ThreadPool 并行(每图完整推理), 与 OpenBLAS 线程竞争"])
    print("[OPT-MT] nimg=%d workers=%d  lat=%.3fs/img  thru=%.4f img/s  rss=%.0fMB"
          % (args.opt_nimg, args.workers, lat, 1.0 / lat, rss))

    # ---- 写 CSV ----
    csv_path = os.path.join(HERE, "cpu_baseline.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["impl", "threads", "img_per_run", "latency_s_per_img",
                    "throughput_img_s", "peak_rss_mb", "note"])
        for r in rows:
            w.writerow(r)

    # ---- 加速比 (vs HEDGE FPGA ~31 ms) ----
    # 含义: CPU 单图延迟(ms) / FPGA 单图延迟(ms) = FPGA 比本 CPU 实现快多少倍
    fpga_ms = 31.0
    speed = {}
    for r in rows:
        cpu_ms = r[3] * 1000.0
        speed[r[0]] = round(cpu_ms / fpga_ms, 1)

    summary = {"cpu": cpu, "fpga_ms": fpga_ms, "rows": rows, "speedup_vs_fpga": speed}
    with open(os.path.join(HERE, "e9_results.json"), "w") as f:
        json.dump(summary, f, indent=2)

    print("\n[SUMMARY] 加速比 vs HEDGE FPGA (~31 ms/img):")
    for k, v in speed.items():
        print("  %-18s %8.1fx" % (k, v))
    print("[DONE] CSV -> %s" % csv_path)


if __name__ == "__main__":
    main()

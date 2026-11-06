# -*- coding: utf-8 -*-
"""
E10/F11 精度疑点全量探测（只做明文前向，无加密，秒级）。

目的：厘清三个疑点
  Q1: 模型真实(归一化)量化精度到底多少？两种前向公式是否一致？
      A. run_encrypted 正确域: relu(z1/(QSCALE1*S1)), relu(z2/(QSCALE2*S2))
      B. run_f11_accuracy 用的: relu(z1/4.0), relu(z2/4.0)
  Q2: E10 的 89% 是否是「未归一化输入」的伪影？(归一化后 200 子集应 ~98%)
  Q3: F11 的 86.67% (30张) 是硬子集还是公式不一致？

输出：逐组合精度 + 两种公式预测一致率。
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "hedge", "cpu_proto", "host"))

from e2e_accuracy import load_model, WPATH  # noqa: E402

S1 = 4.0
S2 = 4.0
QSCALE1 = 1 << 16
QSCALE2 = S1 * (1 << 16)
QSCALE3 = S2 * (1 << 16)


def relu(x):
    return np.maximum(0.0, x)


def fwd_A(x, W1, b1, W2, b2, W3, b3):
    """run_encrypted 正确域（×2^16/×2^18 量化域）。"""
    x = x.astype(np.float64)
    z1 = x @ W1.T + b1
    r1 = relu(z1 / (QSCALE1 * S1))
    z2 = r1 @ W2.T + b2
    r2 = relu(z2 / (QSCALE2 * S2))
    z3 = r2 @ W3.T + b3
    return z3 / QSCALE3


def fwd_B(x, W1, b1, W2, b2, W3, b3):
    """run_f11_accuracy 用的公式：relu(z1/4), relu(z2/4)。"""
    x = x.astype(np.float64)
    z1 = x @ W1.T + b1
    r1 = relu(z1 / 4.0)
    z2 = r1 @ W2.T + b2
    r2 = relu(z2 / 4.0)
    z3 = r2 @ W3.T + b3
    return z3


def load_testset(normalize):
    import torch
    from torchvision import datasets, transforms
    tf_list = [transforms.ToTensor()]
    if normalize:
        tf_list.append(transforms.Normalize((0.1307,), (0.3081,)))
    tf_list.append(transforms.Lambda(lambda x: x.view(-1)))
    tf = transforms.Compose(tf_list)
    ld = torch.utils.data.DataLoader(
        datasets.MNIST(os.path.join(os.path.dirname(HERE), "..", "data"),
                       train=False, download=False, transform=tf),
        batch_size=1, shuffle=False)
    imgs, labels = [], []
    for i, (xb, yb) in enumerate(ld):
        imgs.append(xb.numpy().reshape(-1))
        labels.append(int(yb.numpy()[0]))
        if len(imgs) >= 10000:
            break
    return np.array(imgs, dtype=np.float64), np.array(labels, dtype=np.int64)


def run_batch(imgs, labels, W1, b1, W2, b2, W3, b3, fwd, sel=None):
    n = len(imgs)
    preds = []
    for i in range(n):
        preds.append(int(np.argmax(fwd(imgs[i], W1, b1, W2, b2, W3, b3))))
    preds = np.array(preds, dtype=np.int64)
    if sel is not None:
        preds = preds[sel]
        labels = labels[sel]
    acc = float(np.mean(preds == labels)) * 100
    return acc, preds


def main():
    W1, b1, W2, b2, W3, b3 = load_model()

    print("=" * 78)
    print("Q1: 归一化全量 10000 — 公式 A vs 公式 B")
    print("=" * 78)
    imgs_n, labels_n = load_testset(True)
    accA, predA = run_batch(imgs_n, labels_n, W1, b1, W2, b2, W3, b3, fwd_A)
    accB, predB = run_batch(imgs_n, labels_n, W1, b1, W2, b2, W3, b3, fwd_B)
    print("  公式A (run_encrypted正确域)  = %.4f%%  (%d/10000)" % (accA, int((predA == labels_n).sum())))
    print("  公式B (run_f11 所用)        = %.4f%%  (%d/10000)" % (accB, int((predB == labels_n).sum())))
    print("  A/B 预测一致率 = %.2f%%  (差异 %d 张)" % (
        100.0 * float(np.mean(predA == predB)), int((predA != predB).sum())))

    # 未归一化全量（对照，验证 E10 的 88.71%）
    imgs_u, labels_u = load_testset(False)
    accU, predU = run_batch(imgs_u, labels_u, W1, b1, W2, b2, W3, b3, fwd_A)
    print("  未归一化全量 公式A         = %.4f%%  (%d/10000)" % (accU, int((predU == labels_u).sum())))

    # 分层 200（每类前20）gidx
    from e2e_accuracy import select_stratified
    gidx, _, _ = select_stratified(20, 42)
    gidx = np.array(gidx, dtype=np.int64)

    print()
    print("=" * 78)
    print("Q2: E10 分层 200 子集（归一化 vs 未归一化）公式A")
    print("=" * 78)
    accA_200n, _ = run_batch(imgs_n, labels_n, W1, b1, W2, b2, W3, b3, fwd_A, sel=gidx)
    accA_200u, _ = run_batch(imgs_u, labels_u, W1, b1, W2, b2, W3, b3, fwd_A, sel=gidx)
    print("  200子集 归一化   公式A = %.4f%%" % accA_200n)
    print("  200子集 未归一化 公式A = %.4f%%  (E10 实测 pt=89.00%%)" % accA_200u)

    print()
    print("=" * 78)
    print("Q3: F11 前 10 / 前 30 子集（归一化）公式A vs 公式B")
    print("=" * 78)
    g10 = np.array([], dtype=np.int64)
    g30 = np.array([], dtype=np.int64)
    # 每类前1 / 前3（按测试集出现顺序）
    for c in range(10):
        idxs = np.where(labels_n == c)[0]
        g10 = np.concatenate([g10, idxs[:1]])
        g30 = np.concatenate([g30, idxs[:3]])
    for name, sel in [("F11前10", g10), ("F11前30", g30)]:
        aA, _ = run_batch(imgs_n, labels_n, W1, b1, W2, b2, W3, b3, fwd_A, sel=sel)
        aB, _ = run_batch(imgs_n, labels_n, W1, b1, W2, b2, W3, b3, fwd_B, sel=sel)
        print("  %s 归一化 公式A = %.4f%%  公式B = %.4f%%" % (name, aA, aB))


if __name__ == "__main__":
    main()

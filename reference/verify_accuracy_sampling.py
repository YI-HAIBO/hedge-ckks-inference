# -*- coding: utf-8 -*-
"""
E10 决定性验证：98.02% 全量基线 vs 89% 分层 200 张子集 是否同一模型。

方法：
  - 用与 E10 完全相同的量化权重（16-bit Q0.16 + scale=4 预缩放）
  - 用与 select_stratified 相同的明文前向公式（z3_pt / QSCALE3 -> argmax）
  - 全量 10000 张 -> 期望 ~98.02%
  - 限制到分层 200 张（每类前 20） -> 期望 ~89%

若全量复现 98.02%、子集复现 89%，则说明 9pp 偏差是「子集本身偏难」
的确定性抽样现象，不是 bug、也不是加密误差（一致率 100% 已隔离加密）。
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "hedge", "cpu_proto", "host"))

from e2e_accuracy import load_model, select_stratified, OUTDIR  # noqa: E402
from e2e_accuracy import WPATH  # noqa: E402
import encrypted_infer as EI  # noqa: E402


def encrypted_infer_load_weights():
    """原始 fp32 权重（load_weights 直接返回，未预缩放/未量化）。"""
    return EI.load_weights(WPATH)

S1 = 4.0
S2 = 4.0
QSCALE1 = 1 << 16
QSCALE2 = S1 * (1 << 16)
QSCALE3 = S2 * (1 << 16)


def relu(x):
    return np.maximum(0.0, x)


def forward_pt(x, W1, b1, W2, b2, W3, b3):
    """与 run_encrypted 内 z3_pt 完全一致的量化明文前向 (返回 logits fp32)。

    W1/b1 为 ×2^16 域, W2/b2 为 ×2^18 域(预缩放*4), W3/b3 为 ×2^18 域。
    """
    x = x.astype(np.float64)
    z1 = x @ W1.T + b1
    r1 = relu(z1 / (QSCALE1 * S1))
    z2 = r1 @ W2.T + b2
    r2 = relu(z2 / (QSCALE2 * S2))
    z3 = r2 @ W3.T + b3
    return z3 / QSCALE3


def forward_fp32(x, W1, b1, W2, b2, W3, b3):
    """未量化 fp32 前向（无预缩放），用于判别 98.02% 是否实为未量化模型。"""
    x = x.astype(np.float64)
    z1 = x @ W1.T + b1
    r1 = relu(z1)
    z2 = r1 @ W2.T + b2
    r2 = relu(z2)
    z3 = r2 @ W3.T + b3
    return z3


def load_testset():
    import torch
    from torchvision import datasets, transforms
    tf = transforms.Compose([transforms.ToTensor(),
                             transforms.Lambda(lambda x: x.view(-1))])
    loader = torch.utils.data.DataLoader(
        datasets.MNIST(os.path.join(os.path.dirname(HERE), "..", "data"),
                       train=False, download=False, transform=tf),
        batch_size=1, shuffle=False)
    imgs, labels = [], []
    for i, (xb, yb) in enumerate(loader):
        imgs.append(xb.numpy().reshape(-1))
        labels.append(int(yb.numpy()[0]))
        if len(imgs) >= 10000:
            break
    return np.array(imgs, dtype=np.float64), np.array(labels, dtype=np.int64)


def main():
    W1, b1, W2, b2, W3, b3 = load_model()
    # 未量化 fp32 权重
    W1f, b1f, W2f, b2f, W3f, b3f = encrypted_infer_load_weights()
    imgs, labels = load_testset()
    N = len(imgs)
    print("[verify] 测试集 N=%d" % N)

    # ---- fp32 (未量化, 无预缩放) 全量 ----
    preds_f = []
    for i in range(N):
        logits = forward_fp32(imgs[i], W1f, b1f, W2f, b2f, W3f, b3f)
        preds_f.append(int(np.argmax(logits)))
    preds_f = np.array(preds_f, dtype=np.int64)
    acc_f = float(np.mean(preds_f == labels)) * 100
    print("[verify] fp32(未量化) 全量 Top-1 = %.4f%% (%d/%d)"
          % (acc_f, int((preds_f == labels).sum()), N))

    # ---- 量化模型 全量 ----
    preds = []
    for i in range(N):
        logits = forward_pt(imgs[i], W1, b1, W2, b2, W3, b3)
        preds.append(int(np.argmax(logits)))
    preds = np.array(preds, dtype=np.int64)
    acc_full = float(np.mean(preds == labels)) * 100
    print("[verify] 量化 全量明文 Top-1 = %.4f%% (%d/%d)"
          % (acc_full, int((preds == labels).sum()), N))

    # ---- 分层 200 张子集 (量化模型) ----
    gidx, X, Y = select_stratified(20, 42)
    sub_pred = preds[gidx]
    acc_sub = float(np.mean(sub_pred == Y)) * 100
    print("[verify] 量化 分层200子集 Top-1 = %.4f%% (%d/200)"
          % (acc_sub, int((sub_pred == Y).sum()), 200))

    # ---- 与 E10 CSV 对拍 (pt_pred) ----
    import csv
    csv_path = os.path.join(OUTDIR, "e2e_accuracy.csv")
    mism = 0
    with open(csv_path, "r", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        for r in rd:
            idx = int(r["sample_idx"])
            pt = int(r["pt_pred"])
            if preds[idx] != pt:
                mism += 1
    print("[verify] E10 pt_pred 与本重算不一致数 = %d/200 (应=0)" % mism)

    # ---- 每类子集正确率 ----
    print("[verify] 量化 每类子集正确率:")
    for c in range(10):
        m = Y == c
        n = int(m.sum())
        ok = int((sub_pred[m] == Y[m]).sum())
        print("  class %d: %d/%d = %.1f%%" % (c, ok, n, 100.0 * ok / n))


if __name__ == "__main__":
    main()

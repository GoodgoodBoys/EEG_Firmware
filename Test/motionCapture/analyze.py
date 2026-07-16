#!/usr/bin/env python3
"""
analyze.py — 分析 events/ 下 tap/shake/bump 三类动作，提取判别特征并给阈值建议。
用法:  python analyze.py
"""
import os, glob, math
import numpy as np

ROOT = os.path.join(os.path.dirname(__file__), "events")
GYRO_REV_TH = 250.0      # 陀螺方向反转判定阈值(dps)，与固件 shake 一致
SETTLE_TH   = 200.0      # |a|-1000 回落到此以下视为“静止”(mg)
ACTIVE_TH   = 300.0      # 判活跃段/多峰用的门限(mg)


def load(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    return d  # cols: seq,t_ms,ax,ay,az,gx,gy,gz


def reversals(sig, th):
    """方向反转次数（穿越 +th / -th 且方向翻转）"""
    last = 0; n = 0
    for v in sig:
        d = 1 if v > th else (-1 if v < -th else 0)
        if d != 0 and last != 0 and d != last:
            n += 1
        if d != 0:
            last = d
    return n


def features(d):
    seq = d[:, 0]; t = d[:, 1] / 1000.0
    a = d[:, 2:5]; g = d[:, 5:8]
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.004

    # 重力方向：用触发前基线(seq<0)估计
    base = a[seq < 0]
    if len(base) < 5:
        base = a[:10]
    gmean = base.mean(axis=0)
    ghat = gmean / (np.linalg.norm(gmean) + 1e-9)
    base_vert = base @ ghat            # 基线沿重力轴分量 ≈ +1000

    vert = a @ ghat                    # 每样本沿重力轴分量
    linV = vert - base_vert.mean()     # 去重力后的垂直冲击
    horiz = np.linalg.norm(a - np.outer(vert, ghat), axis=1)
    amag = np.linalg.norm(a, axis=1)
    dev = np.abs(amag - 1000.0)

    # jerk：相邻加速度差 / dt
    jerk = np.linalg.norm(np.diff(a, axis=0), axis=1) / dt if len(a) > 1 else np.array([0.0])

    gmax = np.abs(g).max()
    grms = math.sqrt((g ** 2).sum(axis=1).mean())
    rev = reversals(g[:, 0], GYRO_REV_TH) + reversals(g[:, 1], GYRO_REV_TH) + reversals(g[:, 2], GYRO_REV_TH)

    # 峰值位置 + 冲击后回静止时间
    pk = int(np.argmax(dev))
    after = np.where(dev[pk:] > SETTLE_TH)[0]
    settle_ms = (t[pk + after[-1]] - t[pk]) * 1000 if len(after) else 0.0
    active_ms = (dev > ACTIVE_TH).sum() * dt * 1000

    # 主轴（峰值时刻）
    axis = "XYZ"[int(np.argmax(np.abs(a[pk] - gmean)))]

    return dict(
        peak_dev=dev.max(),
        peak_linV=np.abs(linV).max(),
        peak_horiz=horiz.max(),
        vh_ratio=np.abs(linV).max() / max(horiz.max(), 1.0),
        peak_jerk=jerk.max(),
        gyro_max=gmax,
        gyro_rms=grms,
        reversals=rev,
        settle_ms=settle_ms,
        active_ms=active_ms,
        axis=axis,
    )


def pct(vals, p):
    return float(np.percentile(vals, p))


def summarize(cls, feats):
    keys = ["peak_dev", "peak_linV", "peak_horiz", "vh_ratio", "peak_jerk",
            "gyro_max", "gyro_rms", "reversals", "settle_ms", "active_ms"]
    print(f"\n===== {cls}  (n={len(feats)}) =====")
    print(f"{'feature':<11} {'p10':>9} {'median':>9} {'p90':>9}")
    for k in keys:
        vals = [f[k] for f in feats]
        print(f"{k:<11} {pct(vals,10):>9.0f} {pct(vals,50):>9.0f} {pct(vals,90):>9.0f}")
    axes = {}
    for f in feats:
        axes[f["axis"]] = axes.get(f["axis"], 0) + 1
    print("主轴分布:", axes)


def main():
    data = {}
    for cls in ("tap", "shake", "bump"):
        files = sorted(glob.glob(os.path.join(ROOT, cls, "*.csv")))
        feats = []
        for fp in files:
            try:
                feats.append(features(load(fp)))
            except Exception as e:
                print(f"  ! {fp}: {e}")
        data[cls] = feats
        summarize(cls, feats)

    # ── 简单分类器验证：垂直主导 + 单次冲击 vs 摇/碰 ──
    print("\n\n########## 判别测试 ##########")
    for VH in (1.5, 2.0, 2.5):
        for REV in (2, 3):
            tp = fp = fn = 0
            for cls, feats in data.items():
                for f in feats:
                    is_tap = (f["vh_ratio"] >= VH and f["reversals"] < REV
                              and f["peak_linV"] >= 800)
                    if cls == "tap":
                        tp += is_tap; fn += (not is_tap)
                    else:
                        fp += is_tap
                    _ = fp
            # 重新数（上面 fp 混了，规范化）
            tp = sum(1 for f in data["tap"] if f["vh_ratio"] >= VH and f["reversals"] < REV and f["peak_linV"] >= 800)
            miss = len(data["tap"]) - tp
            fp_shake = sum(1 for f in data["shake"] if f["vh_ratio"] >= VH and f["reversals"] < REV and f["peak_linV"] >= 800)
            fp_bump = sum(1 for f in data["bump"] if f["vh_ratio"] >= VH and f["reversals"] < REV and f["peak_linV"] >= 800)
            print(f"VH>={VH} rev<{REV} linV>=800 : tap命中 {tp}/{len(data['tap'])} "
                  f"漏{miss} | 误判 shake={fp_shake} bump={fp_bump}")


if __name__ == "__main__":
    main()

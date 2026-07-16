# -*- coding: utf-8 -*-
# 纯加速度判据扫描（陀螺不参与，留给现有 shake 检测器做互斥）
import glob, os, numpy as np
from analyze import features, load

ROOT = os.path.join(os.path.dirname(__file__), "events")
D = {c: [features(load(f)) for f in sorted(glob.glob(os.path.join(ROOT, c, "*.csv")))]
     for c in ("tap", "shake", "bump")}

# 纯 accel 特征分布
for k in ("peak_linV", "peak_horiz", "vh_ratio", "peak_jerk", "active_ms", "settle_ms"):
    row = []
    for c in ("tap", "shake", "bump"):
        v = [f[k] for f in D[c]]
        row.append(f"{c}:{np.percentile(v,10):.0f}/{np.percentile(v,50):.0f}/{np.percentile(v,90):.0f}")
    print(f"{k:<11}", "  ".join(row))
print("\n(每格 p10/median/p90)\n")

best = None
for lo in (700, 900, 1000):
    for hi in (2600, 3000, 3500):
        for vh in (1.2, 1.5):
            for jhi in (800000, 1000000, 1300000):
                def ok(f):
                    return (lo <= f["peak_linV"] <= hi and f["vh_ratio"] >= vh
                            and f["peak_jerk"] <= jhi)
                tp = sum(1 for f in D["tap"] if ok(f))
                fpb = sum(1 for f in D["bump"] if ok(f))
                fps = sum(1 for f in D["shake"] if ok(f))
                score = tp - 2 * (fpb + fps)
                cand = (score, tp, fpb, fps, lo, hi, vh, jhi)
                if best is None or score > best[0]:
                    best = cand
print("BEST accel-only (score,tp,fpb,fps,lo,hi,vh,jhi):", best)
s, tp, fpb, fps, lo, hi, vh, jhi = best
print(f"\n推荐纯加速度规则: linV∈[{lo},{hi}]mg, vh_ratio>={vh}, jerk<{jhi}")
print(f"  → tap {tp}/40, shake误 {fps}/30, bump误 {fpb}/20")
# shake 若再叠加“与摇一摇检测器互斥”，fps 实际可归零

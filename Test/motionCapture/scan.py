# -*- coding: utf-8 -*-
import glob, os, numpy as np, math
from analyze import features, load

ROOT = os.path.join(os.path.dirname(__file__), "events")
D = {c: [features(load(f)) for f in sorted(glob.glob(os.path.join(ROOT, c, "*.csv")))]
     for c in ("tap", "shake", "bump")}

for k in ("peak_linV", "peak_jerk", "gyro_max"):
    print("==", k, "==")
    for c in ("tap", "bump"):
        v = sorted(f[k] for f in D[c])
        print(f"{c:5}", " ".join(f"{x:6.0f}" for x in v))
print()

best = None
for gcap in (300, 400, 500):
    for lo in (700, 900):
        for hi in (2200, 2600, 3000, 3500, 99999):
            for jhi in (800000, 1000000, 1300000, 9999999):
                def ok(f):
                    return (lo <= f["peak_linV"] <= hi and f["reversals"] < 2
                            and f["gyro_max"] < gcap and f["peak_jerk"] <= jhi)
                tp = sum(1 for f in D["tap"] if ok(f))
                fpb = sum(1 for f in D["bump"] if ok(f))
                fps = sum(1 for f in D["shake"] if ok(f))
                score = tp - 2 * (fpb + fps)
                cand = (score, tp, fpb, fps, gcap, lo, hi, jhi)
                if best is None or score > best[0]:
                    best = cand
                if tp >= 34 and fpb <= 5:
                    print(f"g<{gcap} linV[{lo},{hi}] jerk<{jhi}: tap {tp}/40 | bump误={fpb} shake误={fps}")
print("\nBEST (score,tp,fpb,fps,gcap,lo,hi,jhi):", best)

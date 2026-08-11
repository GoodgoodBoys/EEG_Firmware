#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""超时参数扫描 + 失败原因归因 + T7 补投验证"""
import sys, statistics, collections
sys.stdout.reconfigure(encoding="utf-8")
import uv_joint_test2 as T

def batch(level, seeds=60, **kw):
    rs = [T.run(level, seed=s, **kw) for s in range(1, seeds+1)]
    why = collections.Counter()
    for r in rs:
        if not r["app_ok"]:
            f = [x for x in r["res"] if x["kind"] == "APP" and not x["ok"]]
            why[f[-1]["why"] if f else "未开始"] += 1
    ts = sorted(r["t"] for r in rs)
    return dict(ok=sum(r["app_ok"] for r in rs)/len(rs),
                med=statistics.median(ts), p90=ts[int(len(ts)*0.9)-1],
                amp=statistics.median([r["amp"] for r in rs]), why=why)

print("="*104)
print("A. 超时参数扫描（P123 档，只变 STEP / LIVE / MAXR）—— 找回被我调坏的可靠性")
print("="*104)
GRID = [
    ("现行 CURR",       None, None, None),
    ("我上轮建议 3000/15000/6", 3000, 15000, 6),
    ("STEP1200 LIVE15000",     1200, 15000, 12),
    ("STEP1200 LIVE30000",     1200, 30000, 20),
    ("STEP1500 LIVE30000",     1500, 30000, 20),
    ("STEP2000 LIVE40000",     2000, 40000, 20),
    ("STEP800  LIVE30000",      800, 30000, 30),
]
for tag, lossp, rtt in [("丢包25% RTT120", 0.25, 60), ("丢包40% RTT400", 0.40, 200)]:
    print(f"\n── 链路：{tag} ──")
    print(f"{'参数':<26}{'成功率':>8}{'中位':>10}{'P90':>10}{'放大':>8}   失败原因")
    for name, st, lv, mr in GRID:
        lvl = "CURR" if st is None else "P123"
        if st is not None:
            _orig = T.Cfg.__init__
            def patched_init(self, level, step=None, live=None, _st=st, _lv=lv, _mr=mr, _o=_orig):
                _o(self, level, _st, _lv); self.MAXR = _mr
            T.Cfg.__init__ = patched_init
        r = batch(lvl, up=T.Link(rtt, loss=lossp), dn=T.Link(rtt, loss=lossp))
        if st is not None: T.Cfg.__init__ = _orig
        w = " ".join(f"{k}×{v}" for k, v in r["why"].most_common(3)) or "—"
        print(f"{name:<26}{r['ok']*100:>7.0f}%{r['med']:>9.0f}ms{r['p90']:>9}ms{r['amp']:>7.2f}x   {w}")

print("\n"+"="*104)
print("B. T7 修正版：App 离线时录音 → 只走伙伴 → App 30s 后上线 → 是否自动补投")
print("="*104)
print(f"{'档位':<8}{'伙伴收到':>10}{'App补投收到':>13}{'文件最终删除':>14}{'App尝试次数':>13}")
for lv in ["CURR", "P123", "FULL"]:
    okp = oka = dele = 0; att = 0
    for s in range(1, 11):
        r = T.run(lv, seed=s, up=T.Link(60), want_app=True, want_peer=True,
                  app_off=True, app_on_at=30000, max_ms=60000)
        okp += r["peer_ok"]; oka += r["app_ok"]; dele += (len(r["left"]) == 0)
        att += len([x for x in r["res"] if x["kind"] == "APP"])
    print(f"{lv:<8}{okp*10:>9}%{oka*10:>12}%{dele*10:>13}%{att/10:>12.1f}")

print("\n"+"="*104)
print("C. T8 深挖：伙伴离线 20s 期间 App 先送达 —— 文件是否被 App 单方面删除")
print("="*104)
for lv in ["CURR", "P123", "FULL"]:
    r = T.run(lv, seed=1, up=T.Link(60), want_app=True, want_peer=True,
              peer_off=True, peer_on_at=20000, max_ms=60000)
    print(f"\n[{lv}] App收齐={r['app_ok']} 伙伴收齐={r['peer_ok']} 剩余槽={r['left']}")
    for x in r["res"]:
        print(f"   · {x['kind']:4} {'OK  ' if x['ok'] else 'FAIL'} why={x['why']:<10} "
              f"acked={x['acked']}/{x['size']} t={x['ms']}ms")

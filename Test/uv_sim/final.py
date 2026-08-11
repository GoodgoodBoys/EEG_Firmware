#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最终参数集全场景回归：CURR vs FINAL(账本解耦 + P0/P1/P2 + 修正后的超时)"""
import sys, statistics
sys.stdout.reconfigure(encoding="utf-8")
import uv_joint_test2 as T

_orig = T.Cfg.__init__
def final_init(self, level, step=None, live=None):
    _orig(self, level, 1200, 30000)          # ★ 修正：STEP 不动，LIVENESS 大幅放宽
    if level != "CURR":
        self.MAXR = 20                        # ★ 8 → 20（关键）
        self.HAND = 10000
T.Cfg.__init__ = final_init

def bat(lv, seeds=60, **kw):
    rs = [T.run(lv, seed=s, **kw) for s in range(1, seeds+1)]
    ts = sorted(r["t"] for r in rs)
    return (sum(r["app_ok"] for r in rs)/len(rs), sum(r["peer_ok"] for r in rs)/len(rs),
            statistics.median(ts), statistics.median([r["amp"] for r in rs]))

CASES = [
 ("理想 RTT120",            dict(up=T.Link(60)), 30),
 ("云端 RTT600",            dict(up=T.Link(300)), 30),
 ("ACK滞留乱序30%@1.5s",     dict(up=T.Link(60), dn=T.Link(60, xd_p=.3, xd=1500)), 40),
 ("丢包25%",                dict(up=T.Link(60, loss=.25), dn=T.Link(60, loss=.25)), 60),
 ("丢包40%+RTT400",         dict(up=T.Link(200, loss=.40), dn=T.Link(200, loss=.40)), 60),
 ("丢包55%+RTT600",         dict(up=T.Link(300, loss=.55), dn=T.Link(300, loss=.55)), 60),
 ("坏槽 voice_0=0B",        dict(up=T.Link(60), bad=True), 10),
 ("双通路并发",              dict(up=T.Link(60), want_peer=True), 20),
]
print("="*100)
print("最终回归：CURR(现行)  vs  FINAL(账本解耦 + P0坏槽 + P1下溢 + P2cwnd + 修正超时)")
print("="*100)
print(f"{'场景':<24}{'CURR App':>10}{'FINAL App':>11}{'CURR放大':>10}{'FINAL放大':>11}"
      f"{'CURR中位':>11}{'FINAL中位':>11}")
for name, kw, sd in CASES:
    a0,_,m0,p0 = bat("CURR", sd, **kw)
    a1,_,m1,p1 = bat("FULL", sd, **kw)
    print(f"{name:<24}{a0*100:>9.0f}%{a1*100:>10.0f}%{p0:>9.2f}x{p1:>10.2f}x"
          f"{m0:>9.0f}ms{m1:>9.0f}ms")

print("\n" + "="*100)
print("解耦专项（用户核心诉求）")
print("="*100)
rows = [
 ("App离线录音→伙伴收 / App 30s后上线补投",
  dict(up=T.Link(60), want_app=True, want_peer=True, app_off=True, app_on_at=30000, max_ms=60000)),
 ("伙伴离线录音→App收 / 伙伴 20s后上线补投",
  dict(up=T.Link(60), want_app=True, want_peer=True, peer_off=True, peer_on_at=20000, max_ms=60000)),
]
print(f"{'场景':<42}{'档位':<8}{'App':>7}{'伙伴':>8}{'文件清理':>10}")
for name, kw in rows:
    for lv in ["CURR", "FULL"]:
        a=p=d=0
        for s in range(1, 11):
            r = T.run(lv, seed=s, **kw)
            a += r["app_ok"]; p += r["peer_ok"]; d += (len(r["left"]) == 0)
        print(f"{name if lv=='CURR' else '':<42}{lv:<8}{a*10:>6}%{p*10:>7}%{d*10:>9}%")

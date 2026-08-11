#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACK 竞争/冒险专项：RTT > UV_STEP_TIMEOUT_MS 时 BEGIN 重传与会话的竞争"""
import sys, statistics
sys.stdout.reconfigure(encoding="utf-8")
import uv_joint_test2 as T

_orig = T.Cfg.__init__
def final_init(self, level, step=None, live=None):
    _orig(self, level, 1200, 30000)
    if level != "CURR":
        self.MAXR = 20; self.HAND = 10000
T.Cfg.__init__ = final_init

print("=" * 100)
print("H1  伙伴接收端：重复 PV_BEGIN 是否会重置已收进度")
print("    构造：单向延迟 > UV_STEP_TIMEOUT_MS/2 → 发送端在收到首个 ACK 前就重发了 BEGIN，")
print("          重发的 BEGIN 在会话已开始传输后才抵达接收端。")
print("=" * 100)
print(f"{'单向延迟':>10}{'RTT':>8}{'伙伴收齐率':>12}{'中位耗时':>11}{'放大':>8}  主要失败原因")
for d in [60, 300, 500, 700, 900, 1200]:
    ok = 0; ts = []; amps = []; whys = {}
    for s in range(1, 31):
        r = T.run("FULL", seed=s, up=T.Link(d), dn=T.Link(d),
                  want_app=False, want_peer=True, max_ms=200000)
        ok += r["peer_ok"]; ts.append(r["t"]); amps.append(r["amp"])
        if not r["peer_ok"]:
            f = [x for x in r["res"] if x["kind"] == "PEER" and not x["ok"]]
            w = f[-1]["why"] if f else "未开始"
            whys[w] = whys.get(w, 0) + 1
    w = " ".join(f"{k}×{v}" for k, v in sorted(whys.items(), key=lambda x: -x[1])[:2]) or "—"
    print(f"{d:>9}ms{d*2:>7}ms{ok/30*100:>11.0f}%{statistics.median(ts):>9.0f}ms"
          f"{statistics.median(amps):>7.2f}x  {w}")

print()
print("=" * 100)
print("H2  同样条件下 App 接收端（voice_service.dart 已有同 xferId 幂等保护）作为对照")
print("=" * 100)
print(f"{'单向延迟':>10}{'RTT':>8}{'App收齐率':>12}{'中位耗时':>11}{'放大':>8}")
for d in [60, 300, 500, 700, 900, 1200]:
    ok = 0; ts = []; amps = []
    for s in range(1, 31):
        r = T.run("FULL", seed=s, up=T.Link(d), dn=T.Link(d),
                  want_app=True, want_peer=False, max_ms=200000)
        ok += r["app_ok"]; ts.append(r["t"]); amps.append(r["amp"])
    print(f"{d:>9}ms{d*2:>7}ms{ok/30*100:>11.0f}%{statistics.median(ts):>9.0f}ms"
          f"{statistics.median(amps):>7.2f}x")

print()
print("=" * 100)
print("H3  单条详细轨迹（单向 700ms）：看伙伴接收端的 next 是否被重复 BEGIN 打回 0")
print("=" * 100)
r = T.run("FULL", seed=3, up=T.Link(700), dn=T.Link(700),
          want_app=False, want_peer=True, max_ms=60000)
print(f"伙伴收齐={r['peer_ok']}  耗时={r['t']}ms  放大={r['amp']:.2f}x")
for x in r["res"]:
    print(f"  · {x['kind']} {'OK' if x['ok'] else 'FAIL'} why={x['why']} "
          f"acked={x['acked']}/{x['size']} stall={x['stalls']} retry={x.get('retry','?')}")

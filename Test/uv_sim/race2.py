#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACK 竞争专项 2：上行乱序（broker 排队尖峰）导致【先发的 PV_BEGIN 后到】"""
import sys, statistics
sys.stdout.reconfigure(encoding="utf-8")
import uv_joint_test2 as T

_orig = T.Cfg.__init__
def final_init(self, level, step=None, live=None):
    _orig(self, level, 1200, 30000)
    if level != "CURR":
        self.MAXR = 20; self.HAND = 10000
T.Cfg.__init__ = final_init

# ── 给 Peer 打上"同 xferId 的重复 BEGIN 幂等"补丁，用于对照 ──
def peer_patched_on_message(s, t, pl, now):
    s.now = now
    b = pl if isinstance(pl, bytes) else pl.encode()
    if not b: return
    if b[:1] == b"{":
        import json
        j = json.loads(b); m = j.get("msg"); x = j.get("xferId", 0)
        if m == "PV_BEGIN":
            if j.get("sn") != "EGG0001" or x == 0: return
            if x == s.did and now - s.dms < T.PV_DEDUP_MS: s._tx("PV_DONE", x); return
            # ★ 补丁：同 xferId 的重复 BEGIN → 幂等回当前进度，绝不重置
            if s.x == x:
                s.last = now; s._tx("PV_ACK", x, s.n); return
            if s.busy: s._tx("PV_BUSY", x); return
            s.x, s.sz, s.n = x, j.get("size", 0), 0
            s.buf = bytearray(s.sz); s.last = now
            s._tx("PV_ACK", x, 0)
            return
    T.Peer.on_message_orig(s, t, pl, now)
T.Peer.on_message_orig = T.Peer.on_message


def run_set(patched_peer, up, dn, seeds=40, **kw):
    T.Peer.on_message = peer_patched_on_message if patched_peer else T.Peer.on_message_orig
    ok = 0; ts = []; amps = []; whys = {}
    for s in range(1, seeds + 1):
        r = T.run("FULL", seed=s, up=up, dn=dn, want_app=False, want_peer=True,
                  max_ms=250000, **kw)
        ok += r["peer_ok"]; ts.append(r["t"]); amps.append(r["amp"])
        if not r["peer_ok"]:
            f = [x for x in r["res"] if x["kind"] == "PEER" and not x["ok"]]
            w = f[-1]["why"] if f else "未开始"
            whys[w] = whys.get(w, 0) + 1
    T.Peer.on_message = T.Peer.on_message_orig
    return ok / seeds, statistics.median(ts), statistics.median(amps), whys

print("=" * 104)
print("H4  上行乱序：broker 排队尖峰让【先发的 BEGIN 后到】——重复 BEGIN 在数据已开始落位后抵达")
print("    左=现行伙伴接收端(无幂等保护)   右=打上'同 xferId 幂等'补丁")
print("=" * 104)
print(f"{'场景':<34}{'现行收齐率':>11}{'现行耗时':>10}{'现行放大':>10}"
      f"{'补丁收齐率':>12}{'补丁耗时':>10}{'补丁放大':>10}")
CASES = [
  ("基线 RTT600 无乱序",        T.Link(300),                        T.Link(300)),
  ("上行20%尖峰+2.5s RTT600",   T.Link(300, xd_p=0.20, xd=2500),    T.Link(300)),
  ("上行35%尖峰+3s  RTT600",    T.Link(300, xd_p=0.35, xd=3000),    T.Link(300)),
  ("上行20%尖峰+2.5s +丢包20%", T.Link(300, loss=0.20, xd_p=0.20, xd=2500), T.Link(300, loss=0.20)),
  ("上行35%尖峰+3s  +丢包30%",  T.Link(300, loss=0.30, xd_p=0.35, xd=3000), T.Link(300, loss=0.30)),
]
for name, up, dn in CASES:
    a0, t0, m0, w0 = run_set(False, up, dn)
    a1, t1, m1, w1 = run_set(True,  up, dn)
    print(f"{name:<34}{a0*100:>10.0f}%{t0:>8.0f}ms{m0:>9.2f}x"
          f"{a1*100:>11.0f}%{t1:>8.0f}ms{m1:>9.2f}x")
    if w0: print(f"{'':34}   现行失败原因: " + " ".join(f"{k}×{v}" for k, v in w0.items()))
    if w1: print(f"{'':34}   补丁失败原因: " + " ".join(f"{k}×{v}" for k, v in w1.items()))

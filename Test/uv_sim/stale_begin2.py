#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
定向验证 2：会话1 的 PV_BEGIN 在【会话2 进行中】才抵达（两个不同 xferId）

构造：
  t=0      会话1(X1) 发 BEGIN；接收端收到并建会话 X1
  t=0..10s 回程被阻断 → 发送端握手超时失败（"无人应答"）
  t=10s    回程恢复；账本退避 5s
  t=15s    会话2(X2) 开始，正常传输
  t=T      重放【会话1 的 BEGIN(X1)】—— 对接收端而言这是"一个不同 xferId 的 BEGIN"，
           现行实现无条件覆盖 → 会话2 的进度被清零、后续 X2 分片全被丢弃
"""
import sys, random, json, hashlib
sys.stdout.reconfigure(encoding="utf-8")
import uv_joint_test2 as T

GUARD_MS = 3000

def peer_guarded(s, t, pl, now):
    b = pl if isinstance(pl, bytes) else pl.encode()
    if b[:1] == b"{":
        j = json.loads(b)
        if j.get("msg") == "PV_BEGIN":
            x = j.get("xferId", 0)
            if s.x != 0 and x != s.x and (now - s.last) < GUARD_MS:
                s.now = now; s._tx("PV_BUSY", x)
                print(f"      ✓ t={now}ms 抢占保护生效：拒绝 xferId={x}（当前 {s.x} 仍活跃）")
                return
    T.Peer.on_message_orig(s, t, pl, now)
T.Peer.on_message_orig = T.Peer.on_message


class AtkBroker(T.Broker):
    def __init__(s, rng, block_until, replay_at):
        super().__init__(rng)
        s.block_until, s.replay_at = block_until, replay_at
        s.first_begin = None; s.replayed = False
    def publish(s, t, pl, lk, now):
        if isinstance(pl, str) and '"PV_BEGIN"' in pl and s.first_begin is None:
            s.first_begin = (t, pl)
        if t == "dev/TID/cmd" and now < s.block_until:   # 阻断回程 → 会话1 握手失败
            s.pub += 1; s.drop += 1; return
        super().publish(t, pl, lk, now)
    def pump(s, now):
        if not s.replayed and s.first_begin and now >= s.replay_at:
            s.replayed = True
            t, pl = s.first_begin
            ep = s.subs.get(t)
            if ep:
                s.q.append((now, ep, t, pl))
                print(f"      ↳ t={now}ms 重放会话1 的 BEGIN(xferId={json.loads(pl)['xferId']})")
        super().pump(now)


def run(guarded, block_until=11000, replay_at=17000, seed=1, size=240000):
    T.Peer.on_message = peer_guarded if guarded else T.Peer.on_message_orig
    rng = random.Random(seed)
    cfg = T.Cfg("FULL", 1200, 30000); cfg.MAXR = 20; cfg.HAND = 10000
    b = AtkBroker(rng, block_until, replay_at)
    dev = T.Device(b, cfg, rng); dev.up = T.Link(120)
    app = T.App(b, T.Link(120), "dev/TID/cmd"); peer = T.Peer(b, T.Link(120), "dev/TID/cmd")
    b.sub("term/EGG0001/voice", app); b.sub("dev/PEERTID/voicePlay", peer)
    b.sub("dev/TID/cmd", dev)
    pay = bytes(rng.randrange(256) for _ in range(size))
    md5 = hashlib.md5(pay).hexdigest()
    dev.slots[0] = pay
    dev.meta[0] = {"app": T.NA, "peer": T.PEND, "atry": 0, "ptry": 0, "anext": 0, "pnext": 0}
    dev.now = 0
    t = 0; prevN = 0; prevX = 0; regress = 0
    while t < 150000:
        b.pump(t)
        if t % 5 == 0: dev.service(t)
        app.tick(t); peer.tick(t)
        if peer.x == prevX and peer.x != 0 and peer.n < prevN:
            regress += 1
            print(f"      ★ t={t}ms 伙伴进度被打回：{prevN} → {peer.n}")
        prevX, prevN = peer.x, peer.n
        pend = any(m["peer"] == T.PEND or m["app"] == T.PEND for m in dev.meta.values())
        if (not dev.active() and not b.q and not pend):
            t += 1; b.pump(t); break
        t += 1
    T.Peer.on_message = T.Peer.on_message_orig
    return md5 in peer.played, t, dev.res, regress


print("=" * 96)
print("旧会话 BEGIN 迟到攻击（两个不同 xferId）")
print("=" * 96)
for tag, guarded in [("现行（无抢占保护）", False), (f"加抢占保护 {GUARD_MS}ms", True)]:
    print(f"\n── {tag} ──")
    ok, dur, res, reg = run(guarded)
    for x in res:
        print(f"      · {x['kind']} {'OK' if x['ok'] else 'FAIL'} why={x['why']:<10} "
              f"acked={x['acked']}/{x['size']} t={x['ms']}ms")
    print(f"   → 伙伴收齐={ok}  总耗时={dur}ms  进度回退次数={reg}")

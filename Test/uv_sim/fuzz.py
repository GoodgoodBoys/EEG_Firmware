#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对抗性 broker + 不变量断言 —— 自动发现协议竞争。

思路：把"重复投递"当成一等公民注入。MQTT QoS0/1 本来就允许重复，
      加上 broker 排队会乱序，因此【任何入站帧处理不幂等的地方都会被打出来】。
      再配一组每步都检查的不变量，违反即打印轨迹。
"""
import sys, random, json, hashlib, collections
sys.stdout.reconfigure(encoding="utf-8")
import uv_joint_test2 as T

# ── 对抗性 broker：在原 Broker 上叠加 复制 / 乱序 / 延迟尖峰 ──
class AdvBroker(T.Broker):
    def __init__(s, rng, dup_p=0.25, dup_delay=(50, 4000), reorder_p=0.25, reorder=(100, 3000)):
        super().__init__(rng)
        s.dup_p, s.dup_delay = dup_p, dup_delay
        s.reorder_p, s.reorder = reorder_p, reorder
        s.dups = 0
    def publish(s, t, pl, lk, now):
        super().publish(t, pl, lk, now)
        # 复制投递：同一帧再送一份，延迟随机（模拟 QoS1 重投 + broker 排队）
        if s.rng.random() < s.dup_p:
            ep = s.subs.get(t)
            if ep:
                d = s.rng.randint(*s.dup_delay)
                s.q.append((now + d, ep, t, pl)); s.dups += 1
        # 乱序：给最近入队的帧追加一个额外延迟，打乱到达顺序
        if s.q and s.rng.random() < s.reorder_p:
            i = len(s.q) - 1
            dt, ep, tt, p2 = s.q[i]
            s.q[i] = (dt + s.rng.randint(*s.reorder), ep, tt, p2)


class Viol(Exception): pass

def check(cond, msg, ctx):
    if not cond:
        raise Viol(f"{msg} | {ctx}")

def run_fuzz(seed, size=120000, want_app=True, want_peer=True, max_ms=300000,
             dup_p=0.25, reorder_p=0.25, patched=True):
    rng = random.Random(seed)
    cfg = T.Cfg("FULL" if patched else "CURR", 1200, 30000)
    if patched: cfg.MAXR = 20; cfg.HAND = 10000
    up = T.Link(120); dn = T.Link(120)
    b = AdvBroker(rng, dup_p=dup_p, reorder_p=reorder_p)
    dev = T.Device(b, cfg, rng); dev.up = up
    app = T.App(b, dn, "dev/TID/cmd"); peer = T.Peer(b, dn, "dev/TID/cmd")
    b.sub("term/EGG0001/voice", app); b.sub("dev/PEERTID/voicePlay", peer)
    b.sub("dev/TID/cmd", dev)
    pay = bytes(rng.randrange(256) for _ in range(size))
    md5 = hashlib.md5(pay).hexdigest()
    dev.slots[0] = pay
    dev.meta[0] = {"app": T.PEND if want_app else T.NA,
                   "peer": T.PEND if want_peer else T.NA,
                   "atry": 0, "ptry": 0, "anext": 0, "pnext": 0}
    dev.now = 0
    if not cfg.ledger:
        if want_app: dev.pull = True
        if want_peer: dev.peer_pend = 0

    # 不变量追踪状态
    hi = {"peer_next": 0, "peer_x": 0}
    app_hi = {}
    t = 0
    while t < max_ms:
        b.pump(t)
        if t % 5 == 0: dev.service(t)
        app.tick(t); peer.tick(t)

        ctx = f"t={t} devApp({dev.app.state},n={dev.app.next},s={dev.app.sent}) " \
              f"devPeer({dev.peer.state},n={dev.peer.next},s={dev.peer.sent}) " \
              f"peerRx(x={peer.x},n={peer.n})"

        # INV1 发送端：窗口右沿不得落在左沿之前（下溢防护是否生效）
        for se in (dev.app, dev.peer):
            if se.state != T.IDLE:
                check(se.sent >= se.next, "INV1 sent<next（窗口下溢）", ctx)
                check(se.next <= se.size, "INV1b next>size", ctx)

        # INV2 伙伴接收端：同一 xferId 内 next 单调不减（竞争2的水位修复依赖此性质）
        if peer.x != 0:
            if peer.x == hi["peer_x"]:
                check(peer.n >= hi["peer_next"], "INV2 伙伴 next 在同一 xferId 内回退", ctx)
            hi["peer_x"], hi["peer_next"] = peer.x, peer.n

        # INV3 App 接收端：同上
        for sn, st in app.rx.items():
            k = (sn, st["x"])
            prev = app_hi.get(k, 0)
            check(st["n"] >= prev, "INV3 App next 在同一 xferId 内回退", ctx)
            app_hi[k] = st["n"]

        # INV4 账本 vs 文件：used ⟺ 文件存在
        for sl, m in dev.meta.items():
            check(sl in dev.slots, "INV4 账本有记录但文件已删（被单路删掉了？）", ctx)

        # INV5 不重复投递
        check(len(app.saved) <= 1, "INV5 App 重复保存同一条语音", ctx)
        check(len(peer.played) <= 1, "INV5 伙伴重复播放同一条语音", ctx)

        idle = (not dev.active() and not app.pf and not b.q and not dev.pull
                and dev.peer_pend is None
                and (not cfg.ledger or (dev.pick(True, t) < 0 and dev.pick(False, t) < 0)))
        if idle:
            t += 1; b.pump(t); app.tick(t); break
        t += 1

    return dict(t=t, app_ok=md5 in app.saved, peer_ok=md5 in peer.played,
                app_n=len(app.saved), peer_n=len(peer.played),
                dups=b.dups, res=dev.res)


if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    print("=" * 96)
    print(f"对抗性 fuzz：复制率 25% + 乱序率 25%，{N} 个种子")
    print("=" * 96)
    viol = collections.Counter(); fail = collections.Counter()
    okA = okP = 0; dups = 0
    for s in range(1, N + 1):
        try:
            r = run_fuzz(s)
            okA += r["app_ok"]; okP += r["peer_ok"]; dups += r["dups"]
            if not r["app_ok"]:
                f = [x for x in r["res"] if x["kind"] == "APP" and not x["ok"]]
                fail["APP:" + (f[-1]["why"] if f else "未开始")] += 1
            if not r["peer_ok"]:
                f = [x for x in r["res"] if x["kind"] == "PEER" and not x["ok"]]
                fail["PEER:" + (f[-1]["why"] if f else "未开始")] += 1
        except Viol as e:
            viol[str(e).split("|")[0].strip()] += 1
            if viol[str(e).split("|")[0].strip()] == 1:
                print(f"\n★ 首次违反 (seed={s}):\n  {e}\n")
    print(f"\nApp 收齐率  : {okA/N*100:.0f}%   ({okA}/{N})")
    print(f"伙伴收齐率  : {okP/N*100:.0f}%   ({okP}/{N})")
    print(f"注入的重复帧: {dups}")
    print(f"\n不变量违反  : {dict(viol) if viol else '无'}")
    print(f"失败原因    : {dict(fail) if fail else '无'}")

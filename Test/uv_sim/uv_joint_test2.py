#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音链路三端联合测试 v2 —— 多种子统计 + 下溢/空转埋点 + 账本解耦对照"""
import sys, random, json, hashlib, statistics
sys.stdout.reconfigure(encoding="utf-8")
from dataclasses import dataclass

U32 = 0xFFFFFFFF
UV_CHUNK, UV_HDR, UV_MAGIC = 4000, 9, 0xD1
UV_MAX_SEND_PER_PUMP = 2
PV_DEDUP_MS = APP_DEDUP_MS = 60000
IDLE, WAIT_READY, SENDING, WAIT_DONE = 0, 1, 2, 3

# ═══ 三个档位 ═══
#  CURR : 现行代码
#  P123 : P1窗口下溢 + P2 cwnd + P3超时重标定 + P0坏槽自愈
#  FULL : P123 + 账本解耦(文件不由单路删除) + presence 门控 + 失败退避重试
class Cfg:
    def __init__(self, level, step=None, live=None):
        self.level = level
        p = level != "CURR"
        self.fix_underflow = p
        self.cwnd_on       = p
        self.badslot_fix   = p
        self.ledger        = (level == "FULL")
        self.presence      = (level == "FULL")
        self.retry         = (level == "FULL")
        if p:
            self.STEP  = step if step else 3000
            self.LIVE  = live if live else 15000
            self.HAND  = 10000
            self.MAXR  = 6
            self.CW_I, self.CW_MIN, self.CW_MAX = 4*UV_CHUNK, 2*UV_CHUNK, 8*UV_CHUNK
        else:
            self.STEP, self.LIVE, self.HAND, self.MAXR = 1200, 6000, 6000, 8
            self.CW_I = self.CW_MIN = self.CW_MAX = 8*UV_CHUNK
        self.TOTAL = 180000

@dataclass
class Link:
    delay: int = 60; jitter: int = 0; loss: float = 0.0
    xd_p: float = 0.0; xd: int = 0

class Broker:
    def __init__(s, rng): s.rng, s.q, s.subs, s.pub, s.drop = rng, [], {}, 0, 0
    def sub(s, t, ep): s.subs[t] = ep
    def publish(s, t, pl, lk, now):
        s.pub += 1
        if s.rng.random() < lk.loss: s.drop += 1; return
        d = lk.delay + (s.rng.randint(0, lk.jitter) if lk.jitter else 0)
        if lk.xd and s.rng.random() < lk.xd_p: d += lk.xd
        ep = s.subs.get(t)
        if ep: s.q.append((now + d, ep, t, pl))
    def pump(s, now):
        due = [x for x in s.q if x[0] <= now]
        if due:
            s.q = [x for x in s.q if x[0] > now]
            for _, ep, t, pl in due: ep.on_message(t, pl, now)

@dataclass
class Sess:
    kind: str; state: int = IDLE; topic: str = ""; xid: int = 0; slot: int = -1
    uid: int = 0; size: int = 0; next: int = 0; sent: int = 0; deadline: int = 0
    retry: int = 0; t0: int = 0; rx: int = 0; cwnd: int = 32000; stalls: int = 0

# ═══ 目的地投递状态（FULL 档账本）═══
NA, PEND, SEND, DONE, GIVEUP = 0, 1, 2, 3, 4
LINK_BK = [5000, 15000, 45000, 120000, 300000]

class Device:
    def __init__(s, b, cfg, rng):
        s.b, s.cfg, s.rng = b, cfg, rng
        s.app, s.peer = Sess("APP"), Sess("PEER")
        s.slots, s.uid = {}, {}
        s.meta = {}                     # slot -> {"app":st,"peer":st,"atry":n,"ptry":n,"anext":ms,"pnext":ms}
        s.pull = False
        s.peer_pend = None
        s.app_online = s.peer_online = True
        s.res, s.wire, s.now = [], 0, 0
        s.underflow = 0; s.idle_pumps = 0
        s.at, s.pt = "term/EGG0001/voice", "dev/PEERTID/voicePlay"
    def _pub(s, t, pl, lk):
        s.wire += len(pl if isinstance(pl, bytes) else pl.encode()); s.b.publish(t, pl, lk, s.now)
    def begin(s, se): s._pub(se.topic, json.dumps({"msg":"PV_BEGIN","xferId":se.xid,"sn":"EGG0001","size":se.size,"uid":se.uid}), s.up)
    def end(s, se):   s._pub(se.topic, json.dumps({"msg":"PV_END","xferId":se.xid}), s.up)
    def chunk(s, se, off):
        d = s.slots.get(se.slot)
        if not d: return -1
        w = min(UV_CHUNK, se.size - off)
        s._pub(se.topic, bytes([UV_MAGIC]) + se.xid.to_bytes(4,"big") + off.to_bytes(4,"big") + d[off:off+w], s.up)
        return 1
    def fill(s, se):
        n0, burst = se.sent, 0
        while True:
            if s.cfg.fix_underflow: inf = se.sent - se.next if se.sent > se.next else 0
            else:
                inf = (se.sent - se.next) & U32
                if se.sent < se.next: s.underflow += 1
            if not (se.sent < se.size and inf < se.cwnd and burst < UV_MAX_SEND_PER_PUMP): break
            ln = min(UV_CHUNK, se.size - se.sent)
            if s.chunk(se, se.sent) < 0: s.stop(se, "读源失败", False); return False
            se.sent += ln; burst += 1
        if se.state == SENDING and se.sent == n0 and se.sent < se.size:
            real = se.sent - se.next if se.sent > se.next else 0
            if real < se.cwnd: s.idle_pumps += 1        # 本可发却没发 = 空转
        return True
    def pump(s, se):
        if se.state == IDLE: return
        n = s.now
        if n - (se.t0 + s.cfg.TOTAL) >= 0: s.stop(se, "总超时", False); return
        if se.state == WAIT_READY:
            if n - (se.t0 + s.cfg.HAND) >= 0: s.stop(se, "无人应答", False); return
        else:
            if n - (se.rx + s.cfg.LIVE) >= 0: s.stop(se, "对端无响应", False); return
        if se.state == SENDING:
            if not s.fill(se): return
            if se.next >= se.size:
                s.end(se); se.state = WAIT_DONE; se.retry = 0; se.deadline = n + s.cfg.STEP; return
            if n - se.deadline >= 0:
                se.retry += 1; se.stalls += 1
                if se.retry > s.cfg.MAXR: s.stop(se, "重试超限", False); return
                if s.cfg.cwnd_on: se.cwnd = max(s.cfg.CW_MIN, se.cwnd // 2)
                se.sent = se.next; se.deadline = n + s.cfg.STEP; s.fill(se)
        else:
            if n - se.deadline >= 0:
                se.retry += 1
                if se.retry > s.cfg.MAXR: s.stop(se, "重试超限", False); return
                (s.begin if se.state == WAIT_READY else s.end)(se)
                se.deadline = n + s.cfg.STEP
    def on_ack(s, se, x, nx):
        if se.state == IDLE or x != se.xid: return
        se.rx = s.now
        if s.cfg.fix_underflow:                      # ★竞争2修复：丢弃滞后 ACK
            if nx < se.ackhigh: return
            se.ackhigh = nx
        if se.state == WAIT_READY:
            se.next = se.sent = nx; se.state = SENDING; se.retry = 0; se.deadline = s.now + s.cfg.STEP
        elif se.state == SENDING:
            if s.cfg.fix_underflow and nx > se.size: return
            if nx > se.next:
                se.next = nx
                if s.cfg.fix_underflow and se.sent < se.next: se.sent = se.next
                if s.cfg.cwnd_on: se.cwnd = min(s.cfg.CW_MAX, se.cwnd + UV_CHUNK)
                se.retry = 0; se.deadline = s.now + s.cfg.STEP
        elif se.state == WAIT_DONE:
            if nx < se.size:
                se.next = se.sent = nx; se.state = SENDING; se.retry = 0; se.deadline = s.now + s.cfg.STEP
    def on_message(s, t, pl, now):
        s.now = now
        try: j = json.loads(pl)
        except Exception: return
        m, x = j.get("msg"), j.get("xferId", 0)
        if m == "PULL_VOICE": s.pull = True; return
        se = s.app if (s.app.state != IDLE and s.app.xid == x) else (s.peer if (s.peer.state != IDLE and s.peer.xid == x) else None)
        if not se: return
        if   m == "PV_ACK":  s.on_ack(se, x, j.get("next", 0))
        elif m == "PV_DONE": se.rx = s.now; s.stop(se, "DONE", True)
        elif m == "PV_FAIL": s.stop(se, "对端 FAIL", False)
        elif m == "PV_BUSY": s.stop(se, "对端忙/勿扰", False)
    # ── uvStop ──
    def stop(s, se, why, ok):
        s.res.append(dict(kind=se.kind, slot=se.slot, ok=ok, why=why, ms=s.now-se.t0,
                          acked=se.next, size=se.size, stalls=se.stalls, cwnd=se.cwnd))
        sl = se.slot
        if s.cfg.ledger:
            mt = s.meta.get(sl)
            if mt is not None:
                k, tk, nk = ("app","atry","anext") if se.kind == "APP" else ("peer","ptry","pnext")
                if ok: mt[k] = DONE; mt[tk] = 0
                else:
                    mt[k] = PEND; mt[tk] += 1
                    if mt[tk] >= len(LINK_BK): mt[k] = GIVEUP
                    else: mt[nk] = s.now + LINK_BK[mt[tk]-1]
                s.settle(sl)
        else:
            if se.kind == "APP":
                if ok and sl >= 0: s.slots.pop(sl, None); s.uid.pop(sl, None)
                elif not ok: s.pull = False
        se.state = IDLE; se.slot = -1
    def settle(s, sl):
        mt = s.meta.get(sl)
        if not mt: return
        if mt["app"] in (DONE,GIVEUP,NA) and mt["peer"] in (DONE,GIVEUP,NA):
            s.slots.pop(sl, None); s.meta.pop(sl, None)
    def _uid(s, sl):
        if sl not in s.uid: s.uid[sl] = s.rng.randrange(1,1<<31)|1
        return s.uid[sl]
    def start(s, se, sl, topic):
        d = s.slots.get(sl)
        if not d: return False
        se.slot, se.uid, se.size = sl, s._uid(sl), len(d)
        se.xid = s.rng.randrange(1,1<<31)|1
        se.next = se.sent = se.retry = se.stalls = 0
        se.t0 = se.rx = s.now; se.deadline = s.now + s.cfg.STEP
        se.cwnd = s.cfg.CW_I; se.ackhigh = 0; se.state = WAIT_READY; se.topic = topic
        s.begin(se); return True
    # ── uvService ──
    def service(s, now):
        s.now = now
        if s.cfg.ledger:
            # 通路 A
            if s.app.state == IDLE and (not s.cfg.presence or s.app_online):
                for _ in range(10):
                    sl = s.pick(True, now)
                    if sl < 0: break
                    if s.start(s.app, sl, s.at): s.meta[sl]["app"] = SEND; break
                    s.meta[sl]["app"] = s.meta[sl]["peer"] = GIVEUP; s.settle(sl)
            # 通路 B
            if s.peer.state == IDLE and (not s.cfg.presence or s.peer_online):
                for _ in range(10):
                    sl = s.pick(False, now)
                    if sl < 0: break
                    if s.start(s.peer, sl, s.pt): s.meta[sl]["peer"] = SEND; break
                    s.meta[sl]["app"] = s.meta[sl]["peer"] = GIVEUP; s.settle(sl)
        else:
            if s.pull and s.app.state == IDLE:
                sl = min(s.slots.keys()) if s.slots else -1
                if sl < 0: s.pull = False
                elif not s.start(s.app, sl, s.at):
                    if s.cfg.badslot_fix:
                        s.slots.pop(sl, None); s.uid.pop(sl, None)   # P0 坏槽自愈
                    else:
                        s.pull = False
            if s.peer_pend is not None and s.peer.state == IDLE and s.peer_online:
                sl = s.peer_pend; s.peer_pend = None; s.start(s.peer, sl, s.pt)
        s.pump(s.app); s.pump(s.peer)
    def pick(s, is_app, now):
        best = -1
        for sl in sorted(s.meta):
            mt = s.meta[sl]
            if mt["app" if is_app else "peer"] != PEND: continue
            if now < mt["anext" if is_app else "pnext"]: continue
            best = sl; break
        return best
    def active(s): return s.app.state != IDLE or s.peer.state != IDLE

class App:
    def __init__(s, b, dn, ack, fin=100):
        s.b, s.dn, s.ack_t, s.fin = b, dn, ack, fin
        s.rx, s.done, s.saved, s.pf, s.online, s.now = {}, {}, [], [], True, 0
    def _ack(s, m, x, n=None):
        if not s.online: return
        d = {"msg": m, "xferId": x}
        if n is not None: d["next"] = n
        s.b.publish(s.ack_t, json.dumps(d), s.dn, s.now)
    def on_message(s, t, pl, now):
        s.now = now
        if not s.online: return
        sn = t.split("/")[1]; b = pl if isinstance(pl, bytes) else pl.encode()
        if b[:1] == b"{":
            j = json.loads(b); m = j.get("msg")
            if m == "PV_BEGIN":
                x, sz, u = j.get("xferId",0), j.get("size",0), j.get("uid",0)
                if x == 0 or sz <= 0: return
                s.done = {k:v for k,v in s.done.items() if now-v < APP_DEDUP_MS}
                key = f"{sn}:u:{u}" if u else f"{sn}:x:{x}"
                if key in s.done: s._ack("PV_DONE", x); return
                e = s.rx.get(sn)
                if e and e["x"] == x: s._ack("PV_ACK", x, e["n"]); return
                s.rx[sn] = {"x":x,"sz":sz,"u":u,"buf":bytearray(sz),"n":0}
                s._ack("PV_ACK", x, 0)
            elif m == "PV_END":
                x = j.get("xferId",0); st = s.rx.get(sn)
                if st and st["x"] == x:
                    if st["n"] >= st["sz"]: s.rx.pop(sn); s.pf.append((now+s.fin, sn, st))
                    else: s._ack("PV_ACK", x, st["n"])
                    return
                if f"{sn}:x:{x}" in s.done: s._ack("PV_DONE", x)
        elif b[0] == UV_MAGIC and len(b) >= 9:
            x = int.from_bytes(b[1:5],"big"); o = int.from_bytes(b[5:9],"big")
            st = s.rx.get(sn)
            if st is None or st["x"] != x: return
            dl = len(b)-9
            if o == st["n"] and o+dl <= st["sz"]: st["buf"][o:o+dl] = b[9:]; st["n"] = o+dl
            s._ack("PV_ACK", x, st["n"])
    def tick(s, now):
        s.now = now
        r = [x for x in s.pf if x[0] <= now]
        if r:
            s.pf = [x for x in s.pf if x[0] > now]
            for _, sn, st in r:
                s.done[f"{sn}:u:{st['u']}"] = now; s.done[f"{sn}:x:{st['x']}"] = now
                s._ack("PV_DONE", st["x"]); s.saved.append(hashlib.md5(bytes(st["buf"])).hexdigest())

class Peer:
    def __init__(s, b, dn, ack):
        s.b, s.dn, s.ack_t = b, dn, ack
        s.x = s.sz = s.n = 0; s.buf = None; s.last = 0
        s.did = 0; s.dms = -10**9; s.played = []; s.busy = False; s.now = 0
    def _tx(s, m, x, n=None):
        d = {"msg": m, "xferId": x}
        if n is not None: d["next"] = n
        s.b.publish(s.ack_t, json.dumps(d), s.dn, s.now)
    def on_message(s, t, pl, now):
        s.now = now; b = pl if isinstance(pl, bytes) else pl.encode()
        if not b: return
        if b[:1] == b"{":
            j = json.loads(b); m = j.get("msg"); x = j.get("xferId",0)
            if m == "PV_BEGIN":
                if j.get("sn") != "EGG0001" or x == 0: return
                if x == s.did and now-s.dms < PV_DEDUP_MS: s._tx("PV_DONE", x); return
                if s.x != 0 and x == s.x:            # ★竞争1修复：重复 BEGIN 幂等，不重置
                    s.last = now; s._tx("PV_ACK", x, s.n); return
                if s.x != 0 and x != s.x and now - s.last < 3000:   # ★竞争6修复：抢占保护
                    s._tx("PV_BUSY", x); return
                if s.busy: s._tx("PV_BUSY", x); return
                s.x, s.sz, s.n = x, j.get("size",0), 0; s.buf = bytearray(s.sz); s.last = now
                s._tx("PV_ACK", x, 0)
            elif m == "PV_END":
                if x and x == s.did and now-s.dms < PV_DEDUP_MS: s._tx("PV_DONE", x); return
                if s.x == 0 or x != s.x: return
                s.last = now
                if s.sz > 0 and s.n == s.sz:
                    s.played.append(hashlib.md5(bytes(s.buf)).hexdigest())
                    s.did, s.dms = x, now; s.x = s.n = s.sz = 0; s._tx("PV_DONE", x)
                else:
                    s.x = s.n = s.sz = 0; s._tx("PV_FAIL", x)
        elif b[0] == UV_MAGIC and len(b) >= 9:
            x = int.from_bytes(b[1:5],"big"); o = int.from_bytes(b[5:9],"big")
            if s.x == 0 or x != s.x: return
            s.last = now; dl = len(b)-9
            if o == s.n: s.buf[o:o+dl] = b[9:]; s.n = o+dl
            s._tx("PV_ACK", x, s.n)
    def tick(s, now):
        if s.x != 0 and now-s.last > 15000: s.x = s.n = s.sz = 0

def run(level, *, size=240000, up=None, dn=None, seed=1, silent_at=None, peer_busy=False,
        want_app=True, want_peer=False, bad=False, app_off=False, peer_off=False,
        app_on_at=None, peer_on_at=None, max_ms=400000, step=None, live=None):
    rng = random.Random(seed); cfg = Cfg(level, step, live)
    up = up or Link(); dn = dn or Link(up.delay, up.jitter, up.loss)
    b = Broker(rng); dev = Device(b, cfg, rng); dev.up = up
    app = App(b, dn, "dev/TID/cmd"); peer = Peer(b, dn, "dev/TID/cmd")
    peer.busy = peer_busy; app.online = not app_off
    dev.app_online = not app_off; dev.peer_online = not peer_off
    b.sub("term/EGG0001/voice", app); b.sub("dev/PEERTID/voicePlay", peer); b.sub("dev/TID/cmd", dev)
    pay = bytes(rng.randrange(256) for _ in range(size)); md5 = hashlib.md5(pay).hexdigest()
    dev.slots[0] = b"" if bad else pay
    if bad: dev.slots[1] = pay
    for sl in dev.slots:
        dev.meta[sl] = {"app": PEND if want_app else NA, "peer": PEND if want_peer else NA,
                        "atry":0,"ptry":0,"anext":0,"pnext":0}
    dev.now = 0
    if not cfg.ledger:
        if want_app: dev.pull = True
        if want_peer: dev.peer_pend = 0
    t = 0
    while t < max_ms:
        b.pump(t)
        if app_on_at is not None and t == app_on_at:
            app.online = True; dev.app_online = True
            for m in dev.meta.values(): m["anext"] = 0
        if peer_on_at is not None and t == peer_on_at:
            dev.peer_online = True
            for m in dev.meta.values(): m["pnext"] = 0
        if silent_at is not None and app.online:
            st = app.rx.get("EGG0001")
            if st and st["n"] >= silent_at: app.online = False; dev.app_online = False
        if t % 5 == 0: dev.service(t)
        app.tick(t); peer.tick(t)
        idle = (not dev.active() and not app.pf and not b.q and
                (not dev.pull) and dev.peer_pend is None and
                (not cfg.ledger or (dev.pick(True,t) < 0 and dev.pick(False,t) < 0)))
        if idle and app_on_at is None and peer_on_at is None:
            t += 1; b.pump(t); app.tick(t); break
        t += 1
    return dict(level=level, t=t, app_ok=md5 in app.saved, peer_ok=md5 in peer.played,
                app_n=len(app.saved), peer_n=len(peer.played), res=dev.res,
                amp=dev.wire/size, left=sorted(dev.slots), pull=dev.pull,
                uf=dev.underflow, idlep=dev.idle_pumps, drop=b.drop, pub=b.pub)

def stat(name, levels, seeds=40, **kw):
    print(f"\n{'='*96}\n{name}\n{'='*96}")
    print(f"{'档位':<8}{'成功率App':>10}{'成功率Peer':>11}{'中位耗时':>10}{'P90耗时':>9}"
          f"{'放大':>7}{'下溢':>7}{'空转':>7}{'重复投递':>9}")
    for lv in levels:
        rs = [run(lv, seed=s, **kw) for s in range(1, seeds+1)]
        ts = sorted(r["t"] for r in rs)
        a = sum(r["app_ok"] for r in rs)/len(rs)
        p = sum(r["peer_ok"] for r in rs)/len(rs)
        dup = sum(max(0, r["app_n"]-1) + max(0, r["peer_n"]-1) for r in rs)
        print(f"{lv:<8}{a*100:>9.0f}%{p*100:>10.0f}%{statistics.median(ts):>9.0f}ms"
              f"{ts[int(len(ts)*0.9)-1]:>8}ms{statistics.median([r['amp'] for r in rs]):>6.2f}x"
              f"{sum(r['uf'] for r in rs):>7}{sum(r['idlep'] for r in rs):>7}{dup:>9}")

if __name__ == "__main__":
    L = ["CURR", "P123", "FULL"]
    stat("T1 理想链路 RTT120  |  App 单路", L, up=Link(60))
    stat("T2 云端 RTT600      |  App 单路", L, up=Link(300))
    stat("T3 ACK滞留乱序 30%@+1.5s (触发 uint32 下溢的对抗场景)", L,
         up=Link(60), dn=Link(60, xd_p=0.30, xd=1500))
    stat("T4 双向丢包 25%", L, up=Link(60, loss=0.25), dn=Link(60, loss=0.25))
    stat("T5 双向丢包 40% + RTT400", L, up=Link(200, loss=0.40), dn=Link(200, loss=0.40))
    stat("T6 坏槽 voice_0=0B（后面还有一条好的）", L, up=Link(60), bad=True, seeds=10)
    stat("T7 ★ App离线 + 伙伴在线（用户要求的核心场景）", L,
         up=Link(60), want_app=True, want_peer=True, app_off=True, seeds=10)
    stat("T8 ★ K1耦合：伙伴离线，App 先送达（伙伴 20s 后上线）", L,
         up=Link(60), want_app=True, want_peer=True, peer_off=True,
         peer_on_at=20000, max_ms=60000, seeds=10)
    stat("T9 双通路并发（都在线）", L, up=Link(60), want_app=True, want_peer=True, seeds=20)

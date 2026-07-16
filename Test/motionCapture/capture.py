#!/usr/bin/env python3
"""
capture.py — 配合 motionCapture.ino，读串口把每个 "# EVENT .. # END" 事件存成 CSV。

用法:
    python capture.py --port COM5 --label tap        # Windows
    python capture.py --port /dev/ttyACM0 --label shake   # Linux/Mac

依赖:  pip install pyserial

按动作分 label 采集，便于后续对比拍/摇/碰:
    events/tap/tap_001.csv, events/tap/tap_002.csv ...
    events/shake/shake_001.csv ...
    events/bump/bump_001.csv ...
文件续号（重跑不覆盖）。Ctrl-C 退出。采完把整个 events/ 目录发给我做分析。
"""
import argparse, os, sys, math

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial:  pip install pyserial")


def summarize(rows):
    """rows: [[seq,t_ms,ax,ay,az,gx,gy,gz], ...] —— 打印一行动作摘要，快速肉眼判断数据是否合理"""
    if not rows:
        return "空事件"
    peak_dev, peak = 0.0, rows[0]
    for r in rows:
        mag = math.sqrt(r[2] ** 2 + r[3] ** 2 + r[4] ** 2)
        dev = abs(mag - 1000)
        if dev > peak_dev:
            peak_dev, peak = dev, r
    axis = max(("X", abs(peak[2])), ("Y", abs(peak[3])), ("Z", abs(peak[4])),
               key=lambda t: t[1])[0]
    gpx = max(abs(r[5]) for r in rows)
    gpy = max(abs(r[6]) for r in rows)
    gpz = max(abs(r[7]) for r in rows)
    return (f"峰值偏移 {peak_dev:4.0f}mg @seq{peak[0]:<4d} 主轴{axis} | "
            f"陀螺峰 gx{gpx} gy{gpy} gz{gpz} dps")


def next_index(outdir, label):
    n = 0
    if os.path.isdir(outdir):
        for f in os.listdir(outdir):
            if f.startswith(label + "_") and f.endswith(".csv"):
                try:
                    n = max(n, int(f[len(label) + 1:-4]))
                except ValueError:
                    pass
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="串口，如 COM5 或 /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--label", default="event", help="动作标签: tap / shake / bump ...")
    ap.add_argument("--out", default="events", help="输出根目录")
    a = ap.parse_args()

    outdir = os.path.join(a.out, a.label)
    os.makedirs(outdir, exist_ok=True)
    n = next_index(outdir, a.label)

    ser = serial.Serial(a.port, a.baud, timeout=1)
    print(f"[capture] {a.port} @ {a.baud} | label={a.label} | 存到 {outdir}/ | 已有 {n} 条 | Ctrl-C 退出\n")

    in_ev, header, rows, raw = False, None, [], []
    try:
        while True:
            line = ser.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            if line.startswith("# EVENT"):
                in_ev, header, rows, raw = True, None, [], []
                continue
            if line.startswith("# END"):
                if in_ev and rows:
                    n += 1
                    path = os.path.join(outdir, f"{a.label}_{n:03d}.csv")
                    with open(path, "w", newline="") as fp:
                        fp.write((header or "seq,t_ms,ax,ay,az,gx,gy,gz") + "\n")
                        fp.write("\n".join(raw) + "\n")
                    print(f"  ✓ {os.path.basename(path)}  {len(rows):3d}样本 | {summarize(rows)}")
                in_ev = False
                continue
            if line.startswith("#"):
                print("  " + line)                 # 设备日志: READY / REST / ERROR
                continue
            if in_ev:
                if header is None and line.startswith("seq"):
                    header = line
                    continue
                raw.append(line)
                try:
                    rows.append([int(x) for x in line.split(",")])
                except ValueError:
                    pass
    except KeyboardInterrupt:
        print("\n[capture] 退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

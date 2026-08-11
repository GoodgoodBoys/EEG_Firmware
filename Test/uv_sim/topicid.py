#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
算出 dev/ 主题用的令牌（与固件 web.cpp topicId() / App topic_id.dart 完全一致）。

    令牌 = HMAC-SHA256(TOPIC_SALT, SN) 的前 8 字节 hex

用法:  python topicid.py EGG0001
"""
import sys, hmac, hashlib

# ⚠ 必须与 web.cpp 的 TOPIC_SALT 一致；改盐后这里也要改
TOPIC_SALT = b"egg-topic-2026-CHANGE-THIS-SALT"

def topic_id(sn: str) -> str:
    return hmac.new(TOPIC_SALT, sn.encode(), hashlib.sha256).hexdigest()[:16]

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    sn = sys.argv[1]
    tid = topic_id(sn)
    print(f"SN       = {sn}")
    print(f"topicId  = {tid}")
    print()
    print("── 设备【订阅】的主题（你往这些发）──")
    print(f"  命令      dev/{tid}/cmd")
    print(f"  视频分片  dev/{tid}/video")
    print(f"  音频分片  dev/{tid}/audio")
    print(f"  伙伴语音  dev/{tid}/voicePlay")
    print()
    print("── 设备【发布】的主题（你订阅这些看）──")
    print(f"  健康黑匣子  term/{sn}/health")
    print(f"  传输回执    term/{sn}/response")
    print(f"  语音上行    term/{sn}/voice")
    print(f"  在线状态    devInfo/{sn}")

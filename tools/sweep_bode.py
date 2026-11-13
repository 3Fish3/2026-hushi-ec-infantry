#!/usr/bin/env python3
"""云台稳向频率扫描: 通过参数协议设正弦注入, 采集后算幅相.

流程(每组):
  1. 关观测 -> 写参数(幅度/频率) -> 开观测   (分时复用 USART1)
  2. 等暂态结束
  3. 采集 CAL 行, 对 heading 与 target 做正弦拟合求幅比与相位差

用法:
  python3 tools/sweep_bode.py --port /dev/ttyUSB0 \
      --freqs 0.3,0.6,1,2,3,5 --amp 0.09 --seconds 8
"""
import argparse
import cmath
import math
import statistics as st
import sys
import time

import serial

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from tune_gimbal import capture, parse_line  # noqa: E402

VEHICLE_SLOT = 10
P_ANGLE_KP, P_ANGLE_KI, P_ANGLE_KD = 10, 11, 12
P_SPEED_KP, P_SPEED_KI, P_SPEED_KD = 13, 14, 15
P_SWEEP_AMP, P_SWEEP_FREQ = 16, 17
P_TELEMETRY = 18


def param_id(slot: int, local: int) -> int:
    return (slot << 8) | local


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else (crc << 1)) & 0xFFFF
    return crc


def encode_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    raw = bytes([0x01, 0x02]) + message_type.to_bytes(2, "little") + \
        sequence.to_bytes(2, "little") + payload
    crc = crc16(raw)
    return b"\x7e" + raw + bytes([crc & 0xFF, (crc >> 8) & 0xFF]) + b"\x7e"


def fit_sine(t, y, freq):
    """对 y(t) 在给定频率上做最小二乘正弦拟合, 返回 (幅度, 相位, 偏置)."""
    w = 2.0 * math.pi * freq
    n = len(t)
    sc = sum(math.cos(w * x) for x in t)
    ss = sum(math.sin(w * x) for x in t)
    scc = sum(math.cos(w * x) ** 2 for x in t)
    sss = sum(math.sin(w * x) ** 2 for x in t)
    scs = sum(math.cos(w * x) * math.sin(w * x) for x in t)
    yc = sum(v * math.cos(w * x) for v, x in zip(y, t))
    ys = sum(v * math.sin(w * x) for v, x in zip(y, t))
    sy = sum(y)
    # 解 [n, sc, ss; sc, scc, scs; ss, scs, sss] [a; b; c] = [sy; yc; ys]
    import numpy as np
    A = np.array([[n, sc, ss], [sc, scc, scs], [ss, scs, sss]], dtype=float)
    B = np.array([sy, yc, ys], dtype=float)
    a, b, c = np.linalg.lstsq(A, B, rcond=None)[0]
    amp = math.hypot(b, c)
    phase = math.atan2(c, b)
    return amp, phase, a / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--freqs", default="0.3,0.6,1,2,3,5")
    ap.add_argument("--amp", type=float, default=0.09)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--akp", type=float, default=None)
    ap.add_argument("--skp", type=float, default=None)
    ap.add_argument("--out", default="bode.csv")
    args = ap.parse_args()

    freqs = [float(x) for x in args.freqs.split(",") if x.strip()]
    ser = serial.Serial(args.port, 921600, timeout=0.5)
    seq = 0

    def write(local, value):
        nonlocal seq
        seq += 1
        ser.write(encode_frame(0x50, seq, __import__("struct").pack("<HHf", param_id(VEHICLE_SLOT, local), 0, value)))
        ser.flush()

    rows = []
    try:
        # 关观测, 写入增益与注入参数, 再开观测.
        write(P_TELEMETRY, 0.0)
        if args.akp is not None:
            write(P_ANGLE_KP, args.akp)
        if args.skp is not None:
            write(P_SPEED_KP, args.skp)
        time.sleep(0.3)

        for f in freqs:
            write(P_SWEEP_AMP, 0.0)          # 先停, 改频率无相位跳变
            write(P_SWEEP_FREQ, f)
            write(P_SWEEP_AMP, args.amp)
            write(P_TELEMETRY, 1.0)
            time.sleep(args.settle)
            samples = capture(args.port, args.seconds)
            write(P_SWEEP_AMP, 0.0)          # 该频点结束, 停止注入
            write(P_TELEMETRY, 0.0)
            time.sleep(0.2)

            if len(samples) < 20:
                print(f"f={f:5.2f}Hz 样本不足 ({len(samples)})")
                continue
            t0 = samples[0]["t"]
            ts = [(s["t"] - t0) * 0.001 for s in samples]
            head = [s["heading"] for s in samples]
            targ = [s["target"] for s in samples]
            ah, ph, _ = fit_sine(ts, head, f)
            at, pt, _ = fit_sine(ts, targ, f)
            if at <= 1e-6:
                print(f"f={f:5.2f}Hz 目标无注入 (amp={at:.4f})")
                continue
            gain = ah / at
            lag_deg = math.degrees((ph - pt + math.pi) % (2 * math.pi) - math.pi)
            rows.append((f, gain, lag_deg, ah, at))
            print(f"f={f:5.2f}Hz  gain={gain:6.3f} ({20*math.log10(max(gain,1e-6)):+6.2f}dB)  "
                  f"phase={lag_deg:+7.1f}deg  amp_in={at:.4f} amp_out={ah:.4f}")
    finally:
        write(P_SWEEP_AMP, 0.0)
        write(P_TELEMETRY, 1.0)
        ser.close()

    with open(args.out, "w", encoding="utf-8") as fh:
        fh.write("freq_hz,gain,phase_deg,amp_in,amp_out\n")
        for r in rows:
            fh.write(f"{r[0]:.3f},{r[1]:.4f},{r[2]:.2f},{r[3]:.5f},{r[4]:.5f}\n")
    print(f"\n结果写入 {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

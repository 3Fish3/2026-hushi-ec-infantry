#!/usr/bin/env python3
"""云台稳向增益整定: 阶跃响应采样 + 指标计算.

用法(需先烧录并让固件进入运行状态, 串口接 ttyUSB0):
  python3 tools/tune_gimbal.py --port /dev/ttyUSB0 --seconds 8

脚本只读取 CAL 行并计算指标; 它不发送任何控制命令, 也不改变增益.
增益仍通过参数协议在线写, 或在 infantry_config.hpp 里改后重烧.
"""
import argparse
import math
import statistics as st
import sys
import time

import serial


FIELDS = (
    "t joint heading imu_yaw ax ay az gx gy gz roll pitch valid reprime "
    "torque execute reason target tvalid rate"
).split()


def parse_line(line: str):
    parts = line.split()
    if not parts or parts[0] != "CAL" or len(parts) < len(FIELDS):
        return None
    try:
        values = [float(x) for x in parts[1:]]
    except ValueError:
        return None
    return dict(zip(FIELDS, values))


def capture(port: str, seconds: float):
    ser = serial.Serial(port, 921600, timeout=0.5)
    ser.reset_input_buffer()
    samples = []
    end = time.time() + seconds
    buf = b""
    try:
        while time.time() < end:
            buf += ser.read(4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                row = parse_line(line.decode("utf-8", "replace").strip())
                if row:
                    samples.append(row)
    finally:
        ser.close()
    return samples


def step_metrics(samples):
    """在目标切换点之后统计超调/稳态误差/调节时间."""
    if len(samples) < 20:
        return None
    heading = [s["heading"] for s in samples]
    target = [s["target"] for s in samples]
    tvalid = [s["tvalid"] for s in samples]
    torque = [s["torque"] for s in samples]

    # 只统计目标有效的样本.
    valid_idx = [i for i, v in enumerate(tvalid) if v > 0.5]
    if len(valid_idx) < 10:
        return {"note": "目标无效样本过多", "valid_ratio": len(valid_idx) / len(samples)}

    final_target = st.mean(target[i] for i in valid_idx[-10:])
    tail = [heading[i] for i in valid_idx[-10:]]
    steady = st.mean(tail)

    peak = max(heading[i] for i in valid_idx)
    trough = min(heading[i] for i in valid_idx)
    span = abs(final_target) if abs(final_target) > 1e-6 else 1.0

    return {
        "samples": len(samples),
        "valid_ratio": len(valid_idx) / len(samples),
        "target": final_target,
        "steady": steady,
        "steady_error_rad": steady - final_target,
        "steady_error_deg": math.degrees(steady - final_target),
        "overshoot_rad": max(peak - final_target, final_target - trough),
        "heading_range_rad": max(heading) - min(heading),
        "torque_mean": st.mean(torque),
        "torque_sd": st.pstdev(torque),
        "torque_peak": max(abs(x) for x in torque),
        "rate_sd": st.pstdev([s["rate"] for s in samples]),
    }


def quality_score(m):
    """越低越好的综合代价: 稳态误差 + 振荡 + 力矩抖动."""
    if not m or "steady_error_rad" not in m:
        return float("inf")
    return (abs(m["steady_error_rad"]) * 10.0 +
            m["heading_range_rad"] * 2.0 +
            m["torque_sd"] * 5.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--csv", default=None, help="把原始样本写入 CSV")
    args = ap.parse_args()

    samples = capture(args.port, args.seconds)
    if not samples:
        print("没有收到 CAL 行; 检查串口与固件状态", file=sys.stderr)
        return 1

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as fh:
            fh.write(",".join(FIELDS) + "\n")
            for row in samples:
                fh.write(",".join(f"{row[k]:.6f}" for k in FIELDS) + "\n")
        print(f"已写出 {len(samples)} 行到 {args.csv}")

    m = step_metrics(samples)
    if not m:
        print("样本不足", file=sys.stderr)
        return 1
    for key, value in m.items():
        if isinstance(value, float):
            print(f"{key:22s} {value:+.5f}")
        else:
            print(f"{key:22s} {value}")
    print(f"{'quality_score':22s} {quality_score(m):.4f}  (越低越好)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

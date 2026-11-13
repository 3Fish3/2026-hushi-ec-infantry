#!/usr/bin/env python3
"""云台稳向增益扫参: 逐组写入增益并评估响应.

前提:
  1. 固件已烧录, 串口可用, 云台已进入执行状态 (execute=1);
  2. 参数协议端口 USART1 与观测共用: 本脚本通过 UART 帧协议写参数;
  3. 物理急停就位 —— 云台会按每组增益反复出力.

用法:
  python3 tools/sweep_gimbal_pid.py --port /dev/ttyUSB0 \
      --angle-kp 2,4,6 --speed-kp 0.35,0.8,1.5 --seconds 6

脚本按网格遍历, 每组: 写入 -> 等待稳定 -> 采集 -> 计算指标.
结果按 quality_score 升序打印, 并写出 CSV.
"""
import argparse
import csv
import itertools
import math
import sys
import time

import serial

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from tune_gimbal import capture, quality_score, step_metrics  # noqa: E402

# 参数 ID: vehicle 槽 (domain_count=10) 的局部序号.
VEHICLE_SLOT = 10
LOCAL = {
    "angle_kp": 10,
    "angle_ki": 11,
    "angle_kd": 12,
    "speed_kp": 13,
    "speed_ki": 14,
    "speed_kd": 15,
}


def param_id(slot: int, local: int) -> int:
    return (slot << 8) | local


def build_frame(sequence: int, payload: bytes) -> bytes:
    """按固件 UartFrame 编码: 0x7e 分帧 + 长度 + CRC8."""
    body = bytes([0x01, 0x02]) + sequence.to_bytes(2, "little") + payload
    crc = 0
    for b in body:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xff if crc & 0x80 else (crc << 1) & 0xff
    return b"\x7e" + body + bytes([crc]) + b"\x7e"


def write_param(ser: serial.Serial, param: int, value: float, sequence: int):
    payload = bytes([0x50, 0x01]) + param.to_bytes(2, "little") + \
        __import__("struct").pack("<f", value)
    ser.write(build_frame(sequence, payload))
    ser.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--angle-kp", default="2")
    ap.add_argument("--speed-kp", default="0.35")
    ap.add_argument("--speed-kd", default="0")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--out", default="sweep_result.csv")
    args = ap.parse_args()

    def grid(text):
        return [float(x) for x in text.split(",") if x.strip()]

    combos = list(itertools.product(grid(args.angle_kp), grid(args.speed_kp),
                                    grid(args.speed_kd)))
    print(f"共 {len(combos)} 组, 每组 {args.seconds:.0f}s + {args.settle:.0f}s 稳定")
    print("物理急停必须就位; 云台会按每组增益出力.\n")

    ser = serial.Serial(args.port, 921600, timeout=0.5)
    seq = 0
    results = []
    try:
        for akp, skp, skd in combos:
            seq += 1
            write_param(ser, param_id(VEHICLE_SLOT, LOCAL["angle_kp"]), akp, seq)
            seq += 1
            write_param(ser, param_id(VEHICLE_SLOT, LOCAL["speed_kp"]), skp, seq)
            seq += 1
            write_param(ser, param_id(VEHICLE_SLOT, LOCAL["speed_kd"]), skd, seq)
            time.sleep(args.settle)

            samples = capture(args.port, args.seconds)
            m = step_metrics(samples) or {}
            score = quality_score(m)
            row = {
                "angle_kp": akp, "speed_kp": skp, "speed_kd": skd,
                "score": score,
                "steady_err_deg": m.get("steady_error_deg", float("nan")),
                "range_rad": m.get("heading_range_rad", float("nan")),
                "torque_sd": m.get("torque_sd", float("nan")),
                "valid_ratio": m.get("valid_ratio", 0.0),
            }
            results.append(row)
            print(f"akp={akp:<5} skp={skp:<6} skd={skd:<5} "
                  f"score={score:9.3f} err={row['steady_err_deg']:+7.2f}deg "
                  f"range={row['range_rad']:.3f} tq_sd={row['torque_sd']:.4f}")
    finally:
        ser.close()

    results.sort(key=lambda r: r["score"])
    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(results[0].keys()))
        writer.writeheader()
        writer.writerows(results)

    print(f"\n最优: {results[0]}")
    print(f"结果已写入 {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""用 J4310 力矩 Sweep 记录离线辨识航向轴，并计算双环候选增益。"""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares


ACTIVE = 1 << 7
TERMINAL = 1 << 8
ANGLE = 1 << 9
VALID_ACTIVE = 0xFF
UINT32_MASK = 0xFFFFFFFF
MAX_GAP_US = 20_000
MIN_FREQUENCIES = 4
MIN_TONE_SNR = 3.0
MAX_RELATIVE_ERROR = 0.30
MIN_PHASE_MARGIN_DEG = 45.0
TORQUE_LIMIT_NM = 2.0
FIRMWARE_KP_LIMIT = {"speed": 5.0, "angle": 20.0}
SIGNALS = (
    "requested_excitation_nm", "applied_excitation_nm",
    "torque_command_nm", "heading_rate_rad_s",
)
COLUMNS = {
    "sequence", "timestamp_us", "reference_epoch", "flags", "reason",
    "sweep_run_id", "sweep_hz", *SIGNALS,
}


def float32(value):
    return float(np.float32(value))


def read_trial(path):
    sidecar = path.with_suffix(path.suffix + ".report.json")
    meta = json.loads(sidecar.read_text(encoding="utf-8"))
    if (meta["schema"] != 2 or meta.get("mode", "torque") != "torque"
            or meta["complete"] is not True or not 0 < meta["amplitude_nm"] <= 0.03):
        raise ValueError("需要完成的 schema 2 力矩 Sweep")
    cycles = meta["cycles"]
    if not isinstance(cycles, int) or not 4 <= cycles <= 12:
        raise ValueError("每频点至少需要 4 个计划周期，去掉包络后才有两个稳态周期")
    frequencies = [float32(value) for value in meta["frequencies"]]
    if (not 1 <= len(frequencies) <= 6 or not all(float32(0.2) <= f <= 5 for f in frequencies)
            or any(a >= b for a, b in itertools.pairwise(frequencies))):
        raise ValueError("报告频点超出 Sweep 范围，或在 float32 量化后未严格递增")
    run_id = meta["sweep_run_id"]
    if not isinstance(run_id, int) or not 0 < run_id <= UINT32_MASK:
        raise ValueError("报告缺少有效的 Sweep run_id")

    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not COLUMNS <= set(reader.fieldnames or ()):
            raise ValueError("CSV 缺少 schema 2 力矩采集字段")
        rows = list(reader)

    if len(rows) != meta["records"] + 1 or len(rows) < 2:
        raise ValueError("CSV 记录数量与试验报告不符")
    flags = [int(row["flags"]) for row in rows]
    if (not flags[-1] & TERMINAL or flags[-1] & (ACTIVE | ANGLE)
            or float(rows[-1]["sweep_hz"]) != 0.0):
        raise ValueError("CSV 没有自然完成的力矩终态，试验可能提前中止")
    if any((flag & VALID_ACTIVE) != VALID_ACTIVE or flag & (TERMINAL | ANGLE)
           or int(row["reason"]) != 0 for flag, row in zip(flags[:-1], rows[:-1])):
        raise ValueError("存在未执行、未提交力矩或传感器无效的运行样本")
    if (any(int(row["sweep_run_id"]) != run_id for row in rows)
            or len({int(row["reference_epoch"]) for row in rows}) != 1):
        raise ValueError("Sweep run_id 或航向参考代次发生变化")

    sequence = [int(row["sequence"]) for row in rows]
    if any((b - a) & UINT32_MASK != 1 for a, b in itertools.pairwise(sequence)):
        raise ValueError("CSV 样本序号不连续")
    stamps = [int(row["timestamp_us"]) for row in rows]
    gaps_us = [(b - a) & UINT32_MASK for a, b in itertools.pairwise(stamps)]
    if not all(0 < gap <= MAX_GAP_US for gap in gaps_us):
        raise ValueError("MCU 样本时间回退或中间断档")
    times = np.r_[0.0, np.cumsum(gaps_us)] * 1e-6

    groups = []
    start = 0
    for hz, group in itertools.groupby(rows[:-1], key=lambda row: float32(row["sweep_hz"])):
        samples = list(group)
        end = start + len(samples)
        values = np.array([[float(row[name]) for name in SIGNALS] for row in samples])
        if not np.isfinite(values).all():
            raise ValueError("CSV 包含非有限的力矩或角速度")
        if abs(times[end] - times[start] - cycles / hz) > MAX_GAP_US * 1e-6:
            raise ValueError(f"{hz:g} Hz 的实测时长与试验计划不符")
        groups.append((hz, times[start:end + 1] - times[start], values))
        start = end
    if [hz for hz, _, _ in groups] != frequencies:
        raise ValueError("CSV 的频点顺序或身份与试验报告不符")
    return groups


def measure(hz, times, values):
    period = 1 / hz
    sample_s = float(np.median(np.diff(times)))
    steady = (times[:-1] >= period) & (times[:-1] <= times[-1] - period)
    time = times[:-1][steady]
    if len(time) < 8 or time[-1] - time[0] + 2 * sample_s < 2 * period:
        raise ValueError("去掉首尾包络后不足两个稳态周期")
    signals = values[steady]
    if (np.max(np.abs(values[:, 0] - values[:, 1])) > 1e-5
            or np.max(np.abs(values[:, 2])) >= TORQUE_LIMIT_NM - 1e-5):
        raise ValueError("注入力矩被裁剪或总力矩达到固件限幅")
    omega_t = 2 * math.pi * hz * time
    basis = np.column_stack((np.ones_like(time), (time - time.mean()) / period,
                             np.cos(omega_t), np.sin(omega_t)))
    coeff, _, rank, _ = np.linalg.lstsq(basis, signals, rcond=None)
    if rank != basis.shape[1]:
        raise ValueError("稳态样本不足以分辨正弦响应")
    tone = coeff[2] - 1j * coeff[3]
    tone_rms = np.abs(tone) / math.sqrt(2)
    residual_rms = np.sqrt(np.mean((signals - basis @ coeff) ** 2, axis=0))
    snr = tone_rms / np.maximum(residual_rms, 1e-12)
    if np.any(snr[[0, 2, 3]] < MIN_TONE_SNR):
        raise ValueError(f"同频信号太弱（注入、总力矩、角速度 RMS 比 {snr[[0, 2, 3]].round(2)}）")

    # 用外部注入 d 提取各量的同频分量，再取 H(角速度,d)/H(总力矩,d)。
    # 总力矩含原闭环输出；直接用角速度除以注入力矩会得到闭环扰动响应。
    # 工程 torque_direction=+1，提交力矩仍是 CAN 编码前的电机侧命令。
    torque_from_injection = tone[2] / tone[0]
    rate_from_injection = tone[3] / tone[0]
    response = complex(rate_from_injection / torque_from_injection)
    return response, {
        "steady_samples": len(time),
        "command_torque_snr": float(snr[2]),
        "heading_rate_snr": float(snr[3]),
        "gain_rad_s_per_nm": abs(response),
        "phase_deg": math.degrees(np.angle(response)),
    }


def plant(params, hz):
    gain, tau_s, delay_s = params
    s = 2j * math.pi * np.asarray(hz)
    return gain * np.exp(-delay_s * s) / (1 + tau_s * s)


def fit_model(hz, response, delay_limit):
    def residual(params):
        error = (plant(params, hz) - response) / np.abs(response)
        return np.r_[error.real, error.imag]

    gain_guess = float(np.max(np.abs(response)))
    tau_guess = 1 / (2 * math.pi * np.median(hz))
    results = [
        least_squares(
            residual, [gain_guess, tau_guess, seed],
            bounds=([0, 0, 0], [np.inf, np.inf, delay_limit]),
            x_scale=[gain_guess, tau_guess, delay_limit],
        )
        for seed in (0.0, delay_limit / 3, 2 * delay_limit / 3)
    ]
    successful = [result for result in results if result.success]
    if not successful:
        raise ValueError("一阶模型优化未收敛")
    return min(successful, key=lambda result: result.cost).x


def tune(params, speed_hz, angle_hz):
    speed_kp = 1 / abs(plant(params, speed_hz))
    speed_margin = 180 - math.degrees(
        math.atan(2 * math.pi * speed_hz * params[1]) + 2 * math.pi * speed_hz * params[2]
    )
    inner_open = speed_kp * plant(params, angle_hz)
    angle_kp = 2 * math.pi * angle_hz / abs(inner_open / (1 + inner_open))
    if any(not 0 < gain <= FIRMWARE_KP_LIMIT[name]
           for name, gain in (("speed", speed_kp), ("angle", angle_kp))):
        raise ValueError("目标交越频率要求的 Kp 超出固件参数范围")
    if speed_margin < MIN_PHASE_MARGIN_DEG:
        raise ValueError(f"模型速度环相位裕度仅 {speed_margin:.1f}°")

    # 一阶幅值单调下降；当 |速度开环|≤1/2 且 ω>航向 Kp 时，
    # 航向开环幅值小于 1，此后无需再搜索交越。
    dc_loop_gain = speed_kp * params[0]
    half_gain_hz = math.sqrt(max(0.0, (2 * dc_loop_gain) ** 2 - 1))
    half_gain_hz /= 2 * math.pi * max(params[1], 1e-12)
    high_hz = max(half_gain_hz, angle_kp / (2 * math.pi), speed_hz) * 1.01
    grid = np.unique(np.r_[np.geomspace(angle_hz / 100, high_hz, 4096), angle_hz])
    inner = speed_kp * plant(params, grid)
    outer_mag = angle_kp * np.abs(inner / (1 + inner)) / (2 * math.pi * grid)
    crossing = np.flatnonzero(np.diff(outer_mag >= 1 - 1e-10))
    if (len(crossing) != 1 or not outer_mag[0] > 1 or not outer_mag[-1] < 1
            or not grid[crossing[0]] <= angle_hz <= grid[crossing[0] + 1]):
        raise ValueError("模型航向环未形成唯一目标交越")
    angle_phase = -math.atan(2 * math.pi * angle_hz * params[1])
    angle_phase -= 2 * math.pi * angle_hz * params[2] + np.angle(1 + inner_open)
    angle_margin = 90 + math.degrees(angle_phase)
    if angle_margin < MIN_PHASE_MARGIN_DEG:
        raise ValueError(f"模型航向环相位裕度仅 {angle_margin:.1f}°")
    return {
        "speed": dict(kp=speed_kp, ki=0.0, kd=0.0, crossover_hz=speed_hz,
                      model_phase_margin_deg=speed_margin, param_ids=["0x0A0D", "0x0A0E", "0x0A0F"]),
        "angle": dict(kp=angle_kp, ki=0.0, kd=0.0, crossover_hz=angle_hz,
                      model_phase_margin_deg=angle_margin, param_ids=["0x0A0A", "0x0A0B", "0x0A0C"]),
    }


def analyze(paths, speed_hz, angle_hz, report):
    if len({path.resolve() for path in paths}) != len(paths):
        raise ValueError("同一份 CSV 不能重复计入")
    measurements = {}
    for path in paths:
        try:
            groups = read_trial(path)
        except (OSError, ValueError, TypeError, KeyError) as error:
            raise ValueError(f"{path.name}：{error}") from error
        for hz, times, values in groups:
            item = {"csv": str(path), "hz": hz}
            report["measurements"].append(item)
            try:
                response, metrics = measure(hz, times, values)
            except ValueError as error:
                item["rejected"] = str(error)
                continue
            item.update(metrics)
            measurements.setdefault(hz, []).append(response)

    frequencies = []
    responses = []
    for hz, runs in sorted(measurements.items()):
        mean = np.mean(runs)
        spread = float(np.max(np.abs(np.asarray(runs) - mean)) / max(abs(mean), 1e-12))
        report["repeat_checks"].append({"hz": hz, "runs": len(runs), "relative_spread": spread})
        if spread <= MAX_RELATIVE_ERROR:
            frequencies.append(hz)
            responses.append(mean)
    if len(frequencies) < MIN_FREQUENCIES:
        raise ValueError(f"只有 {len(frequencies)} 个合格频点，至少需要 {MIN_FREQUENCIES} 个")
    if not frequencies[0] <= angle_hz < speed_hz <= frequencies[-1]:
        raise ValueError("内外环目标交越频率必须在合格实测频段内")

    hz = np.array(frequencies)
    response = np.array(responses)
    delay_limit = 0.5 / hz[-1]
    params = fit_model(hz, response, delay_limit)
    holdout = []
    for index in range(len(hz)):
        selected = np.arange(len(hz)) != index
        model = fit_model(hz[selected], response[selected], delay_limit)
        error = float(abs(plant(model, hz[index]) - response[index]) / abs(response[index]))
        holdout.append({"hz": frequencies[index], "relative_error": error})
    fit_errors = np.abs(plant(params, hz) - response) / np.abs(response)
    report["model"] = {
        "dc_gain_rad_s_per_nm": params[0], "time_constant_s": params[1], "delay_s": params[2],
        "fit_max_relative_error": float(np.max(fit_errors)), "leave_one_frequency_out": holdout,
    }
    if (np.max(fit_errors) > MAX_RELATIVE_ERROR
            or max(item["relative_error"] for item in holdout) > MAX_RELATIVE_ERROR):
        raise ValueError("模型拟合或留出频点误差超过 30%，本轮不出参数")
    report["candidate"] = tune(params, speed_hz, angle_hz)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="+", help="力矩 Sweep CSV，旁边保留对应报告")
    parser.add_argument("--speed-hz", required=True, type=float, help="速度内环目标交越频率")
    parser.add_argument("--angle-hz", required=True, type=float, help="航向外环目标交越频率")
    parser.add_argument("--output", type=Path, help="另存 JSON 报告")
    args = parser.parse_args()
    if (not all(math.isfinite(hz) for hz in (args.speed_hz, args.angle_hz))
            or not 0 < 2 * args.angle_hz <= args.speed_hz):
        parser.error("交越频率必须为正，且速度环至少为航向环的两倍")

    report = {"candidate": None, "measurements": [], "repeat_checks": []}
    try:
        analyze(args.csv, args.speed_hz, args.angle_hz, report)
    except (ValueError, np.linalg.LinAlgError) as error:
        report["reason"] = str(error)
    result = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        try:
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(result + "\n")
        except OSError as error:
            parser.exit(2, f"无法保存报告：{error}\n")
    print(result)
    return 0 if report["candidate"] is not None else 2


if __name__ == "__main__":
    sys.exit(main())

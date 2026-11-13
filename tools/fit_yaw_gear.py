#!/usr/bin/env python3
"""步兵 J4310 外啮合小角度传动比/回差离线拟合.

CSV 必填列: approach,turret_deg,motor_rad
  approach     forward(正向接近) 或 reverse(反向接近), 不是云台角度符号
  turret_deg   云台角, 度; forward 按递增记录, reverse 按递减记录
  motor_rad    电机角, rad; 必须同一参考系, 必要时先解卷绕

可选元数据列:
  source       mit_pos / p_m / xout, 同一份数据只能使用一种位置源
  sampled_ms   采样时刻, ms
  age_ms       反馈时延, ms
  valid        反馈有效性, 1/0 或 true/false; 无效行会被拒绝
  mark         人工标记, 原样保留到残差表
  note         备注, 原样保留到残差表
  data_kind    SYNTHETIC / REAL; 合成数据不会被当作实车标定

GUI 侧契约: 按上述固定 10 列导出; 额外原始寄存器位值、PMAX 与分段信息保存在
JSONL (yaw_gear_sample 事件) 中, 本工具不读取也不依赖它们.

输出:
  默认人可读报告; --json 输出机器可读报告; --residuals FILE 输出每点残差 CSV.

单圈/连续角: 只识别电机角跳变、非单调与跨度异常并提示人工复核; 不推断周期,
不自动解卷绕, 不改写原始读数.

本工具只做离线计算, 不发送控制命令, 不访问硬件, 不写固件参数. 报告只说明拟合
数学是否一致; 未经独立非拟合点验收与实车独立验证, 任何结果都不得宣称可部署.
仅用原三列时, 位置来源与采样时效均未证实, 结果同样不能宣称可部署.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from dataclasses import dataclass

REQUIRED_COLUMNS = ("approach", "turret_deg", "motor_rad")
OPTIONAL_COLUMNS = (
    "source",
    "sampled_ms",
    "age_ms",
    "valid",
    "mark",
    "note",
    "data_kind",
)
POSITION_SOURCES = frozenset({"mit_pos", "p_m", "xout"})
DATA_KINDS = frozenset({"SYNTHETIC", "REAL"})
MIN_TOTAL_SAMPLES = 6
MIN_PER_DIRECTION = 3


@dataclass(frozen=True)
class Sample:
    approach: str
    motor_rad: float
    turret_rad: float
    turret_deg: float
    line: int
    source: str | None
    sampled_ms: float | None
    age_ms: float | None
    mark: str
    note: str
    data_kind: str | None
    valid_reported: bool


def fit_line(samples: list[Sample]):
    """单方向最小二乘直线: 返回 (斜率, 截距)."""
    xm = sum(s.motor_rad for s in samples) / len(samples)
    ym = sum(s.turret_rad for s in samples) / len(samples)
    var = sum((s.motor_rad - xm) ** 2 for s in samples)
    if var < 1e-9:
        raise ValueError("电机角度变化过小，无法标定传动比")
    slope = sum((s.motor_rad - xm) * (s.turret_rad - ym) for s in samples) / var
    intercept = ym - slope * xm
    return slope, intercept


def _cell(row: dict, name: str) -> str:
    value = row.get(name)
    return "" if value is None else value.strip()


def _required_float(row: dict, name: str, number: int) -> float:
    raw = _cell(row, name)
    if not raw:
        raise ValueError(f"第 {number} 行 {name} 为空：请先填入实际测量值")
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"第 {number} 行 {name} 不是数字：{raw!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"第 {number} 行 {name} 包含 NaN 或无穷大")
    return value


def _optional_float(row: dict, name: str, number: int) -> float | None:
    raw = _cell(row, name)
    if not raw:
        return None
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"第 {number} 行 {name} 不是数字：{raw!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"第 {number} 行 {name} 包含 NaN 或无穷大")
    if value < 0:
        raise ValueError(f"第 {number} 行 {name} 不能为负：{value}")
    return value


def _parse_valid(row: dict, number: int) -> tuple[bool, bool]:
    """返回 (是否明确标记过 valid, 是否有效)."""
    raw = _cell(row, "valid")
    if not raw:
        return False, True
    normalized = raw.lower()
    if normalized in {"1", "true", "yes", "ok", "valid"}:
        return True, True
    if normalized in {"0", "false", "no", "invalid"}:
        return True, False
    raise ValueError(f"第 {number} 行 valid 无法识别：{raw!r}（用 1/0 或 true/false）")


def _parse_source(row: dict, number: int) -> str | None:
    raw = _cell(row, "source").lower()
    if not raw:
        return None
    if raw not in POSITION_SOURCES:
        raise ValueError(
            f"第 {number} 行 source 无效：{raw!r}（只允许 "
            + "/".join(sorted(POSITION_SOURCES))
            + "）"
        )
    return raw


def _parse_data_kind(row: dict, number: int) -> str | None:
    raw = _cell(row, "data_kind").upper()
    if not raw:
        return None
    if raw not in DATA_KINDS:
        raise ValueError(f"第 {number} 行 data_kind 无效：{raw!r}（用 SYNTHETIC 或 REAL）")
    return raw


def load(path: str) -> list[Sample]:
    data: list[Sample] = []
    with open(path, newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        fieldnames = reader.fieldnames or []
        if len(set(fieldnames)) != len(fieldnames):
            raise ValueError("CSV 表头存在重复列名")
        missing = [name for name in REQUIRED_COLUMNS if name not in fieldnames]
        if missing:
            raise ValueError(f'CSV 必须包含列: {", ".join(sorted(REQUIRED_COLUMNS))}')
        for number, row in enumerate(reader, 2):
            if all(not _cell(row, name) for name in fieldnames):
                continue
            group = _cell(row, "approach").lower()
            if group not in ("forward", "reverse"):
                raise ValueError(f"第 {number} 行 approach 必须是 forward 或 reverse")
            turret_deg = _required_float(row, "turret_deg", number)
            motor_rad = _required_float(row, "motor_rad", number)
            valid_reported, is_valid = _parse_valid(row, number)
            if not is_valid:
                raise ValueError(
                    f"第 {number} 行反馈标记为无效（valid=0），无法作为标定点"
                )
            source = _parse_source(row, number)
            sampled_ms = _optional_float(row, "sampled_ms", number)
            age_ms = _optional_float(row, "age_ms", number)
            # 与采集 GUI 的 MIT/位置寄存器 500 ms 新鲜度窗口一致, 不是机械精度标准.
            if age_ms is not None and age_ms >= 500.0:
                raise ValueError(f"第 {number} 行位置反馈已过期（age_ms={age_ms:g}，要求 <500 ms）")
            data_kind = _parse_data_kind(row, number)
            data.append(
                Sample(
                    approach=group,
                    motor_rad=motor_rad,
                    turret_rad=math.radians(turret_deg),
                    turret_deg=turret_deg,
                    line=number,
                    source=source,
                    sampled_ms=sampled_ms,
                    age_ms=age_ms,
                    mark=_cell(row, "mark"),
                    note=_cell(row, "note"),
                    data_kind=data_kind,
                    valid_reported=valid_reported,
                )
            )
    if len(data) < MIN_TOTAL_SAMPLES:
        raise ValueError(
            f"至少需要 {MIN_TOTAL_SAMPLES} 个有效测量点，建议正向 5 点与反向 4 点"
        )
    return data


def _single_metadata(values: list[str | None], label: str) -> str | None:
    """同一份数据只允许一种来源/数据类型; 部分缺失同样视为不可确认."""
    present = [value for value in values if value]
    if not present:
        return None
    if len(present) != len(values):
        raise ValueError(f"部分行缺少 {label}，无法确认整份数据来源一致")
    unique = set(present)
    if len(unique) != 1:
        raise ValueError(f"混用 {label}：{', '.join(sorted(unique))}；一份数据只能保留一种")
    return present[0]


def _validate_dataset(data: list[Sample]) -> dict:
    grouped: dict[str, list[Sample]] = defaultdict(list)
    for item in data:
        grouped[item.approach].append(item)
    if set(grouped) != {"forward", "reverse"}:
        raise ValueError("正向与反向各至少需要 3 个独立测量点")
    for name in ("forward", "reverse"):
        if len(grouped[name]) < MIN_PER_DIRECTION:
            raise ValueError(f"{name} 至少需要 {MIN_PER_DIRECTION} 个独立测量点")

    for name, group in grouped.items():
        seen: dict[float, int] = {}
        for item in group:
            key = round(item.turret_deg, 9)
            if key in seen:
                raise ValueError(
                    f"第 {item.line} 行与第 {seen[key]} 行 {name} 标定点 turret_deg "
                    f"重复（{item.turret_deg:g}°）；同方向每个角度只保留一个独立点"
                )
            seen[key] = item.line
        if len(seen) < MIN_PER_DIRECTION:
            raise ValueError(f"{name} 至少需要 {MIN_PER_DIRECTION} 个不同角度")
        ordered = sorted(group, key=lambda s: s.line)
        for previous, current in zip(ordered, ordered[1:]):
            if name == "forward" and current.turret_deg <= previous.turret_deg:
                raise ValueError(
                    f"正向标定点必须按 turret_deg 递增记录（第 {previous.line} 行 "
                    f"{previous.turret_deg:g}° → 第 {current.line} 行 "
                    f"{current.turret_deg:g}°）"
                )
            if name == "reverse" and current.turret_deg >= previous.turret_deg:
                raise ValueError(
                    f"反向标定点必须按 turret_deg 递减记录（第 {previous.line} 行 "
                    f"{previous.turret_deg:g}° → 第 {current.line} 行 "
                    f"{current.turret_deg:g}°）"
                )

    source = _single_metadata([item.source for item in data], "位置来源(source)")
    data_kind = _single_metadata([item.data_kind for item in data], "数据类型(data_kind)")
    source_reported = source is not None
    timeliness_reported = all(
        item.sampled_ms is not None and item.age_ms is not None for item in data
    )
    feedback_valid_reported = all(item.valid_reported for item in data)
    return {
        "grouped": grouped,
        "source": source,
        "data_kind": data_kind,
        "source_reported": source_reported,
        "timeliness_reported": timeliness_reported,
        "feedback_valid_reported": feedback_valid_reported,
        "data_kind_reported": data_kind is not None,
    }


def _angle_continuity(grouped: dict) -> tuple[list[str], list[str]]:
    """识别单圈跨边界/未解卷绕迹象.

    只报告异常并提示复核, 不猜周期, 不展开 MIT 位置, 不修改原始读数.
    返回 (必须复核的异常, 建议复核的提示).
    """
    anomalies: list[str] = []
    advisories: list[str] = []
    for name, group in sorted(grouped.items()):
        ordered = sorted(group, key=lambda s: s.line)
        deltas = []
        for previous, current in zip(ordered, ordered[1:]):
            delta = current.motor_rad - previous.motor_rad
            deltas.append(delta)
            if abs(delta) > math.pi:
                anomalies.append(
                    f"{name} 第 {previous.line}→{current.line} 行电机角跳变 "
                    f"{delta:+.3f} rad（超过 π），疑似单圈跨边界或未解卷绕"
                )
        if deltas and not (all(delta > 0 for delta in deltas) or all(delta < 0 for delta in deltas)):
            advisories.append(f"{name} 电机角不单调，可能跨圈或回读抖动，请复核")
        motor_span = max(s.motor_rad for s in group) - min(s.motor_rad for s in group)
        turret_span = max(s.turret_rad for s in group) - min(s.turret_rad for s in group)
        if motor_span > 2 * math.pi and turret_span > 0:
            anomalies.append(
                f"{name} 电机角跨度 {motor_span:.3f} rad 超过 2π，"
                "小角度标定下可能包含跨圈数据"
            )
    return anomalies, advisories


def _status_reason(
    real_measurements: bool,
    mechanism_sign_ok: bool,
    provenance_complete: bool,
    continuity_anomalies: list[str],
) -> str:
    if not real_measurements:
        return "合成/未指定数据，仅用于工具回归"
    if not mechanism_sign_ok:
        return "k>=0 与外啮合预期不符，判定不可用"
    if continuity_anomalies:
        return "存在跨圈/未解卷绕异常，需复核原始读数后重测"
    if not provenance_complete:
        return "来源/时效/有效性未证实，不能作为实车候选参数"
    return "拟合数学通过且来源完整，但独立非拟合点与实车验证尚未完成"


def analyze(path: str) -> dict:
    data = load(path)
    meta = _validate_dataset(data)
    grouped = meta["grouped"]
    continuity_anomalies, continuity_advisories = _angle_continuity(grouped)

    # 固定效应回归: 正反方向共享同一物理传动比, 各自保留独立截距以吸收回差.
    centered_cov = centered_var = 0.0
    for group in grouped.values():
        xmean = sum(p.motor_rad for p in group) / len(group)
        ymean = sum(p.turret_rad for p in group) / len(group)
        centered_cov += sum((p.motor_rad - xmean) * (p.turret_rad - ymean) for p in group)
        centered_var += sum((p.motor_rad - xmean) ** 2 for p in group)
    if centered_var < 1e-9:
        raise ValueError("运动幅度过小，无法可靠辨识传动比")
    k = centered_cov / centered_var
    if abs(k) < 1e-6:
        raise ValueError("拟合传动比过小，检查角度单位与数据来源")

    offsets: dict[str, float] = {}
    slopes: dict[str, float] = {}
    residual_points: list[dict] = []
    for name, group in sorted(grouped.items()):
        independent_k, _ = fit_line(group)
        slopes[name] = independent_k
        b = sum(p.turret_rad - k * p.motor_rad for p in group) / len(group)
        offsets[name] = b
        for item in group:
            predicted = k * item.motor_rad + b
            residual_points.append(
                {
                    "line": item.line,
                    "approach": item.approach,
                    "turret_deg": item.turret_deg,
                    "motor_rad": item.motor_rad,
                    "predicted_turret_deg": math.degrees(predicted),
                    "residual_deg": math.degrees(item.turret_rad - predicted),
                    "source": item.source,
                    "age_ms": item.age_ms,
                    "mark": item.mark,
                    "note": item.note,
                }
            )
    residual_points.sort(key=lambda point: point["line"])

    motor_zero_forward = -offsets["forward"] / k
    motor_zero_reverse = -offsets["reverse"] / k
    backlash_turret_deg = abs(math.degrees(offsets["forward"] - offsets["reverse"]))
    residuals = [math.radians(point["residual_deg"]) for point in residual_points]
    rms_deg = math.degrees(math.sqrt(sum(x * x for x in residuals) / len(residuals)))
    worst_deg = math.degrees(max(abs(x) for x in residuals))
    relative_spread = abs(slopes["forward"] - slopes["reverse"]) / max(abs(k), 1e-9)

    mechanism_sign_ok = k < 0  # 外啮合固定轴机构预期为负号; 不符只告警, 不翻转实测值
    warnings: list[str] = []
    if not mechanism_sign_ok:
        warnings.append(
            "外啮合固定轴机构预期 k<0，实测拟合却为 k>=0；"
            "已按原值保留，不翻转符号，判定为不可用，需检查角度正方向或传动机构"
        )
    if relative_spread > 0.05:
        warnings.append(
            f"正反向单独拟合斜率相差 {relative_spread:.1%}，"
            "建议检查传动间隙、读数时效与量角误差"
        )

    data_kind = meta["data_kind"]
    real_measurements = data_kind == "REAL"
    legacy_three_column = not any(
        meta[key]
        for key in (
            "source_reported",
            "timeliness_reported",
            "feedback_valid_reported",
            "data_kind_reported",
        )
    )
    provenance_complete = (
        real_measurements
        and meta["source_reported"]
        and meta["timeliness_reported"]
        and meta["feedback_valid_reported"]
    )
    if not real_measurements:
        warnings.append("数据未标记为 REAL（当前为 SYNTHETIC/未指定），参数只能用于工具回归，不能作为实车标定")
    if not meta["source_reported"]:
        warnings.append("缺少位置来源(source)，无法确认所有点来自同一反馈源")
    if not meta["timeliness_reported"]:
        warnings.append("缺少采样时刻(sampled_ms)或时延(age_ms)，无法核对读数时效")
    if not meta["feedback_valid_reported"]:
        warnings.append("部分测量点未显式标记反馈有效(valid)，有效性依赖采集端")
    if legacy_three_column:
        warnings.append(
            "仅使用原三列(approach,turret_deg,motor_rad)：位置来源与采样时效均未证实，"
            "不能据此确认数据可用，更不能宣称可直接部署"
        )
    warnings.extend(continuity_advisories)
    usable_as_candidate = (
        mechanism_sign_ok and provenance_complete and not continuity_anomalies
    )
    warnings.append("尚未验收独立非拟合点，也未完成实车独立验证，不能宣称可部署")

    return {
        "tool": "fit_yaw_gear",
        "data_file": path,
        "samples": len(data),
        "counts": {"forward": len(grouped["forward"]), "reverse": len(grouped["reverse"])},
        "fit_math": "pass",
        "parameters": {
            "k_rad_turret_per_rad_motor": k,
            "equivalent_gear_ratio": 1 / abs(k),
            "motor_zero_forward_rad": motor_zero_forward,
            "motor_zero_reverse_rad": motor_zero_reverse,
            "backlash_turret_deg": backlash_turret_deg,
            "per_direction_slope": slopes,
            "per_direction_slope_spread_relative": relative_spread,
        },
        "residuals_summary": {"rms_deg": rms_deg, "max_abs_deg": worst_deg},
        "residual_points": residual_points,
        "continuity": {
            "unwrap_performed": False,
            "reason_checked": "只识别跨圈/未解卷绕迹象并提示复核，不推断周期",
            "anomalies": continuity_anomalies,
            "advisories": continuity_advisories,
        },
        "provenance": {
            "data_kind": data_kind or "UNSPECIFIED",
            "position_source": meta["source"] or "UNSPECIFIED",
            "source_reported": meta["source_reported"],
            "timeliness_reported": meta["timeliness_reported"],
            "feedback_valid_reported": meta["feedback_valid_reported"],
            "legacy_three_column": legacy_three_column,
            "max_age_ms": max(
                (item.age_ms for item in data if item.age_ms is not None), default=None
            ),
            "real_measurements": real_measurements,
        },
        "status": {
            "fit_math_passed": True,
            "mechanism_sign_ok": mechanism_sign_ok,
            "real_measurements": real_measurements,
            "provenance_complete": provenance_complete,
            "continuity_ok": not continuity_anomalies,
            "independent_non_fit_points_accepted": False,
            "usable_as_candidate": usable_as_candidate,
            "deployable": False,
            "reason": _status_reason(
                real_measurements, mechanism_sign_ok, provenance_complete, continuity_anomalies
            ),
        },
        "warnings": warnings,
    }


def render_text(report: dict) -> str:
    p = report["parameters"]
    r = report["residuals_summary"]
    lines = [
        f'采样点: {report["samples"]}（正向 {report["counts"]["forward"]} / '
        f'反向 {report["counts"]["reverse"]}）',
        f'带符号换算系数 k = {p["k_rad_turret_per_rad_motor"]:+.8f} '
        "[rad_turret / rad_motor]",
        f'相对传动比 |k| = {abs(p["k_rad_turret_per_rad_motor"]):.8f}；'
        f'等效大/小齿轮齿数比 ≈ {p["equivalent_gear_ratio"]:.5f}',
        f'正向单独拟合 k = {p["per_direction_slope"]["forward"]:+.8f}',
        f'反向单独拟合 k = {p["per_direction_slope"]["reverse"]:+.8f}',
        f'正向接近零位的 motor_zero_rad = {p["motor_zero_forward_rad"]:+.8f} rad',
        f'反向接近零位的 motor_zero_rad = {p["motor_zero_reverse_rad"]:+.8f} rad',
        f'方向相关的等效回差（云台端）≈ {p["backlash_turret_deg"]:.4f}°',
        f'拟合残差：RMS = {r["rms_deg"]:.4f}°；最大绝对值 = {r["max_abs_deg"]:.4f}°',
        "",
        "换算示意（采用正向接近零位）：",
        f'  turret_relative_rad = ({p["k_rad_turret_per_rad_motor"]:+.8f}) * '
        f'(motor_unwrapped_rad - ({p["motor_zero_forward_rad"]:+.8f}))',
        "",
        "数据质量：",
        f'  数据类型: {report["provenance"]["data_kind"]}',
        f'  位置来源: {report["provenance"]["position_source"]}',
        f'  采样时刻/时延: '
        f'{"已提供" if report["provenance"]["timeliness_reported"] else "缺失"}',
        f'  反馈有效性标记: '
        f'{"已提供" if report["provenance"]["feedback_valid_reported"] else "部分缺失"}',
        f'  最大反馈时延: '
        + (
            f'{report["provenance"]["max_age_ms"]:.3f} ms'
            if report["provenance"]["max_age_ms"] is not None
            else "未提供"
        ),
        f'  单圈/连续角检查: '
        + (
            "发现异常，需复核（本工具不猜周期、不自动解卷绕）"
            if report["status"]["continuity_ok"] is False
            else "未发现跨圈异常"
        ),
        "",
        "判定：",
        f'  拟合数学: {"通过" if report["status"]["fit_math_passed"] else "未通过"}',
        f'  外啮合符号(k<0): '
        f'{"符合" if report["status"]["mechanism_sign_ok"] else "不符合，参数不可用"}',
        f'  来源/时效/有效性完整: '
        f'{"是" if report["status"]["provenance_complete"] else "否（尚缺，不能宣称可用）"}',
        f'  独立非拟合点验收: '
        f'{"已通过" if report["status"]["independent_non_fit_points_accepted"] else "未进行"}',
        f'  可作为待复核候选参数: '
        f'{"是" if report["status"]["usable_as_candidate"] else "否"}',
        f'  可直接部署: 否（{report["status"]["reason"]}）',
    ]
    if report["continuity"]["anomalies"]:
        lines.append("")
        lines.append("单圈/连续角异常（需人工复核原始读数）：")
        lines.extend(f"  - {issue}" for issue in report["continuity"]["anomalies"])
    if report["continuity"]["advisories"]:
        lines.append("")
        lines.append("连续角提示：")
        lines.extend(f"  - {issue}" for issue in report["continuity"]["advisories"])
    if report["warnings"]:
        lines.append("")
        lines.append("告警：")
        lines.extend(f"  - {warning}" for warning in report["warnings"])
    lines.append("")
    lines.append("注意：以上参数不是实车验证结论；复核并完成独立验证后才考虑回填固件。")
    return "\n".join(lines)


def write_residuals(path: str, report: dict) -> None:
    fields = [
        "line",
        "approach",
        "turret_deg",
        "motor_rad",
        "predicted_turret_deg",
        "residual_deg",
        "source",
        "age_ms",
        "mark",
        "note",
    ]
    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(report["residual_points"])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="步兵 J4310 外啮合小角度传动比/回差离线拟合",
        epilog=(
            "退出码: 0 拟合数学通过(仍不代表可部署); 2 数据被拒绝; "
            "3 拟合完成但判定不可用(外啮合符号不符或存在跨圈异常)"
        ),
    )
    parser.add_argument("csv_file", help="按模板记录的真实测量 CSV")
    parser.add_argument("--json", action="store_true", help="输出 JSON 报告")
    parser.add_argument("--residuals", metavar="FILE", help="写出每点残差 CSV")
    args = parser.parse_args(argv)

    try:
        report = analyze(args.csv_file)
        if args.residuals:
            write_residuals(args.residuals, report)
    except (ValueError, OSError) as exc:
        if args.json:
            print(json.dumps({"tool": "fit_yaw_gear", "fit_math": "rejected", "error": str(exc)}, ensure_ascii=False))
        else:
            print(f"标定失败：{exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(render_text(report))
    usable = report["status"]["mechanism_sign_ok"] and report["status"]["continuity_ok"]
    return 0 if usable else 3


if __name__ == "__main__":
    sys.exit(main())

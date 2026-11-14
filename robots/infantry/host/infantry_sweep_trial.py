#!/usr/bin/env python3
"""一次受监督的 Yaw 多频力矩激励试验 (单串口 USART1, 921600).

业务请求与采集分页严格串行单飞: 同一时刻只有一个请求在外, 因此业务应答不会被
采集帧吞掉, 也不需要另建分流传输层. 必须显式 --arm, 并给出经确认的当前世界航向
目标 (与当前航向相差 <0.1 rad); 不读写任何 PID 或控制参数.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'robots/infantry/host'))

import infantry_client as wire
import infantry_sweep_capture as capture

SWEEP = 0x49
START, CANCEL = 1, 0
REQUEST_TIMEOUT_S = 0.015     # 单次业务请求上限: 两次刷新合计 < 心跳 45 ms
CAPTURE_TIMEOUT_S = 0.015     # 采集分页只读, 失败可快速重发
HEARTBEAT_S = 0.045           # 维护权与目标都需要持续刷新
STARTUP_TIMEOUT_S = 2.0
ARM_WAIT_S = 0.7               # START ACK 只代表 arming 接受, 运行时由固件随后建立
DM_FRESH_BIT, DM_ENABLED_BIT = 0x04, 0x08
MAX_GAP_US = 20000            # 250 Hz 下 4 ms 一条; 5 个周期无记录即视为断档
ACTIVE_BIT, TERMINAL_BIT = 0x80, 0x100
REQUIRED_BITS = 0x0F          # target/heading/DM 反馈/DM 使能
MIN_SAMPLES = 20
# 固件DM反馈TTL为20ms; 主机预检留出请求往返裕量, 不扩大固件新鲜度门限.
DM_FRESH_MS = 10


def _encoded_sequence(message: bytes) -> int:
    """从已编码帧里取外层序号 (跳过转义), 用于只接受本次请求的应答."""
    raw = bytearray()
    escaped = False
    for byte in message:
        if byte == wire.FRAME_START and not raw:
            continue
        if escaped:
            raw.append(byte ^ wire.ESCAPE_XOR)
            escaped = False
        elif byte == wire.FRAME_ESCAPE:
            escaped = True
        else:
            raw.append(byte)
        if len(raw) >= 4:
            break
    return raw[2] | (raw[3] << 8)


class Link:
    """单飞链路: 只接受与本次请求同外层序号的应答, 迟到帧直接丢弃."""

    def __init__(self, inner):
        self.inner = inner

    def send(self, message: bytes) -> None:
        self.inner.send(message)

    def receive(self, timeout_s: float = 0.0):
        return self.inner.receive(timeout_s)

    def exchange(self, message: bytes, timeout_s: float = REQUEST_TIMEOUT_S) -> tuple:
        sequence = _encoded_sequence(message)
        self.inner.send(message)
        deadline = time.monotonic() + min(timeout_s, REQUEST_TIMEOUT_S)
        while (left := deadline - time.monotonic()) > 0:
            frame = self.inner.receive(min(left, 0.005))
            if frame is not None and frame[1] == sequence:
                return frame
        raise TimeoutError(
            f'frame {sequence} timed out after {REQUEST_TIMEOUT_S * 1000:.0f} ms')

    def close(self) -> None:
        self.inner.close()


def accepted(ack: dict, what: str) -> None:
    if ack.get('result', 0) != 0:
        raise RuntimeError(f'{what} rejected: {ack}')


def _main_yaw(page: dict):
    return next((item for item in page.get('domains', [])
                 if item.get('domain') == 'main_yaw'), None)


def _dm_main_yaw(page: dict):
    return next((item for item in page.get('devices', [])
                 if item.get('domain') == 'main_yaw'), None)


def target_ready(page: dict) -> bool:
    """主 yaw 已采纳 local_inertial 位置目标且自身有效.

    不要求 execute: 终态会锁存 yaw 停机, 此时目标仍可正常采纳; 再要求执行态
    会让重复试验永远等不到 START, arming 由固件在 START 之后重新建立.
    """
    item = _main_yaw(page)
    return bool(item) and item.get('validity') == 'valid' \
        and bool(item.get('measurement_valid')) \
        and item.get('mode') == 'position' and item.get('reason') == 'none' \
        and item.get('requested_frame') == 'local_inertial' \
        and item.get('adopted_frame') == 'local_inertial'


def dm_healthy(client) -> bool:
    """DM 在线且反馈新鲜即可: 终态锁存后允许关机状态, 由 START arming 重新使能."""
    # diagnostics(1) 与 status(1) 同属50ms限频页, 连续等待它会错过目标保活.
    # 快状态的机械角有效性直接来自同一DM新鲜反馈, START仍由MCU再次检查设备状态.
    page = client.status(0)
    accepted(page, 'fast DM status')
    item = page.get('joints', {}).get('main_yaw', {})
    return bool(item.get('position_valid')) and bool(item.get('velocity_valid')) \
        and item.get('position_age_ms') is not None \
        and item['position_age_ms'] <= DM_FRESH_MS


def wait_ready(client, heading: float, setpoints: list) -> None:
    """先只读确认航向接近目标, 再取维护权下发目标, 最后等惯性目标与 DM 生效."""
    deadline = time.monotonic() + STARTUP_TIMEOUT_S
    last = None
    while True:
        page0 = client.status(0)
        accepted(page0, 'status page 0')
        last = page0
        platform = page0.get('main_platform', {})
        current = platform.get('heading_rad')
        if platform.get('validity') == 'valid' and isinstance(current, (int, float)) \
                and math.isfinite(current):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError(f'no valid heading within {STARTUP_TIMEOUT_S:.1f} s: {last}')
        time.sleep(0.005)
    # 显式目标必须贴近当前航向; 否则拒绝开环激励而不是把零位当安全值.
    if abs(current - heading) >= 0.1:
        raise RuntimeError(f'target heading {heading:.4f} is not near current {current:.4f}')
    accepted(client.authorize(True, 'maintenance'), 'maintenance authorize')
    accepted(client.targets(setpoints, main_yaw_frame='local_inertial'), 'held heading')
    while time.monotonic() < deadline:
        page1 = client.status(1)
        accepted(page1, 'status page 1')
        last = page1
        if page1.get('source') == 'maintenance' and page1.get('authority') \
                and target_ready(page1) and dm_healthy(client):
            return
        accepted(client.authorize(True, 'maintenance'), 'maintenance refresh')
        accepted(client.targets(setpoints, main_yaw_frame='local_inertial'), 'target refresh')
        # status(1)是50ms限频慢页, 等待时不紧接着重发; 目标刚刷新且TTL100ms.
        time.sleep(0.05)
    raise RuntimeError(f'startup not ready within {STARTUP_TIMEOUT_S:.1f} s: {last}')


def sweep_command(client, op: int, frequencies: list, cycles: int,
                  amplitude_nm: float) -> dict:
    """0x49 控制帧: 与固件契约一致, 并保留外层与 body 双重身份校验."""
    sequence = client._next_sequence()
    client.frame_sequence = (client.frame_sequence + 1) & 0xFFFF
    outer = client.frame_sequence
    body = bytes([wire.PROTOCOL_VERSION, SWEEP]) + struct.pack(
        '<IIBBBf', client.session, sequence, op, len(frequencies), cycles, float(amplitude_nm))
    body += b''.join(struct.pack('<f', value) for value in frequencies)
    rtype, echo, payload = client.transport.exchange(wire.encode_frame(SWEEP, outer, body))
    if rtype != wire.ACK or echo != outer:
        raise RuntimeError(f'sweep {op}: unexpected reply 0x{rtype:02x} echo {echo}')
    ack = wire.decode_ack(payload)
    if (ack['session'], ack['sequence']) != (client.session, sequence):
        raise RuntimeError(f'sweep {op}: ACK identity mismatch')
    accepted(ack, f'sweep op {op}')
    return ack


def capture_page(client, op: int, cursor: int) -> dict:
    """采集分页使用同一客户端的外层序号, 每次只在上一请求返回后才发下一次."""
    client.frame_sequence = (client.frame_sequence + 1) & 0xFFFF
    return capture.request_page(client.transport, client.frame_sequence, op, cursor,
                                CAPTURE_TIMEOUT_S)


def rms(values: list) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def centered_rms(values: list) -> float:
    if not values:
        return 0.0
    mean = sum(values) / len(values)
    return rms([value - mean for value in values])


def per_frequency(path: Path, frequencies: list) -> list:
    """按频点汇总活性样本: 只给测量与注入统计, 不宣称最优或辨识结论."""
    with path.open(newline='', encoding='utf-8') as stream:
        rows = [row for row in csv.DictReader(stream)
                if (int(row['flags']) & ACTIVE_BIT) and not (int(row['flags']) & TERMINAL_BIT)]
    report = []
    for hz in frequencies:
        subset = [row for row in rows if abs(float(row['sweep_hz']) - hz) <= 1e-3]
        heading = [float(row['heading_rad']) for row in subset]
        rate = [float(row['heading_rate_rad_s']) for row in subset]
        applied = [float(row['applied_excitation_nm']) for row in subset]
        submitted = [float(row['requested_excitation_nm']) for row in subset]
        applied_rms = rms(applied)
        report.append(dict(
            hz=hz, samples=len(subset),
            heading_rms_rad=centered_rms(heading),
            heading_rate_rms_rad_s=centered_rms(rate),
            applied_rms_nm=applied_rms,
            applied_peak_nm=max((abs(v) for v in applied), default=0.0),
            submitted_rms_nm=rms(submitted),
            submitted_peak_nm=max((abs(v) for v in submitted), default=0.0),
            usable=len(subset) >= MIN_SAMPLES and applied_rms > 1e-5))
    return report


def acquire(client, args, cursor: int, duration: float, writer, stream, state) -> dict:
    """严格串行地边刷新心跳边按页排空; 由 MCU 时戳与运行位判断结束.

    START 只代表 arming 已被接受: 使能与运行由固件重新建立, 这里最多等 ARM_WAIT_S,
    运行时长从首个 running 记录 (arming 完成) 起按 MCU 时戳计算.
    """
    arm_deadline = time.monotonic() + ARM_WAIT_S
    cutoff = time.monotonic() + duration + 3.0
    heartbeat = time.monotonic()
    setpoints = [('main_yaw', 'position', float(args.target_heading), 0)]
    run_id = first_ts = last_ts = last_epoch = None
    terminal_ts = terminal_flags = terminal_reason = None
    active = 0
    while terminal_ts is None:
        now = time.monotonic()
        if now >= cutoff:
            raise RuntimeError('trial wall-clock deadline exceeded before MCU terminal')
        if run_id is None and now >= arm_deadline:
            raise RuntimeError(f'arming not confirmed within {ARM_WAIT_S * 1000:.0f} ms')
        if now >= heartbeat:
            accepted(client.authorize(True, 'maintenance'), 'maintenance heartbeat')
            accepted(client.targets(setpoints, main_yaw_frame='local_inertial'),
                     'target refresh')
            heartbeat = time.monotonic() + HEARTBEAT_S
        page = capture_page(client, 1, cursor)
        if page['first'] != cursor:
            raise RuntimeError(
                f'capture ring gap: expected {cursor}, first {page["first"]}')
        if not page['rows']:
            time.sleep(0.002)
            continue
        for row in page['rows']:
            if row['sequence'] != cursor:
                raise RuntimeError('capture sequence discontinuity')
            cursor += 1
            capture.validate_sample(row)
            if last_ts is not None:
                gap = (row['timestamp_us'] - last_ts) & 0xFFFFFFFF
                if not 0 < gap <= MAX_GAP_US:
                    raise RuntimeError(f'sample timing invalid: {gap} us')
            last_ts = row['timestamp_us']
            if last_epoch is not None and row['reference_epoch'] != last_epoch:
                raise RuntimeError('reference epoch changed during trial')
            last_epoch = row['reference_epoch']
            flags = int(row['flags'])
            if flags & TERMINAL_BIT:
                if run_id is not None and row['sweep_run_id'] != run_id:
                    raise RuntimeError('terminal row changed the sweep run id')
                terminal_ts, terminal_flags = row['timestamp_us'], flags
                terminal_reason = int(row['reason'])
                writer.writerow(row)
                state.update(cursor=cursor, terminal=True)
                break
            if not flags & ACTIVE_BIT:
                continue  # 启动前的非运行记录不计入活性数据
            if flags & REQUIRED_BITS != REQUIRED_BITS or row['reason'] != 0:
                raise RuntimeError(
                    f'active sample invalid: flags=0x{flags:03x} reason={row["reason"]}')
            if run_id is None:
                run_id = row['sweep_run_id']
                if run_id == 0:
                    raise RuntimeError('active sample carries zero sweep run id')
            elif row['sweep_run_id'] != run_id:
                raise RuntimeError('sweep run id changed during trial')
            if first_ts is None:
                first_ts = row['timestamp_us']
            writer.writerow(row)
            active += 1
        stream.flush()
        state['cursor'] = cursor
    span = 0 if first_ts is None else (terminal_ts - first_ts) & 0xFFFFFFFF
    expected = int(duration * 1e6)
    # 只按 MCU 时戳与运行状态判断正常结束, 不用墙钟提前下结论.
    complete = bool(run_id and active >= MIN_SAMPLES and span + 20000 >= expected)
    return dict(run_id=run_id, active_samples=active, mcu_span_us=span,
                expected_us=expected, complete=complete,
                terminal_flags=terminal_flags, terminal_reason=terminal_reason)


def confirm_disabled(client, cursor: int) -> dict:
    """终态撤销后核验: 维护权已撤, 且高速采集里 DM 使能位为 false 而反馈仍新鲜."""
    status = client.status(0)
    authority = bool(status.get('authority')) if status.get('result', 0) == 0 else None
    # 重新定位到撤权后的最新游标, 不使用取消前缓存中的Enabled样本作停机结论.
    cursor = capture_page(client, 0, 0)['next']
    deadline = time.monotonic() + 0.3
    last = None
    while time.monotonic() < deadline:
        page = capture_page(client, 1, cursor)
        for row in page['rows']:
            cursor = (row['sequence'] + 1) & 0xffffffff
            last = int(row['flags'])
        if last is not None and last & DM_FRESH_BIT and not last & DM_ENABLED_BIT:
            break
        time.sleep(0.005)
    if last is None:
        return dict(confirmed=False, reason='no high-rate sample after terminal')
    fresh, enabled = bool(last & DM_FRESH_BIT), bool(last & DM_ENABLED_BIT)
    return dict(flags=last, feedback_fresh=fresh, dm_enabled=enabled,
                authority=authority, confirmed=fresh and not enabled and authority is False)


def _plan(args) -> float:
    if not args.arm:
        raise RuntimeError('explicit --arm required for motor excitation')
    frequencies = args.frequencies
    if (not frequencies or len(frequencies) > 6
            or not all(math.isfinite(f) for f in frequencies)
            or not 0.2 <= frequencies[0]
            or any(a >= b for a, b in zip(frequencies, frequencies[1:]))
            or frequencies[-1] > 5.0
            or not 0.0 < args.amplitude_nm <= 0.03
            or not 3 <= args.cycles <= 12
            or not math.isfinite(args.target_heading)):
        raise ValueError('invalid plan: 0.2..5 Hz strictly ascending, '
                         'amplitude in (0,0.03] Nm, cycles 3..12')
    duration = sum(args.cycles / hz for hz in frequencies)
    if duration > 120.0:
        raise ValueError('invalid plan: total excitation exceeds 120 s')
    return duration


def run(args, link_factory=None) -> dict:
    duration = _plan(args)
    report_path = Path(str(args.output) + '.report.json')
    if args.output.exists() or report_path.exists():
        raise FileExistsError(f'will not overwrite {args.output} or its summary')
    if args.elf is not None:
        elf_sha256 = capture.file_sha256(args.elf)
    else:
        elf_sha256 = None
    meta = dict(tool='infantry_sweep_trial', schema=capture.SCHEMA, complete=False,
                frequencies=list(args.frequencies), cycles=args.cycles,
                amplitude_nm=args.amplitude_nm, duration_s=duration,
                target_heading_rad=args.target_heading, port=args.serial, baud=args.baud,
                elf_sha256=elf_sha256)
    setpoints = [('main_yaw', 'position', float(args.target_heading), 0)]
    link = client = None
    start_attempted = False
    state = {}
    try:
        factory = link_factory or (lambda: wire.SerialTransport(args.serial, args.baud))
        link = Link(factory())
        client = wire.InfantryClient(link)
        meta['identity'] = client.hello().get('identity')
        wait_ready(client, float(args.target_heading), setpoints)
        with args.output.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=capture.FIELDS)
            writer.writeheader()
            stream.flush()
            start_attempted = True
            sweep_command(client, START, args.frequencies, args.cycles, args.amplitude_nm)
            # 在START受理后取游标, 不把两次请求之间的上次终态归入本次运行.
            cursor = capture_page(client, 0, 0)['next']
            result = acquire(client, args, cursor, duration, writer, stream, state)
        meta.update(records=result['active_samples'], sweep_run_id=result['run_id'],
                    mcu_span_us=result['mcu_span_us'], expected_us=result['expected_us'],
                    terminal_flags=result['terminal_flags'],
                    terminal_reason=result['terminal_reason'])
        meta['frequencies_report'] = per_frequency(args.output, args.frequencies)
        usable = all(item['usable'] for item in meta['frequencies_report'])
        if result['complete'] and usable:
            meta['complete'] = True
        elif not result['active_samples']:
            meta['reason'] = 'no active samples'
        elif not result['complete']:
            meta['reason'] = 'terminal before expected MCU span'
        else:
            meta['reason'] = 'insufficient usable response per frequency'
    except BaseException as error:
        meta['error'] = f'{type(error).__name__}: {error}'
        meta['reason'] = meta.get('reason') or f'error: {type(error).__name__}'
        raise
    finally:
        if client is not None:
            if start_attempted:
                # START 发出即视为可能已启动, 即使 ACK 丢失也必须尝试取消.
                try:
                    sweep_command(client, CANCEL, [], 3, 0.0)
                except BaseException as error:
                    meta['cancel_error'] = str(error)
                    meta['complete'] = False
            if client.session:
                try:
                    accepted(client.targets([], main_yaw_frame='local_inertial'),
                             'clear all targets')
                except BaseException as error:
                    meta['clear_targets_error'] = str(error)
                    meta['complete'] = False
                try:
                    ack = client.authorize(False, 'maintenance')
                    if ack.get('result', 0) != 0:
                        raise RuntimeError(f'revoke rejected: {ack}')
                except BaseException as error:
                    meta['revoke_error'] = str(error)
                    meta['complete'] = False
            if state.get('terminal'):
                # 取消/清目标/撤权之后核验实际 DM 状态, 不能只凭 ACK 认定已停机.
                try:
                    confirmation = confirm_disabled(client, state.get('cursor', 0))
                except BaseException as error:
                    confirmation = dict(confirmed=False, error=str(error))
                meta['disabled_confirmation'] = confirmation
                if not confirmation.get('confirmed'):
                    meta['complete'] = False
                    meta['reason'] = meta.get('reason') or 'DM disabled after terminal not confirmed'
        if link is not None:
            link.close()
        with report_path.open('x', encoding='utf-8') as handle:
            json.dump(meta, handle, ensure_ascii=False, allow_nan=False, indent=2)
            handle.write('\n')
    return meta


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--arm', action='store_true',
                        help='显式允许电机激励; 仍需遥控左下+右下维护组合')
    parser.add_argument('--target-heading', type=float, required=True,
                        help='操作者确认的当前世界航向 (rad); 必须是当前值附近')
    parser.add_argument('--frequencies', type=float, nargs='+', required=True)
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--amplitude-nm', type=float, default=0.01)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--elf', type=Path, help='可选: 记录固件身份')
    args = parser.parse_args()
    try:
        result = run(args)
    except (RuntimeError, ValueError, OSError, TimeoutError) as error:
        print(json.dumps({'complete': False, 'error': f'{type(error).__name__}: {error}'},
                         ensure_ascii=False, indent=2))
        return 1
    print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
    return 0 if result.get('complete') else 2


if __name__ == '__main__':
    sys.exit(main())

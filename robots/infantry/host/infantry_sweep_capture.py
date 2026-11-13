#!/usr/bin/env python3
"""步兵 APP USART1 只读 1 kHz 采样采集 (C 板 UART2, 921600 8N1).

本工具绝不 hello/authorize、不设置目标、不写参数、不使能电机, 也不启动 Sweep.
它只通过 infantry_client 既有的 UartFrame/CRC16 分帧请求环形缓存中的样本.
同一串口只允许一个进程占用, 不可与其他 GUI/主机客户端同时运行.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import struct
import time
from pathlib import Path
from statistics import median

import infantry_client as wire

REQUEST = 0x37
RESPONSE = 0xB7
SCHEMA = 1
HEADER = struct.Struct('<BBBBIII')
RECORD = struct.Struct('<III10fHBB')
FIELDS = (
    'sequence', 'timestamp_us', 'reference_epoch',
    'target_heading_rad', 'heading_rad', 'heading_rate_rad_s',
    'dm_position_rad', 'dm_velocity_rad_s', 'torque_command_nm',
    'torque_feedback_nm', 'speed_target_rad_s', 'speed_p_nm', 'speed_i_nm',
    'flags', 'reason', 'feedback_age_ms',
)
FLOAT_FIELDS = FIELDS[3:13]
# 与 infantry_sweep_capture.hpp 的 Flag 位一一对应; 有效性由 flags 表达, 不用 0/NaN 代替.
FLAG_NAMES = ('target_valid', 'heading_valid', 'dm_feedback_fresh', 'dm_enabled',
              'torque_submitted', 'pid_trace_valid', 'executing')
VALIDITY_FIELDS = {
    'target_heading_rad': 1 << 0,
    'heading_rad': 1 << 1, 'heading_rate_rad_s': 1 << 1,
    'dm_position_rad': 1 << 2, 'dm_velocity_rad_s': 1 << 2,
    'torque_feedback_nm': 1 << 2,
    'torque_command_nm': 1 << 4,
    'speed_target_rad_s': 1 << 5, 'speed_p_nm': 1 << 5, 'speed_i_nm': 1 << 5,
}
assert HEADER.size == 16 and RECORD.size == 56
assert HEADER.size + 4 * RECORD.size == 240

INFLIGHT = 4      # 固件固定 4 槽在途 FIFO: 同时未应答的请求上限
PAGE = 4          # 每页最多 4 条记录 (240 B 上限)
PREFILL_S = 0.016  # 16 ms ≈ 16 条 @1 kHz, 远小于 128 条 (128 ms) 环形深度
DRAIN_SETTLE_S = 0.1  # 结束后有界排空在途应答, 避免遗留 B7 被后续 HELLO 交换误读
RETRY_AFTER_S = 0.05  # 4 槽 FIFO 不覆盖旧应答: 丢应答必须短于 128 ms 环形深度就重发
TAIL_PROBE_S = 0.004  # 尾部空页探测限速: 4 ms ≈ 4 条, 远小于 128 条环形深度


class CaptureError(RuntimeError):
    pass


def _after(a: int, b: int) -> bool:
    """uint32 序号/时戳的模比较: a 在 b 之后 (容许自然回绕, 拒绝复位倒退)."""
    return 0 < ((a - b) & 0xFFFFFFFF) < 0x80000000


def summary_path(output: Path) -> Path:
    return Path(output).with_suffix(Path(output).suffix + '.summary.json')


def file_sha256(path) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 16), b''):
            digest.update(chunk)
    return digest.hexdigest()


def decode_page(payload: bytes, expected_op: int) -> dict:
    """校验页头边界与逐条序号; 不把错误帧按成功布局解码."""
    if len(payload) < HEADER.size:
        raise CaptureError('capture reply header truncated')
    version, kind, op, count, first, next_seq, earliest = HEADER.unpack_from(payload)
    if (version, kind, op) != (SCHEMA, RESPONSE, expected_op):
        raise CaptureError('capture schema, type, or op mismatch')
    if count > 4:
        raise CaptureError(f'page count {count} exceeds 4')
    if expected_op == 0 and (count or first != next_seq):
        raise CaptureError('status reply must be an empty page at next sequence')
    if len(payload) != HEADER.size + count * RECORD.size:
        raise CaptureError('capture payload length mismatch')
    if _after(first, next_seq) or _after(earliest, next_seq) or _after(earliest, first):
        raise CaptureError('invalid ring sequence window')
    if _after((first + count) & 0xFFFFFFFF, next_seq):
        raise CaptureError(f'page [{first},{first+count}) exceeds next {next_seq}')
    rows = [dict(zip(FIELDS, RECORD.unpack_from(payload, HEADER.size + i * RECORD.size)))
            for i in range(count)]
    for i, row in enumerate(rows):
        if row['sequence'] != (first + i) & 0xFFFFFFFF:
            raise CaptureError('record sequence mismatch')
    return dict(first=first, next=next_seq, earliest=earliest, rows=rows)


def validate_sample(row: dict) -> None:
    """仅当 flags 声称该字段有效时要求有限值; 无效字段保留原始值, 由 flags 说明."""
    for field, bit in VALIDITY_FIELDS.items():
        if row['flags'] & bit and not math.isfinite(row[field]):
            raise CaptureError(
                f'non-finite {field} marked valid at sequence {row["sequence"]}')


def request_page(transport, seq: int, op: int, cursor: int, timeout: float = 1.0) -> dict:
    payload = struct.pack('<BBI', SCHEMA, op, cursor)
    transport.send(wire.encode_frame(REQUEST, seq, payload))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = transport.receive(min(0.05, max(0.0, deadline - time.monotonic())))
        if frame is None:
            continue
        kind, reply_seq, body = frame
        if kind != RESPONSE or reply_seq != seq:
            # CRC 错误帧由 FrameDecoder 丢弃; CAL 文本不是二进制帧, 不能当作应答.
            continue
        return decode_page(body, op)
    raise CaptureError(f'capture request timeout (frame seq {seq})')


def _rejected(transport) -> int:
    return getattr(getattr(transport, '_decoder', None), 'rejected', 0)


def collect(transport, output, seconds: float, *, port=None, baud=None, identity=None,
            request_timeout: float = 1.0, stall_timeout: float = 1.0) -> dict:
    """4 页在途的只读流水线: 只请求已产生区间, 按 first/sequence 连续排空.

    status(op 0) 读起点 next, 预填 PREFILL_S 后作为首屏发出至多 4 个不同外层序号的
    分页请求 (cursor 步进 4). 之后只有已知 `next` 覆盖到的游标才发页请求, 不向未来游标盲发.
    """
    if not 0.0 < seconds <= 120.0:
        raise ValueError('seconds must be in (0, 120]')
    output = Path(output)
    summary_file = summary_path(output)
    if output.exists() or summary_file.exists():
        raise FileExistsError(f'will not overwrite existing output: '
                              f'{output if output.exists() else summary_file}')
    output.parent.mkdir(parents=True, exist_ok=True)

    meta = {'schema': SCHEMA, 'tool': 'infantry_sweep_capture',
            'port': port, 'baud': baud, 'requested_seconds': seconds,
            'complete': False, 'records': 0}
    if identity:
        meta.update(identity)

    frame_seq = 0
    cursor = 0
    started_cursor = None
    first_ts = last_ts = None
    last_epoch = None
    periods: list[int] = []
    flag_counts = {name: 0 for name in FLAG_NAMES}
    non_finite = {name: 0 for name in FLOAT_FIELDS}
    total = 0
    overrun = 0
    frame_errors = 0
    lost_replies = 0
    tail_probe_at = None  # 上次尾部探测时刻; 同一 next 上限制探测频率
    csv_file = summary_handle = None
    try:
        # 先原子预留两个输出, 出错时保留 CSV 与摘要证据, 不覆盖既有文件.
        csv_file = output.open('x', newline='', encoding='utf-8')
        summary_handle = summary_file.open('x', encoding='utf-8')
        writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
        writer.writeheader()
        csv_file.flush()

        started = time.monotonic()
        before = _rejected(transport)
        info = request_page(transport, frame_seq, 0, 0, request_timeout)
        frame_errors += max(0, _rejected(transport) - before)
        cursor = started_cursor = info['next']
        known_next = info['next']
        last_progress = time.monotonic()
        target_span_us = int(seconds * 1e6)
        wall_deadline = started + seconds * 1.5 + 2.0
        # 预填: 让环形缓存先积累十几条, 首屏就能发出 4 个各有数据的请求.
        time.sleep(min(PREFILL_S, max(0.0, wall_deadline - time.monotonic())))

        inflight: dict[int, dict] = {}   # outer sequence -> 待应答的页请求
        pending: dict[int, dict] = {}    # cursor -> 已收到但还不能按序落盘的页
        # 丢应答的重试窗口必须短于 stall_timeout 与环形深度, 否则会误报停摆或覆盖样本.
        retry_after = max(0.01, min(RETRY_AFTER_S, request_timeout, stall_timeout * 0.5))

        def request_pages(now: float) -> None:
            """只在 known_next 已覆盖的游标上补齐在途请求; 不向未来游标盲发."""
            nonlocal frame_seq, lost_replies, tail_probe_at
            # 固件 FIFO 4 槽: 超过 retry_after 未应答的请求视为丢失并让出槽位,
            # 该 cursor 之后会被重新请求 (分页读取幂等).
            for seq in [seq for seq, entry in inflight.items()
                        if now - entry['sent'] >= retry_after]:
                del inflight[seq]
                lost_replies += 1
            while len(inflight) < INFLIGHT:
                # 每页覆盖 [cursor, cursor+4): 已请求页覆盖到的游标不再重复请求.
                covered = [(entry['cursor']) for entry in inflight.values()]
                covered += [entry['first'] for entry in pending.values()]
                want = cursor
                for _ in range(INFLIGHT + PAGE):
                    hit = any(not _after(key, want) and _after(
                        (key + PAGE) & 0xFFFFFFFF, want) for key in covered)
                    if not hit:
                        break
                    want = (want + PAGE) & 0xFFFFFFFF
                else:
                    return
                if _after(want, known_next):
                    return  # 未产生的游标不盲发
                if want == known_next:
                    # 尾部探测保持单飞并限速: 空页是重新获知 next 的手段, 但同一 next 上
                    # 每毫秒重发只会把在途槽位浪费在空页上.
                    if any(entry['cursor'] == want for entry in inflight.values()):
                        return
                    if tail_probe_at is not None and now - tail_probe_at < TAIL_PROBE_S:
                        return
                    tail_probe_at = now
                frame_seq = (frame_seq + 1) & 0xFFFF
                inflight[frame_seq] = {'cursor': want, 'sent': now}
                transport.send(wire.encode_frame(
                    REQUEST, frame_seq, struct.pack('<BBI', SCHEMA, 1, want)))

        drain_deadline = None
        while True:
            span = 0 if first_ts is None else (last_ts - first_ts) & 0xFFFFFFFF
            finishing = first_ts is not None and span >= target_span_us
            if finishing:
                # 达到 MCU 目标时长: 不再新发页, 只做有界排空, 让遗留 B7 不落到后续会话.
                if not inflight:
                    break
                if drain_deadline is None:
                    drain_deadline = time.monotonic() + max(
                        DRAIN_SETTLE_S, min(request_timeout, stall_timeout))
                elif time.monotonic() >= drain_deadline:
                    break
            now = time.monotonic()
            if not finishing:
                if now >= wall_deadline:
                    raise CaptureError(f'capture wall-clock deadline: only {span}us of '
                                       f'{target_span_us}us MCU samples collected')
                if now - last_progress >= stall_timeout:
                    raise CaptureError(f'capture stalled: no new MCU samples for '
                                       f'{stall_timeout:.2f}s ({span}us of '
                                       f'{target_span_us}us collected)')
                request_pages(now)
            wait = max(0.001, min(request_timeout, wall_deadline - now))
            before = _rejected(transport)
            frame = transport.receive(wait)
            frame_errors += max(0, _rejected(transport) - before)
            if frame is None:
                continue
            kind, reply_seq, body = frame
            if kind != RESPONSE or reply_seq not in inflight:
                # 不是本次在途请求的应答 (旧帧/其他类型) 不能当作样本.
                continue
            entry = inflight.pop(reply_seq)
            page = decode_page(body, 1)
            for row in page['rows']:
                validate_sample(row)
            # 应答可能乱序返回: next 只按模最大值推进, 不把较早请求的较小 next 当作复位.
            # 真正的复位由后续的序号/时戳回归检查发现.
            if _after(page['next'], known_next):
                known_next = page['next']
            if page['first'] != entry['cursor']:
                if _after(page['first'], entry['cursor']):
                    overrun = (page['first'] - entry['cursor']) & 0xFFFFFFFF
                    raise CaptureError(
                        f'data gap: expected sequence {entry["cursor"]}, earliest '
                        f'retained {page["earliest"]}; host fell behind the ring by '
                        f'{overrun} records')
                # 固件把超出 next 的游标夹回; 该页无新数据, 等 next 增长后重发.
                continue
            if page['rows']:
                pending[entry['cursor']] = page
            else:
                # 空页 (尾部尚未凑满一页或还没产生样本): 稍后再试, 不计入进展.
                time.sleep(0.001)

            # 已经被更早的页覆盖到的待用页 (尾部不足一页导致游标未按 4 对齐) 直接丢弃.
            for stale in [key for key in pending if _after(cursor, key)]:
                del pending[stale]

            # 按游标顺序排空: 只有 cursor 对应的页才能写入 CSV.
            while cursor in pending:
                page = pending.pop(cursor)
                for row in page['rows']:
                    if row['sequence'] != cursor:
                        raise CaptureError('data sequence discontinuity')
                    if last_ts is not None:
                        dt = (row['timestamp_us'] - last_ts) & 0xFFFFFFFF
                        # 回绕表现为小正差; 复位/回退会得到大于半周期的模差.
                        if dt == 0 or dt >= 0x80000000:
                            raise CaptureError(
                                f'MCU timestamp regression/reset at sequence '
                                f'{row["sequence"]}: {dt}us')
                        periods.append(dt)
                    if last_epoch is not None and row['reference_epoch'] != last_epoch:
                        raise CaptureError('reference epoch changed during capture')
                    last_epoch = row['reference_epoch']
                    for name in FLOAT_FIELDS:
                        if not math.isfinite(row[name]):
                            non_finite[name] += 1
                    for bit, name in enumerate(FLAG_NAMES):
                        if row['flags'] & (1 << bit):
                            flag_counts[name] += 1
                    if first_ts is None:
                        first_ts = row['timestamp_us']
                    last_ts = row['timestamp_us']
                    writer.writerow(row)
                    cursor = (cursor + 1) & 0xFFFFFFFF
                    total += 1
                last_progress = time.monotonic()
                csv_file.flush()

        # 结束前把已经到达并排队的应答读干净; 不再发新请求, 也不等待尚未到达的应答.
        for _ in range(INFLIGHT):
            before = _rejected(transport)
            if transport.receive(0.0) is None:
                break
            frame_errors += max(0, _rejected(transport) - before)

        meta['complete'] = True
    except BaseException as error:
        meta['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        span_us = 0 if first_ts is None or last_ts is None else \
            (last_ts - first_ts) & 0xFFFFFFFF
        if periods:
            nominal = float(median(periods))
            gaps = [p for p in periods if p > 2.0 * nominal]
            timing = {
                'nominal_period_us': round(nominal, 3),
                'mean_period_us': round(sum(periods) / len(periods), 3),
                'min_period_us': min(periods),
                'max_period_us': max(periods),
                'gap_count': len(gaps),
                'max_gap_us': max(gaps) if gaps else 0,
                'gap_threshold_us': round(2.0 * nominal, 3),
            }
        else:
            timing = {'nominal_period_us': 0.0, 'mean_period_us': 0.0,
                      'min_period_us': 0, 'max_period_us': 0,
                      'gap_count': 0, 'max_gap_us': 0, 'gap_threshold_us': 0.0}
        meta.update(
            records=total,
            started_cursor=started_cursor,
            next_cursor=cursor if started_cursor is not None else None,
            mcu_span_s=round(span_us / 1e6, 6),
            mcu_rate_hz=round((total - 1) * 1e6 / span_us, 3) if total > 1 and span_us else 0.0,
            timing=timing,
            flags={name: {'count': count,
                          'fraction': round(count / total, 6) if total else 0.0}
                   for name, count in flag_counts.items()},
            non_finite=dict(non_finite),
            frame_rejections=frame_errors,
            lost_replies=lost_replies,
            overrun_records=overrun,
        )
        if summary_handle is not None:
            summary_handle.write(json.dumps(meta, ensure_ascii=False, indent=2) + '\n')
            summary_handle.close()
        if csv_file is not None:
            csv_file.close()
    return meta


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True, help='/dev/serial/by-id/... 设备路径')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--elf', type=Path, help='可选: 记录固件 ELF 的 SHA256 身份')
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 0.0 < args.seconds <= 120.0:
        parser.error('--seconds must be in (0, 120]')
    identity = {'elf': str(args.elf)} if args.elf else {}
    # 目标文件已存在时连串口都不打开, 避免无意义的端口占用.
    for path in (args.output, summary_path(args.output)):
        if path.exists():
            parser.error(f'{path} already exists; refusing to overwrite')
    transport = None
    try:
        if args.elf:
            identity['elf_sha256'] = file_sha256(args.elf)
        transport = wire.SerialTransport(args.serial, args.baud)
        result = collect(transport, args.output, args.seconds,
                         port=args.serial, baud=args.baud, identity=identity)
    except (CaptureError, OSError) as error:
        # 详细失败字段已写入 *.summary.json; 这里只报告本次异常, 不回读可能属于旧文件的摘要.
        print(json.dumps({'complete': False, 'error': f'{type(error).__name__}: {error}'},
                         ensure_ascii=False, indent=2))
        return 1
    finally:
        if transport is not None:
            transport.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print('Read-only collection complete. No PID/target/motor commands were sent.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

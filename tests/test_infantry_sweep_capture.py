"""步兵只读 Sweep 采集主机工具离线测试.

运行: .venv/bin/python -m unittest tests.test_infantry_sweep_capture
原生 golden 帧 (可选): 传 --native-golden <可执行文件>, 由 C++ 侧 --golden 输出一条真实 UartFrame.
所有合成缓存均为 SYNTHETIC 夹具: 不代表实机时序、带宽或控制行为, 不打开串口.
"""
from __future__ import annotations

import argparse
import collections
import contextlib
import csv
import io
import json
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robots' / 'infantry' / 'host'))
import infantry_client as wire
import infantry_sweep_capture as sweep


def golden_argument(argv: list[str]) -> tuple[Path | None, list[str]]:
    """可选 --native-golden PATH; 也接受单个位置路径, 其余交给 unittest."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--native-golden', type=Path)
    known, rest = parser.parse_known_args(argv)
    if known.native_golden:
        return known.native_golden, rest
    if rest and not rest[0].startswith('-') and Path(rest[0]).is_file():
        return Path(rest[0]), rest[1:]
    return None, rest


GOLDEN, sys.argv[1:] = golden_argument(sys.argv[1:])
MASK = 0xFFFFFFFF


def pack_record(sequence: int, timestamp_us: int, reference_epoch: int,
                floats: list[float] | tuple[float, ...] = (0.0,) * 10,
                flags: int = 0, reason: int = 0, feedback_age_ms: int = 0,
                sweep_run_id: int = 0, sweep_fields=(0.0, 0.0, 0.0),
                sweep_phase: int = 0, sweep_stop: int = 0) -> bytes:
    """SYNTHETIC schema 3 记录: '<III10fHBBI3fBB', 74 B."""
    assert len(floats) == 10 and len(sweep_fields) == 3
    return struct.pack('<III10fHBBI3fBB', sequence, timestamp_us, reference_epoch,
                       *floats, flags, reason, feedback_age_ms, sweep_run_id,
                       *sweep_fields, sweep_phase, sweep_stop)


def page(schema: int, op: int, count: int, first: int, next_seq: int,
         earliest: int, rows: bytes = b'') -> bytes:
    return struct.pack('<BBBBIII', schema, sweep.RESPONSE, op, count,
                       first, next_seq, earliest) + rows


def delta(a: int, b: int) -> int:
    """uint32 模差, 只在 SYNTHETIC 夹具内用于回绕比较."""
    return (a - b) & MASK


class FakeRing:
    """SYNTHETIC 1 kHz 环形缓存 (schema 3: 容量 128 条): 记录 74 B, 每页最多 3 条."""

    def __init__(self, *, start_seq: int = 0, start_us: int = 0, period_us: int = 1000,
                 epoch: int = 7, capacity: int = 128, flags: int = 0x7f,
                 limit: int | None = None, epoch_after: int | None = None,
                 epoch_new: int = 8, reset_after: int | None = None,
                 prefilled: int = 0, free_running: bool = False) -> None:
        self.next = start_seq
        self.start_us = start_us
        self.period_us = period_us
        self.epoch = epoch
        self.capacity = capacity
        self.flags = flags
        self.limit = limit
        self.epoch_after = epoch_after
        self.epoch_new = epoch_new
        self.reset_after = reset_after
        self.produced = 0
        self.free_running = free_running
        # 预填的记录视为已过去的时间, 否则时钟要等满 prefill 才继续产出.
        self.started = time.monotonic() - prefilled * period_us / 1e6
        self.records: collections.deque = collections.deque()
        self.produce(prefilled)

    def produce(self, count: int) -> None:
        for _ in range(count):
            if self.limit is not None and self.produced >= self.limit:
                return
            if self.reset_after is not None and self.produced >= self.reset_after:
                timestamp = 1000 + (self.produced - self.reset_after) * self.period_us
            else:
                timestamp = (self.start_us + self.produced * self.period_us) & MASK
            epoch = self.epoch_new if (self.epoch_after is not None
                                       and self.produced >= self.epoch_after) else self.epoch
            values = [self.produced + 0.25 * index for index in range(10)]
            self.records.append((self.next, pack_record(
                self.next, timestamp, epoch, values, self.flags, 3,
                self.produced % 250, sweep_run_id=5,
                sweep_fields=(9.0, 0.25, 0.1))))
            self.next = (self.next + 1) & MASK
            self.produced += 1
            while len(self.records) > self.capacity:
                self.records.popleft()

    def advance_clock(self, page_advance: int, jump: int) -> None:
        """补齐新记录: 自由运行按墙钟, 否则每页固定推进 (确定性夹具)."""
        if self.free_running:
            due = int((time.monotonic() - self.started) * 1e6 / self.period_us) + 1
            self.produce(max(0, due - self.produced) + jump)
        else:
            self.produce(page_advance + jump)

    def answer(self, message_type: int, payload: bytes, sequence: int,
               advance: int, jump: int) -> bytes | None:
        if message_type != sweep.REQUEST or len(payload) != 6:
            return b''
        version, op, cursor = struct.unpack('<BBI', payload)
        if version != sweep.SCHEMA or op not in (0, 1):
            return b''
        if op == 1:  # status 请求本身不产生样本
            self.advance_clock(advance, jump)
        earliest = self.records[0][0] if self.records else self.next
        if op == 0:
            start, rows = self.next, []
        else:
            # cursor 失效 (早于 earliest 或越过 next) 时从 earliest 重新对齐.
            ahead = 0 < delta(cursor, self.next) < 0x80000000
            stale = 0 < delta(earliest, cursor) < 0x80000000
            start = earliest if (ahead or stale) else cursor
            span = delta(self.next, start)
            rows = [record for seq, record in self.records
                    if delta(seq, start) < span][:sweep.PAGE]
        return struct.pack('<BBBBIII', sweep.SCHEMA, sweep.RESPONSE, op, len(rows),
                           start, self.next, earliest) + b''.join(rows)


def bad_crc_frame(body: bytes, sequence: int) -> bytes:
    """转义正确但 CRC 故意错误的应答帧, 由真实 FrameDecoder 判为 rejected."""
    raw = bytes([wire.PROTOCOL_VERSION, sweep.RESPONSE,
                 sequence & 0xFF, sequence >> 8,
                 len(body) & 0xFF, len(body) >> 8]) + body + b'\xde\xad'
    out = bytearray([wire.FRAME_START])
    for byte in raw:
        out += bytes([wire.FRAME_ESCAPE, byte ^ wire.ESCAPE_XOR]) \
            if byte in (wire.FRAME_START, wire.FRAME_ESCAPE) else bytes([byte])
    out.append(wire.FRAME_START)
    return bytes(out)


class FakeTransport:
    """SYNTHETIC 固件: 固定 4 槽在途 FIFO, 每周期最多回 1 页, 真实 FrameDecoder/encode_frame 往返.

    `reorder` 让应答乱序返回, `drop_after_pages` 丢一次应答 (模拟 FIFO 槽满/线路丢帧),
    `ignore_requests` 完全不回. 这些都不改变线协议与记录布局.
    """

    def __init__(self, ring: FakeRing, advance: int = 4, jump_after_pages: int = 0,
                 jump: int = 0, corrupt_first: bool = False,
                 ignore_requests: bool = False, reorder: bool = False,
                 drop_after_pages: int | None = None,
                 max_inflight: int = sweep.INFLIGHT) -> None:
        self.ring = ring
        self.advance = advance
        self.jump_after_pages = jump_after_pages
        self.jump = jump
        self.corrupt_first = corrupt_first
        self.ignore_requests = ignore_requests
        self.reorder = reorder
        self.drop_after_pages = drop_after_pages
        self.max_inflight = max_inflight
        self.pages = 0
        self.held: list[tuple[bytes, int]] = []
        self.inflight = 0
        self.max_inflight_seen = 0
        # 与 SerialTransport 一致: _decoder 是接收侧解码器 (rejected 计接收 CRC 失败).
        self._request = wire.FrameDecoder()
        self._decoder = wire.FrameDecoder()
        self._inbox: collections.deque = collections.deque()
        self.sent: list[bytes] = []
        self.outgoing: list[tuple[int, int, bytes]] = []
        self.page_requests: list[tuple[int, int]] = []  # (cursor, 请求时已产生条数)

    def send(self, message: bytes) -> None:
        self.sent.append(message)
        for byte in message:
            frame = self._request.push(byte)
            if frame is not None:
                self._handle(*frame)

    def _queue(self, frame: bytes) -> None:
        for byte in frame:
            decoded = self._decoder.push(byte)
            if decoded is not None:
                self._inbox.append(decoded)

    def _reply(self, body: bytes, sequence: int) -> None:
        if self.corrupt_first:
            # 先来一帧 CRC 错误: 真实 FrameDecoder 丢弃并计入 rejected, 合法页紧随其后.
            self.corrupt_first = False
            for byte in bad_crc_frame(body, sequence):
                self._decoder.push(byte)
        self._queue(wire.encode_frame(sweep.RESPONSE, sequence, body))

    def _handle(self, message_type: int, sequence: int, payload: bytes) -> None:
        self.outgoing.append((message_type, sequence, payload))
        if message_type != sweep.REQUEST or len(payload) != 6:
            return
        op = payload[1]
        if op == 1:
            cursor = struct.unpack_from('<I', payload, 2)[0]
            self.page_requests.append((cursor, self.ring.next))
            self.inflight += 1
            self.max_inflight_seen = max(self.max_inflight_seen, self.inflight)
            if self.inflight > self.max_inflight:
                # 协议允许的最多 4 个在途请求被突破: 直接失败, 不伪造更多槽位.
                raise AssertionError(f'inflight {self.inflight} > {self.max_inflight}')
            self.pages += 1
            if self.drop_after_pages is not None and self.pages > self.drop_after_pages:
                self.drop_after_pages = None
                self.inflight -= 1
                return  # 该请求被固件接受但应答丢失
            if self.ignore_requests:
                return
            jump = 0
            if self.jump_after_pages and self.pages > self.jump_after_pages:
                jump, self.jump = self.jump, 0
        else:
            if self.ignore_requests:
                return
            jump = 0
        body = self.ring.answer(message_type, payload, sequence, self.advance, jump)
        if not body:
            return
        if self.reorder and op == 1:
            # 乱序: 每个应答保留自己的外层序号, 主机的 frame_seq 配对必须仍然成立.
            self.held.append((body, sequence))
            if len(self.held) < 2:
                return
            for held_body, held_seq in reversed(self.held):
                self._reply(held_body, held_seq)
            self.held.clear()
        else:
            self._reply(body, sequence)

    def receive(self, timeout_s: float = 0.0):
        if not self._inbox and self.held:
            # 固件每周期至多回 1 页: 主机的下一次轮询就足以让暂存的应答全部出队.
            for held_body, held_seq in reversed(self.held):
                self._reply(held_body, held_seq)
            self.held.clear()
        if self._inbox:
            self.inflight = max(0, self.inflight - 1)
            return self._inbox.popleft()
        return None

    def close(self) -> None:
        pass


class PacketTests(unittest.TestCase):
    def test_layout(self):
        # schema 3: 74 B 记录, 页上限 3 条 = 238 B.
        self.assertEqual(sweep.SCHEMA, 3)
        self.assertEqual(sweep.RECORD.size, 74)
        self.assertEqual(sweep.PAGE, 3)
        self.assertEqual(sweep.RECORD.format, '<III10fHBBI3fBB')
        self.assertEqual(sweep.HEADER.size + 3 * sweep.RECORD.size, 238)
        self.assertEqual(sweep.FIELDS[-6:], ('sweep_run_id', 'sweep_hz',
                                            'requested_excitation_nm',
                                            'applied_excitation_nm', 'sweep_phase', 'sweep_stop'))
        self.assertEqual(sweep.FLAG_NAMES[7:],
                         ('sweep_running', 'sweep_inhibited', 'sweep_angle'))
        # 新字段追加在记录末尾, 有效位上限仍为 0x3ff.
        self.assertLessEqual((1 << len(sweep.FLAG_NAMES)) - 1, 0x3FF)

    def test_page(self):
        row = pack_record(4, 1000, 10, [1.5] * 10, 0b111, 13, 2, 9, (40.0, 0.3, 0.2), 3, 5)
        decoded = sweep.decode_page(page(3, 1, 1, 4, 5, 4, row), 1)
        self.assertEqual(decoded['rows'][0]['sequence'], 4)
        self.assertAlmostEqual(decoded['rows'][0]['dm_velocity_rad_s'], 1.5)
        self.assertEqual(decoded['rows'][0]['feedback_age_ms'], 2)
        self.assertEqual(decoded['rows'][0]['sweep_run_id'], 9)
        self.assertEqual(decoded['rows'][0]['reason'], 13)
        self.assertEqual(decoded['rows'][0]['sweep_phase'], 3)
        self.assertEqual(decoded['rows'][0]['sweep_stop'], 5)
        self.assertAlmostEqual(decoded['rows'][0]['sweep_hz'], 40.0)
        self.assertAlmostEqual(decoded['rows'][0]['applied_excitation_nm'], 0.2)

    def test_reject_bad_size(self):
        with self.assertRaises(sweep.CaptureError):
            sweep.decode_page(page(3, 1, 1, 4, 5, 4), 1)
        with self.assertRaises(sweep.CaptureError):  # 4 条已超出 schema 3 的页上限
            sweep.decode_page(page(3, 1, 4, 4, 8, 4, pack_record(4, 0, 10)), 1)

    def test_reject_gap(self):
        # 页内记录序号出现空洞: 声称 first=8 但记录实际是 9.
        row = pack_record(9, 1000, 10)
        with self.assertRaises(sweep.CaptureError):
            sweep.decode_page(page(3, 1, 1, 8, 9, 4, row), 1)
        # 页头窗口本身越界同样拒绝.
        with self.assertRaises(sweep.CaptureError):
            sweep.decode_page(page(3, 1, 1, 8, 9, 10, pack_record(8, 1000, 10)), 1)

    def test_reject_bounds_and_identity(self):
        empty = page(3, 1, 0, 4, 4, 4)
        self.assertEqual(sweep.decode_page(empty, 1)['next'], 4)
        with self.assertRaises(sweep.CaptureError):  # first 4 < earliest 5
            sweep.decode_page(page(3, 1, 0, 4, 4, 5), 1)
        with self.assertRaises(sweep.CaptureError):  # 声称的 count 超出 next
            sweep.decode_page(page(3, 1, 2, 5, 4, 0), 1)
        with self.assertRaises(sweep.CaptureError):  # op 不符
            sweep.decode_page(empty, 0)
        with self.assertRaises(sweep.CaptureError):  # schema 1 已过期
            sweep.decode_page(page(1, 1, 0, 4, 4, 4), 1)

    def test_old_schema_and_all_stop_reasons(self):
        # 旧协议即使空页也明确拒绝, 不误将 APP reason 当成 Sweep stop.
        with self.assertRaises(sweep.CaptureError):
            sweep.decode_page(page(2, 1, 0, 4, 4, 4), 1)
        for phase, stop in [(2, 0)] + [(3, stop) for stop in range(1, 10)]:
            raw = pack_record(4, 1000, 10, reason=13,
                              sweep_phase=phase, sweep_stop=stop)
            row = sweep.decode_page(page(3, 1, 1, 4, 5, 4, raw), 1)['rows'][0]
            self.assertEqual((row['reason'], row['sweep_phase'], row['sweep_stop']),
                             (13, phase, stop))

    def test_status_page_must_be_empty_at_next(self):
        ok = page(3, 0, 0, 9, 9, 2)
        self.assertEqual(sweep.decode_page(ok, 0)['next'], 9)
        with self.assertRaises(sweep.CaptureError):
            sweep.decode_page(page(3, 0, 1, 9, 9, 2, pack_record(9, 0, 1)), 0)

    def test_validity_bits_guard_floats(self):
        row = dict.fromkeys(sweep.FIELDS, 0)
        row.update(flags=0, target_heading_rad=float('nan'),
                   torque_feedback_nm=float('inf'))
        sweep.validate_sample(row)  # 无效字段保留原始值, 由 flags 说明
        row['target_heading_rad'] = 1.0
        self.assertEqual(row['torque_feedback_nm'], float('inf'))
        row['flags'] = (1 << 0) | (1 << 2)  # target + dm_feedback 均声称有效
        with self.assertRaises(sweep.CaptureError):
            sweep.validate_sample(row)  # torque_feedback_nm 为 inf 却标为 fresh


class BaselineTests(unittest.TestCase):
    """原始包内既有用例: 页解析与错误拒绝."""

    def test_status_and_page_decode(self):
        decoded = sweep.decode_page(page(
            3, 1, 2, 126, 130, 2,
            b''.join(pack_record(seq, seq * 1000, 1,
                                 [seq + i * 0.5 for i in range(10)], 0x7f, 0, 1)
                     for seq in (126, 127))), 1)
        self.assertEqual([row['sequence'] for row in decoded['rows']], [126, 127])
        self.assertEqual((decoded['first'], decoded['next'], decoded['earliest']),
                         (126, 130, 2))


class GoldenTests(unittest.TestCase):
    def test_native_golden_frame(self):
        if GOLDEN is None:
            self.skipTest('未提供 --native-golden 可执行文件')
        raw = bytes.fromhex(subprocess.check_output(
            [str(GOLDEN), '--golden'], text=True).strip())
        decoder = wire.FrameDecoder()
        frames = [frame for frame in (decoder.push(byte) for byte in raw)
                  if frame is not None]
        self.assertEqual(len(frames), 1)
        message_type, sequence, body = frames[0]
        self.assertEqual((message_type, sequence), (sweep.RESPONSE, 0x7e7d))
        decoded = sweep.decode_page(body, 1)
        self.assertEqual(decoded['next'], 134)
        self.assertEqual(decoded['earliest'], 6)
        self.assertEqual(decoded['first'], 6)
        self.assertEqual([row['sequence'] for row in decoded['rows']], [6, 7, 8])
        self.assertAlmostEqual(decoded['rows'][0]['target_heading_rad'], 3.25)
        strides = {b['timestamp_us'] - a['timestamp_us']
                   for a, b in zip(decoded['rows'], decoded['rows'][1:])}
        self.assertEqual(strides, {1000})  # 相邻样本固定 1 ms
        values = [row['heading_rad'] for row in decoded['rows']]
        self.assertEqual(len(set(values)), len(values))
        for row in decoded['rows']:
            self.assertTrue(row['flags'] & 0x7f)
            self.assertEqual((row['reason'], row['sweep_phase'], row['sweep_stop']),
                             (13, 3, 5))
            sweep.validate_sample(row)
        self.assertEqual(decoder.rejected, 0)


class CollectTests(unittest.TestCase):
    def collect(self, ring, seconds=0.01, **kwargs):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        transport = FakeTransport(ring, **kwargs)
        output = Path(directory.name) / 'capture.csv'
        meta = sweep.collect(transport, output, seconds,
                             request_timeout=0.2, stall_timeout=0.2)
        return transport, output, meta

    def test_happy_1khz_contiguous(self):
        transport, output, meta = self.collect(FakeRing(), seconds=0.05)
        self.assertTrue(meta['complete'])
        self.assertGreaterEqual(meta['records'], 51)
        self.assertAlmostEqual(meta['mcu_rate_hz'], 1000.0, places=3)
        self.assertEqual(meta['timing']['nominal_period_us'], 1000.0)
        self.assertEqual(meta['timing']['gap_count'], 0)
        self.assertEqual(meta['overrun_records'], 0)
        self.assertEqual(meta['frame_rejections'], 0)
        self.assertGreaterEqual(meta['mcu_span_s'], 0.05)
        self.assertAlmostEqual(meta['flags']['target_valid']['fraction'], 1.0)
        self.assertTrue(all(count == 0 for count in meta['non_finite'].values()))
        with output.open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), meta['records'])
        self.assertEqual([int(row['sequence']) for row in rows],
                         [(meta['started_cursor'] + i) & MASK
                          for i in range(meta['records'])])
        summary = json.loads(sweep.summary_path(output).read_text(encoding='utf-8'))
        self.assertEqual(summary['records'], meta['records'])
        self.assertTrue(summary['complete'])
        self.assertEqual(summary['port'], None)

    def test_only_read_requests_are_sent(self):
        transport, _, meta = self.collect(FakeRing(), seconds=0.02)
        self.assertTrue(meta['complete'])
        self.assertTrue(transport.outgoing)
        for message_type, _, payload in transport.outgoing:
            self.assertEqual(message_type, sweep.REQUEST)
            self.assertEqual(len(payload), 6)
            self.assertEqual(payload[0], sweep.SCHEMA)
            self.assertIn(payload[1], (0, 1))
        self.assertEqual(sum(1 for _, _, payload in transport.outgoing if payload[1] == 0), 1)

    def test_bad_crc_reply_is_rejected_and_counted(self):
        transport, _, meta = self.collect(FakeRing(), seconds=0.02, corrupt_first=True)
        self.assertTrue(meta['complete'])
        self.assertEqual(meta['frame_rejections'], 1)
        self.assertEqual(meta['timing']['gap_count'], 0)

    def test_initialization_timeout_keeps_error_summary(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'noinit.csv'
        with self.assertRaises(sweep.CaptureError) as caught:
            sweep.collect(FakeTransport(FakeRing(), ignore_requests=True), path, 0.05,
                          request_timeout=0.05, stall_timeout=0.05)
        self.assertIn('timeout', str(caught.exception))
        summary = json.loads(sweep.summary_path(path).read_text(encoding='utf-8'))
        self.assertFalse(summary['complete'])
        self.assertEqual(summary['records'], 0)
        self.assertIsNone(summary['started_cursor'])
        self.assertIn('timeout', summary['error'])
        self.assertEqual(path.read_text(encoding='utf-8').splitlines(), [','.join(sweep.FIELDS)])

    def test_zero_data_times_out_with_error_summary(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'empty.csv'
        with self.assertRaises(sweep.CaptureError):
            sweep.collect(FakeTransport(FakeRing(limit=0)), path, 0.2,
                          request_timeout=0.2, stall_timeout=0.05)
        summary = json.loads(sweep.summary_path(path).read_text(encoding='utf-8'))
        self.assertFalse(summary['complete'])
        self.assertEqual(summary['records'], 0)
        self.assertIn('stalled', summary['error'])
        self.assertTrue(path.exists())

    def test_short_mcu_coverage_cannot_complete(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'short.csv'
        with self.assertRaises(sweep.CaptureError) as caught:
            sweep.collect(FakeTransport(FakeRing(limit=20)), path, 0.2,
                          request_timeout=0.2, stall_timeout=0.05)
        self.assertIn('stalled', str(caught.exception))
        summary = json.loads(sweep.summary_path(path).read_text(encoding='utf-8'))
        self.assertFalse(summary['complete'])
        self.assertEqual(summary['records'], 20)
        self.assertLess(summary['mcu_span_s'], 0.2)

    def test_ring_overrun_is_reported_as_gap(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'gap.csv'
        with self.assertRaises(sweep.CaptureError) as caught:
            # 覆盖 128 条环形容量 (schema 3): 一次跳跃必须超过 128 条才产生真实缺口.
            sweep.collect(FakeTransport(FakeRing(), jump_after_pages=1, jump=200),
                          path, 0.5, request_timeout=0.2, stall_timeout=0.2)
        self.assertIn('data gap', str(caught.exception))
        summary = json.loads(sweep.summary_path(path).read_text(encoding='utf-8'))
        self.assertFalse(summary['complete'])
        self.assertGreater(summary['overrun_records'], 0)

    def test_reference_epoch_change_fails(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'epoch.csv'
        with self.assertRaises(sweep.CaptureError) as caught:
            sweep.collect(FakeTransport(FakeRing(epoch_after=20)), path, 0.5,
                          request_timeout=0.2, stall_timeout=0.2)
        self.assertIn('reference epoch', str(caught.exception))

    def test_timestamp_regression_is_detected(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'reset.csv'
        with self.assertRaises(sweep.CaptureError) as caught:
            sweep.collect(FakeTransport(FakeRing(reset_after=20)), path, 0.5,
                          request_timeout=0.2, stall_timeout=0.2)
        self.assertIn('timestamp', str(caught.exception))

    def test_no_valid_target_still_completes_but_is_reported(self):
        # 停机状态样本可以完整采集, 但 target_valid 比例必须如实为 0, 不与完整性混淆.
        transport, output, meta = self.collect(FakeRing(flags=0), seconds=0.02)
        self.assertTrue(meta['complete'])
        self.assertEqual(meta['flags']['target_valid']['fraction'], 0.0)
        self.assertEqual(meta['flags']['dm_feedback_fresh']['count'], 0)

    def test_sequence_wrap_is_allowed(self):
        ring = FakeRing(start_seq=0xFFFFFFFE)
        transport, output, meta = self.collect(ring, seconds=0.02)
        self.assertTrue(meta['complete'])
        self.assertGreaterEqual(meta['records'], 21)
        with output.open(newline='', encoding='utf-8') as stream:
            sequences = [int(row['sequence']) for row in csv.DictReader(stream)]
        self.assertEqual(sequences[0], 0xFFFFFFFE)
        self.assertEqual(sequences[:10], [0xFFFFFFFE, 0xFFFFFFFF, 0, 1, 2, 3, 4, 5, 6, 7])
        self.assertEqual(meta['timing']['gap_count'], 0)

    def test_timestamp_wrap_is_allowed(self):
        ring = FakeRing(start_us=0xFFFFFFFF - 1200)
        transport, output, meta = self.collect(ring, seconds=0.02)
        self.assertTrue(meta['complete'])
        self.assertGreaterEqual(meta['records'], 21)
        self.assertAlmostEqual(meta['mcu_span_s'], (meta['records'] - 1) / 1000.0)
        self.assertEqual(meta['timing']['gap_count'], 0)

    def test_reject_existing_outputs(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        output = Path(directory.name) / 'capture.csv'
        output.write_text('sentinel\n', encoding='utf-8')
        transport = FakeTransport(FakeRing())
        with self.assertRaises(FileExistsError):
            sweep.collect(transport, output, 0.01)
        self.assertEqual(output.read_text(encoding='utf-8'), 'sentinel\n')
        self.assertEqual(transport.sent, [])
        sweep.summary_path(output).write_text('{}\n', encoding='utf-8')
        with self.assertRaises(FileExistsError):
            sweep.collect(FakeTransport(FakeRing()), output, 0.01)

    def test_seconds_range(self):
        transport = FakeTransport(FakeRing())
        with self.assertRaises(ValueError):
            sweep.collect(transport, Path('/nonexistent/x.csv'), 0.0)

    def pipeline(self, seconds=0.05, **kwargs):
        ring = FakeRing(free_running=True, prefilled=16)
        transport = FakeTransport(ring, **kwargs)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        output = Path(directory.name) / 'pipe.csv'
        meta = sweep.collect(transport, output, seconds, request_timeout=0.2,
                             stall_timeout=0.3)
        return transport, output, meta

    def test_pipeline_keeps_four_inflight_and_never_blind_requests(self):
        # SYNTHETIC 4 槽 FIFO: 在途恒不超 4, cursor 只按已知 next 推进, 不请求未来游标.
        transport, _, meta = self.pipeline()
        self.assertTrue(meta['complete'])
        self.assertEqual(transport.max_inflight_seen, sweep.INFLIGHT)
        cursors = [cursor for cursor, _ in transport.page_requests]
        for cursor, produced in transport.page_requests:
            self.assertLessEqual(cursor, produced, '不得向尚未产生的游标发页请求')
        # 只有"尾部空页探测"可能重复同一游标 (空页是重新获知 next 的手段);
        # 已返回过整页的游标绝不再请求.
        repeats = {c for c in cursors if cursors.count(c) > 1}
        for cursor, produced in transport.page_requests:
            if cursor in repeats:
                self.assertEqual(cursor, produced, f'已产生游标 {cursor} 被重复请求')
        # 游标单调前进, 每次最多跨一页; 部分页之后步进会小于 PAGE, 这是正常进展而非跳跃.
        steps = [delta(b, a) for a, b in zip(cursors, cursors[1:])]
        self.assertTrue(all(0 < step <= sweep.PAGE for step in steps), steps)
        self.assertEqual(meta['lost_replies'], 0)
        self.assertEqual(meta['overrun_records'], 0)
        self.assertEqual(meta['timing']['gap_count'], 0)

    def test_pipeline_reassembles_out_of_order_replies(self):
        transport, output, meta = self.pipeline(reorder=True)
        self.assertTrue(meta['complete'])
        self.assertEqual(transport.max_inflight_seen, sweep.INFLIGHT)
        with output.open(newline='', encoding='utf-8') as stream:
            sequences = [int(row['sequence']) for row in csv.DictReader(stream)]
        self.assertEqual(sequences, list(range(meta['started_cursor'],
                                               meta['started_cursor'] + len(sequences))))
        self.assertEqual(meta['timing']['gap_count'], 0)

    def test_pipeline_recovers_from_dropped_reply(self):
        transport, _, meta = self.pipeline(seconds=0.1, drop_after_pages=3)
        self.assertTrue(meta['complete'])
        self.assertGreaterEqual(meta['lost_replies'], 1)
        self.assertEqual(meta['overrun_records'], 0)
        self.assertEqual(meta['timing']['gap_count'], 0)

    def test_pipeline_drains_leftover_replies_after_target(self):
        # 达到 MCU 目标时长后不再新发页, 但必须把在途应答排空, 否则后续 HELLO 会读到遗留 B7.
        transport, _, meta = self.pipeline(seconds=0.03)
        self.assertTrue(meta['complete'])
        self.assertEqual(len(transport._inbox), 0, '结束时不得留下未读的 B7 应答')
        self.assertEqual(transport.inflight, 0)
        self.assertEqual(len(transport.page_requests) + 1, len(transport.outgoing),
                         '除 1 个 status 请求外, 发出的帧必须都是已跟踪的分页请求')
        # 排空阶段不应继续追逐 next: 请求数与"已写出的页数"同量级, 而非无限增长.
        self.assertLessEqual(len(transport.page_requests), meta['records'] // sweep.PAGE + 4)

    def test_pipeline_overrun_refusal_keeps_evidence(self):
        # 主机彻底落后于 128 条环形深度时, 必须报 gap 而不是静默续采.
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        output = Path(directory.name) / 'overrun.csv'
        ring = FakeRing(free_running=True, prefilled=16)
        transport = FakeTransport(ring)
        transport.send = self._stalling_send(transport, ring)
        with self.assertRaises(sweep.CaptureError) as caught:
            sweep.collect(transport, output, 0.3, request_timeout=0.2, stall_timeout=0.3)
        self.assertIn('data gap', str(caught.exception))
        summary = json.loads(sweep.summary_path(output).read_text(encoding='utf-8'))
        self.assertFalse(summary['complete'])
        self.assertGreater(summary['overrun_records'], 0)
        self.assertIn('data gap', summary['error'])

    @staticmethod
    def _stalling_send(transport, ring):
        """让主机每发一页就停 200 ms: 期间环形缓存跑满并覆盖未读记录."""
        original = transport.send

        def send(message):
            original(message)
            time.sleep(0.2)

        return send

    def test_cli_records_port_baud_and_elf_identity(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        elf = Path(directory.name) / 'app.elf'
        elf.write_bytes(b'SYNTHETIC ELF')
        output = Path(directory.name) / 'cli.csv'
        opened = []

        def fake_transport(port, baud):
            opened.append((port, baud))
            return FakeTransport(FakeRing())

        original = sweep.wire.SerialTransport
        sweep.wire.SerialTransport = fake_transport
        self.addCleanup(setattr, sweep.wire, 'SerialTransport', original)
        with contextlib.redirect_stdout(io.StringIO()):
            code = sweep.main(['--serial', '/dev/synthetic', '--seconds', '0.01',
                               '--output', str(output), '--elf', str(elf)])
        self.assertEqual(code, 0)
        self.assertEqual(opened, [('/dev/synthetic', 921600)])
        meta = json.loads(sweep.summary_path(output).read_text(encoding='utf-8'))
        self.assertEqual((meta['port'], meta['baud']), ('/dev/synthetic', 921600))
        self.assertEqual(meta['elf_sha256'], sweep.file_sha256(elf))
        self.assertTrue(meta['complete'])
        # 已存在的 CSV 不会被覆盖, 也不打开串口.
        opened.clear()
        before = output.read_text(encoding='utf-8')
        with contextlib.redirect_stderr(io.StringIO()), \
                self.assertRaises(SystemExit) as caught:
            sweep.main(['--serial', '/dev/synthetic', '--seconds', '0.01',
                        '--output', str(output)])
        self.assertEqual(caught.exception.code, 2)
        self.assertEqual(opened, [])
        self.assertEqual(output.read_text(encoding='utf-8'), before)


if __name__ == '__main__':
    unittest.main()

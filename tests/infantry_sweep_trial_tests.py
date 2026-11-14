"""受监督激励试验的离线测试: 真实 0x7e 分帧 + 伪 MCU 应答, 不打开串口.

覆盖: 单飞链路不再缺 exchange; --arm 与计划校验先于打开传输; 输出不覆盖;
完整流程保持心跳; START ACK 丢失仍取消/清目标/撤权; run id/参考代次/时戳断档;
提前 terminal 不等于正常完成; 全程不写 PID 或其他参数.
"""

import importlib
import json
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robots/infantry/host'))
import infantry_client as wire
import infantry_sweep_trial as trial
import infantry_sweep_capture as capture

SESSION = 0x5A5A
RUN_ID = 0x1234
START_CURSOR = 100
FREQUENCIES = [3.0, 4.0]
CYCLES = 3
AMPLITUDE = 0.01


def _ack(mtype, sequence, result=0):
    """ACK body 必须原样带回请求的 session/sequence, 否则客户端身份校验会拒绝."""
    return bytes([3, mtype]) + struct.pack('<IIBB', SESSION, sequence, result, 0)


def _status0(seq, heading=0.5, authority=True):
    body = bytes([3, wire.STATUS_SNAPSHOT]) + struct.pack(
        '<IIBIII', SESSION, seq, 0, 3, 1, 1000)
    body += struct.pack('<III', 10, 20, 77)
    # bit0 = authority, bit3 = locally_ready.
    body += struct.pack('<BBB', 3, 8 | (1 if authority else 0), 0)
    body += struct.pack('<BBBII', 3, 0, 1, 1, 1)      # 主 yaw 参考有效
    for _ in range(3):
        body += struct.pack('<ffIIB', 0.0, 0.0, 10, 10, 3)
    body += struct.pack('<ffff', 1.0, 0.0, 0.0, 0.0)
    body += struct.pack('<fff', 0.0, 0.0, 0.0)
    body += struct.pack('<fff', heading, 0.0, 1.0)
    body += struct.pack('<I', 123)
    body += struct.pack('<III', 10, 10, 10)
    body += struct.pack('<III', 5, 5, 5)
    body += struct.pack('<BB', 3, 7)
    body += struct.pack('<fff', 0.0, 0.0, 0.0)
    body += struct.pack('<fff', 0.0, 0.0, 0.0)
    body += struct.pack('<IIfI', 10, 0, 0.0, 0)
    body += struct.pack('<f', 1.0)
    return body


def _status1(seq, execute=1):
    body = bytes([3, wire.STATUS_SNAPSHOT]) + struct.pack('<IIBIII', SESSION, seq, 1, 3, 1, 1000)
    body += struct.pack('<BBBBB', 3, 1, 0, 1, 10)
    for domain in wire.DOMAINS:
        if domain == 'main_yaw':
            # requested_frame/adopted_frame 都是 local_inertial, 测量有效.
            frames = 64 | 1 | (1 << 2) | (1 << 4) | 128
            body += struct.pack('<BBBBBffII', 3, 3, execute, 0, frames, 0.5, 0.5, 10, 10)
        else:
            body += struct.pack('<BBBBBffII', 1, 0, 0, 1, 0, 0.0, 0.0, 10, 10)
    return body


def _diagnostics1(seq):
    """DM 在线但未使能: 终态锁存后允许关机, START arming 才重新使能."""
    body = bytes([3, wire.DIAGNOSTICS_PAGE]) + struct.pack('<IIBB', SESSION, seq, 1, 10)
    for domain in wire.DOMAINS:
        flags = 1 | 2 if domain == 'main_yaw' else 1
        body += struct.pack('<BBBBHHIffBB', wire.DOMAINS.index(domain) if domain == 'main_yaw' else 0,
                            flags, 0, 0, 0, 0, 5, 0.0, 0.0, 0, 0)
    return body


def _row(sequence, ts, hz, flags=0x8F, run_id=RUN_ID, epoch=1):
    return struct.pack(
        '<III10fHBBI3f', sequence, ts, epoch,
        0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        flags, 0, 5, run_id, hz, AMPLITUDE, AMPLITUDE)


def _rows(mutate=None):
    """按频点生成与 MCU 时长一致的 250 Hz 记录, 末尾一条 terminal."""
    rows, ts = [], 0
    for hz in FREQUENCIES:
        for _ in range(round(CYCLES / hz * 250)):
            rows.append((ts, _row(START_CURSOR + len(rows), ts, hz)))
            ts += 4000
    rows.append((ts, _row(START_CURSOR + len(rows), ts, 0.0, flags=0x100)))
    if mutate is not None:
        mutate(rows)
    return rows


class FakeDevice:
    """最小伪 MCU: 收真实编码帧, 按业务语义回真实编码应答; 记录调用."""

    def __init__(self, rows=None, drop_start_ack=False):
        self.rows = _rows() if rows is None else rows
        self.drop_start_ack = drop_start_ack
        self.frames = []
        self.written_types = []
        self.authorize_grants = 0
        self.authorized = False
        self.running = False
        self.has_started = False
        self.targets_sent = 0
        self.closed = False
        self._tail_us = None
        self._decoder = wire.FrameDecoder()

    def send(self, message):
        for byte in message:
            frame = self._decoder.push(byte)
            if frame is not None:
                self._handle(*frame)

    def receive(self, timeout_s=0.0):
        deadline = time.monotonic() + timeout_s
        while not self.frames:
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.0005)
        return self.frames.pop(0)

    def close(self):
        self.closed = True

    def _reply(self, mtype, seq, body):
        self.frames.append((mtype, seq, body))

    def _handle(self, mtype, seq, payload):
        self.written_types.append(mtype)
        if mtype == wire.HELLO:
            nonce, = struct.unpack_from('<I', payload, 2)
            body = bytes([3, wire.HELLO_ACK]) + struct.pack(
                '<IIIIII', SESSION, 3, 1, 1000, 77, nonce)
            body += struct.pack('<4B', 0, 1, 1, 1) + b'fake-fw'.ljust(16, b'\0') + bytes([0])
            self._reply(wire.HELLO_ACK, seq, body)
        elif mtype == wire.AUTHORIZE:
            grant, = struct.unpack_from('<B', payload, 10)
            _, sequence = struct.unpack_from('<II', payload, 2)
            if grant:
                self.authorize_grants += 1
            self.authorized = bool(grant)
            self._reply(wire.ACK, seq, _ack(wire.ACK, sequence))
        elif mtype == wire.TARGETS:
            _, sequence = struct.unpack_from('<II', payload, 2)
            self.targets_sent += 1
            self._reply(wire.TARGETS_ACK, seq, _ack(wire.TARGETS_ACK, sequence))
        elif mtype == wire.STATUS:
            _, sequence, page = struct.unpack_from('<IIB', payload, 2)
            self._reply(wire.STATUS_SNAPSHOT, seq,
                        _status0(sequence, authority=self.authorized) if page == 0
                        else _status1(sequence, execute=self.running))
        elif mtype == wire.DIAGNOSTICS:
            _, sequence, _ = struct.unpack_from('<IIB', payload, 2)
            self._reply(wire.DIAGNOSTICS_PAGE, seq, _diagnostics1(sequence))
        elif mtype == trial.SWEEP:
            _, sequence, op = struct.unpack_from('<IIB', payload, 2)
            self.running = op == trial.START
            self.has_started = self.has_started or op == trial.START
            if op == trial.START and self.drop_start_ack:
                return
            self._reply(wire.ACK, seq, _ack(wire.ACK, sequence))
        elif mtype == capture.REQUEST:
            # 采集帧体只有 schema/op/cursor, 不带业务协议的 version+type 前缀.
            version, op, cursor = struct.unpack_from('<BBI', payload, 0)
            self._reply(capture.RESPONSE, seq, self._page(op, cursor))
        else:
            raise AssertionError(f'unexpected request 0x{mtype:02x}')

    def _page(self, op, cursor):
        if op == 0:
            # SYNTHETIC: 取消后当前游标已在旧运行样本之后, 不把旧缓存当最新反馈.
            current = START_CURSOR + len(self.rows) if self.has_started and not self.running else START_CURSOR
            return struct.pack('<BBBBIII', capture.SCHEMA, capture.RESPONSE, 0, 0,
                               current, current, START_CURSOR)
        rows = self.rows[cursor - START_CURSOR:cursor - START_CURSOR + capture.PAGE]
        if not rows and cursor >= START_CURSOR + len(self.rows):
            # 终态之后: 高速采集继续回报 DM 已关机且反馈仍新鲜, 时戳单调连续.
            if self._tail_us is None:
                self._tail_us = (self.rows[-1][0] if self.rows else 0)
            rows = []
            for index in range(capture.PAGE):
                self._tail_us += 4000
                rows.append((self._tail_us, _row(cursor + index, self._tail_us, 0.0, flags=0x04)))
        return struct.pack('<BBBBIII', capture.SCHEMA, capture.RESPONSE, 1, len(rows),
                           cursor, cursor + len(rows), START_CURSOR) + b''.join(
            row for _, row in rows)


def _args(tmp, arm=True, **overrides):
    values = dict(serial='fake', baud=921600, arm=arm, target_heading=0.52,
                  frequencies=list(FREQUENCIES), cycles=CYCLES, amplitude_nm=AMPLITUDE,
                  output=Path(tmp) / 'trial.csv', elf=None)
    values.update(overrides)
    return SimpleNamespace(**values)


class TrialTests(unittest.TestCase):
    def _run(self, device, **overrides):
        with tempfile.TemporaryDirectory() as tmp:
            args = _args(tmp, **overrides)
            meta = trial.run(args, link_factory=lambda: device)
            report = json.loads(Path(str(args.output) + '.report.json').read_text())
            with args.output.open() as stream:
                lines = stream.read().splitlines()
        return meta, report, lines

    def test_link_exchange_skips_capture_frames(self):
        """旧分流传输缺少 exchange; 单飞链路必须自己发请求并忽略 0xB7."""
        device = FakeDevice()
        link = trial.Link(device)
        # 先塞一个不相关外层序号的采集帧: 应答配对必须按序号跳过它.
        device.frames.insert(0, (capture.RESPONSE, 999, b'\0' * 4))
        client = wire.InfantryClient(link)
        self.assertTrue(hasattr(trial.Link, 'exchange'))
        info = client.hello()
        self.assertEqual(info['session'], SESSION)
        self.assertEqual(device.written_types[0], wire.HELLO)
        link.close()

    def test_invalid_plan_fails_before_opening(self):
        opened = []
        for kwargs in ({'arm': False}, {'frequencies': [3.0, 2.0]},
                       {'amplitude_nm': 0.05}, {'cycles': 2}, {'frequencies': [6.0]}):
            with tempfile.TemporaryDirectory() as tmp:
                args = _args(tmp, **kwargs)
                with self.assertRaises((RuntimeError, ValueError)):
                    trial.run(args, link_factory=lambda: opened.append(1))
                self.assertFalse(list(Path(tmp).iterdir()))
        self.assertFalse(opened)

    def test_output_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = _args(tmp)
            args.output.write_text('keep\n')
            with self.assertRaises(FileExistsError):
                trial.run(args, link_factory=lambda: self.fail('must not open link'))
            self.assertEqual(args.output.read_text(), 'keep\n')

    def test_full_flow_completes_with_heartbeat(self):
        device = FakeDevice()
        meta, report, lines = self._run(device)
        self.assertTrue(report['complete'], report)
        self.assertEqual(report['sweep_run_id'], RUN_ID)
        self.assertGreaterEqual(device.authorize_grants, 2)
        self.assertGreaterEqual(device.targets_sent, 2)
        self.assertEqual(lines[0], ','.join(capture.FIELDS))
        self.assertGreater(len(lines), len(FREQUENCIES) * trial.MIN_SAMPLES)
        stats = report['frequencies_report']
        self.assertEqual([item['hz'] for item in stats], FREQUENCIES)
        self.assertTrue(all(item['usable'] for item in stats), stats)
        self.assertEqual(device.written_types.count(trial.wire.PARAM_WRITE), 0)
        self.assertEqual(device.written_types.count(trial.SWEEP), 2)  # START + 收尾 CANCEL
        self.assertTrue(device.closed)

    def test_lost_start_ack_still_cleans_up(self):
        device = FakeDevice(drop_start_ack=True)
        with tempfile.TemporaryDirectory() as tmp:
            args = _args(tmp)
            with self.assertRaises(TimeoutError):
                trial.run(args, link_factory=lambda: device)
            report = json.loads(Path(str(args.output) + '.report.json').read_text())
        self.assertFalse(report['complete'])
        self.assertIn('error', report)
        self.assertEqual(device.written_types.count(trial.SWEEP), 2)  # START + CANCEL
        self.assertEqual(device.authorize_grants, 1)  # 已授权, 收尾不再重复授权
        self.assertGreaterEqual(device.targets_sent, 2)  # 设定 + 清空
        self.assertNotIn('disabled_confirmation', report)  # 未进入采集, 无终态可核验
        self.assertTrue(device.closed)

    def test_run_id_epoch_and_gap_are_failures(self):
        def change_run_id(rows):
            rows[10] = (rows[10][0], _row(START_CURSOR + 10, rows[10][0], 3.0, run_id=RUN_ID + 1))

        def change_epoch(rows):
            rows[10] = (rows[10][0], _row(START_CURSOR + 10, rows[10][0], 3.0, epoch=2))

        def make_gap(rows):
            rows[20] = (rows[20][0], _row(START_CURSOR + 20, rows[19][0] + 40000, 3.0))

        for mutate in (change_run_id, change_epoch, make_gap):
            device = FakeDevice(rows=_rows(mutate))
            with tempfile.TemporaryDirectory() as tmp:
                args = _args(tmp)
                with self.assertRaises(RuntimeError):
                    trial.run(args, link_factory=lambda: device)
                report = json.loads(Path(str(args.output) + '.report.json').read_text())
            self.assertFalse(report['complete'])
            self.assertIn('error', report)
            self.assertTrue(device.closed)

    def test_previous_trial_terminal_cannot_finish_new_trial(self):
        # SYNTHETIC 新 START 之前保留了一条上次试验终态, 新运行必须仍采满并匹配新run_id.
        rows = _rows()
        prefix = _row(START_CURSOR, 0, 0.0, flags=0x100, run_id=RUN_ID-1)
        shifted = []
        for index, (stamp, blob) in enumerate(rows):
            fields = list(struct.unpack('<III10fHBBI3f', blob))
            fields[0] = START_CURSOR + index + 1; fields[1] = stamp + 4000
            shifted.append((stamp+4000, struct.pack('<III10fHBBI3f', *fields)))
        class RepeatedTrial(FakeDevice):
            def _page(self, op, cursor):
                if op == 0 and self.running:
                    return struct.pack('<BBBBIII', capture.SCHEMA, capture.RESPONSE, 0, 0,
                                       START_CURSOR+1, START_CURSOR+1, START_CURSOR)
                return super()._page(op,cursor)
        device = RepeatedTrial(rows=[(0,prefix), *shifted])
        meta, report, _ = self._run(device)
        self.assertTrue(report['complete'], report)
        self.assertEqual(report['sweep_run_id'], RUN_ID)
        self.assertEqual(report['records'], len(_rows())-1)

    def test_disabled_confirmation_ignores_old_enabled_samples(self):
        device = FakeDevice()
        device.has_started = True
        link = trial.Link(device); client = wire.InfantryClient(link); client.hello()
        result = trial.confirm_disabled(client, START_CURSOR)
        self.assertTrue(result['confirmed'], result)
        self.assertFalse(result['dm_enabled'])
        link.close()

    def test_early_terminal_is_not_complete(self):
        def truncate(rows):
            # 只保留 40 条活性记录后立即 terminal: 时戳连续但远早于 MCU 期望时长.
            rows[:] = [(index * 4000, _row(START_CURSOR + index, index * 4000, 3.0))
                       for index in range(40)]
            rows.append((160000, _row(START_CURSOR + 40, 160000, 0.0, flags=0x100)))

        device = FakeDevice(rows=_rows(truncate))
        meta, report, lines = self._run(device)
        self.assertFalse(report['complete'])
        self.assertIn('reason', report)
        # 已收到的有效数据仍写入 CSV, 不被静默丢弃.
        self.assertGreater(report['records'], 0)
        self.assertGreater(len(lines), 1)

    def test_start_while_dm_disabled_then_arming_confirmed(self):
        """START 前 DM 允许关机 (否则重复试验会死锁), 运行记录建立后才注入."""
        device = FakeDevice()
        meta, report, _ = self._run(device)
        self.assertTrue(report['complete'], report)
        # 目标只被采纳, 未执行 (execute=false): 启动阶段仍须通过.
        self.assertFalse(device.running and report['records'] == 0)
        self.assertEqual(report['records'], len(_rows()) - 1)
        # 只读到 terminal 一条终态记录后停止, 不再把锁存尾部当活性数据.
        self.assertFalse(report['terminal_flags'] & 0x80)

    def test_arming_not_confirmed_within_window(self):
        def never_running(rows):
            rows[:] = [(index * 4000, _row(START_CURSOR + index, index * 4000, 0.0, flags=0))
                       for index in range(6)]

        device = FakeDevice(rows=_rows(never_running))
        with tempfile.TemporaryDirectory() as tmp:
            args = _args(tmp)
            started = time.monotonic()
            with self.assertRaises(RuntimeError):
                trial.run(args, link_factory=lambda: device)
            elapsed = time.monotonic() - started
            report = json.loads(Path(str(args.output) + '.report.json').read_text())
        self.assertGreaterEqual(elapsed, trial.ARM_WAIT_S - 0.05)
        self.assertFalse(report['complete'])
        self.assertIn('arming', report['error'])
        self.assertEqual(device.written_types.count(trial.SWEEP), 2)  # START + CANCEL
        self.assertTrue(device.closed)

    def test_heading_target_must_near_current(self):
        device = FakeDevice()
        with tempfile.TemporaryDirectory() as tmp:
            args = _args(tmp, target_heading=1.2)
            with self.assertRaises(RuntimeError):
                trial.run(args, link_factory=lambda: device)
        self.assertEqual(device.written_types.count(trial.SWEEP), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)

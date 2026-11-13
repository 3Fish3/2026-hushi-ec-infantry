"""采集工具离线测试: C++ 字节向量、只读分页、日志标记与 CSV. 不打开真实串口."""
import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'boards/infantry_f407/tools'))
import infantry_capture as cap
from board_monitor import Decoder, encode_frame

exe = Path(sys.argv.pop(1))
golden = {int(line.split()[0]): bytes.fromhex(line.split()[1])
          for line in subprocess.check_output([str(exe), '--golden'], text=True).splitlines()}


def summary(now=100):
    values = [0]*24
    values[:2] = [0x43415031, 168000000]
    return struct.pack('<4BI24I32s14I', cap.SCHEMA, 0, 1, 0, now, *values, b'capture-test', *([0]*14))


class Port:
    def __init__(self):
        self.buffer = bytearray()
        self.written = []
    @property
    def in_waiting(self): return len(self.buffer)
    def read(self, n):
        result = bytes(self.buffer[:n]); del self.buffer[:n]; return result
    def write(self, data): self.written.append(data); return len(data)


class Tests(unittest.TestCase):
    def test_native_motor_golden(self):
        page = cap.decode_page(golden[1])
        m, unseen = page['motors']
        self.assertEqual(m['name'], 'LF')
        self.assertEqual(m['encoder'], 1)  # 单圈跨零, 不发明累计圈数或关节零点.
        self.assertEqual(m['rpm'], 120)
        self.assertEqual(m['effort_raw'], -2)
        self.assertAlmostEqual(m['rotor_rad_s'], 4*math.pi)
        self.assertEqual(m['temperature_c'], 43)
        self.assertEqual(m['sum_cycles'], 168000)
        self.assertEqual(m['intervals'], 1)
        self.assertTrue(m['valid'])
        self.assertIsNone(m['cumulative_rotor_rad'])
        self.assertIsNone(m['joint_rad'])
        self.assertFalse(unseen['valid'])
        self.assertIsNone(unseen['encoder'])
        self.assertIsNone(cap.decode_page(golden[5])['motors'][0]['temperature_c'])

    def test_native_dm_golden(self):
        values = cap.decode_page(golden[9])['registers']
        self.assertEqual(values[2]['rid'], 0x50)
        self.assertEqual(values[2]['value'], 1.25)
        self.assertEqual(values[2]['request_ms'], 100)
        self.assertEqual(values[2]['received_ms'], 104)
        self.assertEqual(values[3]['value'], -0.5)
        self.assertTrue(values[3]['valid'])
        self.assertFalse(values[0]['seen'])
        self.assertIsNone(values[0]['value'])

    def test_expiration_and_rollover(self):
        data = bytearray(golden[1]); struct.pack_into('<I', data, 4, 201)
        self.assertFalse(cap.decode_page(data)['motors'][0]['valid'])
        struct.pack_into('<I', data, 4, 3)
        struct.pack_into('<I', data, 8+16, 0xfffffffe)
        self.assertEqual(cap.decode_page(data)['motors'][0]['age_ms'], 5)
        data = bytearray(golden[9]); struct.pack_into('<I', data, 4, 654)
        self.assertFalse(cap.decode_page(data)['registers'][3]['valid'])

    def test_nan_does_not_become_json_nan(self):
        data = bytearray(golden[9]); struct.pack_into('<I', data, 8+2*20+12, 0x7fc00000)
        result = cap.decode_page(data)
        self.assertFalse(result['registers'][2]['valid'])
        self.assertIsNone(result['registers'][2]['value'])
        json.dumps(result, allow_nan=False)

    def test_bad_schema(self):
        for offset, value in ((0, cap.SCHEMA + 1), (1, 20), (2, 0), (3, 1)):
            data = bytearray(golden[1]); data[offset] = value
            with self.assertRaises(ValueError): cap.decode_page(data)
        with self.assertRaises(ValueError): cap.decode_page(golden[1][:-1])
        data = bytearray(summary()); struct.pack_into('<I', data, 8, 0)
        with self.assertRaises(ValueError): cap.decode_page(data)

    def test_sensor_fields(self):
        data = struct.pack('<4BI3I8hH5B3I2B7f4I', cap.SCHEMA, 6, 1, 0, 200,
                           2, 0, 195, 1, 2, 3, 4, 1234, -5, 6, 7, 0x1234, 1, 2, 1, 1, 0,
                           10, 0, 199, 1, 0, 0, 0, 9.81, 0.1, 0.2, 0.3, 30, 0, 0, 0, 0)
        self.assertEqual(len(data), 101)
        result = cap.decode_page(data)
        self.assertEqual(result['remote']['ch4'], 1234)
        self.assertEqual(result['remote']['mouse_x'], -5)
        self.assertEqual(result['remote']['keys'], 0x1234)
        self.assertTrue(result['imu']['valid'])
        self.assertTrue(result['remote']['valid'])
        fresh = cap.sensor_text(result, 0, 'ONLINE')
        aged = cap.sensor_text(result, 200, 'ONLINE')
        self.assertEqual(len(fresh), len(aged))
        self.assertNotIn('SNAPSHOT_OLD', aged)
        self.assertIn('+0.1000', fresh)
        self.assertEqual(aged.count('ONLINE'), 2)
        self.assertIn('    201 ms', aged)  # 年龄持续显示, 不延长 MCU 的有效期.
        result['imu']['state'] = 'LOST'
        self.assertIn('LOST', cap.sensor_text(result, 0, 'ONLINE'))
        self.assertIn('LINK_LOST', cap.sensor_text(result, 0, 'LOST'))

    def make_client(self):
        port, events, now = Port(), [], [0.0]
        return cap.Client(port, 10, events.append, clock=lambda: now[0]), port, events, now

    def feed(self, client, port, payload):
        port.buffer.extend(encode_frame(0xb0, client.pending[0], payload))
        client.tick()

    def test_client_read_only_and_split(self):
        client, port, events, now = self.make_client()
        client.tick()
        self.assertEqual(Decoder().feed(port.written[0]), [(0x30, 1, b'\0')])
        wire = encode_frame(0xb0, 1, summary())
        port.buffer.extend(wire[:10]); client.tick(); self.assertFalse(client.pages)
        port.buffer.extend(wire[10:]); client.tick(); self.assertIn(0, client.pages)
        now[0] = 0.11; client.tick()
        self.assertEqual(Decoder().feed(port.written[-1]), [(0x30, 2, b'\1')])
        self.feed(client, port, golden[1])
        before = len(port.written)
        mark = client.mark('main zero', 'q=0 rad, fixture A')
        self.assertEqual(len(port.written), before)
        self.assertEqual(mark['label'], 'main zero')
        self.assertEqual(mark['pages'][1]['motors'][0]['encoder'], 1)
        now[0] += 1
        mark = client.mark('late', '')
        self.assertFalse(mark['pages'][1]['motors'][0]['valid_at_mark'])

    def test_focus_and_requested_mark(self):
        client, port, events, now = self.make_client()
        client.tick(); self.feed(client, port, summary())
        client.focus_page = 9
        now[0] = 0.11; client.tick()
        self.assertEqual(client.pending[1], 9)
        self.feed(client, port, golden[9])
        client.request_mark(1, 'fixture', '0 rad')
        self.assertFalse(any(e['event'] == 'mark' for e in events))
        now[0] = 0.12; client.tick()
        self.assertEqual(client.pending[1], 1)
        self.feed(client, port, golden[1])
        mark = events[-1]
        self.assertEqual(mark['event'], 'mark')
        self.assertEqual(mark['requested_host_elapsed_s'], 0.11)
        self.assertIsNone(client.mark_request)
        self.assertTrue(all(Decoder().feed(w)[0][0] == 0x30 for w in port.written))

    def test_reboot_clears_pages(self):
        client, port, events, now = self.make_client()
        client.tick(); self.feed(client, port, summary(200))
        now[0] = 0.11; client.tick(); self.feed(client, port, golden[1])  # 160 < 200, 可观察时间倒退.
        self.assertEqual(client.segment, 1)
        self.assertNotIn(0, client.pages)
        now[0] = 0.22; client.tick(); self.assertEqual(client.pending[1], 0)

    def test_client_wrong_page_and_timeout(self):
        client, port, events, now = self.make_client()
        client.tick()
        with self.assertRaises(ValueError): self.feed(client, port, golden[1])
        client, port, events, now = self.make_client()
        client.tick()
        for t in (0.51, 1.02): now[0] = t; client.tick()
        now[0] = 1.53; client.tick()
        self.assertEqual(client.link_state, 'NO_RESPONSE')
        self.assertEqual(len(port.written), 4)
        self.feed(client, port, summary())
        self.assertEqual(client.link_state, 'ONLINE')
        now[0] = 1.64; client.tick()
        now[0] = 2.15; client.tick()
        self.assertEqual(client.link_state, 'LOST')
        self.assertIn(0, client.pages)  # 保留最后数据, 不因失联清空整个面板.
        payload = bytearray(golden[3]); payload[1] = client.pending[1]
        self.feed(client, port, payload)
        self.assertEqual(client.link_state, 'ONLINE')
        self.assertTrue(any(e['event'] == 'link_state' and e['state'] == 'LOST' for e in events))

    def test_partial_devices_and_report_age(self):
        missing, gm = cap.decode_page(golden[3])['motors']
        self.assertEqual(missing['state'], 'NO_FEEDBACK')
        self.assertEqual(gm['state'], 'ONLINE')
        self.assertEqual((gm['name'], gm['bus'], gm['can_id']), ('fire_yaw', 2, 0x20a))
        self.assertEqual(cap.display_state(gm, 2000, 100, 'ONLINE'), 'ONLINE / SNAPSHOT_OLD')
        self.assertEqual(cap.display_state(gm, 2000, 100, 'LOST'), 'LINK_LOST (cached)')
        data = bytearray(golden[3])
        struct.pack_into('<I', data, 4, 259)
        self.assertEqual(cap.decode_page(data)['motors'][1]['state'], 'LOST')
        self.assertEqual(cap.decode_page(golden[9])['registers'][0]['state'], 'NO_REPLY')
        self.assertEqual(cap.sample_state(0, False, 0, 50, 'INIT_FAILED'), 'INIT_FAILED')

    def test_early_firmware_is_explained_not_guessed(self):
        old = bytearray(summary())
        old[0] = 1
        del old[96:104]
        with self.assertRaisesRegex(cap.CaptureProtocolError, r'schema=1 .*length=184 .*infantry_capture.bin'):
            cap.decode_page(old)

    def gear_mark(self, source='p_m'):
        motor = dict(kind=4, bus=1, can_id=5, node=5, status=0, valid=True,
                     valid_at_mark=True, seen=True, decoded_valid=True,
                     received_ms=150, age_ms=10, p16=32768, raw_hex='050000000000191a')
        registers = [dict(rid=rid, value=value, valid=True, valid_at_mark=True,
                          received_ms=150, age_ms=10, bits=0)
                     for rid, value in ((0x50, 1.25), (0x51, -0.5), (0x15, 12.5))]
        return dict(event='mark', label='SYNTHETIC point', segment=0, host_elapsed_s=1.0,
                    reference=dict(kind='yaw_gear', profile='infantry-yaw', source=source,
                                   approach='forward', turret_deg=-10.0,
                                   stationary_confirmed=True, note='SYNTHETIC', data_kind='SYNTHETIC'),
                    pages=[dict(page=5, host_page_age_s=0.01, motors=[motor]),
                           dict(page=9, host_page_age_s=0.01, registers=registers)])

    def test_yaw_gear_sample_sources_and_no_unwrap(self):
        mark = self.gear_mark()
        row = cap.yaw_gear_sample(mark)
        self.assertEqual(row['motor_rad'], 1.25)
        self.assertEqual(row['source'], 'p_m')
        self.assertEqual(row['age_ms'], 20)
        self.assertEqual(row['data_kind'], 'SYNTHETIC')
        mark['reference']['source'] = 'xout'
        self.assertEqual(cap.yaw_gear_sample(mark)['motor_rad'], -0.5)
        mark['reference']['source'] = 'mit_pos'
        motor = mark['pages'][0]['motors'][0]
        for raw, expected in ((0, -12.5), (65535, 12.5)):
            motor['p16'] = raw
            self.assertEqual(cap.yaw_gear_sample(mark)['motor_rad'], expected)

    def test_yaw_gear_rejects_enabled_stale_and_unconfirmed(self):
        mark = self.gear_mark()
        mark['pages'][0]['motors'][0]['status'] = 1
        with self.assertRaisesRegex(ValueError, '失能'): cap.yaw_gear_sample(mark)
        mark = self.gear_mark(); mark['pages'][1]['registers'][0]['valid_at_mark'] = False
        with self.assertRaisesRegex(ValueError, 'p_m'): cap.yaw_gear_sample(mark)
        mark = self.gear_mark(); mark['reference']['stationary_confirmed'] = False
        with self.assertRaisesRegex(ValueError, '静止'): cap.yaw_gear_sample(mark)
        mark = self.gear_mark(); mark['reference']['profile'] = 'legacy'
        with self.assertRaisesRegex(ValueError, 'infantry-yaw'): cap.yaw_gear_sample(mark)

    def test_yaw_gear_profile_validation(self):
        data = bytearray(golden[5])
        base = 8 + 52
        data[base] = 1
        struct.pack_into('<H', data, base+2, 5)
        data[base+4] = 5
        decoded = cap.decode_page(data, profile='infantry-yaw')
        self.assertEqual(decoded['motors'][1]['name'], 'yaw_motor_output')
        with self.assertRaisesRegex(cap.CaptureProtocolError, 'profile'):
            cap.decode_page(golden[5], profile='infantry-yaw')

    def test_infantry_absent_slots_are_not_misreported_as_feedback_loss(self):
        data = bytearray(golden[3]); data[8:] = bytes(104)
        rows = cap.decode_page(data, profile='infantry-yaw')['motors']
        self.assertTrue(all(row['state'] == 'UNINSTALLED' and not row['valid'] for row in rows))
        with self.assertRaises(ValueError): cap.decode_page(data)

    def test_yaw_marker_refreshes_status_and_position_using_read_only_requests(self):
        client, port, events, now = self.make_client()
        client.profile = 'infantry-yaw'
        client.tick(); self.feed(client, port, summary())
        client.request_yaw_mark('p_m', 'forward', '-10', 'SYNTHETIC point', 'SYNTHETIC', True)
        now[0] = 0.11; client.tick()
        self.assertEqual(client.pending[1], 5)
        status = bytearray(golden[5]); base = 8+52
        status[base] = 1; struct.pack_into('<H', status, base+2, 5)
        status[base+4] = 5; status[base+5] = 8
        struct.pack_into('<H', status, base+6, 3)  # seen + decoded_valid
        struct.pack_into('<I', status, base+8, 1)
        struct.pack_into('<I', status, base+16, 150)
        status[base+44] = 5  # SYNTHETIC disabled + node5
        self.assertTrue(cap.decode_page(status, profile='infantry-yaw')['motors'][1]['valid'])
        self.feed(client, port, status)
        client.tick(); self.assertEqual(client.pending[1], 9)
        self.feed(client, port, golden[9])
        samples = [event for event in events if event['event'] == 'yaw_gear_sample']
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]['source'], 'p_m')
        self.assertTrue(all(Decoder().feed(w)[0][0] == 0x30 for w in port.written))

    def test_yaw_gear_export_is_offline_and_preserves_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            source, dest = Path(tmp)/'marks.jsonl', Path(tmp)/'gear.csv'
            sample = cap.yaw_gear_sample(self.gear_mark())
            source.write_text(json.dumps(dict(event='yaw_gear_sample', **sample))+'\n', encoding='utf-8')
            self.assertEqual(cap.export_gear_csv(source, dest), 1)
            text = dest.read_text(encoding='utf-8-sig')
            self.assertIn('motor_rad', text); self.assertIn('SYNTHETIC', text)
            with self.assertRaises(FileExistsError): cap.export_gear_csv(source, dest)
            sample['source'] = 'xout'
            with source.open('a', encoding='utf-8') as output:
                output.write(json.dumps(dict(event='yaw_gear_sample', **sample))+'\n')
            with self.assertRaisesRegex(ValueError, '来源'):
                cap.export_gear_csv(source, Path(tmp)/'mixed.csv')

    def test_synthetic_marker_export_fitter_end_to_end(self):
        # SYNTHETIC 完整十点链路, 不连接真实串口、不当作实车结果.
        with tempfile.TemporaryDirectory() as tmp:
            source, dest = Path(tmp)/'synthetic.jsonl', Path(tmp)/'synthetic.csv'
            with source.open('w', encoding='utf-8') as output:
                n = 0
                for direction, angles, zero in [('forward', (-10, -5, 0, 5, 10), 0.1),
                                                 ('reverse', (10, 5, 0, -5, -10), 0.14)]:
                    for angle in angles:
                        mark = self.gear_mark()
                        mark['reference'].update(approach=direction, turret_deg=angle)
                        register = mark['pages'][1]['registers'][0]
                        register.update(value=zero + math.radians(angle)/-0.2, received_ms=1000+n*100)
                        n += 1
                        sample = cap.yaw_gear_sample(mark)
                        output.write(json.dumps(dict(event='yaw_gear_sample', **sample))+'\n')
            self.assertEqual(cap.export_gear_csv(source, dest), 10)
            result = subprocess.run([sys.executable, '-I', str(root/'tools/fit_yaw_gear.py'),
                                     str(dest), '--json'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertAlmostEqual(report['parameters']['k_rad_turret_per_rad_motor'], -0.2)
            self.assertFalse(report['status']['deployable'])
            self.assertEqual(report['provenance']['data_kind'], 'SYNTHETIC')

    def test_log_mark_export_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            log_path, csv_path = Path(tmp)/'run.jsonl', Path(tmp)/'run.csv'
            log = cap.CaptureLog(log_path, {'hardware': 'fixture'})
            log.emit({'event': 'page', 'sequence': 2, 'segment': 0, 'host_elapsed_s': 1, 'data': cap.decode_page(golden[9])})
            log.emit({'event': 'mark', 'label': 'zero', 'reference': '0 rad'})
            log.close()
            with self.assertRaises(FileExistsError):
                cap.CaptureLog(log_path, {})
            cap.export_csv(log_path, csv_path)
            text = csv_path.read_text(encoding='utf-8-sig')
            self.assertIn('1.25', text)
            self.assertIn('zero', text)
            with self.assertRaises(FileExistsError):
                cap.export_csv(log_path, csv_path)


if __name__ == '__main__':
    unittest.main()

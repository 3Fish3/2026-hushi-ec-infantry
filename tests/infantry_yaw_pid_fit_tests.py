"""用合成闭环扫频复现步兵 J4310 离线 PID 拟合的关键输入和失败边界。"""

import cmath
import csv
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "fit_infantry_yaw_pid.py"
FREQUENCIES = (0.25, 0.5, 1.0, 2.0, 4.0)
PERIOD_US = 4000
RUN_ID = 3
AMPLITUDE_NM = 0.02
FIELDS = (
    "sequence", "timestamp_us", "reference_epoch", "target_heading_rad", "heading_rad",
    "heading_rate_rad_s", "dm_position_rad", "dm_velocity_rad_s", "torque_command_nm",
    "torque_feedback_nm", "speed_target_rad_s", "speed_p_nm", "speed_i_nm", "flags",
    "reason", "feedback_age_ms", "sweep_run_id", "sweep_hz", "requested_excitation_nm",
    "applied_excitation_nm",
)


def plant(s):
    return 2.0 * cmath.exp(-0.015 * s) / (1 + 0.1 * s)


def fixture(path, cycles=6, *, frequencies=FREQUENCIES, late_abort=False, sequence_gap=False,
            no_yaw_response=False):
    # 假定已有角度 P=2、速度 P=0.35，激励叠加在总力矩上，两个环都保持闭合。
    sequence = 100
    timestamp = 2**32 - 500_000
    active_samples = 0
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        common = {
            "reference_epoch": 7, "target_heading_rad": 0.0, "reason": 0,
            "feedback_age_ms": 1, "sweep_run_id": RUN_ID,
        }
        for plan_hz in frequencies:
            hz, = struct.unpack("<f", struct.pack("<f", plan_hz))
            s = 2j * math.pi * hz
            disturbance = -1j * AMPLITUDE_NM
            torque = disturbance / (1 + plant(s) * 0.35 * (1 + 2 / s))
            rate = 0j if no_yaw_response else plant(s) * torque
            angle = rate / s
            count = round(cycles * 1e6 / (hz * PERIOD_US))
            if late_abort and plan_hz == frequencies[-1]:
                count -= 3  # 比计划终点提前 12 ms，仍落在旧脚本的 20 ms 容差内。
            for i in range(count):
                t = i * PERIOD_US * 1e-6
                period = 1 / hz
                envelope = min(1.0, t / period, (cycles * period - t) / period)
                phasor = envelope * cmath.exp(s * t)
                yaw = (angle * phasor).real
                yaw_rate = (rate * phasor).real
                motor_torque = (torque * phasor).real
                injection = (disturbance * phasor).real
                if sequence_gap and active_samples == 100:
                    sequence += 1
                writer.writerow({
                    **common, "sequence": sequence, "timestamp_us": timestamp % 2**32,
                    "heading_rad": yaw, "heading_rate_rad_s": yaw_rate,
                    "dm_position_rad": yaw, "dm_velocity_rad_s": yaw_rate,
                    "torque_command_nm": motor_torque, "torque_feedback_nm": motor_torque,
                    "speed_target_rad_s": -2 * yaw,
                    "speed_p_nm": motor_torque - injection, "speed_i_nm": 0.0,
                    "flags": 0xFF, "sweep_hz": hz, "requested_excitation_nm": injection,
                    "applied_excitation_nm": injection,
                })
                timestamp += PERIOD_US
                sequence += 1
                active_samples += 1
        writer.writerow({
            **dict.fromkeys(FIELDS, 0), **common, "sequence": sequence,
            "timestamp_us": timestamp % 2**32, "flags": 0x147,
            "sweep_hz": frequencies[-1] if late_abort else 0.0,
        })
    report = {
        "tool": "infantry_sweep_trial", "schema": 2, "complete": True,
        "mode": "torque", "frequencies": frequencies, "cycles": cycles,
        "records": active_samples, "sweep_run_id": RUN_ID, "amplitude_nm": AMPLITUDE_NM,
        "duration_s": sum(cycles / hz for hz in frequencies),
        "expected_us": round(1e6 * sum(cycles / hz for hz in frequencies)),
        "mcu_span_us": active_samples * PERIOD_US,
        "terminal_flags": 0x147, "terminal_reason": 0,
        "disabled_confirmation": {"confirmed": True},
    }
    path.with_suffix(".csv.report.json").write_text(json.dumps(report), encoding="utf-8")


class InfantryYawPidFitTests(unittest.TestCase):
    def run_cli(self, *paths):
        return subprocess.run(
            [sys.executable, "-X", "utf8", str(SCRIPT), *(str(path) for path in paths),
             "--speed-hz", "2", "--angle-hz", "0.5"],
            check=False, capture_output=True, encoding="utf-8", timeout=45,
        )

    def run_fixture(self, **kwargs):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sweep.csv"
            fixture(path, **kwargs)
            return self.run_cli(path)

    def assert_rejected(self, result):
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertNotIn("Traceback (most recent call last)", result.stderr)
        self.assertTrue(result.stderr.strip() or result.stdout.strip(), "缺少失败原因")
        if result.stdout.strip().startswith("{"):
            self.assertFalse(json.loads(result.stdout).get("candidate"), result.stdout)

    def test_recovers_candidates_from_closed_loop_sweep(self):
        result = self.run_fixture()
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        candidates = json.loads(result.stdout)["candidate"]
        expected_speed = 1 / abs(plant(2j * math.pi * 2.0))
        outer_s = 2j * math.pi * 0.5
        inner_open = expected_speed * plant(outer_s)
        expected_angle = abs(outer_s / (inner_open / (1 + inner_open)))
        self.assertAlmostEqual(candidates["speed"]["kp"], expected_speed, delta=0.10)
        self.assertAlmostEqual(candidates["angle"]["kp"], expected_angle, delta=0.50)
        for gains in candidates.values():
            self.assertEqual(gains["ki"], 0)
            self.assertEqual(gains["kd"], 0)

    def test_combines_split_trials_and_keeps_nearby_frequencies_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            low = Path(directory) / "low.csv"
            high = Path(directory) / "high.csv"
            fixture(low, frequencies=(0.25, 0.5, 0.5005))
            fixture(high, frequencies=(1.0, 2.0, 4.0))
            self.assert_rejected(self.run_cli(low))
            self.assert_rejected(self.run_cli(high))
            combined = self.run_cli(low, high)

        self.assertEqual(combined.returncode, 0, combined.stderr + combined.stdout)
        report = json.loads(combined.stdout)
        self.assertIsNotNone(report["candidate"])
        measured = [item["hz"] for item in report["repeat_checks"]]
        self.assertEqual(len(measured), 6)
        self.assertEqual(measured[1], 0.5)
        self.assertAlmostEqual(measured[2], 0.5005, places=7)

    def test_rejects_frequencies_that_alias_after_wire_quantization(self):
        result = self.run_fixture(frequencies=(0.25, 0.5, 0.500000001, 2.0, 4.0))
        self.assert_rejected(result)
        self.assertIn("float32", json.loads(result.stdout)["reason"])

    def test_rejects_late_abort_even_when_sidecar_says_complete(self):
        self.assert_rejected(self.run_fixture(late_abort=True))

    def test_rejects_only_one_steady_period(self):
        self.assert_rejected(self.run_fixture(cycles=3))

    def test_rejects_lost_sample_even_when_sidecar_count_matches(self):
        self.assert_rejected(self.run_fixture(sequence_gap=True))

    def test_rejects_no_identifiable_yaw_response(self):
        self.assert_rejected(self.run_fixture(no_yaw_response=True))


if __name__ == "__main__":
    unittest.main()

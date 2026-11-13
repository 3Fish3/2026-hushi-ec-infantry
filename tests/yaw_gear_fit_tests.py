"""小角度外齿轮拟合工具的离线契约测试; 只用合成数据, 不接触硬件/实车参数."""
import importlib.util
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "fit_yaw_gear.py"
TEMPLATE = ROOT / "tools" / "yaw_gear_samples_template.csv"

_spec = importlib.util.spec_from_file_location("fit_yaw_gear", TOOL)
fit = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = fit  # dataclass 解析注解时需要模块已注册
_spec.loader.exec_module(fit)

K = -0.2
ZERO_FORWARD = 0.10
ZERO_REVERSE = 0.14
HEADER = "approach,turret_deg,motor_rad,source,sampled_ms,age_ms,valid,mark,note,data_kind"
ANGLES_FORWARD = (-10, -5, 0, 5, 10)
ANGLES_REVERSE = (10, 5, 0, -5, -10)


def synthetic_rows(k=K, zero_forward=ZERO_FORWARD, zero_reverse=ZERO_REVERSE,
                   data_kind="SYNTHETIC", source="mit_pos", valid="1", motor_hook=None):
    rows = []
    for approach, angles, zero in (
        ("forward", ANGLES_FORWARD, zero_forward),
        ("reverse", ANGLES_REVERSE, zero_reverse),
    ):
        for index, deg in enumerate(angles):
            # turret = k * motor + b 且 b = -k * zero，故 motor = zero + turret / k
            motor = zero + math.radians(deg) / k
            if motor_hook is not None:
                motor = motor_hook(approach, deg, motor)
            rows.append(
                f"{approach},{deg},{motor:.12f},{source},{1000 + index * 10},4.0,"
                f"{valid},,合成点,{data_kind}"
            )
    return rows


class SyntheticFitTests(unittest.TestCase):
    def run_tool(self, rows, *extra, header=HEADER):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "samples.csv"
        path.write_text("\n".join([header, *rows]) + "\n", encoding="utf-8")
        return fit.analyze(str(path)), path

    def test_recovers_known_ratio_zeros_and_backlash(self):
        report, _ = self.run_tool(synthetic_rows())
        p = report["parameters"]
        self.assertAlmostEqual(p["k_rad_turret_per_rad_motor"], K, places=9)
        self.assertAlmostEqual(p["equivalent_gear_ratio"], 5.0, places=8)
        self.assertAlmostEqual(p["motor_zero_forward_rad"], ZERO_FORWARD, places=9)
        self.assertAlmostEqual(p["motor_zero_reverse_rad"], ZERO_REVERSE, places=9)
        expected_backlash = abs(math.degrees(-K * (ZERO_REVERSE - ZERO_FORWARD)))
        self.assertAlmostEqual(p["backlash_turret_deg"], expected_backlash, places=7)
        self.assertAlmostEqual(p["per_direction_slope"]["forward"], K, places=9)
        self.assertAlmostEqual(p["per_direction_slope"]["reverse"], K, places=9)
        self.assertLess(report["residuals_summary"]["rms_deg"], 1e-9)
        self.assertLess(report["residuals_summary"]["max_abs_deg"], 1e-9)
        self.assertEqual(len(report["residual_points"]), 10)
        for point in report["residual_points"]:
            self.assertAlmostEqual(point["residual_deg"], 0.0, places=7)

    def test_synthetic_is_never_deployable(self):
        report, _ = self.run_tool(synthetic_rows())
        self.assertTrue(report["status"]["fit_math_passed"])
        self.assertTrue(report["status"]["mechanism_sign_ok"])
        self.assertFalse(report["provenance"]["real_measurements"])
        self.assertFalse(report["status"]["usable_as_candidate"])
        self.assertFalse(report["status"]["deployable"])
        self.assertFalse(report["status"]["independent_non_fit_points_accepted"])
        self.assertTrue(any("SYNTHETIC" in w for w in report["warnings"]))

    def test_real_kind_still_not_deployable_without_independent_validation(self):
        report, _ = self.run_tool(synthetic_rows(data_kind="REAL"))
        self.assertTrue(report["status"]["usable_as_candidate"])
        self.assertFalse(report["status"]["deployable"])
        self.assertFalse(report["status"]["independent_non_fit_points_accepted"])

    def test_positive_k_is_warned_not_flipped(self):
        report, _ = self.run_tool(synthetic_rows(k=0.2))
        p = report["parameters"]
        self.assertGreater(p["k_rad_turret_per_rad_motor"], 0)
        self.assertFalse(report["status"]["mechanism_sign_ok"])
        self.assertFalse(report["status"]["usable_as_candidate"])
        self.assertTrue(any("k<0" in w for w in report["warnings"]))

    def test_residuals_csv_and_json_cli(self):
        _, path = self.run_tool(synthetic_rows())
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "residuals.csv"
            result = subprocess.run(
                [sys.executable, "-I", str(TOOL), str(path), "--json", "--residuals", str(out)],
                capture_output=True, text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertEqual(report["fit_math"], "pass")
            self.assertFalse(report["status"]["deployable"])
            lines = out.read_text(encoding="utf-8").strip().splitlines()
            self.assertEqual(len(lines), 11)
            self.assertEqual(lines[0].split(",")[:2], ["line", "approach"])

    def test_legacy_three_column_csv_still_supported(self):
        rows = []
        for approach, angles, zero in (
            ("forward", ANGLES_FORWARD, ZERO_FORWARD),
            ("reverse", ANGLES_REVERSE, ZERO_REVERSE),
        ):
            for deg in angles:
                rows.append(f"{approach},{deg},{zero + math.radians(deg) / K:.12f}")
        report, _ = self.run_tool(rows, header="approach,turret_deg,motor_rad")
        self.assertAlmostEqual(report["parameters"]["k_rad_turret_per_rad_motor"], K, places=9)
        self.assertEqual(report["provenance"]["data_kind"], "UNSPECIFIED")
        self.assertTrue(report["provenance"]["legacy_three_column"])
        self.assertFalse(report["status"]["provenance_complete"])
        self.assertFalse(report["status"]["usable_as_candidate"])
        self.assertFalse(report["status"]["deployable"])
        self.assertTrue(any("原三列" in w for w in report["warnings"]))

    def test_cross_wrap_anomaly_is_reported_not_unwrapped(self):
        # 电机角跨 2π 边界: 工具只报告异常, 不猜周期, 原始 motor_rad 保持不变.
        def wrap(approach, deg, motor):
            if approach == "forward" and deg >= 0:
                return motor - 2 * math.pi
            return motor

        report, path = self.run_tool(synthetic_rows(data_kind="REAL", motor_hook=wrap))
        continuity = report["continuity"]
        self.assertFalse(continuity["unwrap_performed"])
        self.assertTrue(continuity["anomalies"])
        self.assertTrue(any("跨边界" in issue or "跨圈" in issue for issue in continuity["anomalies"]))
        self.assertFalse(report["status"]["continuity_ok"])
        self.assertFalse(report["status"]["usable_as_candidate"])
        self.assertFalse(report["status"]["deployable"])
        self.assertIn("跨圈", report["status"]["reason"])
        reported = {point["line"]: point["motor_rad"] for point in report["residual_points"]}
        self.assertTrue(any(motor < 0 for motor in reported.values()))
        result = subprocess.run(
            [sys.executable, "-I", str(TOOL), str(path), "--json"],
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 3)
        self.assertFalse(json.loads(result.stdout)["continuity"]["unwrap_performed"])

    def test_nonmonotonic_motor_is_advisory(self):
        def jitter(approach, deg, motor):
            return motor + (0.5 if approach == "reverse" and deg == -5 else 0.0)

        report, _ = self.run_tool(synthetic_rows(motor_hook=jitter))
        self.assertTrue(report["continuity"]["advisories"])
        self.assertTrue(any("不单调" in issue for issue in report["continuity"]["advisories"]))

    def test_positive_k_cli_exit_is_nonzero_but_written(self):
        _, path = self.run_tool(synthetic_rows(k=0.2))
        result = subprocess.run(
            [sys.executable, "-I", str(TOOL), str(path), "--json"],
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 3)
        self.assertEqual(json.loads(result.stdout)["fit_math"], "pass")


class RejectionTests(unittest.TestCase):
    def assert_rejected(self, rows, header=HEADER):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "samples.csv"
        path.write_text("\n".join([header, *rows]) + "\n", encoding="utf-8")
        with self.assertRaises(ValueError):
            fit.analyze(str(path))
        result = subprocess.run(
            [sys.executable, "-I", str(TOOL), str(path)],
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("标定失败", result.stderr)

    def test_empty_project_template_is_rejected(self):
        with self.assertRaises(ValueError):
            fit.analyze(str(TEMPLATE))
        result = subprocess.run(
            [sys.executable, "-I", str(TOOL), str(TEMPLATE)],
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("motor_rad 为空", result.stderr)

    def test_missing_required_column(self):
        self.assert_rejected(["forward,-10", "reverse,5"], header="approach,turret_deg")

    def test_nan_value(self):
        rows = synthetic_rows()
        fields = rows[2].split(",")
        fields[2] = "nan"
        rows[2] = ",".join(fields)
        self.assert_rejected(rows)

    def test_too_few_points_per_direction(self):
        rows = synthetic_rows()[0:2] + synthetic_rows()[5:7]
        self.assert_rejected(rows)

    def test_duplicate_angle_in_one_direction(self):
        rows = synthetic_rows()
        rows[1] = rows[1].replace(",-5,", ",-10,", 1)
        self.assert_rejected(rows)

    def test_reverse_must_decrease(self):
        rows = synthetic_rows()
        rows[5], rows[9] = rows[9], rows[5]
        self.assert_rejected(rows)

    def test_mixed_position_source(self):
        rows = synthetic_rows()
        rows[3] = rows[3].replace(",mit_pos,", ",p_m,", 1)
        self.assert_rejected(rows)

    def test_partial_source_is_rejected(self):
        rows = synthetic_rows()
        rows[4] = rows[4].replace(",mit_pos,", ",,", 1)
        self.assert_rejected(rows)

    def test_expired_position_feedback_is_rejected(self):
        rows = synthetic_rows()
        fields = rows[2].split(','); fields[5] = '500'
        rows[2] = ','.join(fields)
        self.assert_rejected(rows)

    def test_invalid_feedback_row(self):
        rows = synthetic_rows()
        rows[5] = rows[5].replace(",1,", ",0,", 1)
        self.assert_rejected(rows)

    def test_unknown_data_kind(self):
        rows = synthetic_rows(data_kind="MEASURED")
        self.assert_rejected(rows)

    def test_unknown_source(self):
        rows = synthetic_rows(source="imu")
        self.assert_rejected(rows)


if __name__ == "__main__":
    unittest.main()

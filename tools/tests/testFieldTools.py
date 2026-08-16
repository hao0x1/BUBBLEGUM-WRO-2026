"""Offline field workflow checks: no real serial ports are opened."""

import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("fieldLogger", ROOT / "tools" / "fieldLogger.py")
logger = importlib.util.module_from_spec(spec)
spec.loader.exec_module(logger)


def port(device, vid=None, pid=None, description="", manufacturer=""):
    return SimpleNamespace(device=device, vid=vid, pid=pid,
                           description=description, manufacturer=manufacturer)


class Lines:
    def __init__(self, lines):
        self.lines = iter(lines)

    def readline(self):
        try:
            return (next(self.lines) + "\n").encode()
        except StopIteration:
            raise KeyboardInterrupt


class LoggerTests(unittest.TestCase):
    def test_detects_esp32_and_excludes_k230(self):
        self.assertEqual(logger.detect_port([
            port("/dev/esp", 0x0403, 0x6001),
            port("/dev/usbserial-k230", 0x1209, 0xABD1, "USB serial")]), "/dev/esp")

    def test_does_not_guess_ambiguous_port(self):
        with self.assertRaises(RuntimeError):
            logger.detect_port([port("/dev/a", 0x0403, 0x6001),
                                port("/dev/b", 0x0403, 0x6001)])

    def test_single_pass_is_not_full_obstacle(self):
        self.assertEqual(logger.observed_profile(
            "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R42 SINGLE PASS - AUTO-STOP"), "single-pass")

    def test_current_source_schema_is_available_without_boot(self):
        self.assertIn("controller_fault_hex", logger.source_columns())
        self.assertIn("laps", logger.source_columns())

    def test_readiness_requires_health_and_zero_faults(self):
        ready = dict(state="WAIT_BUTTON", health="1", controller_fault_hex="0x0", actuator_fault="0")
        self.assertTrue(logger.is_ready(ready))
        for change in (dict(health="0"), dict(controller_fault_hex="40"),
                       dict(actuator_fault="1"), dict(state="RUNNING")):
            self.assertFalse(logger.is_ready(dict(ready, **change)))

    def test_records_and_announces_ready_only_after_matching_banner(self):
        columns = "mode,state,health,controller_fault_hex,actuator_fault"
        header = "CSV_HEADER," + columns
        data = "DATA,AUTONOMOUS,WAIT_BUTTON,1,0x0,0"
        output, saved = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(KeyboardInterrupt):
            logger.record(Lines([header, data,
                "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R42 FULL COURSE - PARKING DISABLED",
                header, data, data]), saved, "obstacle", reset=True)
        self.assertEqual(output.getvalue().count("READY: press"), 1)
        self.assertEqual(saved.getvalue().count(data), 3)

    def test_rejects_wrong_installed_round(self):
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            logger.record(Lines(["BUBBLEGUM AUTONOMOUS OPEN 20260905-R18 - MOTION CAPABLE"]),
                          io.StringIO(), "obstacle", reset=True)

    def test_rejects_malformed_rows(self):
        self.assertIsNone(logger.telemetry("DATA,AUTONOMOUS", ["mode", "state"]))
        self.assertIsNone(logger.telemetry("DATA,CALIBRATION", ["mode"]))


class ShellRoutingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.calls = self.directory / "calls.jsonl"
        for name in ("python", "pio"):
            mock = self.directory / name
            mock.write_text("#!/usr/bin/env python3\n"
                "import json,os,sys\n"
                "with open(os.environ['FIELD_TEST_CALLS'],'a') as f:\n"
                " f.write(json.dumps([os.path.basename(sys.argv[0])]+sys.argv[1:])+'\\n')\n"
                "if '--detect-port' in sys.argv: print('/dev/test-esp')\n")
            mock.chmod(0o755)
        self.environment = dict(os.environ, FIELD_PYTHON=str(self.directory / "python"),
                                PLATFORMIO=str(self.directory / "pio"),
                                FIELD_TEST_CALLS=str(self.calls))
        self.environment.pop("ESP32_PORT", None)

    def run_field(self, action, *args):
        subprocess.run(["bash", str(ROOT / "competition" / "field.sh"), action, *args],
                       check=True, env=self.environment, capture_output=True, text=True)
        return [json.loads(line) for line in self.calls.read_text().splitlines()]

    def test_open_upload_routes_to_open_and_resets_logger(self):
        calls = self.run_field("open", "/dev/selected")
        self.assertEqual(calls[0][0], "pio")
        self.assertIn("autonomousOpen", calls[0])
        self.assertIn("/dev/selected", calls[0])
        self.assertIn("--reset", calls[1])
        self.assertIn("open", calls[1])

    def test_obstacle_detects_port_then_uploads_then_logs(self):
        calls = self.run_field("obstacle")
        self.assertIn("--detect-port", calls[0])
        self.assertIn("autonomousObstacle", calls[1])
        self.assertIn("/dev/test-esp", calls[1])
        self.assertIn("--reset", calls[2])

    def test_attach_does_not_upload_or_reset(self):
        calls = self.run_field("log-obstacle", "/dev/selected")
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], "python")
        self.assertNotIn("--reset", calls[0])

    def test_build_never_invokes_logger_or_port_detection(self):
        calls = self.run_field("build")
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], "pio")
        self.assertIn("autonomousOpen", calls[0])
        self.assertIn("autonomousObstacle", calls[0])
        self.assertNotIn("upload", calls[0])

    def test_ports_only_lists_devices(self):
        calls = self.run_field("ports")
        self.assertEqual(len(calls), 1)
        self.assertIn("--list-ports", calls[0])


if __name__ == "__main__":
    unittest.main()

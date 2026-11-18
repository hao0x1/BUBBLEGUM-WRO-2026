"""Offline field workflow checks: no real serial ports are opened."""

import contextlib
import configparser
import importlib.util
import io
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("fieldLogger", ROOT / "tools" / "fieldLogger.py")
logger = importlib.util.module_from_spec(spec)
spec.loader.exec_module(logger)
flash_spec = importlib.util.spec_from_file_location("flashSavedOpen", ROOT / "tools" / "flashSavedOpen.py")
saved_open = importlib.util.module_from_spec(flash_spec)
flash_spec.loader.exec_module(saved_open)


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

    def test_optical_fallback_has_distinct_profile(self):
        self.assertEqual(logger.observed_profile(
            "BUBBLEGUM AUTONOMOUS OPEN 20260905-R21 OPTICAL FALLBACK - MOTION CAPABLE"), "open-backup")
        self.assertEqual(logger.observed_profile(
            "BUBBLEGUM AUTONOMOUS OPEN 20260905-R20 FIELD RECOVERY - MOTION CAPABLE"), "open")

    def test_primary_and_backup_cannot_be_confused(self):
        banners = {
            "open": "BUBBLEGUM AUTONOMOUS OPEN 20260905-R20 FIELD RECOVERY - MOTION CAPABLE",
            "open-backup": "BUBBLEGUM AUTONOMOUS OPEN 20260905-R21 OPTICAL FALLBACK - MOTION CAPABLE",
        }
        for expected, observed in (("open", "open-backup"), ("open-backup", "open")):
            with self.subTest(expected=expected):
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                    logger.record(Lines([banners[observed]]), io.StringIO(), expected, reset=True)

    def test_obstacle_profiles_have_distinct_identity_and_reject_other_modes(self):
        banners = {
            "obstacle": "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R42 FULL COURSE - PARKING DISABLED",
            "obstacle-backup": "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R43 RECOVERABLE - PARKING DISABLED",
            "obstacle-progress": "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R44 PROGRESS PRIORITY - PARKING DISABLED",
            "obstacle-final-cw": "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R45 CONTINUE CW - PARKING DISABLED",
            "obstacle-final-ccw": "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R45 CONTINUE CCW - PARKING DISABLED",
        }
        for profile, banner in banners.items():
            self.assertEqual(logger.observed_profile(banner), profile)
            for expected in banners:
                if expected == profile:
                    continue
                with self.subTest(expected=expected, observed=profile):
                    with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                        logger.record(Lines([banner]), io.StringIO(), expected, reset=True)

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

    def test_close_start_waits_then_announces_ready_after_repositioning(self):
        header = "CSV_HEADER,mode,state,health,controller_fault_hex,actuator_fault"
        blocked = "DATA,AUTONOMOUS,WAIT_HEALTH,1,0x10000,0"
        ready = "DATA,AUTONOMOUS,WAIT_BUTTON,1,0x0,0"
        output = io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(KeyboardInterrupt):
            logger.record(Lines([
                "BUBBLEGUM AUTONOMOUS OPEN 20260905-R19 FIELD APPROACH - MOTION CAPABLE",
                header, blocked, blocked, ready]), io.StringIO(), "open", reset=True)
        self.assertEqual(output.getvalue().count("WAITING FOR START SPACE"), 1)
        self.assertEqual(output.getvalue().count("READY: press"), 1)

    def test_backup_shows_start_placement_and_ready_messages(self):
        header = "CSV_HEADER,mode,state,health,controller_fault_hex,actuator_fault"
        output = io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(KeyboardInterrupt):
            logger.record(Lines([
                "BUBBLEGUM AUTONOMOUS OPEN 20260905-R21 OPTICAL FALLBACK - MOTION CAPABLE",
                header, "DATA,AUTONOMOUS,WAIT_HEALTH,1,0x10000,0",
                "DATA,AUTONOMOUS,WAIT_BUTTON,1,0x0,0"]), io.StringIO(), "open-backup", reset=True)
        self.assertEqual(output.getvalue().count("WAITING FOR START SPACE"), 1)
        self.assertEqual(output.getvalue().count("READY: press"), 1)

    def test_optical_hold_is_active_state_only_and_announced_once_per_hold(self):
        for state in ("RUN", "DIRECTION_PROBE", "CORNER"):
            self.assertTrue(logger.is_optical_hold(dict(state=state, controller_fault_hex="0x4")))
        for state, fault in (("FAULT_LATCHED", "0x4"), ("RUN", "0x0"),
                             ("WAIT_HEALTH", "0x4"), ("RUN", "0x84"), ("RUN", "bad-data")):
            self.assertFalse(logger.is_optical_hold(dict(state=state, controller_fault_hex=fault)))
        output = io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(KeyboardInterrupt):
            logger.record(Lines([
                "BUBBLEGUM AUTONOMOUS OPEN 20260905-R21 OPTICAL FALLBACK - MOTION CAPABLE",
                "CSV_HEADER,mode,state,controller_fault_hex",
                "DATA,AUTONOMOUS,RUN,0x4", "DATA,AUTONOMOUS,RUN,0x4",
                "DATA,AUTONOMOUS,RUN,0x0", "DATA,AUTONOMOUS,CORNER,0x4",
                "DATA,AUTONOMOUS,FAULT_LATCHED,0x4"]), io.StringIO(), "open-backup", reset=True)
        self.assertEqual(output.getvalue().count("OPTICAL HOLD:"), 2)
        self.assertEqual(output.getvalue().count("STOPPED:"), 1)

    def test_probe_direction_hold_is_not_latched_or_repeated(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(KeyboardInterrupt):
            logger.record(Lines([
                "BUBBLEGUM AUTONOMOUS OPEN 20260905-R21 OPTICAL FALLBACK - MOTION CAPABLE",
                "CSV_HEADER,mode,state,controller_fault_hex",
                "DATA,AUTONOMOUS,DIRECTION_PROBE,0x10000",
                "DATA,AUTONOMOUS,DIRECTION_PROBE,0x10000",
                "DATA,AUTONOMOUS,DIRECTION_PROBE,0x4",
                "DATA,AUTONOMOUS,DIRECTION_PROBE,0x4",
                "DATA,AUTONOMOUS,DIRECTION_PROBE,0x0"]), io.StringIO(), "open-backup", reset=True)
        self.assertEqual(output.getvalue().count("WAITING FOR CORNER DIRECTION:"), 1)
        self.assertEqual(output.getvalue().count("OPTICAL HOLD:"), 1)
        self.assertNotIn("STOPPED:", output.getvalue())
        self.assertIsNone(logger.active_hold_message(dict(
            state="FAULT_LATCHED", controller_fault_hex="0x10000")))

    def test_nonterminal_navigation_and_camera_holds_explain_the_reason(self):
        cases = (
            ("DIRECTION_PROBE", "0x400", "direction probing timed out"),
            ("CORNER", "0x4000", "corner turn timed out"),
            ("RUN", "0x100000", "pillar pass timed out"),
            ("RUN", "0x80", "front clearance is too small"),
            ("RUN", "0x100", "fresh camera data"),
            ("RUN", "0x200", "fresh camera data"),
            ("FINISH_CLEAR", "0x40000", "finish clearance timed out"),
        )
        for state, fault, reason in cases:
            with self.subTest(state=state, fault=fault):
                row = dict(state=state, controller_fault_hex=fault, motion="0",
                           applied_drive_permille="0", actuator_fault="0")
                self.assertIn(reason, logger.active_hold_message(row))
                for update in (dict(motion="1"), dict(applied_drive_permille="300"),
                               dict(actuator_fault="1"), dict(state="FAULT_LATCHED"),
                               dict(state="FINISHED")):
                    self.assertIsNone(logger.active_hold_message(dict(row, **update)))

    def test_optical_and_direction_messages_also_require_zero_output(self):
        for state, fault in (("RUN", "0x4"), ("DIRECTION_PROBE", "0x10000")):
            for changed in (dict(motion="1"), dict(applied_drive_permille="300")):
                row = dict(state=state, controller_fault_hex=fault, motion="0", applied_drive_permille="0")
                self.assertIsNone(logger.active_hold_message(dict(row, **changed)))
        self.assertIsNone(logger.active_hold_message(dict(
            state="RUN", controller_fault_hex="0x100001", motion="0", applied_drive_permille="0")))

    def test_navigation_hold_is_deduplicated_but_terminal_fault_is_distinct(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(KeyboardInterrupt):
            logger.record(Lines([
                "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R43 RECOVERABLE - PARKING DISABLED",
                "CSV_HEADER,mode,state,controller_fault_hex,motion,applied_drive_permille",
                "DATA,AUTONOMOUS,CORNER,0x4000,0,0",
                "DATA,AUTONOMOUS,CORNER,0x4000,0,0",
                "DATA,AUTONOMOUS,CORNER,0x4000,1,300",
                "DATA,AUTONOMOUS,FAULT_LATCHED,0x4000,0,0"]), io.StringIO(), "obstacle-backup", reset=True)
        self.assertEqual(output.getvalue().count("RECOVERY HOLD:"), 1)
        self.assertEqual(output.getvalue().count("STOPPED:"), 1)

    def test_rejects_malformed_rows(self):
        self.assertIsNone(logger.telemetry("DATA,AUTONOMOUS", ["mode", "state"]))
        self.assertIsNone(logger.telemetry("DATA,CALIBRATION", ["mode"]))

    def test_fault_messages_distinguish_direction_and_optical_loss(self):
        self.assertIn("Turn direction", logger.fault_explanation(
            {"controller_fault_hex": "0x10000"}))
        self.assertIn("Front distance stayed unusable", logger.fault_explanation(
            {"controller_fault_hex": "0x4"}))
        self.assertIn("Unrecognized", logger.fault_explanation(
            {"controller_fault_hex": "bad-data"}))


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
                "if '--stop-existing' in sys.argv and os.environ.get('FIELD_TEST_FAIL_HANDOFF'): sys.exit(1)\n"
                "if '--detect-port' in sys.argv: print('/dev/test-esp')\n")
            mock.chmod(0o755)
        self.environment = dict(os.environ, FIELD_PYTHON=str(self.directory / "python"),
                                PLATFORMIO=str(self.directory / "pio"),
                                FIELD_TEST_CALLS=str(self.calls))
        self.environment.pop("ESP32_PORT", None)

    def run_field(self, action, *args):
        subprocess.run(["bash", str(ROOT / "competition" / "field.sh"), action, *args],
                       check=True, env=self.environment, capture_output=True, text=True)
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
        return [call for call in calls if "--stop-existing" not in call]

    def test_logger_handoff_precedes_upload_or_new_log(self):
        for action in ("open", "obstacle", "log-obstacle", "obstacle-progress",
                       "obstacle-final-cw", "log-obstacle-final-ccw"):
            with self.subTest(action=action):
                self.calls.unlink(missing_ok=True)
                self.run_field(action, "/dev/selected")
                calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
                self.assertIn("--stop-existing", calls[0])
                self.assertIn("/dev/selected", calls[0])
                self.assertEqual(calls[0][1], str(ROOT / "tools" / "fieldLogger.py"))

    def test_failed_handoff_prevents_upload_and_logger_start(self):
        self.environment["FIELD_TEST_FAIL_HANDOFF"] = "1"
        with self.assertRaises(subprocess.CalledProcessError):
            self.run_field("open", "/dev/selected")
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
        self.assertEqual(len(calls), 1)
        self.assertIn("--stop-existing", calls[0])

    def test_open_upload_routes_to_open_and_resets_logger(self):
        calls = self.run_field("open", "/dev/selected")
        self.assertEqual(calls[0][0], "python")
        self.assertEqual(calls[0][1], str(ROOT / "tools" / "flashSavedOpen.py"))
        self.assertIn("/dev/selected", calls[0])
        self.assertIn("--reset", calls[1])
        self.assertIn("open", calls[1])
        self.assertIn(str(ROOT / "local" / "competitionFirmware" / "open-r20" / "firmware.bin"), calls[1])
        self.assertFalse(any(call[0] == "pio" for call in calls))

    def test_open_backup_upload_routes_separately_and_resets_logger(self):
        calls = self.run_field("open-backup", "/dev/selected")
        self.assertEqual(calls[0][0], "pio")
        self.assertIn("autonomousOpenBackup", calls[0])
        self.assertNotIn("autonomousOpen", calls[0])
        self.assertIn("--reset", calls[1])
        self.assertIn("open-backup", calls[1])

    def test_backup_attach_does_not_upload_or_reset(self):
        calls = self.run_field("log-open-backup", "/dev/selected")
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], "python")
        self.assertIn("open-backup", calls[0])
        self.assertNotIn("--reset", calls[0])

    def test_obstacle_detects_port_then_uploads_then_logs(self):
        calls = self.run_field("obstacle")
        self.assertIn("--detect-port", calls[0])
        self.assertIn("autonomousObstacle", calls[1])
        self.assertIn("/dev/test-esp", calls[1])
        self.assertIn("--reset", calls[2])

    def test_obstacle_variants_upload_matching_firmware_then_log(self):
        for action, environment in (("obstacle-backup", "autonomousObstacleBackup"),
                                    ("obstacle-progress", "autonomousObstacleProgress"),
                                    ("obstacle-final-cw", "autonomousObstacleContinueCw"),
                                    ("obstacle-final-ccw", "autonomousObstacleContinueCcw")):
            with self.subTest(action=action):
                self.calls.unlink(missing_ok=True)
                calls = self.run_field(action, "/dev/selected")
                self.assertEqual(calls[0][0], "pio")
                self.assertIn(environment, calls[0])
                self.assertNotIn("autonomousObstacle", calls[0])
                self.assertIn("--reset", calls[1])
                self.assertIn(action, calls[1])
                self.assertIn(str(ROOT / "src" / "ESP32-S3-N16R8" / ".pio" / "build" /
                                  environment / "firmware.bin"), calls[1])

    def test_obstacle_variant_logs_never_upload_or_reset(self):
        for profile in ("obstacle-backup", "obstacle-progress",
                        "obstacle-final-cw", "obstacle-final-ccw"):
            with self.subTest(profile=profile):
                self.calls.unlink(missing_ok=True)
                calls = self.run_field("log-" + profile, "/dev/selected")
                self.assertEqual(len(calls), 1)
                self.assertEqual(calls[0][0], "python")
                self.assertIn(profile, calls[0])
                self.assertNotIn("--reset", calls[0])

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
        self.assertIn("autonomousOpenBackup", calls[0])
        self.assertIn("autonomousObstacle", calls[0])
        self.assertIn("autonomousObstacleBackup", calls[0])
        self.assertIn("autonomousObstacleProgress", calls[0])
        self.assertNotIn("upload", calls[0])

    def test_ports_only_lists_devices(self):
        calls = self.run_field("ports")
        self.assertEqual(len(calls), 1)
        self.assertIn("--list-ports", calls[0])


class PlatformIOProfileTests(unittest.TestCase):
    def test_backup_inherits_primary_and_adds_only_fallback_flag(self):
        config = configparser.ConfigParser(interpolation=None)
        config.read(ROOT / "src" / "ESP32-S3-N16R8" / "platformio.ini")
        primary = config["env:autonomousOpen"]
        backup = config["env:autonomousOpenBackup"]
        self.assertEqual(backup["extends"], "env:autonomousOpen")
        self.assertEqual(backup["build_flags"].split(), [
            "${env:autonomousOpen.build_flags}", "-DBUBBLEGUM_OPEN_OPTICAL_FALLBACK=1"])
        self.assertNotIn("BUBBLEGUM_OPEN_OPTICAL_FALLBACK", primary["build_flags"])

    def test_each_obstacle_profile_sets_one_distinct_recovery_policy(self):
        config = configparser.ConfigParser(interpolation=None)
        config.read(ROOT / "src" / "ESP32-S3-N16R8" / "platformio.ini")
        for environment, policy in (("autonomousObstacle", 0), ("autonomousObstacleBackup", 1),
                                    ("autonomousObstacleProgress", 2)):
            with self.subTest(environment=environment):
                profile = config["env:" + environment]
                flags = profile["build_flags"].split()
                self.assertEqual([flag for flag in flags if "OBSTACLE_RECOVERY_POLICY" in flag],
                                 [f"-DBUBBLEGUM_OBSTACLE_RECOVERY_POLICY={policy}"])
                self.assertIn("-DBUBBLEGUM_AUTONOMY_PROFILE=2", flags)
                if policy:
                    self.assertEqual(profile["extends"], "env:autonomousObstacle")


class SavedOpenTests(unittest.TestCase):
    def test_verifies_all_images_and_rejects_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory)
            expected = {}
            for name in saved_open.EXPECTED_SHA256:
                payload = name.encode()
                (bundle / name).write_bytes(payload)
                expected[name] = hashlib.sha256(payload).hexdigest()
            with mock.patch.object(saved_open, "EXPECTED_SHA256", expected):
                saved_open.verify_bundle(bundle)
                (bundle / "firmware.bin").write_bytes(b"changed source build")
                with self.assertRaisesRegex(RuntimeError, "checksum mismatch"):
                    saved_open.verify_bundle(bundle)

    def test_missing_saved_bundle_does_not_substitute_current_build(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, "does not rebuild"):
                saved_open.verify_bundle(Path(directory))

    def test_exact_s3_four_image_offsets_and_flash_settings(self):
        command = saved_open.flash_command(Path("/saved Open R20"), "/dev/test-port", Path("/tool/esptool.py"))
        self.assertEqual(command[command.index("--chip") + 1], "esp32s3")
        self.assertEqual(command[command.index("--flash_mode") + 1], "dio")
        self.assertEqual(command[command.index("--flash_freq") + 1], "80m")
        self.assertEqual(command[command.index("--flash_size") + 1], "16MB")
        self.assertEqual(command[-8:], [
            "0x0", "/saved Open R20/bootloader.bin",
            "0x8000", "/saved Open R20/partitions.bin",
            "0xe000", "/saved Open R20/boot_app0.bin",
            "0x10000", "/saved Open R20/firmware.bin"])

    def test_check_mode_does_not_find_esptool_or_run_command(self):
        with mock.patch("sys.argv", ["flashSavedOpen.py", "--check"]), \
                mock.patch.object(saved_open, "verify_bundle") as verify, \
                mock.patch.object(saved_open, "find_esptool") as find, \
                mock.patch.object(saved_open.subprocess, "run") as run, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(saved_open.main(), 0)
            verify.assert_called_once()
            find.assert_not_called()
            run.assert_not_called()


class LoggerLeaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        patch = mock.patch.object(logger, "LEASES_DIR", self.directory)
        patch.start()
        self.addCleanup(patch.stop)
        self.port = "/dev/test-logger"
        self.identity = f"Sat Sep 5 12:00:00 2026 python -u {logger.SCRIPT_PATH} --port {self.port}"
        self.lease = dict(pid=999999, port=self.port, script=str(logger.SCRIPT_PATH),
                          process_identity=self.identity)

    def save(self):
        logger.lease_path(self.port).write_text(json.dumps(self.lease))

    def test_verified_same_repo_logger_is_terminated_once(self):
        self.save()
        with mock.patch.object(logger, "process_identity", side_effect=[self.identity, None]), \
                mock.patch.object(logger.os, "kill") as kill, contextlib.redirect_stdout(io.StringIO()):
            logger.stop_existing_logger(self.port)
            kill.assert_called_once_with(999999, logger.signal.SIGTERM)
        self.assertFalse(logger.lease_path(self.port).exists())

    def test_stale_reused_pid_is_not_killed(self):
        self.save()
        with mock.patch.object(logger, "process_identity", return_value="another process"), \
                mock.patch.object(logger.os, "kill") as kill:
            logger.stop_existing_logger(self.port)
            kill.assert_not_called()

    def test_other_repository_lease_is_not_killed(self):
        self.lease["script"] = "/another/repository/tools/fieldLogger.py"
        self.save()
        with mock.patch.object(logger.os, "kill") as kill, self.assertRaises(RuntimeError):
            logger.stop_existing_logger(self.port)
        kill.assert_not_called()

    def test_own_pid_is_never_stopped(self):
        self.lease["pid"] = os.getpid()
        self.save()
        with mock.patch.object(logger.os, "kill") as kill:
            logger.stop_existing_logger(self.port)
            kill.assert_not_called()

    def test_unresponsive_logger_cancels_handoff_without_force_kill(self):
        self.save()
        with mock.patch.object(logger, "process_identity", return_value=self.identity), \
                mock.patch.object(logger.os, "kill") as kill, contextlib.redirect_stdout(io.StringIO()), \
                self.assertRaisesRegex(RuntimeError, "upload canceled"):
            logger.stop_existing_logger(self.port, wait_seconds=0)
        kill.assert_called_once_with(999999, logger.signal.SIGTERM)

    def test_lease_published_and_removed_after_exception(self):
        with mock.patch.object(logger, "process_identity", return_value=self.identity):
            with self.assertRaises(ValueError):
                with logger.logger_lease(self.port):
                    saved = json.loads(logger.lease_path(self.port).read_text())
                    self.assertEqual(saved["pid"], os.getpid())
                    self.assertEqual(saved["port"], self.port)
                    raise ValueError("serial failure")
        self.assertFalse(logger.lease_path(self.port).exists())


if __name__ == "__main__":
    unittest.main()

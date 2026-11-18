#!/usr/bin/env python3
"""Save production ESP32 telemetry. Never send serial commands or start motion."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

REPO_ROOT = Path(__file__).resolve().parent.parent
LOGS_DIR = REPO_ROOT / "local" / "logs"
MAIN_SOURCE = REPO_ROOT / "src" / "ESP32-S3-N16R8" / "src" / "main.cpp"
SCRIPT_PATH = Path(__file__).resolve()
LEASES_DIR = REPO_ROOT / "local" / "fieldLoggerLeases"


def lease_path(port: str) -> Path:
    return LEASES_DIR / (hashlib.sha256(port.encode()).hexdigest() + ".json")


def process_identity(pid: int) -> str | None:
    result = subprocess.run(["ps", "-ww", "-p", str(pid), "-o", "lstart=", "-o", "command="],
                            capture_output=True, text=True, timeout=2)
    return result.stdout.strip() if result.returncode == 0 and result.stdout.strip() else None


def read_lease(path: Path) -> dict | None:
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        return None
    except (ValueError, OSError) as exc:
        raise RuntimeError(f"Cannot read logger lease {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid logger lease: {path}")
    return data


def remove_matching_lease(path: Path, lease: dict) -> None:
    if read_lease(path) == lease:
        path.unlink(missing_ok=True)


def verified_logger_identity(lease: dict, port: str) -> bool:
    pid = lease.get("pid")
    identity = lease.get("process_identity")
    return (type(pid) is int and pid > 1 and lease.get("port") == port
            and lease.get("script") == str(SCRIPT_PATH)
            and isinstance(identity, str)
            and re.search(r"(?:^|\s)" + re.escape(str(SCRIPT_PATH)) + r"(?:\s|$)", identity) is not None
            and process_identity(pid) == identity)


def stop_existing_logger(port: str, wait_seconds: float = 4.0) -> None:
    path = lease_path(port)
    lease = read_lease(path)
    if lease is None:
        return
    if lease.get("pid") == os.getpid():
        return
    if not verified_logger_identity(lease, port):
        # A changed start time/command means a stale PID, never a kill target.
        if lease.get("script") == str(SCRIPT_PATH) and lease.get("port") == port:
            remove_matching_lease(path, lease)
            return
        raise RuntimeError("Logger lease does not identify this repository and port; no process stopped.")
    print(f"Closing this repository's previous logger on {port}; its log will be saved.", flush=True)
    try:
        os.kill(lease["pid"], signal.SIGTERM)
    except ProcessLookupError:
        remove_matching_lease(path, lease)
        return
    deadline = time.monotonic() + wait_seconds
    while time.monotonic() < deadline:
        if read_lease(path) != lease or process_identity(lease["pid"]) != lease["process_identity"]:
            remove_matching_lease(path, lease)
            return
        time.sleep(0.05)
    raise RuntimeError("Previous logger did not close within four seconds; upload canceled. No force-kill was used.")


@contextmanager
def logger_lease(port: str):
    LEASES_DIR.mkdir(parents=True, exist_ok=True)
    path = lease_path(port)
    existing = read_lease(path)
    if existing is not None:
        if verified_logger_identity(existing, port):
            raise RuntimeError(f"This repository is already logging {port}; use ./competition/field.sh to switch.")
        remove_matching_lease(path, existing)
    identity = process_identity(os.getpid())
    if identity is None:
        raise RuntimeError("Could not identify this logger process for automatic port handoff.")
    lease = dict(pid=os.getpid(), port=port, script=str(SCRIPT_PATH), process_identity=identity)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(lease, handle)
    try:
        yield
    finally:
        # The enclosing serial and log contexts have closed before this cleanup.
        remove_matching_lease(path, lease)


def port_score(port: object) -> int:
    """Prefer the ESP32 UART adapter, excluding the CanMV K230 USB interface."""
    if (getattr(port, "vid", None), getattr(port, "pid", None)) == (0x1209, 0xABD1):
        return -100
    text = " ".join(str(getattr(port, attr, "")).lower()
                    for attr in ("description", "manufacturer"))
    device = str(getattr(port, "device", "")).lower()
    score = 100 if (getattr(port, "vid", None), getattr(port, "pid", None)) == (0x0403, 0x6001) else 0
    if any(name in text for name in ("ft232", "ftdi", "cp210", "ch340", "ch910", "usb serial")):
        score += 30
    if any(name in device for name in ("usbserial", "wchusbserial", "slab_usbtouart")):
        score += 20
    return score


def detect_port(ports: list[object]) -> str:
    ranked = sorted(((port_score(port), port) for port in ports),
                    key=lambda item: item[0], reverse=True)
    if ranked and ranked[0][0] > 0:
        best = [port for score, port in ranked if score == ranked[0][0]]
        if len(best) == 1:
            return str(best[0].device)
    raise RuntimeError("Could not choose the ESP32 port. Run ./competition/field.sh ports, "
                       "then add the ESP32 port after the command.")


def source_columns() -> list[str]:
    """Support attaching after boot; an observed header overrides this fallback."""
    try:
        match = re.search(r'"CSV_HEADER,([^"\n]+)"', MAIN_SOURCE.read_text())
    except OSError:
        return []
    return match.group(1).split(",") if match else []


def observed_profile(line: str) -> str | None:
    if line.startswith("BUBBLEGUM AUTONOMOUS OPEN ") and " OPTICAL FALLBACK " in line:
        return "open-backup"
    if line.startswith("BUBBLEGUM AUTONOMOUS OPEN "):
        return "open"
    if line.startswith("BUBBLEGUM AUTONOMOUS OBSTACLE "):
        if "SINGLE PASS" in line:
            return "single-pass"
        if " CONTINUE CW " in line:
            return "obstacle-final-cw"
        if " CONTINUE CCW " in line:
            return "obstacle-final-ccw"
        if " PROGRESS PRIORITY " in line:
            return "obstacle-progress"
        if " RECOVERABLE " in line:
            return "obstacle-backup"
        return "obstacle"
    return None


def telemetry(line: str, columns: list[str]) -> dict[str, str] | None:
    if not line.startswith("DATA,"):
        return None
    values = line.split(",")[1:]
    if len(values) != len(columns):
        return None
    row = dict(zip(columns, values))
    return row if row.get("mode") == "AUTONOMOUS" else None


def is_ready(row: dict[str, str]) -> bool:
    try:
        return (row.get("state") == "WAIT_BUTTON" and row.get("health") == "1"
                and int(row.get("controller_fault_hex", "1"), 16) == 0
                and row.get("actuator_fault") == "0")
    except ValueError:
        return False


def has_zero_output(row: dict[str, str]) -> bool:
    # Older compact logs/tests omit these fields. Full production telemetry
    # includes both, and neither may indicate a drive command during a hold.
    try:
        return (row.get("motion", "0") == "0"
                and int(row.get("applied_drive_permille", "0")) == 0
                and row.get("actuator_fault", "0") == "0")
    except ValueError:
        return False


def is_optical_hold(row: dict[str, str]) -> bool:
    try:
        return (row.get("state") in ("RUN", "DIRECTION_PROBE", "CORNER", "FINISH_CLEAR")
                and has_zero_output(row)
                and int(row.get("controller_fault_hex", "0"), 16) == 0x4)
    except ValueError:
        return False


def active_hold_message(row: dict[str, str]) -> str | None:
    if row.get("state") not in ("RUN", "DIRECTION_PROBE", "CORNER", "FINISH_CLEAR") or not has_zero_output(row):
        return None
    if is_optical_hold(row):
        return ("OPTICAL HOLD: WAITING FOR DISTANCE. "
                "Resumes when fresh readings permit.")
    try:
        flags = int(row.get("controller_fault_hex", "0"), 16)
        if (row.get("state") == "DIRECTION_PROBE"
                and flags == 0x10000):
            return ("WAITING FOR CORNER DIRECTION: "
                    "resumes when fresh readings identify a usable turn.")
    except ValueError:
        return None
    reasons = (
        (0x4, "front distance is unusable"),
        (0x80, "front clearance is too small for the current maneuver"),
        (0x300, "waiting for fresh camera data"),
        (0x400, "direction probing timed out"),
        (0x4000, "the corner turn timed out"),
        (0x10000, "the corner direction is still unconfirmed"),
        (0x40000, "finish clearance timed out"),
        (0x100000, "the pillar pass timed out"),
    )
    known_mask = sum(mask for mask, _ in reasons)
    if flags == 0 or flags & ~known_mask:
        return None
    detail = "; ".join(reason for mask, reason in reasons if flags & mask)
    return f"RECOVERY HOLD: {detail}. Will retry when current readings permit."


def fault_explanation(row: dict[str, str]) -> str:
    """Explain observed controller flags without declaring a sensor broken."""
    try:
        flags = int(row.get("controller_fault_hex", "0"), 16)
    except ValueError:
        return "Unrecognized fault value; keep the raw log."
    reasons = []
    if flags & 0x10000:
        reasons.append("Turn direction was not confirmed with enough front clearance.")
    if flags & 0x4:
        reasons.append("Front distance stayed unusable; an online sensor can still reject its optical reading.")
    if flags & 0x80:
        reasons.append("Measured front clearance reached the emergency stop distance.")
    if flags & 0x28:
        reasons.append("Distance data became stale.")
    return " ".join(reasons) or "See the saved sensor readings and controller fault flags."


def install_shutdown_handlers() -> None:
    def interrupt(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt
    for name in ("SIGTERM", "SIGHUP"):
        value = getattr(signal, name, None)
        if value is not None:
            signal.signal(value, interrupt)


def open_with_retry(connection: object, retry_seconds: float) -> None:
    deadline = time.monotonic() + retry_seconds
    while True:
        try:
            connection.open()
            return
        except OSError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.25)


def record(connection: object, log_file: object, profile: str, reset: bool) -> None:
    columns = [] if reset else source_columns()
    banner_seen = False
    ready_announced = False
    start_clearance_blocked = False
    previous_hold_message = None
    previous_state = None
    while True:
        raw = connection.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        stamped = f"{datetime.now().isoformat(timespec='milliseconds')} {line}"
        print(stamped, flush=True)
        log_file.write(stamped + "\n")
        detected = observed_profile(line)
        if detected is not None:
            if detected != profile:
                raise RuntimeError(f"Wrong firmware: expected {profile}, observed {detected}. "
                                   f"Upload with ./competition/field.sh {profile}.")
            banner_seen = True
            columns = []
            ready_announced = False
            start_clearance_blocked = False
            previous_hold_message = None
            previous_state = None
            continue
        if line.startswith("CSV_HEADER,"):
            columns = line.split(",")[1:]
            continue
        row = telemetry(line, columns)
        if row is None:
            continue
        blocked_now = (
            profile in ("open", "open-backup") and row.get("state") == "WAIT_HEALTH"
            and row.get("controller_fault_hex", "").lower() in ("0x10000", "0x10080")
        )
        if blocked_now and not start_clearance_blocked:
            print("\nWAITING FOR START SPACE: front clearance must exceed 650 mm; "
                  "release Start while repositioning.\n", flush=True)
            ready_announced = False
        start_clearance_blocked = blocked_now
        hold_message = active_hold_message(row)
        if hold_message is not None and hold_message != previous_hold_message:
            print(f"\n{hold_message}\n", flush=True)
        previous_hold_message = hold_message
        if banner_seen and is_ready(row) and not ready_announced:
            print("\nREADY: press the physical start button once.\n", flush=True)
            ready_announced = True
        state = row.get("state")
        if state != previous_state:
            if state == "FAULT_LATCHED":
                print(f"\nSTOPPED: fault={row.get('controller_fault_hex')}. "
                      f"{fault_explanation(row)} Log continues.\n", flush=True)
            elif state == "FINISHED":
                print(f"\nFINISHED: laps={row.get('laps')}, corners={row.get('corners')}; log continues.\n", flush=True)
            previous_state = state


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("open", "open-backup", "obstacle",
                                               "obstacle-backup", "obstacle-progress",
                                               "obstacle-final-cw", "obstacle-final-ccw"),
                        default="obstacle")
    parser.add_argument("--port", help="ESP32 serial port; auto-detected if omitted")
    parser.add_argument("--reset", action="store_true",
                        help="Reset the ESP32 once to capture its boot banner; use after upload")
    parser.add_argument("--firmware", type=Path, help="Add this uploaded firmware's SHA-256 to the log")
    parser.add_argument("--log-dir", type=Path, default=LOGS_DIR)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list-ports", action="store_true")
    mode.add_argument("--detect-port", action="store_true")
    mode.add_argument("--stop-existing", action="store_true",
                      help="Close only this repository's verified logger on --port, saving its log")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.stop_existing:
        if not args.port:
            print("--stop-existing requires --port.", file=sys.stderr)
            return 2
        try:
            stop_existing_logger(args.port)
            return 0
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            print(str(exc), file=sys.stderr)
            return 1
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("pyserial is missing. Use PlatformIO's Python or install it with "
              f"{sys.executable} -m pip install pyserial", file=sys.stderr)
        return 2
    try:
        ports = list(list_ports.comports()) if not args.port or args.list_ports else []
        if args.list_ports:
            for item in ports:
                print(f"{item.device}: {item.description}")
            if not ports:
                print("No serial ports found.")
            return 0
        port = args.port or detect_port(ports)
        if args.detect_port:
            print(port)
            return 0
        digest = hashlib.sha256(args.firmware.read_bytes()).hexdigest() if args.firmware else None
        args.log_dir.mkdir(parents=True, exist_ok=True)
        started = datetime.now()
        path = args.log_dir / f"autonomousRun-{started:%Y%m%d-%H%M%S-%f}.txt"
        print(f"Saving to {path}", flush=True)
        print("Listening only; Ctrl+C saves the log and does not stop the robot.", flush=True)
        if not args.reset:
            print("Attaching without reset; firmware profile is unverified until a boot banner arrives.", flush=True)
        install_shutdown_handlers()
        with logger_lease(port), path.open("x", encoding="utf-8", buffering=1) as log_file:
            log_file.write(f"# started={started.isoformat()} expected={args.profile} port={port}\n")
            if digest:
                log_file.write(f"# firmware_sha256={digest}\n")
            # Configure inactive modem-control lines BEFORE opening. Merely
            # attaching to a running robot must not intentionally reset it.
            with serial.Serial(port=None, baudrate=115200, timeout=0.1, exclusive=True) as connection:
                connection.dtr = False
                connection.rts = False
                connection.port = port
                open_with_retry(connection, 12.0 if args.reset else 0.0)
                if args.reset:
                    connection.rts = True
                    time.sleep(0.1)
                    connection.reset_input_buffer()
                    connection.rts = False
                try:
                    record(connection, log_file, args.profile, args.reset)
                except KeyboardInterrupt:
                    print(f"\nSaved: {path}", flush=True)
        return 0
    except (OSError, RuntimeError, serial.SerialException, subprocess.TimeoutExpired) as exc:
        print(f"{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

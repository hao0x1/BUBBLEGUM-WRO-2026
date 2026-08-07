#!/usr/bin/env python3
"""Save production ESP32 telemetry. Never send serial commands or start motion."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
from pathlib import Path
import re
import signal
import sys
import time

REPO_ROOT = Path(__file__).resolve().parent.parent
LOGS_DIR = REPO_ROOT / "local" / "logs"
MAIN_SOURCE = REPO_ROOT / "src" / "ESP32-S3-N16R8" / "src" / "main.cpp"


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
    if line.startswith("BUBBLEGUM AUTONOMOUS OPEN "):
        return "open"
    if line.startswith("BUBBLEGUM AUTONOMOUS OBSTACLE "):
        return "single-pass" if "SINGLE PASS" in line else "obstacle"
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
            previous_state = None
            continue
        if line.startswith("CSV_HEADER,"):
            columns = line.split(",")[1:]
            continue
        row = telemetry(line, columns)
        if row is None:
            continue
        if banner_seen and is_ready(row) and not ready_announced:
            print("\nREADY: press the physical start button once.\n", flush=True)
            ready_announced = True
        state = row.get("state")
        if state != previous_state:
            if state == "FAULT_LATCHED":
                print(f"\nSTOPPED: fault={row.get('controller_fault_hex')}; log continues.\n", flush=True)
            elif state == "FINISHED":
                print(f"\nFINISHED: laps={row.get('laps')}, corners={row.get('corners')}; log continues.\n", flush=True)
            previous_state = state


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("open", "obstacle"), default="obstacle")
    parser.add_argument("--port", help="ESP32 serial port; auto-detected if omitted")
    parser.add_argument("--reset", action="store_true",
                        help="Reset the ESP32 once to capture its boot banner; use after upload")
    parser.add_argument("--firmware", type=Path, help="Add this uploaded firmware's SHA-256 to the log")
    parser.add_argument("--log-dir", type=Path, default=LOGS_DIR)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list-ports", action="store_true")
    mode.add_argument("--detect-port", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
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
        with path.open("x", encoding="utf-8", buffering=1) as log_file:
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
    except (OSError, RuntimeError, serial.SerialException) as exc:
        print(f"{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

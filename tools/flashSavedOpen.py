#!/usr/bin/env python3
"""Verify and flash the saved Open R20 images without rebuilding current sources."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shlex
import subprocess
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BUNDLE = REPO_ROOT / "local" / "competitionFirmware" / "open-r20"
# Pin the known snapshot independently of the private bundle's manifest.
# Bootloader, partitions and app match Open-R20-field-recovery-20260905-095433.
# boot_app0 is the standard image from the same installed Arduino framework.
EXPECTED_SHA256 = {
    "bootloader.bin": "2a71d69b471e20c2bac7fb469f3c6a807b3ebee780e348e5889db0da849ca363",
    "partitions.bin": "bd0f7954aca2ef7d925ee21aaa1f3dc8822d1d6ce5cbbd26a135e5886bfff6ce",
    "boot_app0.bin": "f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0",
    "firmware.bin": "09dfc52b9daa4e0ca57b82038076e58b247d30d7484543aa86b84381ff0340ae",
}
FLASH_PARTS = (
    ("0x0", "bootloader.bin"),
    ("0x8000", "partitions.bin"),
    ("0xe000", "boot_app0.bin"),
    ("0x10000", "firmware.bin"),
)


def verify_bundle(bundle: Path) -> None:
    for name, expected in EXPECTED_SHA256.items():
        path = bundle / name
        if not path.is_file():
            raise RuntimeError(
                f"Saved Open R20 image missing: {path}\n"
                "Restore local/competitionFirmware/open-r20 from the prepared laptop. "
                "This command does not rebuild or substitute newer firmware. "
                "To build the current source separately: ./competition/field.sh build"
            )
        observed = hashlib.sha256(path.read_bytes()).hexdigest()
        if observed != expected:
            raise RuntimeError(f"Saved Open R20 checksum mismatch: {path}; no upload performed.")


def find_esptool(override: Path | None = None) -> Path:
    if override is not None:
        path = override
    else:
        core = Path(os.environ.get("PLATFORMIO_CORE_DIR", str(Path.home() / ".platformio")))
        path = core / "packages" / "tool-esptoolpy" / "esptool.py"
    if not path.is_file():
        raise RuntimeError(f"PlatformIO esptool not found: {path}. Use --esptool PATH.")
    return path


def flash_command(bundle: Path, port: str, esptool: Path) -> list[str]:
    # Matches the prepared PlatformIO ESP32-S3 Arduino uploader. The saved
    # bootloader header confirms DIO, 80 MHz and 16 MB; chip ID is ESP32-S3.
    command = [sys.executable, str(esptool), "--chip", "esp32s3", "--port", port,
               "--baud", "460800", "--before", "default_reset", "--after", "hard_reset",
               "write_flash", "-z", "--flash_mode", "dio", "--flash_freq", "80m",
               "--flash_size", "16MB"]
    for offset, name in FLASH_PARTS:
        command.extend((offset, str(bundle / name)))
    return command


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--port", help="ESP32 UART port; required for an upload")
    parser.add_argument("--esptool", type=Path, help="Override PlatformIO's esptool.py location")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="Verify saved hashes only; no device access")
    mode.add_argument("--dry-run", action="store_true", help="Verify and print the upload command; no device access")
    args = parser.parse_args()
    if not args.check and not args.port:
        parser.error("--port is required unless using --check")
    try:
        verify_bundle(args.bundle)
        print("Saved Open R20: all four firmware images verified.", flush=True)
        if args.check:
            return 0
        command = flash_command(args.bundle, args.port, find_esptool(args.esptool))
        if args.dry_run:
            print(shlex.join(command))
            return 0
        subprocess.run(command, check=True)
        print("Saved Open R20 upload completed; firmware was not rebuilt.", flush=True)
        return 0
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

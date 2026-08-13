"""Host-only check: the deployable installer embeds the current K230 source.

Run with Python 3 on the computer. This parses literals without executing the
installer or touching any serial port or SD card.
"""

import ast
import hashlib
from pathlib import Path


def verify():
    directory = Path(__file__).resolve().parent
    source = directory.parents[1] / "src" / "LCKFB-Lushan-Pi-K230" / "main.py"
    installer = directory / "installProduction.py"
    constants = {}
    wanted = {
        "PAYLOAD_CHUNKS", "EXPECTED_SIZE", "EXPECTED_SHA256",
        "EXPECTED_SHA256_HEX", "TARGET_PATH", "TEMP_PATH", "BACKUP_PATH",
    }
    for node in ast.parse(installer.read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name) and target.id in wanted:
                constants[target.id] = ast.literal_eval(node.value)
    if set(constants) != wanted:
        raise ValueError("installer constants are missing")
    payload = b"".join(constants["PAYLOAD_CHUNKS"])
    data = source.read_bytes()
    digest = hashlib.sha256(data)
    if payload != data:
        raise ValueError("installer embeds a different main.py; do not deploy it")
    if (constants["EXPECTED_SIZE"] != len(data)
            or constants["EXPECTED_SHA256"] != digest.digest()
            or constants["EXPECTED_SHA256_HEX"] != digest.hexdigest()):
        raise ValueError("installer size/hash metadata does not match main.py")
    if [constants[name] for name in ("TARGET_PATH", "TEMP_PATH", "BACKUP_PATH")] != [
            "/sdcard/main.py", "/sdcard/main.py.new", "/sdcard/main.py.bak"]:
        raise ValueError("unexpected installer destination")
    ast.parse(data, filename=str(source))
    ast.parse((directory / "colorCalibration.py").read_text())
    print("K230 INSTALLER VERIFIED: embedded main.py matches repository source")
    print("bytes:", len(data))
    print("sha256:", digest.hexdigest())


if __name__ == "__main__":
    verify()

#!/usr/bin/env bash
# Offline regression checks; never connects to a board.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ESP32="$REPO_ROOT/src/ESP32-S3-N16R8"
TESTS="$REPO_ROOT/tests/host"
CHECK_CXX="${CXX:-clang++}"
CHECK_TMP="$(mktemp -d "${TMPDIR:-/tmp}/bubblegum-checks.XXXXXX")"
trap 'rm -rf -- "$CHECK_TMP"' EXIT
if [[ -x "$HOME/.platformio/penv/bin/python" ]]; then
  CHECK_PYTHON="${FIELD_PYTHON:-$HOME/.platformio/penv/bin/python}"
else
  CHECK_PYTHON="${FIELD_PYTHON:-python3}"
fi

"$CHECK_CXX" -std=c++17 -Wall -Wextra -Werror -I "$ESP32/include" \
  "$TESTS/esp32Tests.cpp" "$ESP32/src/autonomyController.cpp" \
  "$ESP32/src/controllerProfiles.cpp" "$ESP32/src/visionProtocol.cpp" \
  -o "$CHECK_TMP/controller"
"$CHECK_TMP/controller"
"$CHECK_CXX" -std=c++17 -Wall -Wextra -Werror \
  -I "$TESTS/actuatorStubs" -I "$ESP32/include" \
  "$TESTS/vehicleActuatorTests.cpp" "$ESP32/src/vehicleActuators.cpp" \
  -o "$CHECK_TMP/actuators"
"$CHECK_TMP/actuators"
"$CHECK_CXX" -std=c++17 -Wall -Wextra -Werror -I "$ESP32/include" \
  "$TESTS/runBlackBoxTests.cpp" "$ESP32/src/runBlackBox.cpp" \
  -o "$CHECK_TMP/blackbox"
"$CHECK_TMP/blackbox"
"$CHECK_PYTHON" -m unittest discover -s "$REPO_ROOT/tools/tests" -p 'test*.py'
"$CHECK_PYTHON" "$REPO_ROOT/tools/k230/verifyInstaller.py"
bash -n "$REPO_ROOT/competition/field.sh"
printf '%s\n' 'All offline checks passed. No device was accessed.'

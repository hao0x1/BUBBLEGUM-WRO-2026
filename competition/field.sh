#!/usr/bin/env bash
# Select the competition firmware, upload it, and record its telemetry.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ESP32_PROJECT="$REPO_ROOT/src/ESP32-S3-N16R8"
LOGGER="$REPO_ROOT/tools/fieldLogger.py"
SAVED_OPEN="$REPO_ROOT/tools/flashSavedOpen.py"
SAVED_OPEN_BUNDLE="$REPO_ROOT/local/competitionFirmware/open-r20"

usage() {
  cat <<'EOF'
Usage: ./competition/field.sh COMMAND [ESP32_PORT]

  open                   Upload exact saved Open R20 without rebuilding; reset and log.
  open-backup            Build/upload Open R21 optical fallback; reset and log.
  obstacle               Build/upload normal Obstacle R42; reset and log.
  obstacle-backup        Build/upload Obstacle R43 recoverable; reset and log.
  obstacle-progress      Build/upload Obstacle R44 progress priority; reset and log.
  obstacle-final-cw      Build/upload R45 optical-open fallback; turn RIGHT/CW.
  obstacle-final-ccw     Build/upload R45 optical-open fallback; turn LEFT/CCW.
  log-open               Record the primary Open firmware without resetting.
  log-open-backup        Record the Open optical fallback without resetting.
  log-obstacle           Record normal Obstacle firmware without resetting.
  log-obstacle-backup    Record recoverable Obstacle firmware without resetting.
  log-obstacle-progress  Record progress-priority Obstacle firmware without resetting.
  log-obstacle-final-cw  Record R45 clockwise firmware without resetting.
  log-obstacle-final-ccw Record R45 counter-clockwise firmware without resetting.
  build                  Build all competition profiles; no device connection.
  ports                  List serial ports; no upload or reset.

Open and Obstacle both stop after 3 laps. Press the physical start button
once after READY. Ctrl+C closes the log; it does not stop the robot.
Logs and build snapshots belong in local/ (ignored by Git).
The K230 runs the same src/LCKFB-Lushan-Pi-K230/main.py for both rounds.

The ESP32 port is detected if omitted. ESP32_PORT can also select it.
Optional tool overrides: FIELD_PYTHON and PLATFORMIO.
EOF
}

ACTION="${1:-help}"
case "$ACTION" in
  help|-h|--help) usage; exit 0 ;;
  open|open-backup|obstacle|obstacle-backup|obstacle-progress|obstacle-final-cw|obstacle-final-ccw|log-open|log-open-backup|log-obstacle|log-obstacle-backup|log-obstacle-progress|log-obstacle-final-cw|log-obstacle-final-ccw|build|ports) ;;
  *) usage >&2; exit 2 ;;
esac
if (( $# > 2 )); then usage >&2; exit 2; fi

FIELD_PYTHON="${FIELD_PYTHON:-}"
if [[ -z "$FIELD_PYTHON" ]]; then
  if [[ -x "$HOME/.platformio/penv/bin/python" ]]; then
    FIELD_PYTHON="$HOME/.platformio/penv/bin/python"
  else
    FIELD_PYTHON="$(command -v python3 || true)"
  fi
fi
if [[ -z "$FIELD_PYTHON" ]]; then
  echo "Python 3 was not found. Set FIELD_PYTHON to your PlatformIO Python." >&2
  exit 2
fi

if [[ "$ACTION" == ports ]]; then
  exec "$FIELD_PYTHON" "$LOGGER" --list-ports
fi

if [[ "$ACTION" != log-* && "$ACTION" != open ]]; then
  PLATFORMIO="${PLATFORMIO:-}"
  if [[ -z "$PLATFORMIO" ]]; then
    if [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
      PLATFORMIO="$HOME/.platformio/penv/bin/pio"
    else
      PLATFORMIO="$(command -v pio || command -v platformio || true)"
    fi
  fi
  if [[ -z "$PLATFORMIO" ]]; then
    echo "PlatformIO was not found. Set PLATFORMIO to the pio executable." >&2
    exit 2
  fi
fi

if [[ "$ACTION" == build ]]; then
  exec "$PLATFORMIO" run --project-dir "$ESP32_PROJECT" \
    -e autonomousOpen -e autonomousOpenBackup -e autonomousObstacle \
    -e autonomousObstacleBackup -e autonomousObstacleProgress \
    -e autonomousObstacleContinueCw -e autonomousObstacleContinueCcw
fi

PROFILE="${ACTION#log-}"
case "$PROFILE" in
  open) ENVIRONMENT=autonomousOpen ;;
  open-backup) ENVIRONMENT=autonomousOpenBackup ;;
  obstacle) ENVIRONMENT=autonomousObstacle ;;
  obstacle-backup) ENVIRONMENT=autonomousObstacleBackup ;;
  obstacle-progress) ENVIRONMENT=autonomousObstacleProgress ;;
  obstacle-final-cw) ENVIRONMENT=autonomousObstacleContinueCw ;;
  obstacle-final-ccw) ENVIRONMENT=autonomousObstacleContinueCcw ;;
esac
PORT="${2:-${ESP32_PORT:-}}"
if [[ -z "$PORT" ]]; then
  PORT="$("$FIELD_PYTHON" "$LOGGER" --detect-port)"
fi

"$FIELD_PYTHON" "$LOGGER" --stop-existing --port "$PORT"
LOGGER_ARGS=(--profile "$PROFILE" --port "$PORT")
if [[ "$ACTION" == open ]]; then
  echo "Uploading saved Open R20 to $PORT. Close another serial monitor if it owns this port."
  "$FIELD_PYTHON" "$SAVED_OPEN" --bundle "$SAVED_OPEN_BUNDLE" --port "$PORT"
  LOGGER_ARGS+=(--reset --firmware "$SAVED_OPEN_BUNDLE/firmware.bin")
elif [[ "$ACTION" != log-* ]]; then
  echo "Uploading $ENVIRONMENT to $PORT. Close another serial monitor if it owns this port."
  "$PLATFORMIO" run --project-dir "$ESP32_PROJECT" \
    -e "$ENVIRONMENT" -t upload --upload-port "$PORT"
  LOGGER_ARGS+=(--reset --firmware "$ESP32_PROJECT/.pio/build/$ENVIRONMENT/firmware.bin")
fi
exec "$FIELD_PYTHON" -u "$LOGGER" "${LOGGER_ARGS[@]}"

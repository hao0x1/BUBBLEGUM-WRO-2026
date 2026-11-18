# Competition commands

Use this Mac's existing PlatformIO installation. No new setup is needed.

```bash
cd /Users/zhihaowu/futureEngineers/BUBBLEGUM-WRO-2026
```

## Final weak-wall obstacle build (R45)

Use the direction announced for the round. These two commands contain the same
controller; only the fixed square-course turn direction differs:

```bash
# Clockwise course: every corner turns RIGHT
./competition/field.sh obstacle-final-cw /dev/cu.usbserial-A5069RR4

# Counter-clockwise course: every corner turns LEFT
./competition/field.sh obstacle-final-ccw /dev/cu.usbserial-A5069RR4
```

R45 uses the 50 ms side-ToF timing budget. A fresh responding side sensor with
optical status 1, 2, 4, 6, or 7 is treated as open space instead of blocking at
`WAIT_HEALTH` or later fault `0x40`. Measured side ranges still steer away from
a close pillar; camera red/green steering remains active when observations are
available. A measured centre-front distance at or below 180 mm still starts a
bounded reverse, and fresh optical-open rear samples allow the controller to
leave reverse and drive forward again. It counts 12 IMU-confirmed 90-degree
corners, reports three laps, then stops. Parking remains disabled.

Do not cover sensor D to make READY. Uncovered fresh `status=2` is accepted by
this profile. Wait for `READY`, press Start once, and leave the logger running.

| Round | ESP32 firmware | K230 |
| --- | --- | --- |
| Open first choice | `autonomousOpen`, R20 field recovery | Same `src/LCKFB-Lushan-Pi-K230/main.py` |
| Open backup | `autonomousOpenBackup`, R21 optical fallback | Same `src/LCKFB-Lushan-Pi-K230/main.py` |
| Obstacle normal | `autonomousObstacle`, R42 | Same camera program |
| Obstacle recovery backup | `autonomousObstacleBackup`, R43 | Same camera program |
| Obstacle progress backup | `autonomousObstacleProgress`, R44 | Same camera program; measured-route fallback if camera is unavailable |
| Obstacle final clockwise | `autonomousObstacleContinueCw`, R45 | Same camera program; camera optional; turns right |
| Obstacle final counter-clockwise | `autonomousObstacleContinueCcw`, R45 | Same camera program; camera optional; turns left |

Open R18 completed three physical laps at home. R20 subsequently faulted on this field; it is preserved as a separate choice, not declared successful. The latest downloaded black box reports front-distance loss (`0x4`) during a clockwise run. All obstacle choices target twelve corner exits (three laps), then stop. They use a normal start, with parking and pillar-location memory disabled. Their full three-lap field result remains unverified.

## Obstacle choices for the field

Select one command, not all three together:

```bash
./competition/field.sh obstacle
```

Normal R42. This is the version uploaded on September 5 at 10:22. It retains the existing camera-guided controller and bounded wall/pillar backoff.

```bash
./competition/field.sh obstacle-backup
```

R43 recovery backup. Keeps required camera observations and red-right/green-left avoidance. Stopped sensor/camera waits do not consume maneuver timers. A known corner can use fresh heading, camera and a usable side ray during strong optical front loss. Recoverable navigation holds preserve the original turn target and lap count; retry requires new usable evidence.

```bash
./competition/field.sh obstacle-progress
```

R44 progress backup. Includes R43 recovery and permits measured-route travel when camera observations are missing. It can drop an obsolete pillar commitment without claiming a successful pass. New valid detections are used when available. Passing every pillar on the correct side is less certain in this mode.

The backups are not unconditional motor-on programs: measured close objects trigger stop/backoff, electrical/IMU loss stops motion, and the total 170-second run bound remains. Recovery cannot determine a free path from entirely missing distance evidence. Speeds retain the established 400 straight / 450 corner / 300 reverse settings; these are motor power per thousand, not measured velocity.

Every command closes the same-repository logger for that port, flashes only the ESP32, captures the firmware banner, and starts a new log. Logs are preserved. If another application owns the port, close that application's serial monitor. After placing the car at its normal start, wait for `READY` and press physical Start once. The same K230 installation and saved colour calibration are used for all three. Do not reinstall it just to switch profiles.

## Flash and record

Open round:

```bash
./competition/field.sh open
```

Open backup, only if choosing the separate R21 fallback:

```bash
./competition/field.sh open-backup
```

Close the current logger with `Ctrl+C` before switching firmware. Check the banner: primary is `R20 FIELD RECOVERY`; backup is `R21 OPTICAL FALLBACK`. Both commands upload only the ESP32 and start logging. Return to R20 with `./competition/field.sh open`.

The backup changes optical-loss handling, not the physical turning calibration. A fresh optical dropout permits a bounded initial probe and then a stopped hold; useful measurements must return before another probe is permitted. Direction ambiguity holds and keeps checking instead of immediately ending the run. Measured close objects, sensor communication loss, IMU faults, the 170-second total run limit, and the permanent three-lap finish remain enforced. Completely missing direction evidence can still leave it waiting: invalid readings are not proof of open space.

Obstacle round:

```bash
./competition/field.sh obstacle
```

`open` restores the exact saved R20 images from `local/competitionFirmware/open-r20/`, without rebuilding. The other selections build their separate source profiles. All selections upload **only the ESP32**, reset it, and record its boot and run. Wait for `READY`, then press the physical start button once. Powered uploads and pressing the button while USB is connected are supported by this workflow. Use the vehicle switch to stop motion; `Ctrl+C` only closes the logger.

Open R20 waits before Start if measured front clearance is at or below 600 mm. Move to the proper starting position with more than 650 mm ahead, release Start, and wait for READY. During the first approach it uses at most 400 power inside one metre. At the 600 mm boundary it stops to confirm direction. If a direction is confirmed and the car has just driven this straight approach, it may reverse straight at 300 power to create about 900 mm of front clearance, pause, then turn. This is limited to one first-corner recovery, at most 1.5 seconds of powered reverse and 500 mm measured retreat. Missing path evidence, a measured rear obstruction, or persistent ambiguity still stops the run.

If the log reports `0x4`, the front distance stayed unusable. `0x10000` means the controller could not confirm a turn with sufficient clearance. Open R20 can recover a late but confirmed opening; it cannot determine a direction when all four side readings remain invalid. An online sensor with `status=2 valid=no` is communicating but rejecting its optical measurement. A finger test does not establish wall-range performance. Before another field run, check a flat white target about 500 mm directly along each sensor beam while stationary; expect `status=0 valid=yes` and a range close to the measured target distance. Check that the mounted beams intersect the wall faces below their top edges.

If the port cannot be chosen automatically:

```bash
./competition/field.sh ports
./competition/field.sh obstacle /dev/cu.usbserial-A5069RR4
```

Close an existing serial monitor with `Ctrl+C` before another upload. Use the port printed for the ESP32 UART adapter, rather than the K230 USB port.

## Record without flashing

```bash
./competition/field.sh log-open
./competition/field.sh log-open-backup
./competition/field.sh log-obstacle
./competition/field.sh log-obstacle-backup
./competition/field.sh log-obstacle-progress
```

Choose one. These commands attach without requesting a reset and never send start or motor commands. They save all serial output in `local/logs/`. When attaching after boot, the logger may not have seen the firmware identity banner; upload commands capture it automatically.

For another run after `FINISHED` or a latched fault: start the matching log command, press the ESP32 **RST** button once, wait for `READY`, then press the car's start button. No new upload is needed for another run of the same version.

## Prepare offline

```bash
./competition/field.sh build
```

This compiles the current default Open source, R21 backup, and the three Obstacle profiles without touching a device. It does not replace the frozen R20 images used by `open`; preserve `local/competitionFirmware/open-r20/` on this laptop. Run `python3 tools/flashSavedOpen.py --check` to verify that saved bundle offline. Current sources live under `src/ESP32-S3-N16R8/` and `src/LCKFB-Lushan-Pi-K230/`. Public field tools live under `competition/` and `tools/`. Logs, archived firmware, calibration experiments, and reference clones live together in `local/`, which is ignored by Git.

## K230: same program for both rounds

Keep its current calibrated installation. Changing ESP32 rounds does not require changing the K230.

If reinstalling the K230 becomes necessary:

1. Open `tools/k230/installProduction.py` in VS Code/CanMV.
2. Use **Run Active File on K230**.
3. Wait for `INSTALL OK: /sdcard/main.py VERIFIED`, then restart the K230.

This preserves the board's saved `colorThresholds.cfg`. Do not flash an ESP32 `.bin` file to the K230. [K230 instructions](../tools/k230/README.md) include the optional lighting calibration.

Offline installer check:

```bash
python3 tools/k230/verifyInstaller.py
```

# BUBBLEGUM WRO Future Engineers 2026

Competition source and field tools for our ESP32-S3 / Lushan-Pi K230 car.

**Start here: [competition/QUICK_START.md](competition/QUICK_START.md).** It contains the exact upload and logging commands for both rounds. No assistant or internet access is needed on the prepared laptop.

## Which version to use

| Round / board | Selection | Evidence |
| --- | --- | --- |
| Open — ESP32 | `autonomousOpen`, R18 configuration | Previously completed 12 corners / three laps in the physical Open test. Speed settings are unchanged. |
| Obstacle — ESP32 | `autonomousObstacle`, R42 | Latest recovery/progress revision; built and host-tested, awaiting its field run. Three laps, normal start, no parking. |
| Either round — K230 | [main.py](src/LCKFB-Lushan-Pi-K230/main.py) | Same production camera program for both rounds. Existing SD colour calibration is preserved. |

From this repository on the prepared Mac:

```sh
./competition/field.sh open
```

or:

```sh
./competition/field.sh obstacle
```

Each command builds/uploads the selected ESP32 profile and opens a passive log. The car waits for its physical start button. Wait for the logger's `READY` message, then press the start button once. `READY` confirms `WAIT_BUTTON` with healthy telemetry and no controller or actuator fault. Logs are saved in `local/logs/`.

## Current behavior

The ESP32 reads five ToF distance sensors and an XMS-A5/BNO055 heading sensor, drives the steering servo and motor, and counts verified corner exits. Twelve corners form three laps. The K230 reports red/green image observations over UART; it does not command motors.

Open uses its existing R18 speed and corner settings. Obstacle uses 400-permille straight power, 450 for tight corners, and 300 for bounded reverse. It follows live red-right / green-left guidance when usable, continues through measured free space when a target disappears, and attempts recovery when blocked. Pillar-section learning is disabled; a known failed target is briefly suppressed after reversing to avoid immediately repeating the same maneuver. Distance sensors remain active during that suppression. Parking and parking-lot departure are not part of this competition revision.

The front-left/front-right sensors are mounted narrower than the rear pair; the four side rays point outward at approximately 45 degrees. Their raw front-minus-rear distances are not treated as calibrated wall angles. The source pin mapping is in [boardConfig.h](src/ESP32-S3-N16R8/include/boardConfig.h).

## Repository layout

- [src/ESP32-S3-N16R8](src/ESP32-S3-N16R8): current PlatformIO controller, sensors, actuators and configuration.
- [src/LCKFB-Lushan-Pi-K230](src/LCKFB-Lushan-Pi-K230): production camera program.
- [competition](competition): short field guide and upload/log command.
- [tools](tools): portable logger, K230 installer/calibrator and offline checks.
- [tests](tests): host regression tests for the controller and actuator driver.
- [competition.md](competition.md): supplied competition reference.
- [models](models), [schemes](schemes), [other](other): CAD, wiring drawings and component records.
- [t-photos](t-photos), [v-photos](v-photos), [video](video): team/vehicle photos and driving-video link.
- `local/`: ignored logs, firmware copies, old experiments, backups and private test records. Keep this folder on the laptop; it is excluded from Git.

Some hardware drawings and component records predate the latest electronics changes. The current source and field guide identify the configuration prepared for this run.

## Offline checks

```sh
./tools/check.sh
./competition/field.sh build
```

Host tests exercise state transitions, mirrored steering, recovery timing and three-lap completion. They do not establish a successful physical Obstacle run. The field guide identifies the remaining physical verification. No Git commit or push is performed by the field tools.

# ESP32-S3 controller

Use [the competition guide](../../competition/QUICK_START.md) from the repository root.

| PlatformIO environment | Purpose |
| --- | --- |
| `autonomousOpen` | Open round, R18 configuration, three laps |
| `autonomousObstacle` | Obstacle round, R42, three laps, normal start, parking disabled |
| `dryRun` | Diagnostics with actuator code disabled; not a competition driving build |
| `obstacleSinglePassCalibration` | Historical strict single-pillar test; not the full obstacle round |
| `autonomous` | Compatibility alias for `autonomousOpen` |

Build without connecting a board:

```sh
../../competition/field.sh build
```

Or from the repository root:

```sh
./competition/field.sh open
./competition/field.sh obstacle
```

These are alternative uploads to the same ESP32, not two programs installed together. Both use the same K230 camera program. Open does not require camera observations to drive; Obstacle requires a fresh camera connection but no visible pillar is required for ordinary clear-space travel.

`include/controllerProfiles.h` and `src/controllerProfiles.cpp` define the exact tested profiles. `main.cpp` samples sensors, applies decisions and logs telemetry. `autonomyController.cpp` owns navigation and recovery. `vehicleActuators.cpp` owns PWM, direction-neutral timing and the motor command lease.

Positive steering is right; positive drive is forward. The motor starts only after a physical button press. Closing the logger does not stop the car. A completed run or latched fault requires a reset before the next start.

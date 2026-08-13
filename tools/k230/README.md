# K230 competition deployment

Use the same K230 program for both rounds. Open uses ESP32 distance sensors and
the IMU and does not require camera observations. Obstacle uses the K230 camera
when a red or green pillar is visible. Parking is disabled.

The current K230 production program and its color calibration are unchanged by
the final ESP32 recovery update. If this program is already installed and the
camera is working, leave the K230 as it is; switching rounds only requires the
ESP32 firmware change described in the competition guide.

## Install production, only if needed

1. Connect the K230 to the computer and open
   [`installProduction.py`](installProduction.py) in VS Code.
2. Select the K230 in the existing CanMV extension, then run the command
   **CanMV: Run Active File on K230** with this installer as the active file.
3. Wait for this exact terminal message:

   ```text
   INSTALL OK: /sdcard/main.py VERIFIED
   ```

4. Restart the K230. Its startup runs `/sdcard/main.py`; simply running the
   installer from RAM does not start camera detection.

The installer writes `/sdcard/main.py`, verifies the bytes on the SD card, and
retains the prior program as `/sdcard/main.py.bak`. It does not change
`/sdcard/colorThresholds.cfg` or `/sdcard/colorThresholds.bak`. A missing success
message means installation is not confirmed; read the reported error.

Do not save the installer or the calibrator as the board's startup `main.py`.
The actual production source is
[`../../src/LCKFB-Lushan-Pi-K230/main.py`](../../src/LCKFB-Lushan-Pi-K230/main.py).

## Color calibration, only if field lighting requires it

Keep the existing calibration unless pillar detection fails under the new
lighting. This step changes the saved LAB thresholds.

1. Open [`colorCalibration.py`](colorCalibration.py) and use
   **CanMV: Run Active File on K230**.
2. Keep the car still. Fill the preview's RED and GREEN boxes with flat faces
   of the corresponding pillars, keeping the EMPTY MAT box clear.
3. Allow the 15-second alignment period and sampling to finish. A valid result
   prints:

   ```text
   COLOR CALIBRATION SAVED: /sdcard/colorThresholds.cfg
   RED/GREEN SAVED; PARKING/MAGENTA REMAINS DISABLED
   ```

4. Restart the K230 to return to production detection. It loads the saved
   calibration automatically. The startup message
   `PERSISTED COLOR CONFIG LOADED: field detection not yet proven` confirms
   loading; it does not prove the current pillars are being detected.

An invalid sample is not saved. The previous valid file is retained as
`/sdcard/colorThresholds.bak`. Do not manually guess LAB values at the field.

## Verify this package on the computer

```sh
cd /Users/zhihaowu/futureEngineers/BUBBLEGUM-WRO-2026
python3 tools/k230/verifyInstaller.py
```

The check statically extracts the installer's embedded bytes and compares them
with the repository's production source, without running board code. This
package embeds **71,099 bytes**, SHA-256
`afb31d3ea162104ad637e817fcba875e67b478f583328634a1da4dfa348ffa0d`.
If production `main.py` changes later, regenerate the installer before deploying;
this check will reject an outdated embedded payload.

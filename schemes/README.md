# Wiring

This is the working wiring map for our car. It matches the wiring we are checking now.

The readable overview is in [`wiring-overview.png`](wiring-overview.png). We can edit the original drawing in [`wiring-overview.excalidraw`](wiring-overview.excalidraw).

Do not power the car from this page yet. We still need a clear ESP32 carrier photo and a yes/no check of the soldered BNO08x wires. The current firmware is safe diagnostics only, but a wrong pin map can still damage two outputs connected together.

## Start button

The button goes between GPIO4 and GND:

```text
ESP32 GPIO4 ---- normally-open button ---- GND
```

The button has no positive or negative side. On a four-leg tactile button, two legs can already be connected inside. Use a multimeter and choose one leg from each different contact group. The selected legs must be open when released and near 0 ohms only while pressed.

The ESP32 uses `INPUT_PULLUP`:

| Button state | GPIO4 reading |
|---|---|
| Released | HIGH |
| Pressed | LOW |

## ESP32-S3 pin map

| ESP32 GPIO | Connection |
|---:|---|
| 2 | BNO08x `RST` |
| 4 | Start button to GND |
| 5 | SDA for all five ToF sensors |
| 6 | SCL for all five ToF sensors |
| 8 | MPU6050 `SDA` |
| 9 | MPU6050 `SCL` |
| 10 | IBT-2 `R_EN` and `L_EN`, tied together |
| 11 | IBT-2 `RPWM` |
| 12 | IBT-2 `LPWM` |
| 13 | Steering-servo signal |
| 15 | Front VL53L1X `XSHUT` |
| 17 | ESP32 TX to K230 physical pin 13 RX |
| 18 | ESP32 RX from K230 physical pin 11 TX |
| 19 | BNO08x `SCL/SCK/RX`, SPI clock |
| 20 | BNO08x `SDA/MISO/TX`, SPI MISO |
| 21 | BNO08x `AD0/MOSI`, SPI MOSI |
| 38 | BNO08x `PS0/WAKE` |
| 39 | Front-left VL53L4CD `XSHUT` |
| 40 | Back-left VL53L4CD `XSHUT` |
| 41 | Front-right VL53L4CD `XSHUT` |
| 42 | Back-right VL53L4CD `XSHUT` |
| 47 | BNO08x `CS` |
| 48 | BNO08x `INT` |

GPIO19 and GPIO20 are also the ESP32-S3 native USB pins. When the BNO08x harness is connected, use the board's separate USB-to-UART `COM` connector for flashing and serial logs. Do not use the native `USB` connector.

GPIO0, GPIO3, GPIO45, and GPIO46 are boot-configuration pins, so we do not use them for this harness. GPIO35 through GPIO37 are unavailable with the N16R8 octal PSRAM module.

Some dual-USB ESP32-S3 carrier boards put the onboard RGB data input on GPIO48. If ours is one of those boards, the BNO08x interrupt shares that net. Do not initialize the onboard RGB LED. We need the exact carrier revision before we call this final.

## BNO08x rewiring

The working record says only these two wires still need to move:

| BNO08x pin | Soldered now | Move to |
|---|---:|---:|
| `RST` | GPIO45 | GPIO2 |
| `PS0/WAKE` | GPIO0 | GPIO38 |

Keep this full BNO08x map if it matches the real soldered harness:

| BNO08x label | Connection |
|---|---|
| `VCC` or `VCC_3.3V` | ESP32 3.3 V only |
| `GND` | GND |
| `SCL/SCK/RX` | GPIO19 |
| `SDA/MISO/TX` | GPIO20 |
| `AD0/MOSI` | GPIO21 |
| `CS` | GPIO47 |
| `INT` | GPIO48 |
| `RST` | GPIO2 |
| `PS1` | 3.3 V |
| `PS0/WAKE` | GPIO38 |
| `BOOT` test pad | Not connected |

Leave the PS0 and PS1 solder bridges open if the header wires set both pins. Do not hard-wire PS0 to 3.3 V while GPIO38 is also connected. Firmware drives GPIO38 HIGH before it resets the BNO08x so the sensor selects SPI mode.

Before changing the two wires, disconnect every battery, USB cable, buck converter, motor supply, and servo supply. Check continuity after soldering and check for shorts from 3.3 V to GND before power-on.

## Five ToF sensors

All five ToF boards share regulated 3.3 V, GND, SDA GPIO5, and SCL GPIO6.

| Position | Sensor | Direction | XSHUT | Address after boot |
|---|---|---|---:|---:|
| Front-centre | VL53L1X | Forward | GPIO15 | `0x30` |
| Front-left | VL53L4CD | Left | GPIO39 | `0x31` |
| Back-left | VL53L4CD | Left | GPIO40 | `0x32` |
| Front-right | VL53L4CD | Right | GPIO41 | `0x33` |
| Back-right | VL53L4CD | Right | GPIO42 | `0x34` |

Leave each ToF `GPIO` or `GPIO1` pin disconnected. The ESP32 uses the individual XSHUT wires to start and readdress the boards one at a time.

The front module seller description was unclear, so we still need to check that it is really a VL53L1X and not a VL53L0X.

## MPU6050

| MPU6050 pin | Connection |
|---|---|
| `VCC` | 3.3 V |
| `GND` | GND |
| `SDA` | GPIO8 |
| `SCL` | GPIO9 |
| `AD0` | GND, address `0x68` |
| `XDA`, `XCL`, `INT` | Not connected |

The MPU6050 is temporary comparison hardware. We plan to remove it after the BNO08x is physically tested.

## K230 UART

UART is 115200 baud, 8N1, and 3.3 V logic.

| K230 physical header pin | K230 signal | ESP32 |
|---:|---|---:|
| 11 | GPIO5, UART2 TX | GPIO18 RX |
| 13 | GPIO6, UART2 RX | GPIO17 TX |
| 14 | GND | GND |

Connect TX, RX, and GND only. Do not connect 5 V or 3.3 V between the two controllers.

## IBT-2, motor, and steering

| Part | Connection |
|---|---|
| IBT-2 `VCC` | Regulated 5 V logic supply |
| IBT-2 `GND` | Common GND |
| IBT-2 `R_EN` and `L_EN` | GPIO10 |
| IBT-2 `RPWM` | GPIO11 |
| IBT-2 `LPWM` | GPIO12 |
| Steering-servo signal | GPIO13 |
| Steering-servo power | Separate regulated 5 to 6 V supply |
| Steering-servo GND | Common GND |

Never power the servo from ESP32 3.3 V. The current firmware forces GPIO10, GPIO11, GPIO12, and GPIO13 LOW and creates no PWM.

## Power overview

```text
3S LiPo
  |
  +-- main switch and fused branch --> IBT-2 motor power
  +-- correct regulated branch -----> ESP32 board input
  +-- 5 to 6 V regulated branch ----> steering servo
  +-- fused 8 to 24 V branch -------> K230 locked 2-pin input

All controller and signal grounds join at common GND.
```

We still need the real regulator models, fuse values, wire sizes, and physical power-distribution photo before turning this into the final diagram.

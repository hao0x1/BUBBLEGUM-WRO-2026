#pragma once

#include <Arduino.h>

// ESP32-S3-WROOM-1-N16R8 hardware map. The default dryRun build executes the
// full controller but compiles the actuator implementation out. Motion code is
// available only in the explicitly selected autonomous environment.
namespace board {

// The current harness uses one IBT-2 on GPIO10/11/12. The dryRun build forces
// every one of these pins LOW.
constexpr uint8_t DRIVE_ENABLE_PIN = 10;  // R_EN and L_EN tied together
constexpr uint8_t DRIVE_RPWM_PIN = 11;
constexpr uint8_t DRIVE_LPWM_PIN = 12;
constexpr uint8_t MOTOR_SAFE_PINS[] = {
    DRIVE_ENABLE_PIN,
    DRIVE_RPWM_PIN,
    DRIVE_LPWM_PIN,
};

// The dryRun firmware holds this pin LOW. A constant LOW is not a valid
// RC-servo command pulse; the autonomous build generates bounded PWM.
constexpr uint8_t STEERING_SERVO_PIN = 13;

// Three physical I2C harnesses share only two ESP32-S3 hardware controllers.
// Wire stays dedicated to XMS-A5, Wire1 stays dedicated to all five ToF
// modules, and the backup MPU6050 uses software I2C on its independent GPIO8/9
// harness. No live hardware controller is rebound between sensors.
constexpr uint8_t MPU_I2C_SDA_PIN = 8;
constexpr uint8_t MPU_I2C_SCL_PIN = 9;
constexpr uint8_t TOF_I2C_SDA_PIN = 5;
constexpr uint8_t TOF_I2C_SCL_PIN = 6;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint16_t I2C_TIMEOUT_MS = 20;

constexpr uint8_t K230_UART_RX_PIN = 18;  // connect to K230 TX
constexpr uint8_t K230_UART_TX_PIN = 17;  // connect to K230 RX
constexpr uint32_t K230_UART_BAUD = 115200;

constexpr uint8_t START_BUTTON_PIN = 4;  // button to GND, INPUT_PULLUP

// All five ST sensors boot at 0x29.  XSHUT is therefore required to wake and
// re-address them one at a time.  Driving XSHUT LOW holds a sensor in reset;
// INPUT releases it high-Z so the module's own pull-up can wake it.
constexpr uint8_t TOF_DEFAULT_ADDRESS = 0x29;
constexpr uint8_t TOF_FRONT_XSHUT_PIN = 15;
constexpr uint8_t TOF_FRONT_LEFT_XSHUT_PIN = 39;
constexpr uint8_t TOF_BACK_LEFT_XSHUT_PIN = 40;
constexpr uint8_t TOF_FRONT_RIGHT_XSHUT_PIN = 41;
constexpr uint8_t TOF_BACK_RIGHT_XSHUT_PIN = 42;

constexpr uint8_t TOF_FRONT_ADDRESS = 0x30;
constexpr uint8_t TOF_FRONT_LEFT_ADDRESS = 0x31;
constexpr uint8_t TOF_BACK_LEFT_ADDRESS = 0x32;
constexpr uint8_t TOF_FRONT_RIGHT_ADDRESS = 0x33;
constexpr uint8_t TOF_BACK_RIGHT_ADDRESS = 0x34;

// The purple GY-BNO08X carrier was physically identified as an XMS-A5 that is
// BNO055-compatible. It is therefore run in I2C mode, not BNO08x SPI mode.
// PS1 is physically connected to GND. Firmware holds PS0 and ADDR LOW, CS HIGH,
// and uses RST for a deterministic startup. GPIO19/20 are USB D-/D+, so use
// the board's separate USB-to-UART COM connector while this harness is fitted.
constexpr uint8_t XMS_A5_I2C_SDA_PIN = 20;
constexpr uint8_t XMS_A5_I2C_SCL_PIN = 19;
constexpr uint8_t XMS_A5_ADDR_PIN = 21;
constexpr uint8_t XMS_A5_CS_PIN = 47;
constexpr uint8_t XMS_A5_INT_PIN = 48;
constexpr uint8_t XMS_A5_RST_PIN = 2;
constexpr uint8_t XMS_A5_PS0_PIN = 38;
constexpr uint8_t XMS_A5_I2C_ADDRESS = 0x28;
constexpr uint32_t XMS_A5_I2C_CLOCK_HZ = 100000;

constexpr uint8_t MPU6050_ADDRESS = 0x68;

#ifndef BUBBLEGUM_ENABLE_MPU6050_COMPARISON
#define BUBBLEGUM_ENABLE_MPU6050_COMPARISON 1
#endif

constexpr bool ENABLE_MPU6050_COMPARISON =
    BUBBLEGUM_ENABLE_MPU6050_COMPARISON != 0;

constexpr uint32_t SENSOR_SUMMARY_INTERVAL_MS = 500;
constexpr uint32_t TOF_STALE_AFTER_MS = 250;
constexpr uint32_t IMU_SAMPLE_INTERVAL_MS = 100;
constexpr uint32_t VISION_SUMMARY_INTERVAL_MS = 250;
constexpr uint32_t VISION_FRESH_MAX_AGE_MS = 250;

constexpr uint8_t ALL_ASSIGNED_PINS[] = {
    DRIVE_ENABLE_PIN,
    DRIVE_RPWM_PIN,
    DRIVE_LPWM_PIN,
    STEERING_SERVO_PIN,
    MPU_I2C_SDA_PIN,
    MPU_I2C_SCL_PIN,
    TOF_I2C_SDA_PIN,
    TOF_I2C_SCL_PIN,
    K230_UART_RX_PIN,
    K230_UART_TX_PIN,
    START_BUTTON_PIN,
    TOF_FRONT_XSHUT_PIN,
    TOF_FRONT_LEFT_XSHUT_PIN,
    TOF_BACK_LEFT_XSHUT_PIN,
    TOF_FRONT_RIGHT_XSHUT_PIN,
    TOF_BACK_RIGHT_XSHUT_PIN,
    XMS_A5_I2C_SDA_PIN,
    XMS_A5_I2C_SCL_PIN,
    XMS_A5_ADDR_PIN,
    XMS_A5_CS_PIN,
    XMS_A5_INT_PIN,
    XMS_A5_RST_PIN,
    XMS_A5_PS0_PIN,
};

constexpr bool assignedPinsAreUnique() {
  constexpr size_t pinCount =
      sizeof(ALL_ASSIGNED_PINS) / sizeof(ALL_ASSIGNED_PINS[0]);
  for (size_t i = 0; i < pinCount; ++i) {
    for (size_t j = i + 1U; j < pinCount; ++j) {
      if (ALL_ASSIGNED_PINS[i] == ALL_ASSIGNED_PINS[j]) return false;
    }
  }
  return true;
}

static_assert(K230_UART_RX_PIN != K230_UART_TX_PIN, "UART pins must differ");
static_assert(MPU_I2C_SDA_PIN != MPU_I2C_SCL_PIN,
              "MPU I2C pins must differ");
static_assert(TOF_I2C_SDA_PIN != TOF_I2C_SCL_PIN,
              "ToF I2C pins must differ");
static_assert(XMS_A5_I2C_SDA_PIN != XMS_A5_I2C_SCL_PIN,
              "XMS-A5 I2C pins must differ");
static_assert(TOF_FRONT_ADDRESS >= 0x08 && TOF_BACK_RIGHT_ADDRESS <= 0x77,
              "ToF addresses must be valid 7-bit I2C addresses");
static_assert(assignedPinsAreUnique(), "ESP32 pin assignments must not overlap");

}  // namespace board

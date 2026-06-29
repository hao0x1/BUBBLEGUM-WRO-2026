#include "imuSensors.h"

#include <math.h>
#include <utility/imumaths.h>

#include "boardConfig.h"

namespace {

constexpr int32_t xmsA5SensorId = 55;
constexpr uint32_t xmsA5ResetSettleMs = 900U;
constexpr adafruit_bno055_opmode_t xmsA5OperatingMode =
    OPERATION_MODE_NDOF;
constexpr uint8_t xmsA5EulerHeadingLsbRegister = 0x1AU;
constexpr uint8_t xmsA5CalibrationStatusRegister = 0x35U;
constexpr float xmsA5EulerDegreesPerCount = 1.0F / 16.0F;

// The ESP32-S3 has only two hardware I2C controllers. Wire is dedicated to
// the XMS-A5 and Wire1 is dedicated to the five ToF sensors, so the backup
// MPU6050 uses this small open-drain software-I2C master on its own GPIO8/9
// harness. The delay keeps the bus below 100 kHz even with GPIO overhead.
constexpr uint32_t mpuSoftI2cHalfPeriodUs = 5U;
constexpr uint32_t mpuSoftI2cLineTimeoutUs = 1000U;
constexpr uint8_t mpuRegisterSampleRateDivider = 0x19;
constexpr uint8_t mpuRegisterConfig = 0x1A;
constexpr uint8_t mpuRegisterGyroConfig = 0x1B;
constexpr uint8_t mpuRegisterAccelConfig = 0x1C;
constexpr uint8_t mpuRegisterAccelXoutHigh = 0x3B;
constexpr uint8_t mpuRegisterPowerManagement1 = 0x6B;
constexpr uint8_t mpuRegisterPowerManagement2 = 0x6C;
constexpr uint8_t mpuRegisterWhoAmI = 0x75;

void setOutput(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, level);
}

void printAge(Print& log, uint32_t nowMs, uint32_t lastMs,
              uint32_t sampleCount) {
  if (sampleCount == 0U) {
    log.print(F("never"));
  } else {
    log.print(static_cast<uint32_t>(nowMs - lastMs));
    log.print(F("ms"));
  }
}

void pullLineLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void releaseLine(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
}

bool waitForLineHigh(uint8_t pin) {
  const uint32_t startedUs = micros();
  while (digitalRead(pin) == LOW) {
    if (static_cast<uint32_t>(micros() - startedUs) >=
        mpuSoftI2cLineTimeoutUs) {
      return false;
    }
    yield();
  }
  return true;
}

int16_t signedWord(const uint8_t* data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8U) |
                              static_cast<uint16_t>(data[1]));
}

int16_t signedLittleEndianWord(const uint8_t* data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8U));
}

}  // namespace

bool ImuSensors::recoverMpuSoftI2c() {
  releaseLine(board::MPU_I2C_SDA_PIN);
  releaseLine(board::MPU_I2C_SCL_PIN);
  if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;

  // A slave interrupted during a byte may still be holding SDA low. Clock it
  // through the remainder of that byte, then generate a STOP condition.
  for (uint8_t pulse = 0; pulse < 9U &&
                          digitalRead(board::MPU_I2C_SDA_PIN) == LOW;
       ++pulse) {
    pullLineLow(board::MPU_I2C_SCL_PIN);
    delayMicroseconds(mpuSoftI2cHalfPeriodUs);
    releaseLine(board::MPU_I2C_SCL_PIN);
    if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;
    delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  }

  pullLineLow(board::MPU_I2C_SDA_PIN);
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  releaseLine(board::MPU_I2C_SCL_PIN);
  if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  releaseLine(board::MPU_I2C_SDA_PIN);
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  return digitalRead(board::MPU_I2C_SDA_PIN) == HIGH;
}

bool ImuSensors::beginMpuSoftI2c() {
  mpuSoftI2cReady_ = recoverMpuSoftI2c();
  return mpuSoftI2cReady_;
}

bool ImuSensors::startMpuSoftI2c() {
  if (!mpuSoftI2cReady_) return false;
  releaseLine(board::MPU_I2C_SDA_PIN);
  releaseLine(board::MPU_I2C_SCL_PIN);
  if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;
  if (digitalRead(board::MPU_I2C_SDA_PIN) == LOW && !recoverMpuSoftI2c()) {
    return false;
  }
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  pullLineLow(board::MPU_I2C_SDA_PIN);
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  pullLineLow(board::MPU_I2C_SCL_PIN);
  return true;
}

bool ImuSensors::stopMpuSoftI2c() {
  pullLineLow(board::MPU_I2C_SDA_PIN);
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  releaseLine(board::MPU_I2C_SCL_PIN);
  if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  releaseLine(board::MPU_I2C_SDA_PIN);
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  return digitalRead(board::MPU_I2C_SDA_PIN) == HIGH;
}

bool ImuSensors::writeMpuSoftI2cByte(uint8_t value) {
  for (uint8_t bit = 0; bit < 8U; ++bit) {
    if ((value & 0x80U) == 0U) {
      pullLineLow(board::MPU_I2C_SDA_PIN);
    } else {
      releaseLine(board::MPU_I2C_SDA_PIN);
    }
    delayMicroseconds(mpuSoftI2cHalfPeriodUs);
    releaseLine(board::MPU_I2C_SCL_PIN);
    if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;
    delayMicroseconds(mpuSoftI2cHalfPeriodUs);
    pullLineLow(board::MPU_I2C_SCL_PIN);
    value <<= 1U;
  }

  // Ninth clock: the slave must acknowledge by pulling SDA low.
  releaseLine(board::MPU_I2C_SDA_PIN);
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  releaseLine(board::MPU_I2C_SCL_PIN);
  if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;
  const bool acknowledged = digitalRead(board::MPU_I2C_SDA_PIN) == LOW;
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  pullLineLow(board::MPU_I2C_SCL_PIN);
  return acknowledged;
}

bool ImuSensors::readMpuSoftI2cByte(uint8_t& value, bool acknowledge) {
  value = 0U;
  releaseLine(board::MPU_I2C_SDA_PIN);
  for (uint8_t bit = 0; bit < 8U; ++bit) {
    value <<= 1U;
    delayMicroseconds(mpuSoftI2cHalfPeriodUs);
    releaseLine(board::MPU_I2C_SCL_PIN);
    if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;
    if (digitalRead(board::MPU_I2C_SDA_PIN) == HIGH) value |= 1U;
    delayMicroseconds(mpuSoftI2cHalfPeriodUs);
    pullLineLow(board::MPU_I2C_SCL_PIN);
  }

  if (acknowledge) {
    pullLineLow(board::MPU_I2C_SDA_PIN);
  } else {
    releaseLine(board::MPU_I2C_SDA_PIN);
  }
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  releaseLine(board::MPU_I2C_SCL_PIN);
  if (!waitForLineHigh(board::MPU_I2C_SCL_PIN)) return false;
  delayMicroseconds(mpuSoftI2cHalfPeriodUs);
  pullLineLow(board::MPU_I2C_SCL_PIN);
  releaseLine(board::MPU_I2C_SDA_PIN);
  return true;
}

bool ImuSensors::writeMpuRegister(uint8_t reg, uint8_t value) {
  bool okay = startMpuSoftI2c();
  if (okay) okay = writeMpuSoftI2cByte(board::MPU6050_ADDRESS << 1U);
  if (okay) okay = writeMpuSoftI2cByte(reg);
  if (okay) okay = writeMpuSoftI2cByte(value);
  const bool stopped = stopMpuSoftI2c();
  if (!okay || !stopped) recoverMpuSoftI2c();
  return okay && stopped;
}

bool ImuSensors::readMpuRegisters(uint8_t firstReg, uint8_t* data,
                                  size_t length) {
  if (data == nullptr || length == 0U) return false;
  bool okay = startMpuSoftI2c();
  if (okay) okay = writeMpuSoftI2cByte(board::MPU6050_ADDRESS << 1U);
  if (okay) okay = writeMpuSoftI2cByte(firstReg);
  // Repeated START keeps the register pointer transaction atomic.
  if (okay) okay = startMpuSoftI2c();
  if (okay) {
    okay = writeMpuSoftI2cByte(
        static_cast<uint8_t>((board::MPU6050_ADDRESS << 1U) | 1U));
  }
  for (size_t index = 0; okay && index < length; ++index) {
    okay = readMpuSoftI2cByte(data[index], index + 1U < length);
  }
  const bool stopped = stopMpuSoftI2c();
  if (!okay || !stopped) recoverMpuSoftI2c();
  return okay && stopped;
}

bool ImuSensors::initializeMpu6050(Print& log) {
  if (!beginMpuSoftI2c()) {
    log.println(F("[mpu6050] GPIO8/9 software-I2C bus stuck (nonfatal)"));
    return false;
  }

  uint8_t whoAmI = 0U;
  if (!readMpuRegisters(mpuRegisterWhoAmI, &whoAmI, 1U) ||
      (whoAmI != 0x68U && whoAmI != 0x69U)) {
    log.print(F("[mpu6050] software-I2C WHO_AM_I invalid: 0x"));
    log.println(whoAmI, HEX);
    return false;
  }

  // Wake the part, use the X-axis gyro PLL, 1 kHz internal rate / 10 = 100 Hz,
  // and keep the default +/-2 g and +/-250 degree/s full-scale ranges.
  const bool configured =
      writeMpuRegister(mpuRegisterPowerManagement1, 0x01U) &&
      writeMpuRegister(mpuRegisterPowerManagement2, 0x00U) &&
      writeMpuRegister(mpuRegisterConfig, 0x03U) &&
      writeMpuRegister(mpuRegisterSampleRateDivider, 0x09U) &&
      writeMpuRegister(mpuRegisterGyroConfig, 0x00U) &&
      writeMpuRegister(mpuRegisterAccelConfig, 0x00U);
  if (!configured) {
    log.println(F("[mpu6050] software-I2C configuration failed (nonfatal)"));
    return false;
  }
  delay(100);

  uint8_t powerManagement1 = 0xFFU;
  if (!readMpuRegisters(mpuRegisterPowerManagement1, &powerManagement1, 1U) ||
      (powerManagement1 & 0x40U) != 0U) {
    log.println(F("[mpu6050] wake-state readback failed (nonfatal)"));
    return false;
  }

  log.print(F("[mpu6050] independent GPIO8/9 software-I2C, WHO_AM_I=0x"));
  log.print(whoAmI, HEX);
  log.println(F(": online (backup/comparison)"));
  return true;
}

bool ImuSensors::readXmsA5Registers(uint8_t firstReg, uint8_t* data,
                                    size_t length) const {
  if (wire_ == nullptr || data == nullptr || length == 0U || length > 32U) {
    return false;
  }

  wire_->beginTransmission(board::XMS_A5_I2C_ADDRESS);
  if (wire_->write(firstReg) != 1U) {
    wire_->endTransmission(true);
    return false;
  }
  if (wire_->endTransmission(false) != 0U) {
    return false;
  }

  const size_t received = wire_->requestFrom(
      board::XMS_A5_I2C_ADDRESS, length, static_cast<bool>(true));
  if (received != length || static_cast<size_t>(wire_->available()) < length) {
    while (wire_->available() > 0) wire_->read();
    return false;
  }
  for (size_t index = 0; index < length; ++index) {
    data[index] = static_cast<uint8_t>(wire_->read());
  }
  return true;
}

void ImuSensors::begin(TwoWire& wire, bool i2cAvailable, Print& log) {
  wire_ = &wire;

  if (board::ENABLE_MPU6050_COMPARISON) {
    mpuOnline_ = initializeMpu6050(log);
  } else if (!board::ENABLE_MPU6050_COMPARISON) {
    log.println(F("[mpu6050] disabled at compile time"));
  }

  // Physical state: PS1 is GND. Hold the remaining carrier mode pins at the
  // same levels that proved this XMS-A5 responds as a BNO055-compatible part.
  setOutput(board::XMS_A5_PS0_PIN, LOW);
  setOutput(board::XMS_A5_ADDR_PIN, LOW);
  setOutput(board::XMS_A5_CS_PIN, HIGH);
  setOutput(board::XMS_A5_RST_PIN, LOW);
  pinMode(board::XMS_A5_INT_PIN, INPUT_PULLUP);

  delay(25);
  digitalWrite(board::XMS_A5_RST_PIN, HIGH);

  if (!i2cAvailable) {
    log.println(F("[xms-a5] dedicated GPIO20/19 I2C controller start failed"));
  } else {
    delay(xmsA5ResetSettleMs);
    xmsA5_.emplace(xmsA5SensorId, board::XMS_A5_I2C_ADDRESS, wire_);

    // Use the exact driver startup mode that was proven by bnoModeTest and the
    // earlier plane project. The XMS-A5 clone acknowledged in IMUPLUS but then
    // returned zero-filled Euler/calibration registers, so ACK alone was not a
    // valid live-data check.
    bnoOnline_ = xmsA5_->begin(xmsA5OperatingMode);
    if (bnoOnline_ && xmsA5_->getMode() != xmsA5OperatingMode) {
      bnoOnline_ = false;
      log.println(F("[xms-a5] operating-mode readback failed (nonfatal)"));
    }
    if (bnoOnline_) {
      log.println(F("[xms-a5] BNO055-compatible primary online at 0x28"));
      log.println(F("[xms-a5] NDOF mode enabled (same proven path as bnoModeTest)"));
    } else {
      log.println(F("[xms-a5] BNO055-compatible startup failed (nonfatal)"));
      xmsA5_.reset();
    }
  }
  log.println(F("[i2c] Wire stays on XMS-A5; Wire1 stays on ToF"));
}

void ImuSensors::poll(uint32_t nowMs, Print& log) {
  (void)log;
  if (static_cast<uint32_t>(nowMs - lastImuPollMs_) <
      board::IMU_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastImuPollMs_ = nowMs;

  if (mpuOnline_) {
    uint8_t motion[14] = {};
    if (readMpuRegisters(mpuRegisterAccelXoutHigh, motion,
                         sizeof(motion))) {
      ax_ = signedWord(&motion[0]);
      ay_ = signedWord(&motion[2]);
      az_ = signedWord(&motion[4]);
      gx_ = signedWord(&motion[8]);
      gy_ = signedWord(&motion[10]);
      gz_ = signedWord(&motion[12]);
      lastMpuReadMs_ = nowMs;
      ++mpuSampleCount_;
    } else {
      ++mpuBusErrorCount_;
    }
  }

  if (!bnoOnline_ || !xmsA5_.has_value()) return;
  // The XMS-A5 clone starts correctly through the Adafruit driver, but its
  // runtime OPR_MODE reads are not reliable. Poll the same raw Euler registers
  // used by the proven standalone test, with explicit I2C success checks. Do
  // not precede reads with an address-only scan transaction: some clones stop
  // responding after those empty writes.
  uint8_t eulerData[6] = {};
  uint8_t calibrationStatus = 0U;
  if (!readXmsA5Registers(xmsA5EulerHeadingLsbRegister, eulerData,
                          sizeof(eulerData)) ||
      !readXmsA5Registers(xmsA5CalibrationStatusRegister,
                          &calibrationStatus, 1U)) {
    ++bnoBusErrorCount_;
    return;
  }

  const float yaw =
      static_cast<float>(signedLittleEndianWord(&eulerData[0])) *
      xmsA5EulerDegreesPerCount;
  const float roll =
      static_cast<float>(signedLittleEndianWord(&eulerData[2])) *
      xmsA5EulerDegreesPerCount;
  const float pitch =
      static_cast<float>(signedLittleEndianWord(&eulerData[4])) *
      xmsA5EulerDegreesPerCount;
  if (!isfinite(yaw) || !isfinite(roll) || !isfinite(pitch)) {
    ++bnoBusErrorCount_;
    return;
  }

  systemCalibration_ = static_cast<uint8_t>((calibrationStatus >> 6U) & 0x03U);
  gyroCalibration_ = static_cast<uint8_t>((calibrationStatus >> 4U) & 0x03U);
  accelerometerCalibration_ =
      static_cast<uint8_t>((calibrationStatus >> 2U) & 0x03U);
  magnetometerCalibration_ = static_cast<uint8_t>(calibrationStatus & 0x03U);
  yawDegrees_ = yaw;
  rollDegrees_ = roll;
  pitchDegrees_ = pitch;
  bnoAccuracy_ = gyroCalibration_;
  lastBnoReadMs_ = nowMs;
  ++bnoSampleCount_;
}

void ImuSensors::printSummary(Print& log, uint32_t nowMs) const {
  log.print(F("[mpu6050] online="));
  log.print(mpuOnline_ ? F("yes") : F("no"));
  if (mpuOnline_) {
    log.print(F(" raw_a="));
    log.print(ax_);
    log.print(',');
    log.print(ay_);
    log.print(',');
    log.print(az_);
    log.print(F(" raw_g="));
    log.print(gx_);
    log.print(',');
    log.print(gy_);
    log.print(',');
    log.print(gz_);
    log.print(F(" age="));
    printAge(log, nowMs, lastMpuReadMs_, mpuSampleCount_);
    log.print(F(" n="));
    log.print(mpuSampleCount_);
    log.print(F(" bus_errors="));
    log.print(mpuBusErrorCount_);
  }
  log.println();

  log.print(F("[xms-a5/bno055] primary online="));
  log.print(bnoOnline_ ? F("yes") : F("no"));
  if (bnoOnline_) {
    log.print(F(" yaw="));
    log.print(yawDegrees_, 2);
    log.print(F("deg roll="));
    log.print(rollDegrees_, 2);
    log.print(F("deg pitch="));
    log.print(pitchDegrees_, 2);
    log.print(F("deg cal=sys/gyro/accel/mag:"));
    log.print(systemCalibration_);
    log.print('/');
    log.print(gyroCalibration_);
    log.print('/');
    log.print(accelerometerCalibration_);
    log.print('/');
    log.print(magnetometerCalibration_);
    log.print(F(" age="));
    printAge(log, nowMs, lastBnoReadMs_, bnoSampleCount_);
    log.print(F(" n="));
    log.print(bnoSampleCount_);
    log.print(F(" bus_errors="));
    log.print(bnoBusErrorCount_);
  }
  log.println();
}

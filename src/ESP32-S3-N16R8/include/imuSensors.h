#pragma once

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <optional>
#include <Wire.h>

class ImuSensors {
 public:
  void begin(TwoWire& wire, bool i2cAvailable, Print& log);
  void poll(uint32_t nowMs, Print& log);
  void printSummary(Print& log, uint32_t nowMs) const;

  bool bnoOnline() const { return bnoOnline_; }
  bool bnoFresh(uint32_t nowMs, uint32_t maximumAgeMs) const {
    return bnoOnline_ && bnoSampleCount_ > 0U &&
           static_cast<uint32_t>(nowMs - lastBnoReadMs_) <= maximumAgeMs;
  }
  float yawDegrees() const { return yawDegrees_; }
  uint8_t bnoAccuracy() const { return bnoAccuracy_; }
  uint32_t bnoSampleCount() const { return bnoSampleCount_; }
  uint32_t lastBnoReadMs() const { return lastBnoReadMs_; }

 private:
  bool beginMpuSoftI2c();
  bool recoverMpuSoftI2c();
  bool startMpuSoftI2c();
  bool stopMpuSoftI2c();
  bool writeMpuSoftI2cByte(uint8_t value);
  bool readMpuSoftI2cByte(uint8_t& value, bool acknowledge);
  bool writeMpuRegister(uint8_t reg, uint8_t value);
  bool readMpuRegisters(uint8_t firstReg, uint8_t* data, size_t length);
  bool initializeMpu6050(Print& log);
  bool readXmsA5Registers(uint8_t firstReg, uint8_t* data,
                          size_t length) const;

  std::optional<Adafruit_BNO055> xmsA5_;
  TwoWire* wire_ = nullptr;

  bool mpuSoftI2cReady_ = false;
  bool mpuOnline_ = false;
  bool bnoOnline_ = false;
  int16_t ax_ = 0;
  int16_t ay_ = 0;
  int16_t az_ = 0;
  int16_t gx_ = 0;
  int16_t gy_ = 0;
  int16_t gz_ = 0;
  float yawDegrees_ = 0.0F;
  float rollDegrees_ = 0.0F;
  float pitchDegrees_ = 0.0F;
  uint8_t bnoAccuracy_ = 0;
  uint8_t systemCalibration_ = 0;
  uint8_t gyroCalibration_ = 0;
  uint8_t accelerometerCalibration_ = 0;
  uint8_t magnetometerCalibration_ = 0;
  uint32_t lastMpuReadMs_ = 0;
  uint32_t lastBnoReadMs_ = 0;
  uint32_t lastImuPollMs_ = 0;
  uint32_t mpuSampleCount_ = 0;
  uint32_t bnoSampleCount_ = 0;
  uint32_t mpuBusErrorCount_ = 0;
  uint32_t bnoBusErrorCount_ = 0;
};

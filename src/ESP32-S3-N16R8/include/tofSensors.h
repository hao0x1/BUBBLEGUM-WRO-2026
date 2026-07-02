#pragma once

#include <Arduino.h>
#include <VL53L1X.h>
#include <VL53L4CD.h>
#include <Wire.h>

class TofSensors {
 public:
  enum SideIndex : size_t {
    FrontLeft = 0,
    BackLeft = 1,
    FrontRight = 2,
    BackRight = 3,
    SideCount = 4,
  };

  struct Sample {
    bool online = false;
    bool valid = false;
    uint16_t rangeMm = 0;
    uint8_t rangeStatus = 0xFF;
    uint8_t i2cStatus = 0xFF;
    uint32_t lastUpdateMs = 0;
    uint32_t sampleCount = 0;
    uint32_t busErrorCount = 0;
  };

  void begin(TwoWire& wire, bool busAvailable, Print& log);
  void poll(uint32_t nowMs);
  void printSummary(Print& log, uint32_t nowMs) const;

  const Sample& front() const { return frontSample_; }
  const Sample& side(size_t index) const { return sideSamples_[index]; }
  const Sample& frontLeft() const { return sideSamples_[FrontLeft]; }
  const Sample& backLeft() const { return sideSamples_[BackLeft]; }
  const Sample& frontRight() const { return sideSamples_[FrontRight]; }
  const Sample& backRight() const { return sideSamples_[BackRight]; }
  static bool sampleFresh(const Sample& sample, uint32_t nowMs,
                          uint32_t maximumAgeMs) {
    return sample.online && sample.valid && sample.sampleCount > 0U &&
           static_cast<uint32_t>(nowMs - sample.lastUpdateMs) <= maximumAgeMs;
  }

 private:
  static constexpr size_t SIDE_SENSOR_COUNT = SideCount;

  void holdAllInReset();
  static void holdInReset(uint8_t pin);
  static void releaseHighZ(uint8_t pin);
  bool initializeFront(Print& log);
  bool initializeSide(size_t index, Print& log);
  static bool addressResponds(TwoWire& wire, uint8_t address);
  static void printOne(Print& log, const __FlashStringHelper* name,
                       uint8_t address, const Sample& sample, uint32_t nowMs);

  TwoWire* wire_ = nullptr;
  VL53L1X frontSensor_;
  VL53L4CD sideSensors_[SIDE_SENSOR_COUNT];
  Sample frontSample_;
  Sample sideSamples_[SIDE_SENSOR_COUNT];
};

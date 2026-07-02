#include "tofSensors.h"

#include "boardConfig.h"

namespace {

constexpr uint8_t SIDE_XSHUT_PINS[] = {
    board::TOF_FRONT_LEFT_XSHUT_PIN,
    board::TOF_BACK_LEFT_XSHUT_PIN,
    board::TOF_FRONT_RIGHT_XSHUT_PIN,
    board::TOF_BACK_RIGHT_XSHUT_PIN,
};

constexpr uint8_t SIDE_ADDRESSES[] = {
    board::TOF_FRONT_LEFT_ADDRESS,
    board::TOF_BACK_LEFT_ADDRESS,
    board::TOF_FRONT_RIGHT_ADDRESS,
    board::TOF_BACK_RIGHT_ADDRESS,
};

const __FlashStringHelper* const SIDE_NAMES[] = {
    F("front-left"), F("back-left"), F("front-right"), F("back-right")};

static_assert(sizeof(SIDE_XSHUT_PINS) == 4U, "four VL53L4CD XSHUT pins");
static_assert(sizeof(SIDE_ADDRESSES) == 4U, "four VL53L4CD addresses");

}  // namespace

void TofSensors::holdInReset(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void TofSensors::releaseHighZ(uint8_t pin) {
  // Do not drive XSHUT high.  The breakout's own pull-up releases the sensor,
  // while INPUT keeps the ESP32 pin electrically high impedance.
  pinMode(pin, INPUT);
}

void TofSensors::holdAllInReset() {
  holdInReset(board::TOF_FRONT_XSHUT_PIN);
  for (uint8_t pin : SIDE_XSHUT_PINS) holdInReset(pin);
}

bool TofSensors::addressResponds(TwoWire& wire, uint8_t address) {
  wire.beginTransmission(address);
  return wire.endTransmission() == 0U;
}

bool TofSensors::initializeFront(Print& log) {
  log.println(F("[tof] release front VL53L1X XSHUT high-Z"));
  releaseHighZ(board::TOF_FRONT_XSHUT_PIN);
  delay(20);

  frontSensor_.setBus(wire_);
  frontSensor_.setTimeout(150);
  if (!frontSensor_.init()) {
    log.println(F("[tof] front VL53L1X missing/init failed (nonfatal)"));
    holdInReset(board::TOF_FRONT_XSHUT_PIN);
    return false;
  }
  frontSensor_.setAddress(board::TOF_FRONT_ADDRESS);
  delay(2);
  if (!addressResponds(*wire_, board::TOF_FRONT_ADDRESS) ||
      !frontSensor_.setDistanceMode(VL53L1X::Long) ||
      !frontSensor_.setMeasurementTimingBudget(33000U)) {
    log.println(F("[tof] front VL53L1X configure/address check failed"));
    holdInReset(board::TOF_FRONT_XSHUT_PIN);
    return false;
  }
  frontSensor_.startContinuous(40U);
  frontSample_.online = true;
  log.print(F("[tof] front VL53L1X online at 0x"));
  log.println(board::TOF_FRONT_ADDRESS, HEX);
  return true;
}

bool TofSensors::initializeSide(size_t index, Print& log) {
  log.print(F("[tof] release "));
  log.print(SIDE_NAMES[index]);
  log.println(F(" VL53L4CD XSHUT high-Z"));
  releaseHighZ(SIDE_XSHUT_PINS[index]);
  delay(20);

  VL53L4CD& sensor = sideSensors_[index];
  sensor.setBus(wire_);
  sensor.setTimeout(150);
  if (!sensor.init(true, false)) {
    log.print(F("[tof] "));
    log.print(SIDE_NAMES[index]);
    log.println(F(" VL53L4CD missing/init failed (nonfatal)"));
    holdInReset(SIDE_XSHUT_PINS[index]);
    return false;
  }
  sensor.setAddress(SIDE_ADDRESSES[index]);
  delay(2);
  if (!addressResponds(*wire_, SIDE_ADDRESSES[index]) ||
      !sensor.setRangeTiming(20U, 0U)) {
    log.print(F("[tof] "));
    log.print(SIDE_NAMES[index]);
    log.println(F(" VL53L4CD configure/address check failed"));
    holdInReset(SIDE_XSHUT_PINS[index]);
    return false;
  }
  sensor.startContinuous();
  sideSamples_[index].online = true;
  log.print(F("[tof] "));
  log.print(SIDE_NAMES[index]);
  log.print(F(" VL53L4CD online at 0x"));
  log.println(SIDE_ADDRESSES[index], HEX);
  return true;
}

void TofSensors::begin(TwoWire& wire, bool busAvailable, Print& log) {
  wire_ = busAvailable ? &wire : nullptr;
  holdAllInReset();
  delay(10);

  if (wire_ != nullptr) {
    initializeFront(log);
    for (size_t i = 0; i < SIDE_SENSOR_COUNT; ++i) {
      initializeSide(i, log);
    }
    if (addressResponds(*wire_, board::TOF_DEFAULT_ADDRESS)) {
      log.println(F("[tof/bus] WARNING: device still responds at 0x29; "
                    "check XSHUT wiring and sensor models"));
    } else {
      log.println(F("[tof/bus] default 0x29 clear after address assignment"));
    }
  } else {
    log.println(F("[tof/bus] skipped because Wire1 did not start"));
  }
}

void TofSensors::poll(uint32_t nowMs) {
  if (wire_ == nullptr) return;

  bool frontReady = false;
  if (frontSample_.online) {
    frontReady = frontSensor_.dataReady();
    if (frontSensor_.last_status != 0U) {
      frontSample_.i2cStatus = frontSensor_.last_status;
      frontSample_.valid = false;
      ++frontSample_.busErrorCount;
    }
  }
  if (frontSample_.online && frontReady && frontSensor_.last_status == 0U) {
    const uint16_t range = frontSensor_.read(false);
    frontSample_.i2cStatus = frontSensor_.last_status;
    if (frontSample_.i2cStatus == 0U && !frontSensor_.timeoutOccurred()) {
      frontSample_.rangeMm = range;
      frontSample_.rangeStatus =
          static_cast<uint8_t>(frontSensor_.ranging_data.range_status);
      frontSample_.valid =
          frontSensor_.ranging_data.range_status == VL53L1X::RangeValid ||
          frontSensor_.ranging_data.range_status ==
              VL53L1X::RangeValidNoWrapCheckFail;
      frontSample_.lastUpdateMs = nowMs;
      ++frontSample_.sampleCount;
    } else {
      frontSample_.valid = false;
      ++frontSample_.busErrorCount;
    }
  }

  for (size_t i = 0; i < SIDE_SENSOR_COUNT; ++i) {
    Sample& sample = sideSamples_[i];
    VL53L4CD& sensor = sideSensors_[i];
    if (!sample.online) continue;
    const bool ready = sensor.dataReady();
    if (sensor.last_status != 0U) {
      sample.i2cStatus = sensor.last_status;
      sample.valid = false;
      ++sample.busErrorCount;
      continue;
    }
    if (!ready) continue;
    const uint16_t range = sensor.read(false);
    sample.i2cStatus = sensor.last_status;
    if (sample.i2cStatus == 0U && !sensor.timeoutOccurred()) {
      sample.rangeMm = range;
      sample.rangeStatus = sensor.ranging_data.range_status;
      sample.valid = range != 0U && sample.rangeStatus == 0U &&
                     sensor.ranging_data.number_of_spad != 0U;
      sample.lastUpdateMs = nowMs;
      ++sample.sampleCount;
    } else {
      sample.valid = false;
      ++sample.busErrorCount;
    }
  }

  if (frontSample_.sampleCount > 0U &&
      static_cast<uint32_t>(nowMs - frontSample_.lastUpdateMs) >
          board::TOF_STALE_AFTER_MS) {
    frontSample_.valid = false;
  }
  for (Sample& sample : sideSamples_) {
    if (sample.sampleCount > 0U &&
        static_cast<uint32_t>(nowMs - sample.lastUpdateMs) >
            board::TOF_STALE_AFTER_MS) {
      sample.valid = false;
    }
  }
}

void TofSensors::printOne(Print& log, const __FlashStringHelper* name,
                              uint8_t address, const Sample& sample,
                              uint32_t nowMs) {
  log.print(F("  "));
  log.print(name);
  log.print(F(" 0x"));
  if (address < 0x10U) log.print('0');
  log.print(address, HEX);
  if (!sample.online) {
    log.println(F(" offline"));
    return;
  }
  log.print(F(" range="));
  log.print(sample.rangeMm);
  log.print(F("mm status="));
  log.print(sample.rangeStatus);
  log.print(F(" valid="));
  log.print(sample.valid ? F("yes") : F("no"));
  log.print(F(" age="));
  if (sample.sampleCount == 0U) {
    log.print(F("never"));
  } else {
    log.print(static_cast<uint32_t>(nowMs - sample.lastUpdateMs));
    log.print(F("ms"));
  }
  log.print(F(" n="));
  log.print(sample.sampleCount);
  log.print(F(" bus_errors="));
  log.println(sample.busErrorCount);
}

void TofSensors::printSummary(Print& log, uint32_t nowMs) const {
  log.println(F("[tof] nonblocking range/status/receipt-age"));
  printOne(log, F("front"), board::TOF_FRONT_ADDRESS, frontSample_, nowMs);
  for (size_t i = 0; i < SIDE_SENSOR_COUNT; ++i) {
    printOne(log, SIDE_NAMES[i], SIDE_ADDRESSES[i], sideSamples_[i], nowMs);
  }
}

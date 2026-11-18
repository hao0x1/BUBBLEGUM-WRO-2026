#include <Arduino.h>
#include <Wire.h>

#include <SparkFun_BNO080_Arduino_Library.h>

#include "boardConfig.h"
#include "vehicleActuators.h"

namespace {

constexpr uint8_t kBno080Address = 0x4B;
constexpr uint32_t kBnoReportPeriodUs = 20000U;
constexpr uint32_t kImuTimeoutMs = 250U;
constexpr float kForwardCommand = 0.25F;
constexpr float kHeadingGain = 0.02F;
constexpr float kMaximumSteering = 0.25F;
// Change to -1.0F if a positive yaw error requires steering left on the car.
constexpr float kSteeringSign = 1.0F;

BNO080 imu;
VehicleActuators actuators;

bool imuReady = false;
bool headingSet = false;
float targetYawRadians = 0.0F;
uint32_t lastImuMs = 0U;

float wrapRadians(float angle) {
  while (angle > PI) angle -= 2.0F * PI;
  while (angle < -PI) angle += 2.0F * PI;
  return angle;
}

void stopVehicle(const __FlashStringHelper* reason) {
  actuators.faultStop();
  Serial.println(reason);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(board::XMS_A5_I2C_SDA_PIN, board::XMS_A5_I2C_SCL_PIN,
            board::XMS_A5_I2C_CLOCK_HZ);

  if (!imu.begin(kBno080Address, Wire)) {
    Serial.println(F("BNO080 not found; vehicle stopped"));
    return;
  }
  imu.enableRotationVector(kBnoReportPeriodUs / 1000U);
  imuReady = true;

  if (!actuators.beginSafe()) {
    Serial.println(F("Actuator startup failed; vehicle stopped"));
    imuReady = false;
    return;
  }

  Serial.println(F("BNO080 forward test ready"));
}

void loop() {
  const uint32_t nowMs = millis();

  if (!imuReady) {
    actuators.stop();
    delay(10);
    return;
  }

  if (imu.dataAvailable()) {
    const float yawRadians = imu.getYaw();
    if (!headingSet) {
      targetYawRadians = yawRadians;
      headingSet = true;
    }
    lastImuMs = nowMs;

    const float headingError = wrapRadians(targetYawRadians - yawRadians);
    const float steering = constrain(
        kSteeringSign * headingError * kHeadingGain,
        -kMaximumSteering, kMaximumSteering);
    actuators.setSteering(steering);
  }

  if (!headingSet || static_cast<uint32_t>(nowMs - lastImuMs) > kImuTimeoutMs) {
    stopVehicle(F("BNO080 data stale; vehicle stopped"));
    while (true) delay(1000);
  }

  if (actuators.setDrive(kForwardCommand) == false) {
    stopVehicle(F("Drive command rejected; vehicle stopped"));
    while (true) delay(1000);
  }

  delay(5);
}
#include <Arduino.h>
#include <Wire.h>

#include "autonomyController.h"
#include "boardConfig.h"
#include "controllerProfiles.h"
#include "imuSensors.h"
#include "tofSensors.h"
#include "visionProtocol.h"

#ifndef BUBBLEGUM_ENABLE_ACTUATION
#define BUBBLEGUM_ENABLE_ACTUATION 0
#endif

#ifndef BUBBLEGUM_ACTUATOR_CALIBRATION_CONFIRMED
#define BUBBLEGUM_ACTUATOR_CALIBRATION_CONFIRMED 0
#endif

#define BUBBLEGUM_AUTONOMY_PROFILE_OPEN 1
#define BUBBLEGUM_AUTONOMY_PROFILE_OBSTACLE 2

#ifndef BUBBLEGUM_AUTONOMY_PROFILE
#define BUBBLEGUM_AUTONOMY_PROFILE BUBBLEGUM_AUTONOMY_PROFILE_OPEN
#endif

#ifndef BUBBLEGUM_OBSTACLE_SINGLE_PASS_CALIBRATION
#define BUBBLEGUM_OBSTACLE_SINGLE_PASS_CALIBRATION 0
#endif

#ifndef BUBBLEGUM_CALIBRATION_MAX_RUN_MS
#define BUBBLEGUM_CALIBRATION_MAX_RUN_MS 8000
#endif

static_assert(BUBBLEGUM_ENABLE_ACTUATION == 0 ||
                  BUBBLEGUM_ENABLE_ACTUATION == 1,
              "BUBBLEGUM_ENABLE_ACTUATION must be 0 or 1");
static_assert(BUBBLEGUM_ACTUATOR_CALIBRATION_CONFIRMED == 0 ||
                  BUBBLEGUM_ACTUATOR_CALIBRATION_CONFIRMED == 1,
              "BUBBLEGUM_ACTUATOR_CALIBRATION_CONFIRMED must be 0 or 1");
static_assert(BUBBLEGUM_AUTONOMY_PROFILE == BUBBLEGUM_AUTONOMY_PROFILE_OPEN ||
                  BUBBLEGUM_AUTONOMY_PROFILE ==
                      BUBBLEGUM_AUTONOMY_PROFILE_OBSTACLE,
              "BUBBLEGUM_AUTONOMY_PROFILE must select OPEN (1) or OBSTACLE (2)");
static_assert(BUBBLEGUM_OBSTACLE_SINGLE_PASS_CALIBRATION == 0 ||
                  BUBBLEGUM_OBSTACLE_SINGLE_PASS_CALIBRATION == 1,
              "BUBBLEGUM_OBSTACLE_SINGLE_PASS_CALIBRATION must be 0 or 1");

#if BUBBLEGUM_OBSTACLE_SINGLE_PASS_CALIBRATION
static_assert(BUBBLEGUM_ENABLE_ACTUATION == 1,
              "single-pass calibration requires actuation");
static_assert(BUBBLEGUM_AUTONOMY_PROFILE ==
                  BUBBLEGUM_AUTONOMY_PROFILE_OBSTACLE,
              "single-pass calibration requires the Obstacle profile");
static_assert(BUBBLEGUM_CALIBRATION_MAX_RUN_MS >= 1000,
              "single-pass calibration runtime is too short");
#endif

#if BUBBLEGUM_ENABLE_ACTUATION && \
    !BUBBLEGUM_ACTUATOR_CALIBRATION_CONFIRMED
#error "Run the wheels-up actuator calibration, then set BUBBLEGUM_ACTUATOR_CALIBRATION_CONFIRMED=1"
#endif

#if BUBBLEGUM_ENABLE_ACTUATION
#include "runBlackBox.h"
#include "vehicleActuators.h"
#endif

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t HEALTH_STABLE_MS = 1000;
constexpr uint32_t STARTUP_FRONT_CLEAR_HOLD_MS = 250;
constexpr uint8_t MINIMUM_START_GYRO_CALIBRATION = 2;
constexpr uint32_t SERVO_SETTLE_MS = 700;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;
constexpr uint32_t SENSOR_SUMMARY_INTERVAL_MS = 1000;

HardwareSerial k230Serial(1);
TofSensors tof;
ImuSensors imuSensors;
vision::Decoder visionDecoder;

#if BUBBLEGUM_AUTONOMY_PROFILE == BUBBLEGUM_AUTONOMY_PROFILE_OBSTACLE
constexpr autonomy::ChallengeProfile CONTROLLER_PROFILE =
    autonomy::ChallengeProfile::Obstacle;
#else
constexpr autonomy::ChallengeProfile CONTROLLER_PROFILE =
    autonomy::ChallengeProfile::Open;
#endif

autonomy::Config makeBuildControllerConfig() {
  autonomy::Config config =
      CONTROLLER_PROFILE == autonomy::ChallengeProfile::Obstacle
          ? autonomy::makeGuardedObstacleConfig()
          : autonomy::makeControllerConfig(CONTROLLER_PROFILE);
#if BUBBLEGUM_OBSTACLE_SINGLE_PASS_CALIBRATION
  config.stopAfterFirstVerifiedObstaclePass = true;
  config.preferForwardObstacleProgress = false;
  config.maxRunMs = BUBBLEGUM_CALIBRATION_MAX_RUN_MS;
#endif
  return config;
}

autonomy::Controller controller(makeBuildControllerConfig());

#if BUBBLEGUM_ENABLE_ACTUATION
VehicleActuators::Config makeActuatorConfig() {
  VehicleActuators::Config config;
  // Measured on the assembled Ackermann linkage on 2026-09-01. The physical
  // endpoints were 900/2100 us, so autonomous travel keeps a 20 us inset.
  config.steeringLeftPulseUs = 920;
  config.steeringCenterPulseUs = 1500;
  config.steeringRightPulseUs = 2080;
  config.steeringReversed = false;
  config.forwardUsesRpwm = true;
  return config;
}

VehicleActuators actuators(makeActuatorConfig());
runblackbox::Recorder runBlackBox;
bool actuatorsStarted = false;
bool actuatorFault = false;
uint32_t actuatorsStartedAtMs = 0;
uint16_t appliedDrivePermille = 0;
uint32_t lastAppliedDriveRampMs = 0;

uint16_t rampAppliedDrive(uint16_t requestedPermille, uint32_t nowMs) {
  if (requestedPermille <= appliedDrivePermille) {
    appliedDrivePermille = requestedPermille;
    lastAppliedDriveRampMs = nowMs;
    return appliedDrivePermille;
  }
  if (appliedDrivePermille == 0U &&
      requestedPermille >= controller.config().minimumMovingSpeedPermille) {
    appliedDrivePermille = controller.config().minimumMovingSpeedPermille;
    lastAppliedDriveRampMs = nowMs;
    return appliedDrivePermille;
  }
  const uint32_t elapsedMs =
      static_cast<uint32_t>(nowMs - lastAppliedDriveRampMs);
  const uint32_t rise =
      (static_cast<uint32_t>(controller.config().accelerationPermillePerSecond) *
       elapsedMs) /
      1000U;
  if (rise == 0U) return appliedDrivePermille;
  const uint32_t next = static_cast<uint32_t>(appliedDrivePermille) + rise;
  appliedDrivePermille = static_cast<uint16_t>(
      next < requestedPermille ? next : requestedPermille);
  lastAppliedDriveRampMs = nowMs;
  return appliedDrivePermille;
}
#endif

bool xmsI2cStarted = false;
bool tofI2cStarted = false;
bool protocolOkay = false;
bool healthCandidateActive = false;
bool healthStable = false;
uint32_t healthCandidateSinceMs = 0;
bool startupFrontClearHeld = false;
uint32_t startupFrontClearSeenAtMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastSensorSummaryMs = 0;
uint32_t bootSessionId = 0;
uint32_t currentRunId = 0;

autonomy::RunState lastState = autonomy::RunState::WaitingForHealth;
autonomy::CourseDirection lastDirection = autonomy::CourseDirection::Unknown;
autonomy::ObstacleTarget lastObstacle = autonomy::ObstacleTarget::None;
uint32_t lastFaultFlags = autonomy::FaultNone;
uint8_t lastCornerCount = 0;
bool previousStartLatched = false;

void forceOutputsLow() {
  for (uint8_t pin : board::MOTOR_SAFE_PINS) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  digitalWrite(board::STEERING_SERVO_PIN, LOW);
  pinMode(board::STEERING_SERVO_PIN, OUTPUT);
  digitalWrite(board::STEERING_SERVO_PIN, LOW);
}

bool beginI2cBus(TwoWire& wire, const __FlashStringHelper* name,
                 uint8_t sdaPin, uint8_t sclPin,
                 uint32_t clockHz = board::I2C_CLOCK_HZ) {
  const bool started = wire.begin(sdaPin, sclPin, clockHz);
  if (started) {
    wire.setTimeOut(board::I2C_TIMEOUT_MS);
    wire.setClock(clockHz);
  }
  Serial.print(F("[i2c/"));
  Serial.print(name);
  Serial.print(F("] SDA=GPIO"));
  Serial.print(sdaPin);
  Serial.print(F(" SCL=GPIO"));
  Serial.print(sclPin);
  Serial.print(F(" begin="));
  Serial.println(started ? F("ok") : F("failed"));
  return started;
}

const char* stateName(autonomy::RunState state) {
  switch (state) {
    case autonomy::RunState::WaitingForHealth:
      return "WAIT_HEALTH";
    case autonomy::RunState::WaitingForStart:
      return "WAIT_BUTTON";
    case autonomy::RunState::DirectionProbe:
      return "DIRECTION_PROBE";
    case autonomy::RunState::Running:
      return "STRAIGHT";
    case autonomy::RunState::Cornering:
      return "CORNER";
    case autonomy::RunState::FinishDelay:
      return "FINISH_CLEAR";
    case autonomy::RunState::Finished:
      return "FINISHED";
    case autonomy::RunState::FaultLatched:
      return "FAULT_LATCHED";
    case autonomy::RunState::Parking:
      return "PARKING_LOCKED";
  }
  return "UNKNOWN";
}

const char* directionName(autonomy::CourseDirection direction) {
  switch (direction) {
    case autonomy::CourseDirection::Unknown:
      return "UNKNOWN";
    case autonomy::CourseDirection::Clockwise:
      return "CLOCKWISE";
    case autonomy::CourseDirection::CounterClockwise:
      return "COUNTERCLOCKWISE";
  }
  return "UNKNOWN";
}

const char* obstacleName(autonomy::ObstacleTarget obstacle) {
  switch (obstacle) {
    case autonomy::ObstacleTarget::None:
      return "NONE";
    case autonomy::ObstacleTarget::RedPassRight:
      return "RED_PASS_RIGHT";
    case autonomy::ObstacleTarget::GreenPassLeft:
      return "GREEN_PASS_LEFT";
  }
  return "NONE";
}

autonomy::RangeReading rangeReading(const TofSensors::Sample& sample) {
  autonomy::RangeReading reading;
  reading.online = sample.online && sample.sampleCount > 0U &&
                   sample.i2cStatus == 0U;
  reading.valid = reading.online && sample.valid;
  reading.rangeMm = sample.rangeMm;
  reading.updatedAtMs = sample.lastUpdateMs;
  reading.rangeStatus = sample.rangeStatus;
  reading.i2cStatus = sample.i2cStatus;
  return reading;
}

autonomy::BlobReading blobReading(const vision::Blob& blob, bool present) {
  autonomy::BlobReading reading;
  reading.present = present;
  if (!present) return reading;
  reading.centerX = blob.centerX;
  reading.bottomY = blob.bottomY;
  reading.width = blob.width;
  reading.height = blob.height;
  reading.confidence = blob.confidence;
  return reading;
}

bool cameraRawHealthy(uint32_t nowMs) {
  if (!visionDecoder.observationFresh(nowMs,
                                      controller.config().visionStaleMs)) {
    return false;
  }
  return visionDecoder.observation().has(vision::CameraValid);
}

bool tofResponding(const TofSensors::Sample& sample, uint32_t nowMs,
                   uint32_t maximumAgeMs) {
  return sample.online && sample.i2cStatus == 0U && sample.sampleCount > 0U &&
         static_cast<uint32_t>(nowMs - sample.lastUpdateMs) <= maximumAgeMs;
}

bool opticalNoTargetStatus(uint8_t status) {
  // 7 is ST's wrapped-target/phase-mismatch result. Status 6 is also a fresh
  // nonfatal measurement (valid without the wraparound check). On side
  // VL53L4CDs we keep it degraded rather than inventing a usable range; on the
  // front VL53L1X the acquisition layer accepts its numeric range directly.
  // None of these statuses is an I2C or module-presence failure.
  return status == 1U || status == 2U || status == 4U || status == 6U ||
         status == 7U;
}

bool sideGeometryRawHealthy(uint32_t nowMs, bool allowCrossPair) {
  const uint32_t maximumAgeMs = controller.config().sensorStaleMs;
  const TofSensors::Sample* const sides[] = {
      &tof.frontLeft(), &tof.backLeft(), &tof.frontRight(), &tof.backRight()};
  bool valid[4] = {false, false, false, false};

  for (uint8_t i = 0; i < 4U; ++i) {
    // An invalid range is tolerable only when the module itself is still
    // responding. A loose wire or stale sensor continues to block motion.
    if (!tofResponding(*sides[i], nowMs, maximumAgeMs)) return false;
    if (!sides[i]->valid && !opticalNoTargetStatus(sides[i]->rangeStatus)) {
      return false;
    }
    valid[i] = TofSensors::sampleFresh(*sides[i], nowMs, maximumAgeMs) &&
               sides[i]->rangeMm <=
                   controller.config().openingReferenceMaximumMm;
  }

  // Open Challenge has no pillars and does not need a numeric wall solution
  // merely to arm. A real no-target can occur at the legal start when an
  // angled ray looks past a 100 mm wall. All four modules were still required
  // above to answer electrically, freshly, and with a recognized status.
  // Obstacle Challenge keeps the stricter geometry gate.
  if (!controller.config().obstacleAvoidanceEnabled) return true;
  if (controller.config().guardedObstaclePassEnabled) {
    return valid[0] || valid[1] || valid[2] || valid[3];
  }

  // Accept any actual straight-geometry solution: complete A/B on the left,
  // complete C/D on the right, or both forward A/C rays. While stopped, the
  // caller may also allow one local numeric ray on each side when the centre
  // front is independently clear. That cross-pair is only an arming gate and
  // never receives wall/lateral steering authority. All four modules were
  // already required to answer electrically and freshly above, so this does
  // not hide a loose wire, stale module, or bus failure.
  const bool anyLeft = valid[0] || valid[1];
  const bool anyRight = valid[2] || valid[3];
  return (valid[0] && valid[1]) || (valid[2] && valid[3]) ||
         (valid[0] && valid[2]) ||
         (allowCrossPair && anyLeft && anyRight);
}

bool sensorsRawHealthy(uint32_t nowMs) {
  const TofSensors::Sample& front = tof.front();
  // A fresh optical no-target is allowed at the start: the wall may simply be
  // beyond the current ray. Status 3 is different—it means too close.
  const bool frontResponding =
      tofResponding(front, nowMs, controller.config().sensorStaleMs) &&
      (front.valid || opticalNoTargetStatus(front.rangeStatus));
  const bool frontNumericNow =
      TofSensors::sampleFresh(front, nowMs,
                              controller.config().sensorStaleMs);
  const bool frontClearNow =
      frontNumericNow &&
      front.rangeMm > controller.config().directionProbeAbortFrontMm;
  if (frontClearNow) {
    startupFrontClearHeld = true;
    startupFrontClearSeenAtMs = nowMs;
  } else if (!frontResponding || front.rangeStatus != 7U ||
             (startupFrontClearHeld &&
              static_cast<uint32_t>(nowMs - startupFrontClearSeenAtMs) >=
                  STARTUP_FRONT_CLEAR_HOLD_MS)) {
    startupFrontClearHeld = false;
    startupFrontClearSeenAtMs = 0;
  }
  const bool frontClearForCross =
      frontClearNow ||
      (startupFrontClearHeld &&
       static_cast<uint32_t>(nowMs - startupFrontClearSeenAtMs) <
           STARTUP_FRONT_CLEAR_HOLD_MS);
  return protocolOkay &&
         imuSensors.bnoFresh(nowMs, controller.config().sensorStaleMs) &&
         imuSensors.bnoAccuracy() >= MINIMUM_START_GYRO_CALIBRATION &&
         frontResponding &&
         sideGeometryRawHealthy(nowMs, frontClearForCross) &&
         (!controller.config().requireVisionForMotion || cameraRawHealthy(nowMs));
}

void updateHealthGate(uint32_t nowMs) {
  const bool rawHealthy = sensorsRawHealthy(nowMs);
  if (!rawHealthy) {
    healthCandidateActive = false;
    healthStable = false;
    return;
  }
  if (!healthCandidateActive) {
    healthCandidateActive = true;
    healthCandidateSinceMs = nowMs;
  }
  healthStable = static_cast<uint32_t>(nowMs - healthCandidateSinceMs) >=
                 HEALTH_STABLE_MS;
}

autonomy::Inputs collectInputs(uint32_t nowMs, bool runAlreadyStarted) {
  autonomy::Inputs inputs;
  inputs.nowMs = nowMs;
#if BUBBLEGUM_ENABLE_ACTUATION
  inputs.driveFeedbackAvailable = true;
  inputs.appliedDrivePermille = appliedDrivePermille;
#endif
  inputs.buttonPressed = digitalRead(board::START_BUTTON_PIN) == LOW;
  inputs.startupFrontClearRecent = startupFrontClearHeld;
  inputs.imu.online = imuSensors.bnoOnline();
  inputs.imu.valid = imuSensors.bnoSampleCount() > 0U;
  inputs.imu.accuracy = imuSensors.bnoAccuracy();
  inputs.imu.yawDegrees = imuSensors.yawDegrees();
  inputs.imu.updatedAtMs = imuSensors.lastBnoReadMs();
  inputs.front = rangeReading(tof.front());
  inputs.sides[autonomy::FrontLeft] = rangeReading(tof.frontLeft());
  inputs.sides[autonomy::BackLeft] = rangeReading(tof.backLeft());
  inputs.sides[autonomy::FrontRight] = rangeReading(tof.frontRight());
  inputs.sides[autonomy::BackRight] = rangeReading(tof.backRight());

  const bool heartbeatReady = visionDecoder.heartbeatReadyAndFresh(
      nowMs, controller.config().visionStaleMs);
  inputs.vision.online = heartbeatReady;
  if (visionDecoder.haveObservation()) {
    const vision::Observation& observation = visionDecoder.observation();
    inputs.vision.valid = observation.has(vision::CameraValid);
    inputs.vision.pipelineOverrun =
        observation.has(vision::PipelineOverrun);
    inputs.vision.frameQuality = observation.frameQuality;
    // This makes the controller's age check include processing time as well
    // as time since the ESP32 received the packet.
    inputs.vision.updatedAtMs =
        observation.receivedAtMs - observation.processingAgeMs;
    inputs.vision.red = blobReading(
        observation.red, observation.has(vision::RedPresent));
    inputs.vision.green = blobReading(
        observation.green, observation.has(vision::GreenPresent));
    // Parking markers are an atomic pair. The V1 decoder validates each slot,
    // while this consumer additionally refuses a one-sided packet.
    const bool magentaPairPresent =
        observation.has(vision::MagentaLeftPresent) &&
        observation.has(vision::MagentaRightPresent);
    inputs.vision.magentaLeft =
        blobReading(observation.magentaLeft, magentaPairPresent);
    inputs.vision.magentaRight =
        blobReading(observation.magentaRight, magentaPairPresent);
  }

  // Before the button, require one uninterrupted second of complete health.
  // After the run starts, expose each real status so any dropout latches.
  if (!runAlreadyStarted && !healthStable) {
    inputs.imu.online = false;
    inputs.front.online = false;
    for (autonomy::RangeReading& side : inputs.sides) side.online = false;
    inputs.vision.online = false;
  }
  return inputs;
}

void pollK230(uint32_t nowMs) {
  while (k230Serial.available() > 0) {
    visionDecoder.consume(static_cast<uint8_t>(k230Serial.read()), nowMs);
  }
}

void printRangeCsv(const TofSensors::Sample& sample) {
  if (sample.valid && sample.sampleCount > 0U) {
    Serial.print(sample.rangeMm);
  } else {
    Serial.print(-1);
  }
}

void printRangeTelemetryCsv(const TofSensors::Sample& sample,
                            uint32_t nowMs) {
  printRangeCsv(sample);
  Serial.print(',');
  Serial.print(sample.rangeStatus);
  Serial.print(',');
  Serial.print(sample.i2cStatus);
  Serial.print(',');
  if (sample.sampleCount > 0U) {
    Serial.print(static_cast<uint32_t>(nowMs - sample.lastUpdateMs));
  } else {
    Serial.print(-1);
  }
}

void printEventChanges(uint32_t nowMs, const autonomy::Decision& decision) {
  static autonomy::DriveDirection lastDriveDirection = autonomy::DriveDirection::Stopped;
  static uint8_t lastRecoveryPhase = 0;
  static uint8_t lastRecoveryHazards = 0;
  static uint8_t lastRecoveryExitReason = 0;
  if (decision.driveDirection != lastDriveDirection) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",drive_direction,"));
    Serial.println(decision.driveDirection == autonomy::DriveDirection::Reverse
                       ? F("REVERSE")
                       : decision.driveDirection == autonomy::DriveDirection::Forward
                             ? F("FORWARD") : F("STOPPED"));
    lastDriveDirection = decision.driveDirection;
  }
  if (decision.wallRecoveryPhase != lastRecoveryPhase) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",wall_recovery_phase,"));
    Serial.println(decision.wallRecoveryPhase);
    lastRecoveryPhase = decision.wallRecoveryPhase;
  }
  if (decision.wallRecoveryHazards != lastRecoveryHazards) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",wall_recovery_hazards,"));
    Serial.println(decision.wallRecoveryHazards);
    lastRecoveryHazards = decision.wallRecoveryHazards;
  }
  if (decision.wallRecoveryExitReason != lastRecoveryExitReason) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",wall_recovery_exit_reason,"));
    Serial.println(decision.wallRecoveryExitReason);
    lastRecoveryExitReason = decision.wallRecoveryExitReason;
  }
  if (decision.state != lastState) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",state,"));
    Serial.print(stateName(lastState));
    Serial.print(',');
    Serial.println(stateName(decision.state));
    lastState = decision.state;
  }
  if (decision.direction != lastDirection) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",direction,"));
    Serial.println(directionName(decision.direction));
    lastDirection = decision.direction;
  }
  if (decision.obstacle != lastObstacle) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",obstacle,"));
    Serial.println(obstacleName(decision.obstacle));
    lastObstacle = decision.obstacle;
  }
  if (decision.cornerCount != lastCornerCount) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",corner,"));
    Serial.print(decision.cornerCount);
    Serial.print(F(",lap,"));
    Serial.println(decision.lapCount);
    lastCornerCount = decision.cornerCount;
  }
  if (decision.faultFlags != lastFaultFlags) {
    Serial.print(F("EVENT,"));
    Serial.print(nowMs);
    Serial.print(F(",fault_flags,0x"));
    Serial.println(decision.faultFlags, HEX);
    lastFaultFlags = decision.faultFlags;
  }
}

void printTelemetry(uint32_t nowMs, const autonomy::Inputs& inputs,
                    const autonomy::Decision& decision) {
  Serial.print(F("DATA,"));
  Serial.print(nowMs);
  Serial.print(',');
#if BUBBLEGUM_ENABLE_ACTUATION
  Serial.print(F("AUTONOMOUS"));
#else
  Serial.print(F("DRY_RUN"));
#endif
  Serial.print(',');
  Serial.print(F("0x"));
  Serial.print(bootSessionId, HEX);
  Serial.print(',');
  Serial.print(currentRunId);
  Serial.print(',');
  Serial.print(stateName(decision.state));
  Serial.print(',');
  Serial.print(directionName(decision.direction));
  Serial.print(',');
  Serial.print(inputs.buttonPressed ? 1 : 0);
  Serial.print(',');
  Serial.print(healthStable ? 1 : 0);
  Serial.print(',');
  Serial.print(decision.motionAllowed ? 1 : 0);
  Serial.print(',');
  Serial.print(decision.speedPermille);
  Serial.print(',');
  Serial.print(decision.steeringPermille);
  Serial.print(F(",0x"));
  Serial.print(decision.faultFlags, HEX);
  Serial.print(',');
#if BUBBLEGUM_ENABLE_ACTUATION
  Serial.print(actuatorFault ? 1 : 0);
  Serial.print(',');
  Serial.print(appliedDrivePermille);
#else
  Serial.print(0);
  Serial.print(',');
  Serial.print(0);
#endif
  Serial.print(',');
  Serial.print(imuSensors.bnoOnline() ? 1 : 0);
  Serial.print(',');
  Serial.print(
      imuSensors.bnoFresh(nowMs, controller.config().sensorStaleMs) ? 1 : 0);
  Serial.print(',');
  Serial.print(imuSensors.yawDegrees(), 2);
  Serial.print(',');
  Serial.print(imuSensors.bnoAccuracy());
  Serial.print(',');
  printRangeTelemetryCsv(tof.front(), nowMs);
  Serial.print(',');
  printRangeTelemetryCsv(tof.frontLeft(), nowMs);
  Serial.print(',');
  printRangeTelemetryCsv(tof.backLeft(), nowMs);
  Serial.print(',');
  printRangeTelemetryCsv(tof.frontRight(), nowMs);
  Serial.print(',');
  printRangeTelemetryCsv(tof.backRight(), nowMs);
  Serial.print(',');
  Serial.print(cameraRawHealthy(nowMs) ? 1 : 0);
  Serial.print(',');
  Serial.print(inputs.vision.red.present ? inputs.vision.red.centerX : 0);
  Serial.print(',');
  Serial.print(inputs.vision.red.present ? inputs.vision.red.bottomY : 0);
  Serial.print(',');
  Serial.print(inputs.vision.red.present ? inputs.vision.red.confidence : 0);
  Serial.print(',');
  Serial.print(inputs.vision.green.present ? inputs.vision.green.centerX : 0);
  Serial.print(',');
  Serial.print(inputs.vision.green.present ? inputs.vision.green.bottomY : 0);
  Serial.print(',');
  Serial.print(inputs.vision.green.present ? inputs.vision.green.confidence : 0);
  Serial.print(',');
  Serial.print(obstacleName(decision.obstacle));
  Serial.print(',');
  Serial.print(decision.leftOpeningMm, 1);
  Serial.print(',');
  Serial.print(decision.rightOpeningMm, 1);
  Serial.print(',');
  Serial.print(decision.cornerProgressDegrees, 1);
  Serial.print(',');
  Serial.print(decision.cornerCount);
  Serial.print(',');
  Serial.print(decision.lapCount);
  Serial.print(',');
  Serial.print(decision.learnedRedSections, HEX);
  Serial.print(',');
  Serial.print(decision.learnedGreenSections, HEX);
  Serial.print(',');
  Serial.print(decision.wallLateralOffsetMm, 1);
  Serial.print(',');
  Serial.print(decision.wallAngleErrorMm, 1);
  Serial.print(',');
  Serial.print(decision.outerWallDistanceMm, 1);
  Serial.print(',');
  Serial.print(decision.outerWallUsable ? 1 : 0);
  Serial.print(',');
  Serial.print(decision.outerWallPair ? 1 : 0);
  Serial.print(',');
  Serial.print(decision.cornerDetectionArmed ? 1 : 0);
  Serial.print(',');
  Serial.print(decision.cornerTargetHeadingDegrees, 1);
  Serial.print(',');
  Serial.print(decision.headingErrorDegrees, 1);
  Serial.print(',');
  Serial.println(decision.targetLateralOffsetMm);
  if (controller.config().guardedObstaclePassEnabled) {
    // Preserve the established DATA schema while recording apparent pillar
    // size. Image bearing alone cannot establish chassis clearance.
    Serial.print(F("VISION_GEOMETRY,"));
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(inputs.vision.updatedAtMs);
    for (const autonomy::BlobReading* blob :
         {&inputs.vision.red, &inputs.vision.green}) {
      Serial.print(',');
      Serial.print(blob->present ? 1 : 0);
      Serial.print(',');
      Serial.print(blob->centerX);
      Serial.print(',');
      Serial.print(blob->bottomY);
      Serial.print(',');
      Serial.print(blob->width);
      Serial.print(',');
      Serial.print(blob->height);
    }
    Serial.println();
  }
}

bool applyDecision(uint32_t nowMs, const autonomy::Decision& decision) {
#if BUBBLEGUM_ENABLE_ACTUATION
  if (actuatorsStarted && !actuatorFault && actuators.leaseExpired()) {
    actuatorFault = true;
    actuators.faultStop();
    Serial.println(F("[SAFETY] drive-command lease expired; restart required"));
  }

  if (decision.startLatched && !actuatorsStarted && !actuatorFault) {
    actuatorsStarted = actuators.beginSafe();
    actuatorsStartedAtMs = nowMs;
    appliedDrivePermille = 0;
    lastAppliedDriveRampMs = nowMs;
    if (!actuatorsStarted) {
      actuatorFault = true;
      forceOutputsLow();
      Serial.println(F("[SAFETY] actuator PWM setup failed; motion locked"));
    } else {
      Serial.println(F("[actuators] attached after valid start; settling servo"));
    }
  }

  if (actuatorFault) {
    appliedDrivePermille = 0;
    lastAppliedDriveRampMs = nowMs;
    if (actuatorsStarted) actuators.faultStop();
    else forceOutputsLow();
    return true;
  }
  if (!actuatorsStarted) {
    forceOutputsLow();
    return true;
  }
  if (decision.state == autonomy::RunState::FaultLatched ||
      decision.state == autonomy::RunState::Finished) {
    appliedDrivePermille = 0;
    lastAppliedDriveRampMs = nowMs;
    actuators.faultStop();
    return true;
  }
  if (!decision.motionAllowed) {
    appliedDrivePermille = 0;
    lastAppliedDriveRampMs = nowMs;
    actuators.stop();
    if (decision.wallRecoveryPhase != 0U) {
      // Set the recovery wheel angle during neutral so reverse starts with
      // the wheels already positioned, rather than the previous forward lock.
      actuators.setSteering(
          static_cast<float>(decision.steeringPermille) / 1000.0F);
    }
    return true;
  }

  actuators.setSteering(
      static_cast<float>(decision.steeringPermille) / 1000.0F);
  if (static_cast<uint32_t>(nowMs - actuatorsStartedAtMs) < SERVO_SETTLE_MS) {
    appliedDrivePermille = 0;
    lastAppliedDriveRampMs = nowMs;
    actuators.stop();
    return true;
  }
  const bool stoppedDirection =
      decision.driveDirection == autonomy::DriveDirection::Stopped;
  if (stoppedDirection != (decision.speedPermille == 0U)) {
    appliedDrivePermille = 0;
    lastAppliedDriveRampMs = nowMs;
    actuatorFault = true;
    actuators.faultStop();
    Serial.println(F("[SAFETY] inconsistent drive direction; motion locked"));
    return true;
  }
  if (stoppedDirection) {
    appliedDrivePermille = 0;
    lastAppliedDriveRampMs = nowMs;
    actuators.stop();
    return true;
  }
  const uint16_t safeAppliedDrive =
      rampAppliedDrive(decision.speedPermille, nowMs);
  if (safeAppliedDrive == 0U) {
    actuators.stop();
    return true;
  }
  float driveCommand = static_cast<float>(safeAppliedDrive) / 1000.0F;
  if (decision.driveDirection == autonomy::DriveDirection::Reverse) {
    driveCommand = -driveCommand;
  }
  const VehicleActuators::DriveResult driveResult =
      actuators.setDriveWithResult(driveCommand);
  if (driveResult != VehicleActuators::DriveResult::Applied) {
    appliedDrivePermille = 0;
    lastAppliedDriveRampMs = nowMs;
    if (driveResult == VehicleActuators::DriveResult::WaitingForNeutral) {
      // The controller's millisecond neutral clock starts before this loop
      // physically stops the bridge. Let the actuator's microsecond interlock
      // finish its full interval, then retry without latching a false fault.
      // setDriveWithResult() has already disabled drive on this path.
      return true;
    }
    actuatorFault = true;
    actuators.faultStop();
    Serial.println(F("[SAFETY] drive command rejected; motion locked"));
    return true;
  }
  return false;
#else
  (void)nowMs;
  (void)decision;
  // This invariant is first and last in every dry-run loop. No PWM/servo
  // implementation is linked into this firmware image.
  forceOutputsLow();
  return true;
#endif
}

#if BUBBLEGUM_ENABLE_ACTUATION
uint16_t blackBoxHealthBits(const autonomy::Inputs& inputs) {
  uint16_t bits = 0U;
  if (inputs.imu.online) bits |= runblackbox::HealthImuOnline;
  if (inputs.imu.valid) bits |= runblackbox::HealthImuValid;
  if (inputs.front.online) bits |= runblackbox::HealthFrontOnline;
  if (inputs.front.valid) bits |= runblackbox::HealthFrontValid;

  bool allSidesOnline = true;
  for (const autonomy::RangeReading& side : inputs.sides) {
    allSidesOnline = allSidesOnline && side.online;
  }
  if (allSidesOnline) bits |= runblackbox::HealthAllSidesOnline;
  const bool leftPair = inputs.sides[autonomy::FrontLeft].valid &&
                        inputs.sides[autonomy::BackLeft].valid;
  const bool rightPair = inputs.sides[autonomy::FrontRight].valid &&
                         inputs.sides[autonomy::BackRight].valid;
  const bool forwardPair = inputs.sides[autonomy::FrontLeft].valid &&
                           inputs.sides[autonomy::FrontRight].valid;
  if (leftPair || rightPair || forwardPair) {
    bits |= runblackbox::HealthSideGeometryValid;
  }
  if (inputs.vision.online) bits |= runblackbox::HealthVisionOnline;
  if (inputs.vision.valid) bits |= runblackbox::HealthVisionValid;
  if (healthStable) bits |= runblackbox::HealthStartGateStable;
  if (protocolOkay) bits |= runblackbox::HealthProtocolOkay;
  return bits;
}

runblackbox::Summary blackBoxSummary(const autonomy::Inputs& inputs,
                                     const autonomy::Decision& decision) {
  runblackbox::Summary summary;
  summary.faultFlags = decision.faultFlags;
  summary.minimumValidFrontMm =
      inputs.front.valid ? inputs.front.rangeMm
                         : runblackbox::kNoValidFrontRange;
  const int32_t signedSteering = decision.steeringPermille;
  summary.maximumAbsoluteSteeringPermille = static_cast<uint16_t>(
      signedSteering < 0 ? -signedSteering : signedSteering);
  summary.healthBits = blackBoxHealthBits(inputs);
  summary.direction = static_cast<int8_t>(decision.direction);
  summary.cornerCount = decision.cornerCount;
  summary.lapCount = decision.lapCount;
  summary.actuatorFault = actuatorFault;
  summary.lastObstacle = static_cast<uint8_t>(decision.obstacle);
  summary.learnedRedSections = decision.learnedRedSections;
  summary.learnedGreenSections = decision.learnedGreenSections;
  return summary;
}
#endif

}  // namespace

void setup() {
  forceOutputsLow();
  pinMode(board::START_BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(SERIAL_BAUD);
  delay(100);
  bootSessionId = esp_random();
  Serial.println();
#if BUBBLEGUM_ENABLE_ACTUATION
#if BUBBLEGUM_OBSTACLE_SINGLE_PASS_CALIBRATION
  Serial.println(F("BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R42 SINGLE PASS - AUTO-STOP"));
#else
  Serial.println(autonomy::controllerSourceIdentity(CONTROLLER_PROFILE));
#endif
  Serial.println(F("WARNING: lift wheels until servo and motor direction are calibrated"));
  runBlackBox.begin(Serial);
#else
  Serial.println(F("BUBBLEGUM FULL AUTONOMY DRY RUN"));
  Serial.println(F("ACTUATION COMPILED OUT: GPIO10/11/12/13 forced LOW"));
#endif
  Serial.println(F("servo signal=GPIO13 start button=GPIO4-to-GND"));
  Serial.println(F("arming requires XMS-A5 gyro calibration >= 2 for 1 second"));
  Serial.println(F("direction: first side-wall opening; laps: 12 verified corner exits"));
#if BUBBLEGUM_AUTONOMY_PROFILE == BUBBLEGUM_AUTONOMY_PROFILE_OBSTACLE
  Serial.println(F("challenge=OBSTACLE pillars=ACTIVE vision=REQUIRED parking=DISABLED"));
#else
  Serial.println(F("challenge=OPEN pillars=IGNORED parking=DISABLED"));
#endif
  Serial.print(F("CONFIG,cruise="));
  Serial.print(controller.config().cruiseSpeedPermille);
  Serial.print(F(",probe="));
  Serial.print(controller.config().directionProbeSpeedPermille);
  Serial.print(F(",corner="));
  Serial.print(controller.config().cornerSpeedPermille);
  Serial.print(F(",front_trigger_mm="));
  Serial.print(controller.config().frontCornerTriggerMm);
  Serial.print(F(",direction_open_delta_mm="));
  Serial.print(controller.config().directionProbeOpeningDeltaMm, 0);
  Serial.print(F(",outer_target_mm="));
  Serial.print(controller.config().openOuterWallTargetMm);
  Serial.print(F(",side_pairs_parallel="));
  Serial.print(controller.config().sideSensorPairsAreParallel ? 1 : 0);
  Serial.print(F(",single_pass="));
  Serial.print(controller.config().stopAfterFirstVerifiedObstaclePass ? 1 : 0);
  Serial.print(F(",guarded_obstacle="));
  Serial.print(controller.config().guardedObstaclePassEnabled ? 1 : 0);
  Serial.print(F(",wall_recovery="));
  Serial.print(controller.config().wallRecoveryEnabled ? 1 : 0);
  Serial.print(F(",forward_progress="));
  Serial.print(controller.config().preferForwardObstacleProgress ? 1 : 0);
  Serial.print(F(",remember_pillars="));
  Serial.print(controller.config().rememberObstacleSections ? 1 : 0);
  Serial.print(F(",body_clear_front_mm="));
  Serial.print(controller.config().obstacleBodyClearFrontGuardMm);
  Serial.print(F(",body_clear_ms="));
  Serial.print(controller.config().obstacleSinglePassBodyClearMs);
  Serial.print(F(",recover_before_pass="));
  Serial.print(controller.config().recoverCenteredEscapeBeforePass ? 1 : 0);
  Serial.print(F(",active_pillar_bottom="));
  Serial.print(controller.config().obstacleMinimumBottomY);
  Serial.print(F(",pass_proof_bottom="));
  Serial.print(controller.config().obstaclePassProofMinimumBottomY);
  Serial.print(F(",pass_timeout_ms="));
  Serial.print(controller.config().obstacleMaximumCommitMs);
  Serial.print(F(",escape_deadline_ms="));
  Serial.print(controller.config().obstacleCenteredEscapeDeadlineMs);
  Serial.print(F(",escape_clock="));
  Serial.print(controller.config().guardedObstaclePassEnabled
                   ? F("applied_drive") : F("wall_time"));
  Serial.print(F(",max_run_ms="));
  Serial.println(controller.config().maxRunMs);
  if (controller.config().guardedObstaclePassEnabled) {
    Serial.println(F("VISION_GEOMETRY_HEADER,ms,vision_sample_ms,red_present,red_x,red_bottom,red_width,red_height,green_present,green_x,green_bottom,green_width,green_height"));
  }
  Serial.println(F("CSV_HEADER,ms,mode,boot_id_hex,run_id,state,direction,button,health,motion,requested_speed_permille,steer_permille,controller_fault_hex,actuator_fault,applied_drive_permille,xms_a5_online,xms_a5_fresh,yaw_deg,gyro_calibration,front_mm,front_status,front_i2c,front_age_ms,front_left_mm,front_left_status,front_left_i2c,front_left_age_ms,back_left_mm,back_left_status,back_left_i2c,back_left_age_ms,front_right_mm,front_right_status,front_right_i2c,front_right_age_ms,back_right_mm,back_right_status,back_right_i2c,back_right_age_ms,camera_fresh,red_x,red_bottom,red_conf,green_x,green_bottom,green_conf,obstacle,left_open_mm,right_open_mm,corner_deg,corners,laps,learned_red_sections_hex,learned_green_sections_hex,wall_offset_mm,wall_angle_mm,outer_wall_mm,outer_wall_usable,outer_wall_pair,corner_armed,corner_target_deg,heading_error_deg,target_offset_mm"));
  Serial.print(F("BOOT_ID,0x"));
  Serial.println(bootSessionId, HEX);

  protocolOkay = vision::protocolSelfTest();
  Serial.print(F("[protocol] V1 self-test: "));
  Serial.println(protocolOkay ? F("PASS") : F("FAIL - motion locked"));

  xmsI2cStarted = beginI2cBus(
      Wire, F("xms-a5"), board::XMS_A5_I2C_SDA_PIN,
      board::XMS_A5_I2C_SCL_PIN, board::XMS_A5_I2C_CLOCK_HZ);
  tofI2cStarted = beginI2cBus(Wire1, F("tof"), board::TOF_I2C_SDA_PIN,
                              board::TOF_I2C_SCL_PIN);
  tof.begin(Wire1, tofI2cStarted, Serial);
  imuSensors.begin(Wire, xmsI2cStarted, Serial);

  k230Serial.begin(board::K230_UART_BAUD, SERIAL_8N1,
                   board::K230_UART_RX_PIN, board::K230_UART_TX_PIN);
  Serial.print(F("[k230] UART1 RX=GPIO"));
  Serial.print(board::K230_UART_RX_PIN);
  Serial.print(F(" TX=GPIO"));
  Serial.println(board::K230_UART_TX_PIN);
  // The 150 ms drive lease stops the motor first. This task watchdog is the
  // second layer for a larger loop deadlock and is fed by the Arduino core.
  enableLoopWDT();
  Serial.println(F("[ready] wait for one second of healthy data, then press button"));
}

void loop() {
#if !BUBBLEGUM_ENABLE_ACTUATION
  forceOutputsLow();
#endif
  const uint32_t sensorPollMs = millis();

  pollK230(sensorPollMs);
  imuSensors.poll(sensorPollMs, Serial);
  tof.poll(sensorPollMs);

  // Use time captured after potentially blocking sensor I/O for every
  // freshness decision and controller update.
  const uint32_t nowMs = millis();
  updateHealthGate(nowMs);

  const bool runAlreadyStarted = controller.decision().startLatched;
  const autonomy::Inputs inputs = collectInputs(nowMs, runAlreadyStarted);
  const autonomy::Decision decision = controller.update(inputs);
  const bool justStarted = decision.startLatched && !previousStartLatched;
  if (justStarted) ++currentRunId;
  previousStartLatched = decision.startLatched;

  printEventChanges(nowMs, decision);
  const bool outputsStopped = applyDecision(nowMs, decision);

#if BUBBLEGUM_ENABLE_ACTUATION
  const runblackbox::Summary summary = blackBoxSummary(inputs, decision);
  if (justStarted) {
    runBlackBox.startRun(nowMs, summary, Serial);
  }
  runBlackBox.observe(nowMs, summary);
  if (decision.state == autonomy::RunState::Finished) {
    runBlackBox.persistTerminal(runblackbox::TerminalStatus::Finished,
                                outputsStopped, Serial);
  } else if (decision.state == autonomy::RunState::FaultLatched ||
             actuatorFault) {
    // An actuator fault is also terminal at the vehicle level: applyDecision()
    // has already latched the driver off even if the pure controller has not
    // independently raised a sensor/navigation fault.
    runBlackBox.persistTerminal(runblackbox::TerminalStatus::FaultLatched,
                                outputsStopped, Serial);
  } else if (justStarted) {
    const bool insideServoSettle =
        actuatorsStarted &&
        static_cast<uint32_t>(nowMs - actuatorsStartedAtMs) < SERVO_SETTLE_MS;
    runBlackBox.persistActive(outputsStopped && insideServoSettle, Serial);
  }
#else
  (void)justStarted;
  (void)outputsStopped;
#endif

  if (static_cast<uint32_t>(nowMs - lastTelemetryMs) >=
      TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry(nowMs, inputs, decision);
  }
  if (static_cast<uint32_t>(nowMs - lastSensorSummaryMs) >=
      SENSOR_SUMMARY_INTERVAL_MS) {
    lastSensorSummaryMs = nowMs;
    tof.printSummary(Serial, nowMs);
    imuSensors.printSummary(Serial, nowMs);
    const vision::DecoderStats& stats = visionDecoder.stats();
    Serial.print(F("[k230] observations="));
    Serial.print(stats.validObservations);
    Serial.print(F(" crc_bad="));
    Serial.print(stats.crcErrors);
    Serial.print(F(" payload_bad="));
    Serial.print(stats.payloadErrors);
    Serial.print(F(" dropped="));
    Serial.println(stats.droppedFrames);
  }

#if !BUBBLEGUM_ENABLE_ACTUATION
  forceOutputsLow();
#endif
  yield();
}

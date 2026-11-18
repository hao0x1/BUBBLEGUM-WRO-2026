#include "autonomyController.h"
#include "controllerProfiles.h"
#include "visionProtocol.h"

#include <cmath>
#include <cstring>
#include <cstdint>
#include <iostream>

namespace {

using autonomy::BackLeft;
using autonomy::BackRight;
using autonomy::Controller;
using autonomy::CourseDirection;
using autonomy::Decision;
using autonomy::DriveDirection;
using autonomy::DriveDirectionInterlock;
using autonomy::FaultDirectionTimeout;
using autonomy::FaultDirectionAmbiguous;
using autonomy::FaultCornerOvershoot;
using autonomy::FaultCornerTimeout;
using autonomy::FaultConfiguration;
using autonomy::FaultFrontTooClose;
using autonomy::FaultFrontTofUnavailable;
using autonomy::FaultFrontTofStale;
using autonomy::FaultFinishTimeout;
using autonomy::FaultHeadingJump;
using autonomy::FaultImuStale;
using autonomy::FaultObstaclePassTimeout;
using autonomy::FaultRunTimeout;
using autonomy::FaultSideTofStale;
using autonomy::FaultSideTofUnavailable;
using autonomy::FaultVisionStale;
using autonomy::FaultWrongWay;
using autonomy::FrontLeft;
using autonomy::FrontRight;
using autonomy::Inputs;
using autonomy::ObstacleTarget;
using autonomy::ParkingBaySide;
using autonomy::ParkingPhase;
using autonomy::RunState;

int failures = 0;

void check(bool condition, const char* message) {
  if (condition) return;
  ++failures;
  std::cerr << "FAIL: " << message << '\n';
}

void checkNear(float actual, float expected, float tolerance,
               const char* message) {
  check(std::fabs(actual - expected) <= tolerance, message);
}

float wrapYaw(float degrees) {
  while (degrees <= -180.0F) degrees += 360.0F;
  while (degrees > 180.0F) degrees -= 360.0F;
  return degrees;
}

Inputs healthyInputs(uint32_t nowMs, float yawDegrees = 0.0F) {
  Inputs inputs;
  inputs.nowMs = nowMs;
  inputs.imu = {true, true, yawDegrees, nowMs, 3};
  inputs.front = {true, true, 900, nowMs};
  for (auto& side : inputs.sides) side = {true, true, 300, nowMs};
  inputs.vision.online = true;
  inputs.vision.valid = true;
  inputs.vision.updatedAtMs = nowMs;
  return inputs;
}

autonomy::Config fixtureConfig() {
  autonomy::Config config;
  // Most cases isolate another invariant and therefore use already-confirmed
  // opening evidence. A separate regression below exercises the real 100 ms
  // confirmation default.
  config.openingConfirmationMs = 0;
  config.cornerRefractoryMs = 0;
  config.cornerRearmStableMs = 0;
  return config;
}

autonomy::Config legacyOpenFixtureConfig() {
  autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  // These fixtures exercise established heading, lap, and optical invariants,
  // not production Open's first-direction acquisition count.
  config.firstDirectionApproachHoldEnabled = false;
  config.directionMinimumDistinctSamples = 2;
  config.cornerDriveDelayMs = 0;
  // Their compact setup enters the first turn at 600 mm. Keep that synthetic
  // setup outside the legacy abort boundary without changing production Open.
  config.directionProbeAbortFrontMm = 280;
  return config;
}

Decision updateFresh(Controller& controller, Inputs& inputs, uint32_t nowMs,
                     float yawDegrees) {
  inputs.nowMs = nowMs;
  inputs.imu.updatedAtMs = nowMs;
  inputs.imu.yawDegrees = yawDegrees;
  inputs.front.updatedAtMs = nowMs;
  for (auto& side : inputs.sides) side.updatedAtMs = nowMs;
  inputs.vision.updatedAtMs = nowMs;
  return controller.update(inputs);
}

Decision startController(Controller& controller, Inputs& inputs,
                         uint32_t baseMs = 0, float yawDegrees = 0.0F) {
  inputs = healthyInputs(baseMs, yawDegrees);
  inputs.buttonPressed = false;
  Decision decision = controller.update(inputs);
  check(decision.state == RunState::WaitingForStart,
        "healthy reset must reach WaitingForStart without motion");
  check(!decision.motionAllowed && decision.speedPermille == 0,
        "waiting state must hold drive stopped");

  decision = updateFresh(controller, inputs, baseMs + 40, yawDegrees);
  inputs.buttonPressed = true;
  decision = updateFresh(controller, inputs, baseMs + 50, yawDegrees);
  check(decision.state == RunState::WaitingForStart,
        "button transition must not bypass debounce");
  decision = updateFresh(controller, inputs, baseMs + 90, yawDegrees);
  check(decision.state == RunState::DirectionProbe,
        "one debounced press must start direction probing");
  check(decision.startLatched && decision.motionAllowed,
        "accepted start must latch exactly one moving run");
  return decision;
}

Decision startDirectionProbeWithBaselines(Controller& controller,
                                          Inputs& inputs,
                                          bool leftForwardNumeric,
                                          bool rightForwardNumeric,
                                          bool rearRaysNumeric,
                                          uint16_t forwardBaselineMm = 604) {
  inputs = healthyInputs(0);
  inputs.front.rangeMm = 1201;
  for (auto& side : inputs.sides) {
    side.online = true;
    side.valid = false;
    side.rangeMm = 0;
    side.rangeStatus = 2;
    side.i2cStatus = 0;
  }
  if (leftForwardNumeric) {
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeMm = forwardBaselineMm;
    inputs.sides[FrontLeft].rangeStatus = 0;
  }
  if (rightForwardNumeric) {
    inputs.sides[FrontRight].valid = true;
    inputs.sides[FrontRight].rangeMm = forwardBaselineMm;
    inputs.sides[FrontRight].rangeStatus = 0;
  }
  if (rearRaysNumeric) {
    inputs.sides[BackLeft].valid = true;
    inputs.sides[BackLeft].rangeMm = 300;
    inputs.sides[BackLeft].rangeStatus = 0;
    inputs.sides[BackRight].valid = true;
    inputs.sides[BackRight].rangeMm = 300;
    inputs.sides[BackRight].rangeStatus = 0;
  }

  Decision decision = controller.update(inputs);
  check(decision.state == RunState::WaitingForStart,
        "probe-baseline fixture must arm while stationary");
  updateFresh(controller, inputs, 40, 0.0F);
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, 50, 0.0F);
  decision = updateFresh(controller, inputs, 90, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "probe-baseline fixture must start with unknown direction");
  return decision;
}

void testAngleWrapping() {
  checkNear(Controller::shortestAngleDeltaDegrees(179.0F, -179.0F),
            2.0F, 0.001F, "+179 to -179 must unwrap as +2 degrees");
  checkNear(Controller::shortestAngleDeltaDegrees(-179.0F, 179.0F),
            -2.0F, 0.001F, "-179 to +179 must unwrap as -2 degrees");
  checkNear(Controller::shortestAngleDeltaDegrees(10.0F, 190.0F),
            180.0F, 0.001F, "half turn must use the documented +180 sign");
}

void testHeldButtonCannotStart() {
  Controller controller(fixtureConfig());
  Inputs inputs = healthyInputs(0);
  inputs.buttonPressed = true;
  Decision decision = controller.update(inputs);
  decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::WaitingForStart,
        "button held from boot must not start the car");
  check(!decision.startLatched && !decision.motionAllowed,
        "held boot button must leave motion inhibited");

  inputs.buttonPressed = false;
  updateFresh(controller, inputs, 120, 0.0F);
  updateFresh(controller, inputs, 160, 0.0F);
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, 170, 0.0F);
  decision = updateFresh(controller, inputs, 210, 0.0F);
  check(decision.state == RunState::DirectionProbe && decision.startLatched,
        "release followed by a debounced press must start once");
}

void setStraightGeometry(Inputs& inputs, uint16_t frontMm = 1200) {
  inputs.front.rangeMm = frontMm;
  for (auto& side : inputs.sides) {
    side.online = true;
    side.valid = true;
    side.rangeMm = 300;
  }
}

void setLeftOpening(Inputs& inputs, uint16_t frontMm = 600) {
  setStraightGeometry(inputs, frontMm);
  inputs.sides[FrontLeft].rangeMm = 550;
  inputs.sides[BackLeft].rangeMm = 300;
}

void setRightOpening(Inputs& inputs, uint16_t frontMm = 600) {
  setStraightGeometry(inputs, frontMm);
  inputs.sides[FrontRight].rangeMm = 550;
  inputs.sides[BackRight].rangeMm = 300;
}

void testDirectionOpeningAndHeadingWrap() {
  {
    Controller controller(fixtureConfig());
    Inputs inputs;
    startController(controller, inputs);
    Decision decision = updateFresh(controller, inputs, 120, -30.0F);
    check(decision.direction == CourseDirection::Unknown &&
              decision.state == RunState::DirectionProbe,
          "yaw alone must never guess a course direction");

    setLeftOpening(inputs, 701);
    decision = updateFresh(controller, inputs, 130, -30.0F);
    check(decision.direction == CourseDirection::Unknown &&
              decision.state == RunState::DirectionProbe,
          "opening evidence too far from the first corner must be ignored");

    setLeftOpening(inputs, 600);
    inputs.sides[FrontRight].rangeMm = 550;
    inputs.sides[BackRight].rangeMm = 300;
    decision = updateFresh(controller, inputs, 140, -30.0F);
    check(decision.direction == CourseDirection::Unknown &&
              decision.state == RunState::DirectionProbe,
          "tied left/right openings must remain ambiguous instead of guessing");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.directionDecisionHoldMs = 300;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
  setLeftOpening(inputs, 280);
  inputs.sides[FrontRight].rangeMm = 550;
  inputs.sides[BackRight].rangeMm = 300;
  Decision decision = updateFresh(controller, inputs, 120, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed && decision.speedPermille == 0,
          "ambiguous close-front evidence must stop while it is checked");
    decision = updateFresh(controller, inputs, 419, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "ambiguity hold must remain stopped until its exact timeout");
    decision = updateFresh(controller, inputs, 420, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultDirectionAmbiguous) != 0,
          "persistent ambiguity at the close front limit must fault safely");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.openingConfirmationMs = 100;
    config.directionDecisionHoldMs = 300;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs, 280);
    Decision decision = updateFresh(controller, inputs, 120, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "a first close-front opening sample must stop during confirmation");
    decision = updateFresh(controller, inputs, 220, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == CourseDirection::CounterClockwise,
          "a stable close-front opening must finish confirmation without racing the ambiguity fault");
  }
  {
    Controller controller(fixtureConfig());
    Inputs inputs;
    startController(controller, inputs, 0, 179.0F);
    setLeftOpening(inputs);
    Decision decision = updateFresh(controller, inputs, 100, 179.0F);
    check(decision.direction == CourseDirection::CounterClockwise,
          "a winning left opening must lock counter-clockwise");
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.steeringPermille < 0,
          "left opening must begin a bounded powered left corner");
    decision = updateFresh(controller, inputs, 140, -151.0F);
    checkNear(decision.unwrappedHeadingDegrees, 30.0F, 0.001F,
              "first corner yaw must unwrap continuously across +180");
    check(decision.configuredClockwiseYawSign == -1 &&
              decision.cornerProgressDegrees > 0.0F,
          "first left corner must use the explicit mounted BNO yaw sign");
  }
  {
    Controller controller(fixtureConfig());
    Inputs inputs;
    startController(controller, inputs, 0, -179.0F);
    setRightOpening(inputs);
    Decision decision = updateFresh(controller, inputs, 100, -179.0F);
    check(decision.direction == CourseDirection::Clockwise,
          "a winning right opening must lock clockwise");
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.steeringPermille > 0,
          "right opening must begin a bounded powered right corner");
    decision = updateFresh(controller, inputs, 140, 151.0F);
    checkNear(decision.unwrappedHeadingDegrees, -30.0F, 0.001F,
              "first corner yaw must unwrap continuously across -180");
  }
}

void testOpeningConfirmation() {
  Controller controller;
  Inputs inputs;
  startController(controller, inputs);
  setLeftOpening(inputs);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "one opening sample must not choose the course direction");
  decision = updateFresh(controller, inputs, 199, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "opening confirmation must not complete one millisecond early");
  decision = updateFresh(controller, inputs, 200, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::CounterClockwise,
        "stable opening evidence must lock direction after 100 ms");
}

void testWallGuidanceSignsAndClamp() {
  autonomy::Config config = fixtureConfig();
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  setLeftOpening(inputs);
  updateFresh(controller, inputs, 100, 0.0F);
  updateFresh(controller, inputs, 120, 40.0F);
  setStraightGeometry(inputs);
  Decision decision = updateFresh(controller, inputs, 140, 90.0F);
  check(decision.state == RunState::Running,
        "wall-guidance fixture must first leave the centred direction probe");
  decision = updateFresh(controller, inputs, 150, 90.0F);
  check(decision.steeringPermille == 0,
        "equal parallel wall ranges must command centred steering");

  inputs.sides[FrontLeft].rangeMm = 200;
  inputs.sides[BackLeft].rangeMm = 200;
  inputs.sides[FrontRight].rangeMm = 400;
  inputs.sides[BackRight].rangeMm = 400;
  decision = updateFresh(controller, inputs, 160, 90.0F);
  check(decision.wallLateralOffsetMm < 0.0F &&
            decision.steeringPermille > 0,
        "vehicle close to left wall must steer right");

  inputs.sides[FrontLeft].rangeMm = 400;
  inputs.sides[BackLeft].rangeMm = 400;
  inputs.sides[FrontRight].rangeMm = 200;
  inputs.sides[BackRight].rangeMm = 200;
  decision = updateFresh(controller, inputs, 170, 90.0F);
  check(decision.wallLateralOffsetMm > 0.0F &&
            decision.steeringPermille < 0,
        "vehicle close to right wall must steer left");

  inputs.sides[FrontLeft].rangeMm = 600;
  inputs.sides[BackLeft].rangeMm = 150;
  inputs.sides[FrontRight].rangeMm = 100;
  inputs.sides[BackRight].rangeMm = 600;
  decision = updateFresh(controller, inputs, 180, 90.0F);
  check(decision.wallAngleErrorMm > 0.0F &&
            decision.steeringPermille < 0,
        "nose pointing right relative to walls must steer left");
  check(decision.steeringPermille >= -1000 &&
            decision.steeringPermille <= 1000,
        "wall steering must remain inside normalized actuator bounds");
}

autonomy::BlobReading usableBlob(int16_t centerX, uint16_t bottomY,
                                 uint16_t height, uint16_t confidence) {
  autonomy::BlobReading blob;
  blob.present = true;
  blob.centerX = centerX;
  blob.bottomY = bottomY;
  blob.width = 1000;
  blob.height = height;
  blob.confidence = confidence;
  return blob;
}

void testObstacleSideSelection() {
  const auto select = [](const autonomy::BlobReading& red,
                         const autonomy::BlobReading& green) {
    Controller controller(fixtureConfig());
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.red = red;
    inputs.vision.green = green;
    return updateFresh(controller, inputs, 100, 0.0F);
  };

  Decision decision = select(usableBlob(-1000, 4500, 3000, 900),
                             usableBlob(1000, 5000, 1000, 500));
  check(decision.obstacle == ObstacleTarget::GreenPassLeft,
        "lower image position must deterministically win target priority");

  decision = select(usableBlob(-1000, 5000, 1800, 600),
                    usableBlob(1000, 5000, 1800, 600));
  check(decision.obstacle == ObstacleTarget::RedPassRight,
        "an exact red/green priority tie must resolve deterministically");

  decision = select(usableBlob(10000, 5000, 1800, 600), {});
  check(decision.obstacle == ObstacleTarget::RedPassRight &&
            decision.targetLateralOffsetMm > 0,
        "an extreme red image position must always retain a right-pass offset");
  decision = select({}, usableBlob(-10000, 5000, 1800, 600));
  check(decision.obstacle == ObstacleTarget::GreenPassLeft &&
            decision.targetLateralOffsetMm < 0,
        "an extreme green image position must always retain a left-pass offset");
}

void testObstacleRangeCannotBecomeCorner() {
  autonomy::Config config = fixtureConfig();
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  setLeftOpening(inputs);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::Cornering,
        "fixture must enter the first verified left corner");
  decision = updateFresh(controller, inputs, 140, 40.0F);
  setStraightGeometry(inputs);
  decision = updateFresh(controller, inputs, 180, 90.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1,
        "fixture must reach straight running after one verified exit");

  inputs.vision.red = usableBlob(0, 6000, 2000, 700);
  setStraightGeometry(inputs, 300);
  decision = updateFresh(controller, inputs, 200, 90.0F);
  check(decision.state == RunState::Running &&
            decision.obstacle == ObstacleTarget::RedPassRight,
        "an actionable pillar and short front range must remain avoidance");

  inputs.vision.red = {};
  setLeftOpening(inputs, 300);
  decision = updateFresh(controller, inputs, 220, 90.0F);
  check(decision.state == RunState::Running &&
            decision.obstacle == ObstacleTarget::RedPassRight,
        "an active obstacle must block otherwise-valid corner evidence");
  decision = updateFresh(controller, inputs, 349, 90.0F);
  check(decision.state == RunState::Running &&
            decision.obstacle == ObstacleTarget::RedPassRight,
        "same-color absence must reach its full 150 ms before pass clearing can start");

  setStraightGeometry(inputs, 300);
  inputs.sides[FrontLeft].rangeMm = 200;
  inputs.sides[BackLeft].rangeMm = 300;
  decision = updateFresh(controller, inputs, 449, 90.0F);
  check(decision.obstacle == ObstacleTarget::RedPassRight,
        "a selected-pair baseline alone must preserve the commitment");
  inputs.sides[FrontLeft].rangeMm = 320;
  decision = updateFresh(controller, inputs, 450, 90.0F);
  check(decision.obstacle == ObstacleTarget::RedPassRight,
        "the exact 120 mm relative rise must begin pass-clear confirmation");
  decision = updateFresh(controller, inputs, 629, 90.0F);
  check(decision.obstacle == ObstacleTarget::RedPassRight,
        "active obstacle confirmation must not finish one millisecond early");
  decision = updateFresh(controller, inputs, 630, 90.0F);
  check(decision.obstacle == ObstacleTarget::None &&
            decision.targetLateralOffsetMm > 0,
        "a fully proven pass must clear into a bounded recenter phase");

  setLeftOpening(inputs, 300);
  decision = updateFresh(controller, inputs, 929, 90.0F);
  check(decision.state == RunState::Running,
        "the recenter phase must block a corner until its exact boundary");
  decision = updateFresh(controller, inputs, 930, 90.0F);
  check(decision.state == RunState::Cornering &&
            decision.cornerCount == 1,
        "a proven pass may recover the next close corner only after recenter completes");
}

void testSpeedEnvelopeAndEmergencyStop() {
  autonomy::Config config = fixtureConfig();
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  config.cornerRefractoryMs = 0;
  config.accelerationPermillePerSecond = 10000;
  config.decelerationPermillePerSecond = 10000;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  Decision decision = updateFresh(controller, inputs, 190, 0.0F);
  check(decision.speedPermille == controller.config().directionProbeSpeedPermille,
        "direction probe must use its conservative configured speed");

  inputs.front.rangeMm = 400;
  decision = updateFresh(controller, inputs, 200, 0.0F);
  check(decision.speedPermille >=
            controller.config().minimumMovingSpeedPermille &&
            decision.speedPermille <
                controller.config().directionProbeSpeedPermille,
        "direction probe must obey the front stopping envelope without stalling");

  // Complete one physical corner so this comparison exercises cruise guidance
  // rather than the deliberately conservative direction-probe speed.
  setLeftOpening(inputs);
  decision = updateFresh(controller, inputs, 230, 0.0F);
  check(decision.state == RunState::Cornering,
        "speed fixture must enter a verified first corner");
  setStraightGeometry(inputs);
  decision = updateFresh(controller, inputs, 250, 40.0F);
  decision = updateFresh(controller, inputs, 270, 90.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1,
        "speed fixture must reach straight running after one corner");

  decision = updateFresh(controller, inputs, 370, 90.0F);
  const uint16_t centredCruiseSpeed = decision.speedPermille;
  check(decision.steeringPermille == 0,
        "centred parallel walls must establish the cruise-speed baseline");

  inputs.sides[FrontLeft].rangeMm = 100;
  inputs.sides[BackLeft].rangeMm = 150;
  inputs.sides[FrontRight].rangeMm = 600;
  inputs.sides[BackRight].rangeMm = 600;
  decision = updateFresh(controller, inputs, 380, 90.0F);
  check(std::abs(decision.steeringPermille) >= 700,
        "large wall offset must command a strong steering turn");
  check(decision.speedPermille >= controller.config().minimumMovingSpeedPermille &&
            decision.speedPermille < centredCruiseSpeed,
        "a large steering turn must slow below centred cruise without stalling");

  inputs.front.rangeMm = controller.config().frontEmergencyStopMm;
  decision = updateFresh(controller, inputs, 390, 90.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultFrontTooClose) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0,
        "emergency front range must stop and latch in the same update");

  inputs.front.rangeMm = 900;
  decision = updateFresh(controller, inputs, 400, 90.0F);
  check(decision.state == RunState::FaultLatched &&
            !decision.motionAllowed && decision.speedPermille == 0,
        "healthy data must not silently clear a motion fault");
}

void testProductionProfileContracts() {
  const autonomy::Config open =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  const autonomy::Config obstacle =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);

  check(!open.sideSensorPairsAreParallel &&
            !obstacle.sideSensorPairsAreParallel &&
            open.cornerRefractoryMs == 250 &&
            open.cornerRearmStableMs == 100 &&
            open.cornerRearmFrontClearMm == 1200 &&
            open.frontCornerTriggerMm == 1000 &&
            open.directionProbeFrontMaximumMm == 1000 &&
            open.directionProbeAbortFrontMm == 600 &&
            obstacle.directionProbeFrontMaximumMm == 1000 &&
            obstacle.directionProbeAbortFrontMm == 600,
        "both profiles must model the physical diagonal rays and retain the 1000/600 direction window");
  check(open.maximumWallCorrectionPermille == 0,
        "production Open must disable range-derived wall steering");
  check(open.directionProbeSpeedPermille == 600 &&
            open.cruiseSpeedPermille == 1000 &&
            open.cornerSpeedPermille == 450 &&
            open.cornerBlindSpeedPermille == 200 &&
            open.cornerSteeringPermille == 760 &&
            open.cornerDriveDelayMs == 0,
        "production Open must use the 600/1000/450/200 speed stages and start drive with its 760 steering command");
  check(obstacle.directionProbeSpeedPermille == 400 &&
            obstacle.cruiseSpeedPermille == 600 &&
            obstacle.cornerSpeedPermille == 450 &&
            obstacle.cornerBlindSpeedPermille == 450 &&
            obstacle.cornerSteeringPermille == 760 &&
            obstacle.cornerDriveDelayMs == 0,
        "production Obstacle must use its 400/600 cruise stages and the proven 450/760 corner command");
  check(open.openingConfirmationMs == 100 &&
            open.openingMinimumDistinctSamples == 2 &&
            open.directionMinimumDistinctSamples == 3 &&
            open.directionOpeningWinnerMarginMm == 100.0F &&
            open.directionProbeOpeningDeltaMm == 35.0F &&
            open.cornerOpeningDeltaMm == 180.0F &&
            obstacle.directionProbeOpeningDeltaMm == 35.0F &&
            obstacle.cornerOpeningDeltaMm == 180.0F,
        "production direction selection must use the physical FL/FR current-space threshold while retaining temporal corner openings");
  check(obstacle.obstacleCloseBottomY == 4500 &&
            obstacle.obstacleFullSteeringBottomY == 5500 &&
            obstacle.obstacleMinimumCommitMs == 300 &&
            obstacle.obstacleMaximumCommitMs == 8000 &&
            obstacle.obstacleBlobLossMs == 150 &&
            obstacle.obstaclePreCloseLossClearMs == 500 &&
            obstacle.obstaclePassClearDeltaMm == 120 &&
            obstacle.obstaclePassClearConfirmationMs == 180 &&
            obstacle.obstacleRecenterMs == 300 &&
            obstacle.obstacleRecommitCooldownMs == 300 &&
            obstacle.obstacleMinimumOffsetMm == 160 &&
            obstacle.obstacleOffsetMm == 180 &&
            obstacle.obstacleMaximumOffsetMm == 300 &&
            obstacle.obstacleMaximumImageSteeringPermille == 500 &&
            obstacle.obstacleSideClearanceGuardMm == 140 &&
            obstacle.obstacleSideClearanceSpeedPermille == 300 &&
            obstacle.obstacleSideClearanceSteeringPermille == 650 &&
            obstacle.maximumWallCorrectionPermille == 500,
        "production obstacle profile must retain temporal pass proof and R23 wide-car steering bounds");
  check(!open.parkingEnabled && !obstacle.parkingEnabled &&
            !open.stopAfterFirstVerifiedObstaclePass &&
            !obstacle.stopAfterFirstVerifiedObstaclePass &&
            !open.parkingCalibrationValidated &&
            !obstacle.parkingCalibrationValidated &&
            !open.parkingRearClearanceValidated &&
            !obstacle.parkingRearClearanceValidated &&
            open.parkingBaySide == ParkingBaySide::Unknown &&
            obstacle.parkingBaySide == ParkingBaySide::Unknown &&
            open.parkingReverseNeutralMs ==
                autonomy::kMinimumDriveReversalNeutralMs &&
            obstacle.parkingReverseNeutralMs ==
                autonomy::kMinimumDriveReversalNeutralMs,
        "R20 production profiles must keep uncalibrated parking disabled");
  check(std::strcmp(
            autonomy::controllerSourceIdentity(
                autonomy::ChallengeProfile::Obstacle),
            "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R42 FULL COURSE - PARKING DISABLED") ==
            0 &&
            std::strcmp(
                autonomy::controllerSourceIdentity(
                    autonomy::ChallengeProfile::Open),
                "BUBBLEGUM AUTONOMOUS OPEN 20260905-R20 FIELD RECOVERY - MOTION CAPABLE") ==
                0,
        "board source identities must distinguish R42 full obstacle motion from the R20 Open field recovery");
}

void testR40GuardedSpeedsPreserveCornerTorqueWithoutStraightSurge() {
  const autonomy::Config config = autonomy::makeGuardedObstacleConfig();
  check(config.cruiseSpeedPermille == 400 &&
            config.directionProbeSpeedPermille == 400 &&
            config.obstacleSpeedPermille == 400 &&
            config.cornerSpeedPermille == 450 &&
            config.cornerBlindSpeedPermille == 450 &&
            config.minimumMovingSpeedPermille == 300 &&
            config.accelerationPermillePerSecond == 600,
        "R40 guarded profile must cap straight, probe, and pillar speed at400 while retaining450 corner torque and300 moving floor");
  const autonomy::Config open =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  check(open.cruiseSpeedPermille == 1000 &&
            open.directionProbeSpeedPermille == 600 &&
            open.cornerSpeedPermille == 450 &&
            open.minimumMovingSpeedPermille == 200 &&
            open.accelerationPermillePerSecond == 1600,
        "R40 guarded speed reduction must leave the demonstrated R18 Open profile unchanged");

  for (const bool clockwise : {false, true}) {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    const float sign = clockwise ? 1.0F : -1.0F;
    if (clockwise) setRightOpening(inputs, 900);
    else setLeftOpening(inputs, 900);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 150, 0.0F);
    Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.speedPermille >= 300 &&
              std::abs(decision.steeringPermille) == 760,
          "R40 an actual mirrored full-lock corner must begin above its moving floor");
    setStraightGeometry(inputs, 1500);
    decision = updateFresh(controller, inputs, 600, sign * 45.0F);
    check(decision.state == RunState::Cornering &&
              decision.speedPermille == 450 &&
              std::abs(decision.steeringPermille) == 760,
          "R40 an actual mirrored full-lock corner must reach450 even though straight cruise is400");
    updateFresh(controller, inputs, 1000, sign * 85.0F);
    updateFresh(controller, inputs, 1200, sign * 94.44F);
    decision = updateFresh(controller, inputs, 1400, sign * 94.44F);
    check(decision.state == RunState::Running && decision.cornerCount == 1,
          "R40 speed replay must complete one actual observed corner before testing its straight handoff");
    for (const uint32_t elapsedMs : {0U, 50U, 100U, 149U, 150U,
                                     200U, 300U, 500U, 1000U}) {
      decision = updateFresh(controller, inputs, 1400 + elapsedMs,
                             sign * 94.44F);
      check(decision.state == RunState::Running &&
                decision.obstacle == ObstacleTarget::None &&
                decision.speedPermille >= 300 &&
                decision.speedPermille <= 400 &&
                decision.cornerCount == 1 && decision.lapCount == 0,
            "R40 blank-camera corner exit must remain300to400 throughout and after the150ms unwind window instead of surging to595");
    }
    auto& pillar = clockwise ? inputs.vision.red : inputs.vision.green;
    pillar = usableBlob(clockwise ? 5000 : -5000, 8033, 3750, 940);
    decision = updateFresh(controller, inputs, 2500, sign * 94.44F);
    check(decision.obstacle == (clockwise ? ObstacleTarget::RedPassRight
                                         : ObstacleTarget::GreenPassLeft) &&
              decision.speedPermille >= 300 &&
              decision.speedPermille <= 400 &&
              (clockwise ? decision.steeringPermille > 0
                         : decision.steeringPermille < 0),
          "R40 accepted near pillars must keep mirrored away steering above the moving floor within the400 speed cap");
  }
}

void testR41FreshClearRouteReleasesMissingPillarWithoutCourseCredit() {
  for (const bool redPillar : {false, true}) {
    for (int blockedEvidence = 0; blockedEvidence < 4; ++blockedEvidence) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      config.preferForwardObstacleProgress = true;
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs);
      setStraightGeometry(inputs, 1500);
      auto& oldPillar = redPillar ? inputs.vision.red : inputs.vision.green;
      oldPillar = usableBlob(redPillar ? 5000 : -5000, 8000, 3000, 900);
      const ObstacleTarget oldTarget = redPillar ? ObstacleTarget::RedPassRight
                                                : ObstacleTarget::GreenPassLeft;
      Decision decision = updateFresh(controller, inputs, 100, 0.0F);
      check(decision.obstacle == oldTarget && decision.speedPermille >= 300 &&
                (redPillar ? decision.steeringPermille > 0
                           : decision.steeringPermille < 0),
            "R41 fresh accepted red and green must still choose their respective right and left steering");
      oldPillar = {};
      if (blockedEvidence == 1) inputs.vision.valid = false;
      if (blockedEvidence == 2) inputs.front.rangeMm = 350;
      if (blockedEvidence == 3) inputs.sides[FrontLeft].rangeMm = 200;
      updateFresh(controller, inputs, 200, 0.0F);
      decision = updateFresh(controller, inputs, 350, 0.0F);
      if (blockedEvidence != 0) {
        check(decision.obstacle == oldTarget &&
                  decision.cornerCount == 0 && decision.lapCount == 0 &&
                  decision.state != RunState::Finished,
              "R41 stale camera, front350, or leading200 must not manufacture an absent-pillar clearance release");
        continue;
      }
      check(decision.obstacle == ObstacleTarget::None &&
                decision.state == RunState::DirectionProbe &&
                decision.cornerCount == 0 && decision.lapCount == 0 &&
                decision.targetLateralOffsetMm == 0 &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.speedPermille > 0,
            "R41 fresh blank camera and a clear route must release the old target without strict pass proof or invented course credit");
      auto& nextPillar = redPillar ? inputs.vision.green : inputs.vision.red;
      nextPillar = usableBlob(redPillar ? -5000 : 5000, 8000, 3000, 900);
      decision = updateFresh(controller, inputs, 450, 0.0F);
      check(decision.obstacle == (redPillar ? ObstacleTarget::GreenPassLeft
                                           : ObstacleTarget::RedPassRight) &&
                decision.speedPermille >= 300 &&
                (redPillar ? decision.steeringPermille < 0
                           : decision.steeringPermille > 0) &&
                decision.cornerCount == 0 && decision.lapCount == 0,
            "R41 after releasing a missing pillar a fresh opposite color must take over with correct away steering and no false corner");
    }
  }
}

void testR41ForwardEscapeUsesFreshLeadingAndTailClearance() {
  for (const bool leftHazard : {false, true}) {
    for (int routeCase = 0; routeCase < 8; ++routeCase) {
      Controller controller(autonomy::makeGuardedObstacleConfig());
      Inputs inputs;
      startController(controller, inputs);
      setStraightGeometry(inputs, 1500);
      for (auto& side : inputs.sides) side.rangeMm = 600;
      const uint8_t nearRay = leftHazard ? FrontLeft : FrontRight;
      const uint8_t otherRay = leftHazard ? FrontRight : FrontLeft;
      const uint8_t tailRay = leftHazard ? BackLeft : BackRight;
      inputs.sides[nearRay].rangeMm = 185;
      if (routeCase == 1) inputs.front.rangeMm = 350;
      if (routeCase == 2) inputs.sides[nearRay].rangeMm = 140;
      if (routeCase == 3) inputs.sides[otherRay].rangeMm = 300;
      if (routeCase == 4) inputs.sides[tailRay].rangeMm = 300;
      if (routeCase == 5) inputs.sides[tailRay].i2cStatus = 4;
      if (routeCase == 6) inputs.vision.valid = false;
      if (routeCase == 7) inputs.front.rangeMm = 180;
      Decision decision = updateFresh(controller, inputs, 100, 0.0F);
      if (routeCase == 0) {
        check(decision.state == RunState::DirectionProbe &&
                  decision.faultFlags == autonomy::FaultNone &&
                  decision.driveDirection == DriveDirection::Forward &&
                  decision.speedPermille == 400 &&
                  decision.steeringPermille == (leftHazard ? 650 : -650) &&
                  decision.cornerCount == 0 && decision.lapCount == 0,
              "R41 a single leading185 hazard with a fresh clear forward and tail route must make mirrored forward progress");
        inputs.sides[nearRay].rangeMm = 260;
        decision = updateFresh(controller, inputs, 200, 0.0F);
        check(decision.state == RunState::DirectionProbe &&
                  decision.faultFlags == autonomy::FaultNone &&
                  decision.speedPermille > 0 &&
                  decision.cornerCount == 0 && decision.lapCount == 0,
              "R41 cleared leading hazard must return to navigation without inventing a corner or obstacle pass");
      } else {
        check(!(decision.driveDirection == DriveDirection::Forward &&
                decision.speedPermille > 0),
              "R41 front350 or180, hard leading140, opposite300, tail300, and invalid transport must not authorize a forward escape");
      }
    }
  }
}

void testR41CornerCoastCanUseClearForwardRouteWithoutFakeProgress() {
  for (const bool clockwise : {false, true}) {
    autonomy::Config config = autonomy::makeGuardedObstacleConfig();
    config.openingConfirmationMs = 0;
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    const float sign = clockwise ? 1.0F : -1.0F;
    if (clockwise) setRightOpening(inputs, 650);
    else setLeftOpening(inputs, 650);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 200, sign * 45.0F);
    const uint8_t nearRay = clockwise ? FrontLeft : FrontRight;
    const uint8_t otherRay = clockwise ? FrontRight : FrontLeft;
    const uint8_t tailRay = clockwise ? BackLeft : BackRight;
    const uint8_t otherRear = clockwise ? BackRight : BackLeft;
    inputs.front.rangeMm = 327;
    inputs.sides[nearRay].rangeMm = 192;
    inputs.sides[otherRay].rangeMm = 618;
    inputs.sides[tailRay].rangeMm = 566;
    inputs.sides[otherRear].valid = false;
    inputs.sides[otherRear].rangeStatus = 2;
    inputs.sides[otherRear].rangeMm = 0;
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    Decision decision = updateFresh(controller, inputs, 250, sign * 50.0F);
    check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
          "R41 front327 and near leading192 must first stop in neutral");
    inputs.front.rangeMm = 375;
    inputs.sides[nearRay].rangeMm = 182;
    inputs.sides[otherRay].rangeMm = 667;
    inputs.sides[tailRay].rangeMm = 602;
    decision = updateFresh(controller, inputs, 549, sign * 61.0F);
    check(decision.state == RunState::Cornering &&
              decision.speedPermille == 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "R41 observed neutral coasting must not consume reverse heading allowance or bypass299ms neutral");
    inputs.front.rangeMm = 377;
    inputs.sides[nearRay].rangeMm = 176;
    decision = updateFresh(controller, inputs, 550, sign * 62.5F);
    check(decision.state == RunState::Cornering &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.driveDirection == DriveDirection::Forward &&
              decision.speedPermille == 450 &&
              decision.steeringPermille == (clockwise ? 760 : -760) &&
              decision.cornerCount == 0 && decision.lapCount == 0 &&
              std::fabs(decision.cornerTargetHeadingDegrees - sign * 90.0F) < 0.01F,
          "R41 actual377center176leading route after12degree neutral coast must resume the same mirrored corner instead of latching0x80");
  }
}

void testR41ForwardEscapeIsBoundedByAppliedDriveHeadingAndTime() {
  for (const bool leftHazard : {false, true}) {
    for (int limit = 0; limit < 3; ++limit) {
      Controller controller(autonomy::makeGuardedObstacleConfig());
      Inputs inputs;
      startController(controller, inputs);
      setStraightGeometry(inputs, 1500);
      for (auto& side : inputs.sides) side.rangeMm = 600;
      inputs.sides[leftHazard ? FrontLeft : FrontRight].rangeMm = 185;
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      updateFresh(controller, inputs, 100, 0.0F);
      Decision decision = updateFresh(controller, inputs, 600, 0.0F);
      check(decision.driveDirection == DriveDirection::Forward &&
                decision.speedPermille == 400,
            "R41 servo or actuator pause must not consume the500ms powered forward-escape budget");
      uint32_t boundaryMs = 1100;
      float boundaryYaw = 0.0F;
      if (limit == 0) {
        inputs.appliedDrivePermille = 400;
        decision = updateFresh(controller, inputs, 1099, 0.0F);
        check(decision.driveDirection == DriveDirection::Forward &&
                  decision.speedPermille == 400,
              "R41 bounded forward escape may continue through499ms actual drive");
      } else if (limit == 1) {
        boundaryMs = 700;
        boundaryYaw = leftHazard ? 20.0F : -20.0F;
      } else {
        boundaryMs = 1600;
        decision = updateFresh(controller, inputs, 1599, 0.0F);
        check(decision.driveDirection == DriveDirection::Forward &&
                  decision.speedPermille == 400,
              "R41 an unpowered forward escape must remain bounded by its independent1500ms wall timer");
      }
      decision = updateFresh(controller, inputs, boundaryMs, boundaryYaw);
      check(!(decision.driveDirection == DriveDirection::Forward &&
              decision.speedPermille > 0) &&
                decision.cornerCount == 0 && decision.lapCount == 0,
            "R41 exact500ms powered,20degree,or1500ms wall limit must end forward escape without awarding progress");
      decision = updateFresh(controller, inputs, boundaryMs + 10, boundaryYaw);
      check(!(decision.driveDirection == DriveDirection::Forward &&
              decision.speedPermille > 0),
            "R41 uncleared bounded escape cannot silently start another forward episode on the next tick");
    }
  }
}

void testR41SameColorSuccessorRefreshesPlanWithoutPassCredit() {
  for (const bool redPillar : {false, true}) {
    Controller controller(autonomy::makeGuardedObstacleConfig());
    Inputs inputs;
    startController(controller, inputs);
    setStraightGeometry(inputs, 1500);
    for (auto& side : inputs.sides) side.rangeMm = 600;
    auto& pillar = redPillar ? inputs.vision.red : inputs.vision.green;
    pillar = usableBlob(0, 10000, 3000, 900);
    const float sign = redPillar ? 1.0F : -1.0F;
    updateFresh(controller, inputs, 100, 0.0F);
    pillar.bottomY = 7000;
    updateFresh(controller, inputs, 200, sign * 10.0F);
    updateFresh(controller, inputs, 280, sign * 10.0F);
    updateFresh(controller, inputs, 300, sign * 22.0F);
    Decision decision = updateFresh(controller, inputs, 380, sign * 22.0F);
    check(decision.obstacle == (redPillar ? ObstacleTarget::RedPassRight
                                         : ObstacleTarget::GreenPassLeft) &&
              decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille > 0
                         : decision.steeringPermille < 0) &&
              decision.cornerCount == 0 && decision.lapCount == 0,
          "R41 a farther same-color blob confirmed over80ms must reset the heading plan without waiting for an old ToF pass transition or awarding a corner");
    updateFresh(controller, inputs, 480, sign * 32.0F);
    decision = updateFresh(controller, inputs, 560, sign * 32.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille < 0
                         : decision.steeringPermille > 0) &&
              decision.cornerCount == 0 && decision.lapCount == 0,
          "R41 the successor maneuver must recover relative to its newly observed heading instead of the abandoned pillar heading");
  }
}

void testR41ForwardCornerEscapeTapersAndUsesNormalExitProof() {
  for (const bool clockwise : {false, true}) {
    Controller controller(autonomy::makeGuardedObstacleConfig());
    Inputs inputs;
    startController(controller, inputs);
    const float sign = clockwise ? 1.0F : -1.0F;
    if (clockwise) setRightOpening(inputs, 900);
    else setLeftOpening(inputs, 900);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 150, 0.0F);
    updateFresh(controller, inputs, 200, 0.0F);
    setStraightGeometry(inputs, 1500);
    for (auto& side : inputs.sides) side.rangeMm = 600;
    updateFresh(controller, inputs, 600, sign * 45.0F);
    inputs.sides[clockwise ? FrontLeft : FrontRight].rangeMm = 185;
    Decision decision = updateFresh(controller, inputs, 1000, sign * 85.0F);
    check(decision.state == RunState::Cornering &&
              decision.speedPermille == 450 &&
              decision.steeringPermille == (clockwise ? 190 : -190) &&
              decision.cornerCount == 0 && decision.lapCount == 0,
          "R41 forward corner escape entering at85degrees must taper to190 instead of starting another full-lock20degree arc");
    decision = updateFresh(controller, inputs, 1050, sign * 88.0F);
    check(decision.state == RunState::Cornering &&
              decision.steeringPermille == (clockwise ? 76 : -76) &&
              decision.cornerCount == 0,
          "R41 at88degrees forward escape must taper to76 and await ordinary corner-exit confirmation");
    decision = updateFresh(controller, inputs, 1100, sign * 90.0F);
    check(decision.state == RunState::Cornering &&
              decision.steeringPermille == 0 && decision.cornerCount == 0,
          "R41 the absolute90degree target must remove further away steering while normal200ms exit proof remains pending");
    decision = updateFresh(controller, inputs, 1249, sign * 90.0F);
    check(decision.state == RunState::Cornering && decision.cornerCount == 0,
          "R41 a forward escape must not award the corner before its normal200ms confirmation boundary");
    decision = updateFresh(controller, inputs, 1250, sign * 90.0F);
    check(decision.state == RunState::Running &&
              decision.cornerCount == 1 && decision.lapCount == 0 &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0 && decision.speedPermille <= 400,
          "R41 normal confirmed corner exit inside a bounded forward arc must count once and immediately return to the400 straight envelope");
    inputs.sides[clockwise ? FrontLeft : FrontRight].rangeMm = 600;
    decision = updateFresh(controller, inputs, 1300, sign * 90.0F);
    check(decision.state == RunState::Running && decision.cornerCount == 1 &&
              decision.lapCount == 0 && decision.speedPermille <= 400,
          "R41 a cleared escape after the confirmed corner must not count the same corner twice");
  }
}

void testR42ObstacleSectionMemoryIsOptionalWithoutChangingLiveSteering() {
  const autonomy::Config production = autonomy::makeGuardedObstacleConfig();
  check(!production.rememberObstacleSections,
        "R42 full course must disable learned pillar section locations");
  check(autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle)
            .rememberObstacleSections,
        "R42 the historical obstacle profile must retain its explicit memory default");
  for (const bool redPillar : {false, true}) {
    for (const bool remember : {false, true}) {
      autonomy::Config config = production;
      config.rememberObstacleSections = remember;
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs);
      setStraightGeometry(inputs, 1500);
      auto& pillar = redPillar ? inputs.vision.red : inputs.vision.green;
      pillar = usableBlob(redPillar ? 5000 : -5000, 8000, 3000, 900);
      const Decision decision = updateFresh(controller, inputs, 100, 0.0F);
      check(decision.obstacle == (redPillar ? ObstacleTarget::RedPassRight
                                           : ObstacleTarget::GreenPassLeft) &&
                decision.speedPermille > 0 &&
                (redPillar ? decision.steeringPermille > 0
                           : decision.steeringPermille < 0),
            "R42 disabling location memory must retain live red-right and green-left steering");
      check(decision.learnedRedSections == (remember && redPillar ? 1 : 0) &&
                decision.learnedGreenSections == (remember && !redPillar ? 1 : 0),
            "R42 learned section bits must remain zero only when their profile memory switch is disabled");
    }
  }
}

void testR42OpticalRecoveryHoldReplaysFreshInvalidLeadingRay() {
  for (const bool clockwise : {false, true}) {
    autonomy::Config config = autonomy::makeGuardedObstacleConfig();
    config.openingConfirmationMs = 0;
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    const float sign = clockwise ? 1.0F : -1.0F;
    if (clockwise) setRightOpening(inputs, 650);
    else setLeftOpening(inputs, 650);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 200, sign * 45.0F);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.front.rangeMm = 0;
    const uint8_t nearRay = clockwise ? FrontRight : FrontLeft;
    const uint8_t otherRay = clockwise ? FrontLeft : FrontRight;
    const uint8_t openRear = clockwise ? BackRight : BackLeft;
    const uint8_t numericRear = clockwise ? BackLeft : BackRight;
    inputs.sides[nearRay].rangeMm = 140;
    inputs.sides[otherRay].rangeMm = 363;
    inputs.sides[numericRear].rangeMm = 703;
    inputs.sides[openRear].valid = false;
    inputs.sides[openRear].rangeStatus = 2;
    inputs.sides[openRear].rangeMm = 0;
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    Decision decision = updateFresh(controller, inputs, 250, sign * 60.0F);
    check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
          "R42 optical-front corner collision must first enter stopped side recovery");
    inputs.sides[nearRay].valid = false;
    inputs.sides[nearRay].rangeStatus = 0;
    inputs.sides[nearRay].rangeMm = 0;
    decision = updateFresh(controller, inputs, 266, sign * 65.0F);
    check(decision.state == RunState::Cornering &&
              decision.wallRecoveryPhase == 1 &&
              decision.speedPermille == 0 && !decision.motionAllowed &&
              decision.cornerCount == 0 && decision.lapCount == 0,
          "R42 the actual16ms-later fresh invalid status0 leading ray must hold zero instead of latching geometry fault0x40");
    decision = updateFresh(controller, inputs, 4000, sign * 87.0F);
    check(decision.state == RunState::Cornering &&
              decision.speedPermille == 0 && !decision.motionAllowed,
          "R42 an optical readiness hold may outlast the active recovery clock while drive remains stopped");
    inputs.sides[nearRay].valid = true;
    inputs.sides[nearRay].rangeStatus = 0;
    inputs.sides[nearRay].rangeMm = 53;
    inputs.front.valid = true;
    inputs.front.rangeStatus = 0;
    inputs.front.rangeMm = 3;
    decision = updateFresh(controller, inputs, 4010, sign * 87.0F);
    check(decision.state == RunState::Cornering &&
              decision.wallRecoveryPhase == 2 &&
              decision.driveDirection == DriveDirection::Reverse &&
              decision.speedPermille == 300 &&
              decision.cornerCount == 0 && decision.lapCount == 0,
          "R42 restoring fresh leading53 front3 and measured rear clearance must allow bounded reverse without another button or another powered-time charge");
    inputs.appliedDrivePermille = 300;
    decision = updateFresh(controller, inputs, 4011, sign * 87.0F);
    check(decision.driveDirection == DriveDirection::Reverse &&
              decision.speedPermille == 300 &&
              decision.faultFlags == autonomy::FaultNone,
          "R42 stopped readiness time must not consume the reverse powered limit or use the pre-coast heading as a reverse-error baseline");
  }
}

void testR42ReverseReadinessNeverTreatsUnknownRearAsClear() {
  for (const uint8_t rear : {uint8_t{BackLeft}, uint8_t{BackRight}}) {
    for (int faultCase = 0; faultCase < 7; ++faultCase) {
      const autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs);
      inputs.sides[BackLeft].rangeMm = 500;
      inputs.sides[BackRight].rangeMm = 500;
      inputs.front.rangeMm = 300;
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      updateFresh(controller, inputs, 100, 0.0F);
      updateFresh(controller, inputs, 400, 0.0F);
      inputs.appliedDrivePermille = 300;
      if (faultCase < 4) {
        inputs.sides[rear].valid = false;
        inputs.sides[rear].rangeStatus =
            faultCase == 0 ? 0 : faultCase == 1 ? 1 : faultCase == 2 ? 6 : 7;
        inputs.sides[rear].rangeMm = 0;
      }
      if (faultCase == 4) inputs.sides[rear].i2cStatus = 4;
      if (faultCase == 6) inputs.sides[rear].rangeMm = 200;
      Decision decision;
      if (faultCase == 5) {
        inputs.nowMs = 500;
        inputs.imu.updatedAtMs = 500;
        inputs.vision.updatedAtMs = 500;
        inputs.front.updatedAtMs = 500;
        for (auto& side : inputs.sides) side.updatedAtMs = 500;
        inputs.sides[rear].updatedAtMs = 0;
        decision = controller.update(inputs);
      } else {
        decision = updateFresh(controller, inputs, 500, 0.0F);
      }
      check(decision.speedPermille == 0 && !decision.motionAllowed &&
                decision.driveDirection == DriveDirection::Stopped,
            "R42 reverse must stop immediately for either rear's invalid0 weak1/6/7 electrical failure stale receipt or hard200 boundary");
      if (faultCase >= 4) continue;
      inputs.appliedDrivePermille = 0;
      decision = updateFresh(controller, inputs, 5000, 0.0F);
      check(decision.state == RunState::DirectionProbe &&
                decision.speedPermille == 0 &&
                decision.driveDirection == DriveDirection::Stopped,
            "R42 unknown rear clearance must remain a stopped hold beyond3000ms rather than authorize blind reverse");
      inputs.sides[rear].valid = true;
      inputs.sides[rear].rangeStatus = 0;
      inputs.sides[rear].rangeMm = 500;
      decision = updateFresh(controller, inputs, 5010, 0.0F);
      check(decision.state == RunState::DirectionProbe &&
                decision.driveDirection == DriveDirection::Reverse &&
                decision.speedPermille == 300 &&
                decision.faultFlags == autonomy::FaultNone,
            "R42 fresh rear clearance must resume the bounded reverse without counting stopped sensor-wait time as powered travel");
      inputs.sides[rear].valid = false;
      inputs.sides[rear].rangeStatus = 0;
      inputs.sides[rear].rangeMm = 0;
      updateFresh(controller, inputs, 5020, 0.0F);
      decision = updateFresh(controller, inputs, 90 + config.maxRunMs, 0.0F);
      check(decision.state == RunState::FaultLatched &&
                (decision.faultFlags & FaultRunTimeout) != 0U &&
                decision.speedPermille == 0,
            "R42 prolonged stopped readiness holds must still terminate at the absolute170s course limit");
    }
  }
}

void testR42SuccessfulBackoffSkipsOnlyOldColorForOneSecond() {
  for (const bool redPillar : {false, true}) {
    for (int nextCase = 0; nextCase < 4; ++nextCase) {
      Controller controller(autonomy::makeGuardedObstacleConfig());
      Inputs inputs;
      startController(controller, inputs);
      setStraightGeometry(inputs, 1500);
      for (auto& side : inputs.sides) side.rangeMm = 600;
      auto& oldPillar = redPillar ? inputs.vision.red : inputs.vision.green;
      oldPillar = usableBlob(redPillar ? 5000 : -5000, 8000, 3000, 900);
      updateFresh(controller, inputs, 100, 0.0F);
      inputs.front.rangeMm = 300;
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      updateFresh(controller, inputs, 200, 0.0F);
      Decision decision = updateFresh(controller, inputs, 500, 0.0F);
      check(decision.wallRecoveryPhase == 2 &&
                decision.driveDirection == DriveDirection::Reverse,
            "R42 old-target skip fixture must actually begin a guarded backoff");
      inputs.appliedDrivePermille = nextCase == 3 ? 0 : 300;
      inputs.front.rangeMm = 700;
      decision = updateFresh(controller, inputs, 800, 0.0F);
      check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
            "R42 measured front clearance after powered backoff must begin exit neutral");
      inputs.appliedDrivePermille = 0;
      decision = updateFresh(controller, inputs, 1100, 0.0F);
      if (nextCase == 3) {
        check(decision.wallRecoveryPhase == 0 &&
                  decision.obstacle == (redPillar ? ObstacleTarget::RedPassRight
                                                 : ObstacleTarget::GreenPassLeft) &&
                  decision.cornerCount == 0 && decision.lapCount == 0,
              "R42 a commanded but never applied reverse must not be mislabeled a successful powered backoff and skip the visible target");
        continue;
      }
      check(decision.wallRecoveryPhase == 0 &&
                decision.obstacle == ObstacleTarget::None &&
                decision.cornerCount == 0 && decision.lapCount == 0 &&
                decision.learnedRedSections == 0 && decision.learnedGreenSections == 0,
            "R42 successful powered backoff must release the old target without pass credit or learned pillar locations");
      if (nextCase == 1) {
        auto& newPillar = redPillar ? inputs.vision.green : inputs.vision.red;
        newPillar = usableBlob(redPillar ? -5000 : 5000, 8000, 3000, 950);
      }
      if (nextCase == 2) inputs.front.rangeMm = 180;
      decision = updateFresh(controller, inputs, 1110, 0.0F);
      if (nextCase == 1) {
        check(decision.obstacle == (redPillar ? ObstacleTarget::GreenPassLeft
                                             : ObstacleTarget::RedPassRight) &&
                  (redPillar ? decision.steeringPermille < 0
                             : decision.steeringPermille > 0),
              "R42 skipping one old color must not suppress a fresh opposite-color obstacle");
      } else if (nextCase == 2) {
        check(!(decision.driveDirection == DriveDirection::Forward &&
                decision.speedPermille > 0),
              "R42 temporary old-color suppression must never bypass the live front180 collision guard");
      } else {
        check(decision.obstacle == ObstacleTarget::None &&
                  decision.speedPermille > 0,
              "R42 a still-visible old pillar must not immediately restart the same maneuver after backoff");
        decision = updateFresh(controller, inputs, 2099, 0.0F);
        check(decision.obstacle == ObstacleTarget::None,
              "R42 the old-color backoff quarantine must remain active through999ms");
        decision = updateFresh(controller, inputs, 2100, 0.0F);
        check(decision.obstacle == (redPillar ? ObstacleTarget::RedPassRight
                                             : ObstacleTarget::GreenPassLeft) &&
                  decision.speedPermille > 0 &&
                  (redPillar ? decision.steeringPermille > 0
                             : decision.steeringPermille < 0) &&
                  decision.cornerCount == 0 && decision.lapCount == 0,
              "R42 the same color must become eligible again at1000ms from fresh observations without a permanent blacklist");
      }
    }
  }
}

void testR20ParkingScaffoldingFailsClosed() {
  autonomy::Config enabled =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
  enabled.parkingEnabled = true;
  enabled.parkingCalibrationValidated = true;
  enabled.parkingRearClearanceValidated = true;
  enabled.parkingBaySide = ParkingBaySide::Right;
  enabled.parkingReverseNeutralMs =
      autonomy::kMinimumDriveReversalNeutralMs;

  Controller lockedController(enabled);
  Inputs inputs = healthyInputs(0);
  Decision decision = lockedController.update(inputs);
  check(decision.state == RunState::WaitingForHealth &&
            (decision.faultFlags & FaultConfiguration) != 0U &&
            decision.parkingPhase == ParkingPhase::Disabled &&
            decision.driveDirection == DriveDirection::Stopped &&
            !decision.startLatched && !decision.motionAllowed &&
            decision.speedPermille == 0U,
        "R20 must reject parking even when flags are forced true because calibrated motion is not implemented");

  autonomy::Config obstacle =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
  Controller legacyController(obstacle);
  decision = startController(legacyController, inputs);
  inputs.vision.magentaLeft = {true, -2500, 6500, 500, 1200, 900};
  inputs.vision.magentaRight = {true, 2500, 6500, 500, 1200, 900};
  decision = updateFresh(legacyController, inputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.parkingPhase == ParkingPhase::Disabled &&
            decision.driveDirection == DriveDirection::Forward &&
            decision.obstacle == ObstacleTarget::None,
        "an atomic magenta pair must remain inert while parking is disabled");
}

void testR21SinglePassCalibrationStopsOnlyAfterVerifiedClear() {
  {
    autonomy::Config invalid =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
    invalid.stopAfterFirstVerifiedObstaclePass = true;
    Controller controller(invalid);
    Inputs inputs = healthyInputs(0);
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags & FaultConfiguration) != 0U &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "single-pass containment must fail closed if paired with Open");
  }

  {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.stopAfterFirstVerifiedObstaclePass = true;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.red = usableBlob(-3500, 4499, 2000, 700);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.vision.red = {};
    updateFresh(controller, inputs, 110, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 610, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::None &&
              decision.motionAllowed,
          "a far pillar disappearance must abandon and recenter, not claim a completed calibration pass");
  }

  for (const bool redPillar : {true, false}) {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.stopAfterFirstVerifiedObstaclePass = true;
    config.maxRunMs = 5000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setStraightGeometry(inputs);
    const uint8_t selectedFront = redPillar ? FrontLeft : FrontRight;
    const uint8_t selectedBack = redPillar ? BackLeft : BackRight;
    inputs.sides[selectedFront].rangeMm = 200;
    inputs.sides[selectedBack].rangeMm = 300;
    if (redPillar) {
      inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 5200, 2000, 700);
    }
    const ObstacleTarget expected =
        redPillar ? ObstacleTarget::RedPassRight
                  : ObstacleTarget::GreenPassLeft;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.obstacle == expected && decision.motionAllowed,
          "single-pass calibration must first commit the intended close pillar");

    inputs.vision.red = {};
    inputs.vision.green = {};
    updateFresh(controller, inputs, 110, 0.0F);
    inputs.sides[selectedFront].rangeMm = 320;
    decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.state != RunState::Finished &&
              decision.obstacle == expected,
          "verified side transition must begin, not skip, pass confirmation");
    decision = updateFresh(controller, inputs, 579, 0.0F);
    check(decision.state != RunState::Finished,
          "single-pass calibration must not stop one millisecond early");
    decision = updateFresh(controller, inputs, 580, 0.0F);
    check(decision.state == RunState::Finished &&
              decision.obstacle == ObstacleTarget::None &&
              !decision.motionAllowed && decision.speedPermille == 0 &&
              decision.steeringPermille == 0 &&
              decision.driveDirection == DriveDirection::Stopped,
          "a strictly verified red or green pass must auto-stop the calibration firmware");
    decision = updateFresh(controller, inputs, 700, 0.0F);
    check(decision.state == RunState::Finished &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "single-pass calibration stop must remain terminal until reset");
  }
}

void testDriveDirectionNeutralInterlock() {
  DriveDirectionInterlock interlock;
  constexpr uint32_t neutralMs =
      autonomy::kMinimumDriveReversalNeutralMs;

  check(interlock.update(DriveDirection::Forward, 0U, neutralMs) ==
            DriveDirection::Forward,
        "a stopped drivetrain may accept its first forward request");
  check(interlock.update(DriveDirection::Reverse, 10U, neutralMs) ==
            DriveDirection::Stopped,
        "a direct forward-to-reverse request must insert neutral");
  check(interlock.update(DriveDirection::Reverse, 10U + neutralMs - 1U,
                         neutralMs) == DriveDirection::Stopped,
        "reverse must remain stopped one millisecond before neutral completes");
  check(interlock.update(DriveDirection::Reverse, 10U + neutralMs,
                         neutralMs) == DriveDirection::Reverse,
        "reverse may be exposed exactly when the neutral interval completes");

  interlock.reset();
  check(interlock.update(DriveDirection::Forward, 1000U, neutralMs) ==
            DriveDirection::Forward &&
            interlock.update(DriveDirection::Stopped, 1010U, neutralMs) ==
                DriveDirection::Stopped &&
            interlock.update(DriveDirection::Reverse,
                             1010U + neutralMs - 1U, neutralMs) ==
                DriveDirection::Stopped &&
            interlock.update(DriveDirection::Reverse, 1010U + neutralMs,
                             neutralMs) == DriveDirection::Reverse,
        "an explicit stop must start the same host-tested neutral interval");
}

void testProductionOpenDirectionProbeUsesCurrentForwardSpace() {
  const autonomy::Config open =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);

  const auto startForwardSpaceProbe = [](Controller& controller,
                                         Inputs& inputs, bool clockwise,
                                         uint8_t openStatus) {
    inputs = healthyInputs(0);
    inputs.front.rangeMm = 1240;
    const uint8_t closedFront = clockwise ? FrontLeft : FrontRight;
    const uint8_t openFront = clockwise ? FrontRight : FrontLeft;
    inputs.sides[closedFront].valid = true;
    inputs.sides[closedFront].rangeMm = 595;
    inputs.sides[closedFront].rangeStatus = 0;
    inputs.sides[openFront].valid = false;
    inputs.sides[openFront].rangeMm = 0;
    inputs.sides[openFront].rangeStatus = openStatus;
    inputs.sides[BackLeft].rangeMm = 120;
    inputs.sides[BackRight].rangeMm = 1100;

    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart,
          "the 1240 mm mirrored forward-space fixture must arm");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    decision = updateFresh(controller, inputs, 90, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone,
          "the mirrored forward-space fixture must start without guessing");
    return decision;
  };

  // R16's physical clockwise run began with FL near 595 mm and a fresh FR
  // status-2 no-target. While approaching the wall FL rose to 734 mm, so the
  // retired same-ray detector chose left. Current symmetric FL/FR space must
  // instead choose the open FR (right/CW); mirror the trace for left/CCW.
  for (const bool clockwise : {true, false}) {
    Controller controller(open);
    Inputs inputs;
    startForwardSpaceProbe(controller, inputs, clockwise, 2);
    const uint8_t closedFront = clockwise ? FrontLeft : FrontRight;
    const CourseDirection expectedDirection =
        clockwise ? CourseDirection::Clockwise
                  : CourseDirection::CounterClockwise;
    const int16_t expectedSteering =
        clockwise ? static_cast<int16_t>(open.cornerSteeringPermille)
                  : -static_cast<int16_t>(open.cornerSteeringPermille);

    inputs.front.rangeMm = 1001;
    inputs.sides[closedFront].rangeMm = 615;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 150, 0.0F);
    Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "fresh mirrored space above the exact 1000 mm front gate must not pre-confirm direction");

    inputs.front.rangeMm = 1000;
    inputs.sides[closedFront].rangeMm = 643;
    inputs.sides[BackLeft].rangeMm = 120;
    inputs.sides[BackRight].rangeMm = 1100;
    decision = updateFresh(controller, inputs, 201, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.motionAllowed && decision.speedPermille == 0,
          "the first eligible FL/FR/front acquisition must stop without skipping confirmation");

    inputs.front.rangeMm = 956;
    inputs.sides[closedFront].rangeMm = 673;
    inputs.sides[BackLeft].rangeMm = 1100;
    inputs.sides[BackRight].rangeMm = 120;
    decision = updateFresh(controller, inputs, 251, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.speedPermille == 0,
          "two distinct mirrored acquisitions must not satisfy the three-sample contract");

    // Advance controller time to the exact 100 ms boundary without a new FL,
    // FR, or center-front acquisition. Rear, IMU, and vision updates cannot be
    // counted as another direction sample.
    inputs.nowMs = 301;
    inputs.imu.updatedAtMs = 301;
    inputs.sides[BackLeft].updatedAtMs = 301;
    inputs.sides[BackRight].updatedAtMs = 301;
    inputs.vision.updatedAtMs = 301;
    decision = controller.update(inputs);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.speedPermille == 0,
          "held FL/FR/front timestamps must not confirm on controller loops alone");

    inputs.front.rangeMm = 638;
    inputs.sides[closedFront].rangeMm = 734;
    inputs.front.updatedAtMs = 301;
    inputs.sides[FrontLeft].updatedAtMs = 301;
    inputs.sides[FrontRight].updatedAtMs = 301;
    decision = controller.update(inputs);
    const float retiredWrongSideRise =
        clockwise ? decision.leftOpeningMm : decision.rightOpeningMm;
    check(decision.state == RunState::Cornering &&
              decision.direction == expectedDirection &&
              decision.motionAllowed && decision.speedPermille == 0 &&
              decision.steeringPermille == expectedSteering &&
              decision.cornerCount == 0 &&
              std::fabs(retiredWrongSideRise - 139.0F) < 0.01F &&
              decision.faultFlags == autonomy::FaultNone,
          "the third 100 ms sample must follow current open space despite the opposite +139 mm rise");

    decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.speedPermille > 0 &&
              decision.speedPermille <= open.cornerSpeedPermille &&
              decision.steeringPermille == expectedSteering,
          "the first fresh tick after a same-timestamp direction commit must drive while retaining full corner steering");
    decision = updateFresh(controller, inputs, 401, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.speedPermille <= open.cornerSpeedPermille &&
              decision.steeringPermille == expectedSteering,
          "confirmed Open corner motion must not regain a steering-only delay");
    decision = updateFresh(controller, inputs, 600, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille == open.cornerSpeedPermille &&
              decision.steeringPermille == expectedSteering &&
              decision.faultFlags == autonomy::FaultNone,
          "a confirmed Open corner at 638 mm must reach the proven 450 command instead of remaining front-capped at 200");

    inputs.front.rangeMm = open.frontEmergencyStopMm;
    decision = updateFresh(controller, inputs, 610, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTooClose) != 0 &&
              !decision.motionAllowed && decision.speedPermille == 0 &&
              decision.steeringPermille == 0,
          "bypassing the proportional Open-corner envelope must retain the hard 180 mm stop");
  }

  // Every member of the synchronized direction sample is authoritative. Hold
  // FL, FR, then center-front in turn while controller time and all other
  // sources advance. Releasing the held member yields only sample two; a third
  // complete acquisition is still required.
  for (const uint8_t heldReading :
       {static_cast<uint8_t>(FrontLeft), static_cast<uint8_t>(FrontRight),
        static_cast<uint8_t>(4U)}) {
    Controller controller(open);
    Inputs inputs;
    startForwardSpaceProbe(controller, inputs, true, 2);
    inputs.front.rangeMm = 1000;
    Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "the held-timestamp fixture must begin with one eligible sample");

    for (const uint32_t nowMs : {250U, 300U}) {
      inputs.nowMs = nowMs;
      inputs.imu.updatedAtMs = nowMs;
      inputs.front.updatedAtMs = heldReading == 4U ? 200U : nowMs;
      for (auto& side : inputs.sides) side.updatedAtMs = nowMs;
      if (heldReading != 4U) inputs.sides[heldReading].updatedAtMs = 200U;
      inputs.vision.updatedAtMs = nowMs;
      decision = controller.update(inputs);
      check(decision.state == RunState::DirectionProbe &&
                decision.direction == CourseDirection::Unknown,
            "holding any FL/FR/front timestamp must prevent a new direction sample");
    }

    decision = updateFresh(controller, inputs, 301, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "releasing one held timestamp must produce sample two, not confirmation");
    decision = updateFresh(controller, inputs, 302, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == CourseDirection::Clockwise &&
              decision.motionAllowed && decision.speedPermille > 0,
          "a third complete synchronized acquisition may confirm after the 100 ms span");
  }

  // Weak optical outcomes are unknown, not open space, even if every sample is
  // fresh. Exercise both directions because swapping the invalid ray must not
  // accidentally swap a weak status into a turn authorization.
  for (const uint8_t weakStatus : {1U, 7U}) {
    for (const bool clockwise : {true, false}) {
      Controller controller(open);
      Inputs inputs;
      startForwardSpaceProbe(controller, inputs, clockwise, weakStatus);
      inputs.front.rangeMm = 1000;
      updateFresh(controller, inputs, 200, 0.0F);
      updateFresh(controller, inputs, 250, 0.0F);
      updateFresh(controller, inputs, 300, 0.0F);
      const Decision decision = updateFresh(controller, inputs, 350, 0.0F);
      check(decision.state == RunState::DirectionProbe &&
                decision.direction == CourseDirection::Unknown &&
                decision.motionAllowed &&
                decision.faultFlags == autonomy::FaultNone,
            "fresh side status 1/7 must remain non-authorizing current-space evidence");
    }
  }

  // Two simultaneous strong no-target rays are symmetric and therefore carry
  // no direction. Numeric current-space contrast, however, is a valid mirrored
  // winner without relying on an old same-ray baseline.
  {
    Controller controller(open);
    Inputs inputs;
    startForwardSpaceProbe(controller, inputs, true, 2);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeMm = 0;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.front.rangeMm = 1000;
    updateFresh(controller, inputs, 200, 0.0F);
    updateFresh(controller, inputs, 250, 0.0F);
    updateFresh(controller, inputs, 300, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 350, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone,
          "two fresh strong open rays must remain directionally ambiguous");
  }

  for (const bool clockwise : {true, false}) {
    Controller controller(open);
    Inputs inputs;
    startForwardSpaceProbe(controller, inputs, clockwise, 2);
    const uint8_t fartherFront = clockwise ? FrontRight : FrontLeft;
    const uint8_t nearerFront = clockwise ? FrontLeft : FrontRight;
    inputs.sides[fartherFront].valid = true;
    inputs.sides[fartherFront].rangeStatus = 0;
    inputs.sides[fartherFront].rangeMm = 695;
    inputs.sides[nearerFront].valid = true;
    inputs.sides[nearerFront].rangeStatus = 0;
    inputs.sides[nearerFront].rangeMm = 595;
    inputs.front.rangeMm = 1000;
    updateFresh(controller, inputs, 200, 0.0F);
    updateFresh(controller, inputs, 250, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 300, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction ==
                  (clockwise ? CourseDirection::Clockwise
                             : CourseDirection::CounterClockwise) &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.speedPermille <= open.cornerSpeedPermille &&
              decision.faultFlags == autonomy::FaultNone,
          "a mirrored 100 mm numeric FL/FR contrast must confirm from current space");
  }
}

void testR19FirstDirectionApproachAndStoppedRetry() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  check(config.firstDirectionApproachHoldEnabled &&
            !autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle)
                 .firstDirectionApproachHoldEnabled &&
            !autonomy::makeGuardedObstacleConfig()
                 .firstDirectionApproachHoldEnabled,
        "R19 approach holding must be enabled only for production Open");

  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  setStraightGeometry(inputs, 1201);
  Decision decision = updateFresh(controller, inputs, 500, 0.0F);
  check(decision.speedPermille == 600 &&
            decision.state == RunState::DirectionProbe,
        "R19 must retain proven 600 power before the first 1000 mm approach window");
  inputs.front.rangeMm = 1000;
  decision = updateFresh(controller, inputs, 501, 0.0F);
  check(decision.speedPermille > 0 && decision.speedPermille <= 400 &&
            decision.faultFlags == autonomy::FaultNone,
        "R19 must immediately cap first approach at 400 without changing corner torque");
  inputs.front.rangeMm = 600;
  decision = updateFresh(controller, inputs, 600, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown &&
            decision.speedPermille == 0 &&
            decision.faultFlags == autonomy::FaultNone,
        "R19 must hold at 600 instead of latching before direction samples can settle");
  inputs.front.rangeMm = 528;
  decision = updateFresh(controller, inputs, 700, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.speedPermille == 0 &&
            decision.faultFlags == autonomy::FaultNone,
        "R19 must remain stopped during the observed coast from 600 to 528 mm");
  decision = updateFresh(controller, inputs, 3099, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.speedPermille == 0,
        "R19 stopped retry must remain recoverable one millisecond before 2500 ms");
  decision = updateFresh(controller, inputs, 3100, 0.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultDirectionAmbiguous) != 0 &&
            decision.speedPermille == 0 && !decision.motionAllowed,
        "R19 must retain a bounded 2500 ms ambiguous stop instead of blind motion");

  // A fresh direction below the mechanical 600 mm turn bound cannot bypass it.
  for (const bool clockwise : {true, false}) {
    Controller held(config);
    Inputs heldInputs;
    startController(held, heldInputs);
    const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
    const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
    heldInputs.front.rangeMm = 600;
    heldInputs.sides[openRay].valid = false;
    heldInputs.sides[openRay].rangeStatus = 2;
    heldInputs.sides[closedRay].rangeMm = 730;
    updateFresh(held, heldInputs, 100, 0.0F);
    updateFresh(held, heldInputs, 150, 0.0F);
    decision = updateFresh(held, heldInputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.speedPermille == 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "confirmed mirrored side contrast cannot authorize a turn at 600 mm");
    heldInputs.front.rangeMm = 700;
    decision = updateFresh(held, heldInputs, 210, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == (clockwise ? CourseDirection::Clockwise
                                              : CourseDirection::CounterClockwise) &&
              decision.faultFlags == autonomy::FaultNone,
          "fresh regained front clearance must release a stopped, confirmed mirrored direction");
    heldInputs.front.rangeMm = config.frontEmergencyStopMm;
    decision = updateFresh(held, heldInputs, 220, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTooClose) != 0 &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R19 first-turn recovery must preserve the hard 180 mm emergency stop");
  }
}

Decision prepareR20BackoffApproach(Controller& controller, Inputs& inputs,
                                   bool clockwise, bool recordApproach = true) {
  startController(controller, inputs);
  inputs.driveFeedbackAvailable = true;
  // The field's far numeric reading arrived during servo settle, before the
  // actual bridge began driving. It may anchor a later real approach but may
  // not itself contribute any powered travel time.
  inputs.appliedDrivePermille = 0;
  inputs.front.rangeMm = 1200;
  updateFresh(controller, inputs, 100, 0.0F);
  inputs.front.rangeMm = 1001;
  inputs.appliedDrivePermille = recordApproach ? 600 : 0;
  for (uint32_t nowMs = 200; nowMs <= 800; nowMs += 100) {
    updateFresh(controller, inputs, nowMs, 0.0F);
  }
  const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
  const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
  inputs.front.rangeMm = 514;
  inputs.sides[closedRay].rangeMm = 718;
  inputs.sides[openRay].valid = false;
  inputs.sides[openRay].rangeStatus = 2;
  for (const uint8_t rear : {BackLeft, BackRight}) {
    inputs.sides[rear].valid = false;
    inputs.sides[rear].rangeStatus = 2;
  }
  inputs.appliedDrivePermille = recordApproach ? 400 : 0;
  updateFresh(controller, inputs, 900, 0.0F);
  inputs.appliedDrivePermille = 0;
  updateFresh(controller, inputs, 950, 0.0F);
  return updateFresh(controller, inputs, 1000, 0.0F);
}

void testR20FirstCornerBackoffRestoresMeasuredTurnRoom() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  for (const bool clockwise : {true, false}) {
    Controller controller(config);
    Inputs inputs;
    Decision decision = prepareR20BackoffApproach(controller, inputs, clockwise);
    uint32_t reverseStartedAt = 0;
    uint32_t reverseStoppedAt = 0;
    bool resumedCorner = false;
    uint16_t frontMm = 514;
    for (uint32_t nowMs = 1050; nowMs <= 3500; nowMs += 50) {
      inputs.appliedDrivePermille = decision.speedPermille;
      if (decision.driveDirection == DriveDirection::Reverse) {
        if (frontMm < 900) frontMm = static_cast<uint16_t>(frontMm + 50);
        if (frontMm > 900) frontMm = 900;
      }
      inputs.front.rangeMm = frontMm;
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      if (decision.driveDirection == DriveDirection::Reverse &&
          decision.speedPermille != 0) {
        if (reverseStartedAt == 0) reverseStartedAt = nowMs;
        check(nowMs >= 1250 && decision.speedPermille == 300 &&
                  decision.steeringPermille == 0 &&
                  decision.cornerCount == 0,
              "R20 must use actual neutral before a straight 300-power backoff without corner credit");
      } else if (reverseStartedAt != 0 && reverseStoppedAt == 0 &&
                 decision.speedPermille == 0) {
        reverseStoppedAt = nowMs;
        check(inputs.front.rangeMm >= 900,
              "R20 successful backoff must stop on regained numeric 900 mm front clearance");
      }
      if (inputs.front.rangeMm >= 900 &&
          decision.state != RunState::Cornering) {
        check(decision.speedPermille == 0,
              "R20 must immediately stop reverse at 900 mm and hold the forward neutral interval");
      }
      if (decision.state == RunState::Cornering &&
          decision.driveDirection == DriveDirection::Forward &&
          decision.speedPermille > 0) {
        check(reverseStoppedAt != 0 && nowMs >= reverseStoppedAt + 300 &&
                  decision.direction == (clockwise ? CourseDirection::Clockwise
                                                  : CourseDirection::CounterClockwise) &&
                  decision.steeringPermille ==
                      (clockwise ? config.cornerSteeringPermille
                                 : -config.cornerSteeringPermille) &&
                  decision.cornerCount == 0 &&
                  decision.faultFlags == autonomy::FaultNone,
              "R20 must resume the carried mirrored first corner only after fresh clearance and actual forward neutral");
        resumedCorner = true;
        break;
      }
    }
    check(reverseStartedAt != 0 && reverseStoppedAt != 0 && resumedCorner,
          "R20 observed 514/718/optical opening must back away and resume forward instead of remaining stuck");
  }
}

void testR20FirstCornerBackoffRejectsUnprovedRetreat() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  enum class Rejection {
    NoPoweredHistory, NoFeedback, NearRear, WeakRear1, WeakRear6, WeakRear7,
    RearI2c, RearStale, InvalidFront, OppositeCandidate, HeadingError
  };
  for (const bool clockwise : {true, false}) {
    for (const Rejection rejection : {
             Rejection::NoPoweredHistory, Rejection::NoFeedback,
             Rejection::NearRear, Rejection::WeakRear1, Rejection::WeakRear6,
             Rejection::WeakRear7, Rejection::RearI2c, Rejection::RearStale,
             Rejection::InvalidFront, Rejection::OppositeCandidate,
             Rejection::HeadingError}) {
      Controller controller(config);
      Inputs inputs;
      Decision decision = prepareR20BackoffApproach(
          controller, inputs, clockwise,
          rejection != Rejection::NoPoweredHistory);
      const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
      const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
      float yaw = 0.0F;
      if (rejection == Rejection::NoFeedback) inputs.driveFeedbackAvailable = false;
      if (rejection == Rejection::NearRear) {
        inputs.sides[BackLeft].valid = true;
        inputs.sides[BackLeft].rangeStatus = 0;
        inputs.sides[BackLeft].rangeMm = 350;
      }
      if (rejection == Rejection::WeakRear1) inputs.sides[BackLeft].rangeStatus = 1;
      if (rejection == Rejection::WeakRear6) inputs.sides[BackLeft].rangeStatus = 6;
      if (rejection == Rejection::WeakRear7) inputs.sides[BackLeft].rangeStatus = 7;
      if (rejection == Rejection::RearI2c) inputs.sides[BackLeft].i2cStatus = 1;
      if (rejection == Rejection::InvalidFront) {
        inputs.front.valid = false;
        inputs.front.rangeStatus = 7;
        inputs.front.rangeMm = 0;
      }
      if (rejection == Rejection::OppositeCandidate) {
        inputs.sides[closedRay].valid = false;
        inputs.sides[closedRay].rangeStatus = 2;
        inputs.sides[openRay].valid = true;
        inputs.sides[openRay].rangeStatus = 0;
        inputs.sides[openRay].rangeMm = 718;
      }
      if (rejection == Rejection::HeadingError) yaw = clockwise ? 8.1F : -8.1F;
      for (uint32_t nowMs = 1050; nowMs <= 2000; nowMs += 50) {
        inputs.appliedDrivePermille = 0;
        inputs.nowMs = nowMs;
        inputs.imu.updatedAtMs = nowMs;
        inputs.imu.yawDegrees = yaw;
        inputs.front.updatedAtMs = nowMs;
        for (auto& side : inputs.sides) side.updatedAtMs = nowMs;
        inputs.vision.updatedAtMs = nowMs;
        if (rejection == Rejection::RearStale) {
          inputs.sides[BackLeft].updatedAtMs = 700;
        }
        decision = controller.update(inputs);
        if (decision.speedPermille != 0) {
          std::cerr << "R20 rejection=" << static_cast<int>(rejection)
                    << " cw=" << clockwise << " at=" << nowMs << '\n';
        }
        check(decision.speedPermille == 0 &&
                  decision.state != RunState::Cornering &&
                  decision.cornerCount == 0,
              "R20 cannot retreat or start a late corner without recent path history and current sensor/heading/feedback proof");
      }
    }
  }
}

void testR20FirstCornerBackoffStopsForRearAndReverseBudget() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  for (const bool rearBecomesClose : {true, false}) {
    Controller controller(config);
    Inputs inputs;
    Decision decision = prepareR20BackoffApproach(controller, inputs, true);
    uint32_t firstReverseAt = 0;
    uint32_t poweredReverseMs = 0;
    bool stoppedAfterReverse = false;
    for (uint32_t nowMs = 1050; nowMs <= 4500; nowMs += 50) {
      inputs.appliedDrivePermille = decision.speedPermille;
      if (decision.driveDirection == DriveDirection::Reverse &&
          inputs.appliedDrivePermille != 0) poweredReverseMs += 50;
      if (rearBecomesClose && firstReverseAt != 0) {
        inputs.sides[BackRight].valid = true;
        inputs.sides[BackRight].rangeStatus = 0;
        inputs.sides[BackRight].rangeMm = 350;
      }
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      if (decision.driveDirection == DriveDirection::Reverse &&
          decision.speedPermille != 0 && firstReverseAt == 0) firstReverseAt = nowMs;
      if (firstReverseAt != 0 && decision.speedPermille == 0) {
        check(poweredReverseMs <= 1500 && decision.cornerCount == 0,
              "R20 must stop a failed reverse before exceeding 1500 ms of powered retreat");
        if (rearBecomesClose) {
          check(nowMs == firstReverseAt + 50,
                "R20 a rear range at 350 mm must stop reverse on the same fresh update");
        }
        stoppedAfterReverse = true;
        inputs.appliedDrivePermille = 0;
        inputs.sides[BackRight].valid = false;
        inputs.sides[BackRight].rangeStatus = 2;
        for (const uint32_t later : {nowMs + 100U, nowMs + 500U}) {
          const Decision afterStop = updateFresh(controller, inputs, later, 0.0F);
          check(afterStop.speedPermille == 0 &&
                    afterStop.state != RunState::Cornering,
                "R20 a failed bounded backoff must not restart another blind retreat when rear data clears");
        }
        break;
      }
    }
    check(firstReverseAt != 0 && stoppedAfterReverse,
          "R20 failed retreat must end in a stopped state rather than spin indefinitely");
  }
}

void testR20BackoffWallDeadlineAndDistanceCap() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  for (const bool excessRetreat : {true, false}) {
    Controller controller(config);
    Inputs inputs;
    Decision decision = prepareR20BackoffApproach(controller, inputs, true);
    uint32_t firstReverseAt = 0;
    bool stopped = false;
    for (uint32_t nowMs = 1050; nowMs <= 6500; nowMs += 50) {
      // An unresponsive drivetrain cannot consume a powered timer; the
      // independent total recovery deadline must still end the attempt.
      inputs.appliedDrivePermille = 0;
      if (excessRetreat && firstReverseAt != 0) inputs.front.rangeMm = 1015;
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      if (decision.driveDirection == DriveDirection::Reverse &&
          decision.speedPermille > 0 && firstReverseAt == 0) firstReverseAt = nowMs;
      if (firstReverseAt != 0 && decision.speedPermille == 0) {
        check(decision.state != RunState::Cornering && decision.cornerCount == 0,
              "R20 exceeded retreat or recovery deadline must not become a forward turn");
        if (excessRetreat) {
          check(nowMs == firstReverseAt + 50,
                "R20 more than500 mm measured retreat must stop in the same update");
        } else {
          check(nowMs <= 6000,
                "R20 motor feedback remaining zero must not bypass the five-second recovery deadline");
        }
        stopped = true;
        break;
      }
    }
    check(firstReverseAt != 0 && stopped,
          "R20 must independently bound measured retreat and total recovery wall time");
  }
}

void testR20ApproachOpticalGapRetainsAnchorWithoutTravelCredit() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  for (const bool enoughNumericTravel : {true, false}) {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    inputs.front.rangeMm = 1200;
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.appliedDrivePermille = 600;
    inputs.front.rangeMm = 1001;
    for (const uint32_t nowMs : {200U, 300U, 400U}) {
      updateFresh(controller, inputs, nowMs, 0.0F);
    }
    // This long optical gap must neither erase the still-young far anchor nor
    // count as powered travel. At least one numeric side remains observable.
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    inputs.front.rangeMm = 0;
    for (uint32_t nowMs = 450; nowMs <= 1050; nowMs += 100) {
      updateFresh(controller, inputs, nowMs, 0.0F);
    }
    inputs.front.valid = enoughNumericTravel;
    inputs.front.rangeStatus = enoughNumericTravel ? 0 : 2;
    inputs.front.rangeMm = enoughNumericTravel ? 1001 : 0;
    for (uint32_t nowMs = 1100; nowMs <= 1500; nowMs += 100) {
      updateFresh(controller, inputs, nowMs, 0.0F);
    }
    inputs.appliedDrivePermille = 0;
    inputs.front.valid = true;
    inputs.front.rangeStatus = 0;
    inputs.front.rangeMm = 514;
    inputs.sides[FrontLeft].rangeMm = 718;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    for (const uint8_t rear : {BackLeft, BackRight}) {
      inputs.sides[rear].valid = false;
      inputs.sides[rear].rangeStatus = 2;
    }
    Decision decision;
    bool reverseSeen = false;
    for (uint32_t nowMs = 1600; nowMs <= 2500; nowMs += 50) {
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      reverseSeen = reverseSeen ||
          (decision.driveDirection == DriveDirection::Reverse &&
           decision.speedPermille > 0);
      if (!enoughNumericTravel) {
        check(decision.speedPermille == 0 &&
                  decision.state != RunState::Cornering,
              "R20 optical-gap duration must not manufacture the600 ms actual numeric-path history");
      }
    }
    check(reverseSeen == enoughNumericTravel,
          "R20 retains a far anchor through optical gaps but only real numeric powered intervals authorize backoff");
  }
}

void testR20AllBlindCasesRemainNonRecoverable() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.front.rangeMm = 1200;
    inputs.appliedDrivePermille = 600;
    for (uint32_t nowMs = 100; nowMs <= 800; nowMs += 100) {
      updateFresh(controller, inputs, nowMs, 0.0F);
    }
    // Latest field pattern: front has an accepted wrapped499 mm range but
    // every side is low signal. Previous path history does not choose a turn.
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.front.rangeMm = 499;
    inputs.appliedDrivePermille = 0;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    Decision decision;
    for (uint32_t nowMs = 900; nowMs <= 3500; nowMs += 50) {
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      check(decision.speedPermille == 0 &&
                decision.direction == CourseDirection::Unknown &&
                decision.cornerCount == 0,
            "R20 wrapped499 mm front with four low-signal sides must never invent reverse or turn direction");
    }
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultDirectionAmbiguous) != 0,
          "R20 all-side-blind decision must retain the bounded0x10000 stop");
  }
  {
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    inputs.front.rangeMm = 0;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    controller.update(inputs);
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    updateFresh(controller, inputs, 90, 0.0F);
    Decision decision;
    for (uint32_t nowMs = 100; nowMs <= config.straightFrontOpticalGraceMs + 200U;
         nowMs += 50) {
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      check(decision.direction == CourseDirection::Unknown &&
                decision.driveDirection != DriveDirection::Reverse &&
                decision.state != RunState::Cornering,
            "R20 five blind sensors from Start cannot authorize backoff or a first corner");
    }
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTofUnavailable) != 0 &&
              decision.speedPermille == 0,
          "R20 fully blind front/side sensing retains the existing0x4 optical-grace fault");
  }
}

Decision startR21BlindProbe(Controller& controller, Inputs& inputs) {
  inputs = healthyInputs(0);
  inputs.driveFeedbackAvailable = true;
  inputs.appliedDrivePermille = 0;
  inputs.front.valid = false;
  inputs.front.rangeStatus = 2;
  inputs.front.rangeMm = 0;
  for (auto& side : inputs.sides) {
    side.valid = false;
    side.rangeStatus = 2;
    side.rangeMm = 0;
  }
  Decision decision = controller.update(inputs);
  check(decision.state == RunState::WaitingForStart,
        "R21 classified fresh optical blindness may wait for the physical Start button");
  updateFresh(controller, inputs, 40, 0.0F);
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, 50, 0.0F);
  decision = updateFresh(controller, inputs, 90, 0.0F);
  inputs.buttonPressed = false;
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "R21 blind Start must enter an unknown-direction bounded probe");
  return decision;
}

Decision exhaustR21BlindProbe(Controller& controller, Inputs& inputs) {
  Decision decision = startR21BlindProbe(controller, inputs);
  for (uint32_t nowMs = 100; nowMs <= 800; nowMs += 100) {
    decision = updateFresh(controller, inputs, nowMs, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0 && decision.speedPermille <= 350,
          "R21 must not spend its actual-motion budget during700 ms of zero-output servo settle");
  }
  // main supplies the drive applied during the preceding loop interval.
  // First positive feedback at900 therefore charges800..900, after the full
  //700 ms of zero-feedback startup settling has already elapsed.
  inputs.appliedDrivePermille = 350;
  for (uint32_t nowMs = 900; nowMs <= 2250; nowMs += 50) {
    decision = updateFresh(controller, inputs, nowMs, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0 && decision.speedPermille <= 350 &&
              decision.direction == CourseDirection::Unknown &&
              decision.driveDirection != DriveDirection::Reverse,
          "R21 blind creep must stay bounded at350 with no invented direction or reverse");
  }
  decision = updateFresh(controller, inputs, 2299, 0.0F);
  check(decision.speedPermille > 0 && decision.speedPermille <= 350,
        "R21 actual blind-probe budget must remain available at1499 ms");
  decision = updateFresh(controller, inputs, 2300, 0.0F);
  check(decision.speedPermille == 0 &&
            decision.state != RunState::FaultLatched &&
            (decision.faultFlags & ~(FaultFrontTofUnavailable | FaultDirectionAmbiguous)) == 0 &&
            decision.direction == CourseDirection::Unknown,
        "R21 must enter a stopped nonlatched hold at1500 ms of actual blind motion");
  inputs.appliedDrivePermille = 0;
  return decision;
}

void testR21AlternatingNumericFramesCannotEvadeBlindMotionBudget() {
  Controller controller(autonomy::makeOpenFallbackConfig());
  Inputs inputs;
  Decision decision = startR21BlindProbe(controller, inputs);
  for (uint32_t nowMs = 100; nowMs <= 800; nowMs += 100) {
    decision = updateFresh(controller, inputs, nowMs, 0.0F);
  }
  uint32_t actualBlindMs = 0;
  uint32_t heldAt = 0;
  bool previousFrontBlind = true;
  for (uint32_t nowMs = 810; nowMs <= 4800; nowMs += 10) {
    // Applied feedback belongs to the prior command interval. Numeric frames
    // alternate with blind frames and never supply three useful observations
    // in a row; no isolated blind interval is allowed to disappear from time.
    inputs.appliedDrivePermille = decision.speedPermille > 0 ? 350 : 0;
    if (previousFrontBlind && decision.speedPermille > 0 &&
        decision.driveDirection == DriveDirection::Forward &&
        inputs.appliedDrivePermille > 0) {
      actualBlindMs += 10;
    }
    const bool numeric = ((nowMs - 810U) / 10U) % 2U == 0U;
    inputs.front.valid = numeric;
    inputs.front.rangeStatus = numeric ? 0 : 2;
    inputs.front.rangeMm = numeric ? 1200 : 0;
    decision = updateFresh(controller, inputs, nowMs, 0.0F);
    previousFrontBlind = !numeric;
    check(actualBlindMs <= 1500 &&
              decision.state != RunState::FaultLatched &&
              decision.direction == CourseDirection::Unknown &&
              decision.driveDirection != DriveDirection::Reverse,
          "R21 alternating isolated numeric/blind frames must still charge every actual blind command interval");
    if (decision.speedPermille == 0 && heldAt == 0) {
      heldAt = nowMs;
      check(actualBlindMs == 1500,
            "R21 flickering numeric returns cannot renew the1500 ms blind budget without full useful-observation proof");
    }
    if (heldAt != 0) {
      check(decision.speedPermille == 0,
            "R21 isolated numeric frames after exhausted blind budget must never automatically restart movement");
    }
  }
  check(heldAt != 0,
        "R21 alternating numeric/blind frames must eventually stop instead of granting indefinite powered probing");
}

void testR21OpticalFallbackIsSeparateAndReacquiresBothDirections() {
  const autonomy::Config config = autonomy::makeOpenFallbackConfig();
  check(config.openOpticalRecoveryEnabled &&
            !autonomy::Config{}.openOpticalRecoveryEnabled &&
            !autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open)
                 .openOpticalRecoveryEnabled &&
            !autonomy::makeGuardedObstacleConfig().openOpticalRecoveryEnabled &&
            std::strcmp(autonomy::openFallbackSourceIdentity(),
                "BUBBLEGUM AUTONOMOUS OPEN 20260905-R21 OPTICAL FALLBACK - MOTION CAPABLE") == 0,
        "R21 optical fallback must have its own factory and identity without enabling it in R20 or R42");
  for (const bool clockwise : {true, false}) {
    Controller controller(config);
    Inputs inputs;
    Decision decision = exhaustR21BlindProbe(controller, inputs);
    for (const uint32_t nowMs : {3000U, 5000U, 13000U, 20000U}) {
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      check(decision.speedPermille == 0 &&
                decision.state != RunState::FaultLatched &&
                (decision.faultFlags & ~(FaultFrontTofUnavailable | FaultDirectionAmbiguous)) == 0 &&
                decision.direction == CourseDirection::Unknown,
            "R21 persistent all-five optical blindness must neither time-restart motion nor guess a direction");
    }
    inputs.front.valid = true;
    inputs.front.rangeStatus = 0;
    inputs.front.rangeMm = 1200;
    decision = updateFresh(controller, inputs, 20100, 0.0F);
    check(decision.speedPermille == 0,
          "R21 first useful numeric observation must not release an optical hold");
    decision = updateFresh(controller, inputs, 20150, 0.0F);
    check(decision.speedPermille == 0,
          "R21 second useful observation must not bypass the three-sample contract");
    decision = updateFresh(controller, inputs, 20249, 0.0F);
    check(decision.speedPermille == 0,
          "R21 three useful samples spanning149 ms must remain stopped");
    inputs.nowMs = 20250;
    inputs.imu.updatedAtMs = 20250;
    inputs.vision.updatedAtMs = 20250;
    decision = controller.update(inputs);
    check(decision.speedPermille == 0,
          "R21 controller-time advancement without a fresh range receipt must not complete recovery proof");
    decision = updateFresh(controller, inputs, 20251, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "R21 useful observations spanning150 ms must restore normal forward probing");
    const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
    const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
    inputs.front.rangeMm = 900;
    inputs.sides[closedRay].valid = true;
    inputs.sides[closedRay].rangeStatus = 0;
    inputs.sides[closedRay].rangeMm = 718;
    inputs.sides[openRay].valid = false;
    inputs.sides[openRay].rangeStatus = 2;
    updateFresh(controller, inputs, 20300, 0.0F);
    updateFresh(controller, inputs, 20350, 0.0F);
    decision = updateFresh(controller, inputs, 20400, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == (clockwise ? CourseDirection::Clockwise
                                              : CourseDirection::CounterClockwise) &&
              decision.faultFlags == autonomy::FaultNone,
          "R21 recovery must retain normal fresh three-sample mirrored first-corner selection");
  }
}

void testR21CloseThenBlindCannotCreepAndAbsoluteRunDeadlineRemains() {
  const autonomy::Config config = autonomy::makeOpenFallbackConfig();
  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    inputs.front.rangeMm = 550;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.speedPermille == 0 &&
              decision.state != RunState::FaultLatched,
          "R21 close550 mm front with unknown direction must stop before optical fallback");
    for (const uint32_t nowMs : {1000U, 3000U}) {
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      check(decision.speedPermille == 0 &&
                decision.direction == CourseDirection::Unknown &&
                decision.state != RunState::FaultLatched,
            "R21 numeric close-wall ambiguity must remain a recoverable stop beyond the old2500 ms latch");
    }
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    inputs.front.rangeMm = 0;
    for (const uint32_t nowMs : {3050U, 3400U, 5000U, 13000U}) {
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      check(decision.speedPermille == 0 &&
                decision.direction == CourseDirection::Unknown &&
                decision.state != RunState::FaultLatched,
            "R21 recent close-wall evidence must block blind creep and reverse despite elapsed hold time");
    }
  }
  {
    Controller controller(config);
    Inputs inputs;
    exhaustR21BlindProbe(controller, inputs);
    Decision decision = updateFresh(controller, inputs, 90U + config.maxRunMs - 1U, 0.0F);
    check(decision.speedPermille == 0 && decision.state != RunState::FaultLatched,
          "R21 optical hold must remain stopped until the absolute run limit");
    decision = updateFresh(controller, inputs, 90U + config.maxRunMs, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultRunTimeout) != 0 &&
              decision.speedPermille == 0,
          "R21 optical waiting must never pause the170-second absolute run timeout");
  }
}

void testR21OpticalHoldPreservesPendingCornerAndPausesOnlyItsClock() {
  autonomy::Config config = autonomy::makeOpenFallbackConfig();
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  for (const bool clockwise : {true, false}) {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 400;
    const float sign = clockwise ? 1.0F : -1.0F;
    const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
    const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
    inputs.front.rangeMm = 900;
    inputs.sides[closedRay].rangeMm = 718;
    inputs.sides[openRay].valid = false;
    inputs.sides[openRay].rangeStatus = 2;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 150, 0.0F);
    Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::Cornering,
          "R21 corner optical fixture must begin a confirmed first corner");
    updateFresh(controller, inputs, 400, sign * 30.0F);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    inputs.front.rangeMm = 0;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    uint32_t heldAt = 0;
    float heldYaw = 0;
    for (uint32_t nowMs = 450; nowMs <= 2300; nowMs += 50) {
      inputs.appliedDrivePermille = 350;
      heldYaw = sign * (35.0F + static_cast<float>(nowMs - 450U) * 0.02F);
      decision = updateFresh(controller, inputs, nowMs, heldYaw);
      if (decision.speedPermille == 0 && decision.state == RunState::Cornering) {
        heldAt = nowMs;
        break;
      }
    }
    check(heldAt != 0 && decision.cornerCount == 0,
          "R21 blind midcorner must exhaust its bounded probe without inventing an exit");
    if (heldAt == 0) continue;
    inputs.appliedDrivePermille = 0;
    for (const uint32_t nowMs : {heldAt + 50U, heldAt + 500U, heldAt + 6000U}) {
      decision = updateFresh(controller, inputs, nowMs, heldYaw);
      check(decision.state == RunState::Cornering &&
                decision.speedPermille == 0 && decision.cornerCount == 0 &&
                decision.cornerTargetHeadingDegrees == sign * 90.0F &&
                (decision.faultFlags & FaultCornerTimeout) == 0,
            "R21 stopped optical hold must preserve direction and cardinal target while pausing the corner timeout");
    }
    setStraightGeometry(inputs, 1200);
    updateFresh(controller, inputs, heldAt + 6100U, heldYaw);
    updateFresh(controller, inputs, heldAt + 6150U, heldYaw);
    decision = updateFresh(controller, inputs, heldAt + 6250U, heldYaw);
    check(decision.state == RunState::Cornering && decision.speedPermille > 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "R21 numeric optical recovery must resume its pending corner after a long stopped hold");
    updateFresh(controller, inputs, heldAt + 6300U, sign * 80.0F);
    decision = updateFresh(controller, inputs, heldAt + 6350U, sign * 90.0F);
    check(decision.state == RunState::Running && decision.cornerCount == 1 &&
              decision.faultFlags == autonomy::FaultNone,
          "R21 only a real cardinal exit after optical recovery may count the corner");
  }
}

void testR21DirectionTimeoutReacquiresFreshCandidate() {
  const autonomy::Config config = autonomy::makeOpenFallbackConfig();
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  inputs.driveFeedbackAvailable = true;
  inputs.appliedDrivePermille = 600;
  inputs.front.rangeMm = 1200;
  Decision decision;
  for (uint32_t nowMs = 100; nowMs < 90U + config.directionTimeoutMs; nowMs += 100) {
    decision = updateFresh(controller, inputs, nowMs, 0.0F);
  }
  const uint32_t deadline = 90U + config.directionTimeoutMs;
  decision = updateFresh(controller, inputs, deadline, 0.0F);
  check(decision.state != RunState::FaultLatched &&
            decision.speedPermille == 0 &&
            decision.direction == CourseDirection::Unknown,
        "R21 an expired first-direction timer must become a stopped recoverable hold");
  inputs.appliedDrivePermille = 0;
  inputs.front.rangeMm = 900;
  inputs.sides[FrontLeft].rangeMm = 718;
  inputs.sides[FrontRight].valid = false;
  inputs.sides[FrontRight].rangeStatus = 2;
  for (uint32_t nowMs = deadline + 50U; nowMs <= deadline + 600U; nowMs += 50) {
    decision = updateFresh(controller, inputs, nowMs, 0.0F);
    if (decision.state == RunState::Cornering) break;
  }
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::Clockwise &&
            decision.faultFlags == autonomy::FaultNone,
        "R21 fresh confirmed direction after a timeout hold must reach a corner instead of immediately re-entering the expired timer hold");
}

void testR21OpticalHoldNeverMasksHardFaults() {
  const autonomy::Config config = autonomy::makeOpenFallbackConfig();
  for (const uint8_t failure : {0U, 1U, 2U, 3U, 4U, 5U, 6U}) {
    Controller controller(config);
    Inputs inputs;
    exhaustR21BlindProbe(controller, inputs);
    uint32_t expectedFault = FaultFrontTofUnavailable;
    if (failure == 0U) {
      inputs.front.valid = true;
      inputs.front.rangeStatus = 0;
      inputs.front.rangeMm = 180;
      expectedFault = FaultFrontTooClose;
    } else if (failure == 1U) {
      inputs.front.rangeStatus = 3;
      expectedFault = FaultFrontTooClose;
    } else if (failure == 2U) {
      inputs.front.rangeStatus = 0;
    } else if (failure == 3U) {
      inputs.front.i2cStatus = 1;
    } else if (failure == 4U) {
      expectedFault = FaultFrontTofStale;
    } else if (failure == 5U) {
      inputs.imu.online = false;
      expectedFault = autonomy::FaultImuUnavailable;
    } else {
      inputs.sides[FrontLeft].i2cStatus = 1;
      expectedFault = FaultSideTofUnavailable;
    }
    Decision decision;
    for (const uint32_t nowMs : {2500U, 3000U, 3500U}) {
      inputs.nowMs = nowMs;
      inputs.imu.updatedAtMs = nowMs;
      inputs.front.updatedAtMs = failure == 4U ? 100U : nowMs;
      for (auto& side : inputs.sides) side.updatedAtMs = nowMs;
      inputs.vision.updatedAtMs = nowMs;
      decision = controller.update(inputs);
      check(decision.speedPermille == 0 &&
                decision.driveDirection != DriveDirection::Reverse,
            "R21 hard faults must never receive optical motion permission");
    }
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & expectedFault) != 0,
          "R21 optical hold must preserve near-front, unclassified, electrical, stale and IMU fault latches");
  }
}

void testObstacleBackupProfileContractsAndGuidedOpticalCorners() {
  const autonomy::Config normal = autonomy::makeGuardedObstacleConfig();
  const autonomy::Config recoverable = autonomy::makeObstacleFallbackConfig(false);
  const autonomy::Config progress = autonomy::makeObstacleFallbackConfig(true);
  check(normal.obstacleRecoveryPolicy == 0U && normal.requireVisionForMotion &&
            recoverable.obstacleRecoveryPolicy == 1U &&
            progress.obstacleRecoveryPolicy == 2U &&
            recoverable.requireVisionForMotion && !progress.requireVisionForMotion &&
            recoverable.cruiseSpeedPermille == normal.cruiseSpeedPermille &&
            progress.cruiseSpeedPermille == normal.cruiseSpeedPermille &&
            !normal.rememberObstacleSections && !recoverable.rememberObstacleSections &&
            !progress.rememberObstacleSections && !normal.parkingEnabled &&
            !recoverable.parkingEnabled && !progress.parkingEnabled &&
            normal.cruiseSpeedPermille == 400 && recoverable.cornerSpeedPermille == 450 &&
            progress.cornerSpeedPermille == 450 && recoverable.minimumMovingSpeedPermille == 300 &&
            progress.minimumMovingSpeedPermille == 300 &&
            std::strcmp(autonomy::obstacleFallbackSourceIdentity(false),
                "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R43 RECOVERABLE - PARKING DISABLED") == 0 &&
            std::strcmp(autonomy::obstacleFallbackSourceIdentity(true),
                "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R44 PROGRESS PRIORITY - PARKING DISABLED") == 0,
        "R43/R44 must remain separately selectable profiles while normal R42 and its motor settings are preserved");
  for (const bool progressPriority : {false, true}) {
    for (const bool clockwise : {true, false}) {
      autonomy::Config config = autonomy::makeObstacleFallbackConfig(progressPriority);
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs);
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 450;
      const float sign = clockwise ? 1.0F : -1.0F;
      const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
      const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
      inputs.front.rangeMm = 900;
      inputs.sides[closedRay].rangeMm = 718;
      inputs.sides[openRay].valid = false;
      inputs.sides[openRay].rangeStatus = 2;
      updateFresh(controller, inputs, 100, 0.0F);
      updateFresh(controller, inputs, 150, 0.0F);
      Decision decision = updateFresh(controller, inputs, 200, 0.0F);
      check(decision.state == RunState::Cornering,
            "obstacle backup optical fixture must first confirm its real mirrored corner");
      inputs.front.valid = false;
      inputs.front.rangeStatus = 2;
      inputs.front.rangeMm = 0;
      inputs.sides[closedRay].rangeMm = 400;
      const uint32_t times[] = {300, 700, 1200, 1700, 2200, 2700, 3000};
      const float headings[] = {5, 20, 40, 60, 75, 82, 85};
      for (size_t i = 0; i < sizeof(times) / sizeof(times[0]); ++i) {
        decision = updateFresh(controller, inputs, times[i], sign * headings[i]);
      }
      check(decision.state == RunState::Cornering &&
                decision.speedPermille == 450 &&
                decision.cornerCount == 0 &&
                decision.faultFlags == autonomy::FaultNone,
            "R43/R44 a fresh numeric side, camera and IMU must preserve the450 corner drive beyond old2500 ms front-optical grace");
      updateFresh(controller, inputs, 3100, sign * 88.0F);
      decision = updateFresh(controller, inputs, 3300, sign * 90.0F);
      check(decision.state == RunState::Running && decision.cornerCount == 1 &&
                decision.lapCount == 0 &&
                decision.faultFlags == autonomy::FaultNone,
            "R43/R44 guided optical corners must count only the stable mirrored cardinal exit");
    }
  }
}

void testR45FreshOpticalOpenKeepsMovingTurnsAndRecovers() {
  for (const CourseDirection direction : {CourseDirection::Clockwise,
                                           CourseDirection::CounterClockwise}) {
    autonomy::Config config = autonomy::makeObstacleContinueConfig(direction);
    check(config.obstacleRecoveryPolicy == 3U &&
              config.obstacleFallbackDirection == direction &&
              !config.requireVisionForMotion &&
              config.straightFrontOpticalGraceMs == config.maxRunMs &&
              config.cornerFrontOpticalGraceMs == config.maxRunMs,
          "R45 must be a direction-selected optical-open continuation profile");
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 1500U;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeMm = 0U;
      side.rangeStatus = 2U;
      side.i2cStatus = 0U;
    }

    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone,
          "R45 fresh status2 on every uncovered side ray must reach WAIT_BUTTON");
    updateFresh(controller, inputs, 40U, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50U, 0.0F);
    decision = updateFresh(controller, inputs, 90U, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone,
          "R45 status2 sides must start without a hand-created numeric D reading");

    inputs.front.valid = false;
    inputs.front.rangeMm = 0U;
    inputs.front.rangeStatus = 2U;
    decision = updateFresh(controller, inputs, 13000U, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0U,
          "R45 fresh optical-open front and sides must outlive direction and geometry grace without stopping");

    inputs.front.valid = true;
    inputs.front.rangeMm = 800U;
    inputs.front.rangeStatus = 0U;
    decision = updateFresh(controller, inputs, 13100U, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == direction &&
              decision.speedPermille > 0U &&
              (direction == CourseDirection::Clockwise
                   ? decision.steeringPermille > 0
                   : decision.steeringPermille < 0),
          "R45 a measured front wall must immediately start the announced-direction turn without side geometry");
  }

  for (const CourseDirection direction : {CourseDirection::Clockwise,
                                           CourseDirection::CounterClockwise}) {
    Controller controller(autonomy::makeObstacleContinueConfig(direction));
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 1500U;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeMm = 0U;
      side.rangeStatus = 2U;
      side.i2cStatus = 0U;
    }
    controller.update(inputs);
    updateFresh(controller, inputs, 40U, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50U, 0.0F);
    updateFresh(controller, inputs, 90U, 0.0F);
    uint32_t nowMs = 100U;
    float yaw = 0.0F;
    const float yawSign = direction == CourseDirection::Clockwise ? 1.0F : -1.0F;
    Decision decision;
    for (uint8_t corner = 0U; corner < 12U; ++corner) {
      if (corner > 0U) {
        inputs.front.rangeMm = 1500U;
        nowMs += 260U;
        updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
        nowMs += 110U;
        updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      }
      inputs.front.rangeMm = 800U;
      nowMs += 20U;
      decision = updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      check(decision.state == RunState::Cornering &&
                decision.cornerCount == corner,
            "R45 must start every front-only corner with all four side rays status2");
      inputs.front.rangeMm = 1500U;
      yaw += yawSign * 40.0F;
      nowMs += 200U;
      updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      yaw += yawSign * 40.0F;
      nowMs += 200U;
      updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      yaw += yawSign * 10.0F;
      nowMs += 100U;
      updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      nowMs += 210U;
      decision = updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      check(decision.cornerCount == static_cast<uint8_t>(corner + 1U) &&
                decision.faultFlags == autonomy::FaultNone,
            "R45 IMU corner completion must count through fresh optical-open side samples");
    }
    check(decision.state == RunState::FinishDelay &&
              decision.cornerCount == 12U && decision.lapCount == 3U,
          "R45 must reach the three-lap finish after twelve front-plus-IMU corners");
    nowMs += controller.config().finishDriveMs;
    updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
    nowMs += controller.config().finishStableMs;
    decision = updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
    check(decision.state == RunState::Finished &&
              decision.speedPermille == 0U && !decision.motionAllowed,
          "R45 must stop permanently after the third lap");
  }

  // A real measured close object keeps the reversal behavior. Optical-open
  // rear rays allow the bounded backout to complete and forward motion to
  // resume instead of ending in FAULT_LATCHED or a phase-3 hold.
  autonomy::Config config =
      autonomy::makeObstacleContinueConfig(CourseDirection::Clockwise);
  Controller controller(config);
  Inputs inputs = healthyInputs(0);
  inputs.front.rangeMm = 1500U;
  for (auto& side : inputs.sides) {
    side.valid = false;
    side.rangeMm = 0U;
    side.rangeStatus = 2U;
  }
  controller.update(inputs);
  updateFresh(controller, inputs, 40U, 0.0F);
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, 50U, 0.0F);
  updateFresh(controller, inputs, 90U, 0.0F);
  inputs.front.rangeMm = 170U;
  Decision decision = updateFresh(controller, inputs, 200U, 0.0F);
  check(decision.state != RunState::FaultLatched &&
            decision.wallRecoveryPhase == 1U &&
            decision.speedPermille == 0U,
        "R45 measured close front must enter neutral recovery rather than latch");
  updateFresh(controller, inputs, 510U, 0.0F);
  decision = updateFresh(controller, inputs, 820U, 0.0F);
  check(decision.state != RunState::FaultLatched &&
            decision.wallRecoveryPhase == 2U &&
            decision.driveDirection == DriveDirection::Reverse &&
            decision.speedPermille > 0U,
        "R45 optical-open rear rays must authorize the bounded reverse");
  inputs.front.rangeMm = 700U;
  updateFresh(controller, inputs, 1200U, 0.0F);
  updateFresh(controller, inputs, 1510U, 0.0F);
  decision = updateFresh(controller, inputs, 1610U, 0.0F);
  check(decision.state != RunState::FaultLatched &&
            decision.wallRecoveryPhase == 0U &&
            decision.speedPermille > 0U,
        "R45 must leave reverse recovery and command forward progress again");
  check(std::strcmp(
            autonomy::obstacleContinueSourceIdentity(CourseDirection::Clockwise),
            "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R45 CONTINUE CW - PARKING DISABLED") == 0 &&
            std::strcmp(autonomy::obstacleContinueSourceIdentity(
                            CourseDirection::CounterClockwise),
                        "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R45 CONTINUE CCW - PARKING DISABLED") == 0,
        "R45 clockwise and counter-clockwise binaries must have distinct logger identities");
}

void testObstacleBackupWeakOrUnguidedOpticalCannotTurnBlind() {
  for (const bool progressPriority : {false, true}) {
    for (const uint8_t frontStatus : {1U, 2U, 6U, 7U}) {
      Controller controller(autonomy::makeObstacleFallbackConfig(progressPriority));
      Inputs inputs;
      startController(controller, inputs);
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 450;
      inputs.front.rangeMm = 900;
      inputs.sides[FrontLeft].rangeMm = 718;
      inputs.sides[FrontRight].valid = false;
      inputs.sides[FrontRight].rangeStatus = 2;
      updateFresh(controller, inputs, 100, 0.0F);
      updateFresh(controller, inputs, 150, 0.0F);
      updateFresh(controller, inputs, 200, 0.0F);
      inputs.front.valid = false;
      inputs.front.rangeStatus = frontStatus;
      inputs.front.rangeMm = 0;
      if (frontStatus == 2U) {
        for (auto& side : inputs.sides) {
          side.valid = false;
          side.rangeStatus = 2;
        }
      }
      inputs.appliedDrivePermille = 0;
      Decision decision;
      for (const uint32_t nowMs : {300U, 800U, 3000U}) {
        decision = updateFresh(controller, inputs, nowMs, 20.0F);
      }
      check(decision.speedPermille == 0 && decision.cornerCount == 0 &&
                decision.driveDirection != DriveDirection::Reverse,
            "R43/R44 weak front1/6/7 or entirely blind sides cannot authorize guided corner motion");
    }
  }
}

Decision beginObstacleBackupCorner(Controller& controller, Inputs& inputs,
                                    bool clockwise) {
  startController(controller, inputs);
  inputs.driveFeedbackAvailable = true;
  inputs.appliedDrivePermille = 400;
  inputs.front.rangeMm = 900;
  const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
  const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
  inputs.sides[closedRay].rangeMm = 718;
  inputs.sides[openRay].valid = false;
  inputs.sides[openRay].rangeStatus = 2;
  updateFresh(controller, inputs, 100, 0.0F);
  updateFresh(controller, inputs, 150, 0.0F);
  return updateFresh(controller, inputs, 200, 0.0F);
}

void testObstacleBackupStoppedHealthHoldsPreserveCornerAndHeading() {
  for (const bool progressPriority : {false, true}) {
    for (const bool clockwise : {true, false}) {
      for (const bool cameraLoss : {false, true}) {
        if (progressPriority && cameraLoss) continue;  // R44 camera loss has a measured-route policy.
        Controller controller(autonomy::makeObstacleFallbackConfig(progressPriority));
        Inputs inputs;
        Decision decision = beginObstacleBackupCorner(controller, inputs, clockwise);
        check(decision.state == RunState::Cornering,
              "obstacle backup health-hold fixture must enter a confirmed corner");
        const float sign = clockwise ? 1.0F : -1.0F;
        setStraightGeometry(inputs, 1200);
        updateFresh(controller, inputs, 250, sign * 20.0F);
        inputs.appliedDrivePermille = 0;
        if (cameraLoss) {
          inputs.vision.online = false;
          inputs.vision.valid = false;
        } else {
          inputs.front.valid = false;
          inputs.front.rangeStatus = 7;
          inputs.front.rangeMm = 0;
        }
        decision = updateFresh(controller, inputs, 300, sign * 20.0F);
        // Existing weak-optical grace is unchanged. Measure paused time only
        // after that grace has actually stopped the controller.
        const uint32_t heldAt = cameraLoss ? 300U : 2800U;
        if (!cameraLoss) decision = updateFresh(controller, inputs, heldAt, sign * 20.0F);
        check(decision.speedPermille == 0 && decision.cornerCount == 0,
              "R43/R44 required-observation health holds must stop the pending corner");
        updateFresh(controller, inputs, heldAt + 400U, sign * 40.0F);
        decision = updateFresh(controller, inputs, heldAt + 5700U, sign * 65.0F);
        check(decision.state == RunState::Cornering &&
                  decision.speedPermille == 0 && decision.cornerCount == 0 &&
                  decision.lapCount == 0 &&
                  decision.cornerTargetHeadingDegrees == sign * 90.0F &&
                  (decision.faultFlags & (FaultCornerTimeout | FaultHeadingJump)) == 0,
              "R43/R44 actual-stopped health time must not expire the corner or discard fresh20-to40-to65 degree IMU progress");
        inputs.vision.online = true;
        inputs.vision.valid = true;
        inputs.front.valid = true;
        inputs.front.rangeStatus = 0;
        inputs.front.rangeMm = 1200;
        const uint32_t restoredAt = heldAt + 5800U;
        decision = updateFresh(controller, inputs, restoredAt, sign * 65.0F);
        check(decision.state == RunState::Cornering &&
                  decision.speedPermille > 0 && decision.cornerCount == 0 &&
                  decision.faultFlags == autonomy::FaultNone,
              "R43/R44 fully healthy data must resume the same corner without a false heading jump");
        updateFresh(controller, inputs, restoredAt + 50U, sign * 88.0F);
        updateFresh(controller, inputs, restoredAt + 250U, sign * 90.0F);
        decision = updateFresh(controller, inputs, restoredAt + 450U, sign * 90.0F);
        check(decision.state == RunState::Running && decision.cornerCount == 1 &&
                  decision.lapCount == 0 && decision.faultFlags == autonomy::FaultNone,
              "R43/R44 only fresh stable cardinal exit evidence after health recovery may count the corner");
      }
    }
  }
}

void testObstacleBackupDirectionHoldAndAbsoluteDeadline() {
  for (const bool progressPriority : {false, true}) {
    Controller controller(autonomy::makeObstacleFallbackConfig(progressPriority));
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    inputs.front.rangeMm = 550;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    decision = updateFresh(controller, inputs, 13000, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0 &&
              decision.direction == CourseDirection::Unknown &&
              decision.cornerCount == 0,
          "R43/R44 numeric close-wall ambiguity must remain stopped and recoverable beyond old direction deadlines");
    inputs.front.rangeMm = 900;
    inputs.sides[FrontLeft].rangeMm = 718;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    updateFresh(controller, inputs, 13100, 0.0F);
    updateFresh(controller, inputs, 13150, 0.0F);
    decision = updateFresh(controller, inputs, 13249, 0.0F);
    check(decision.speedPermille == 0 && decision.state == RunState::DirectionProbe,
          "R43/R44 timeout rearm may not skip150 ms of fresh numeric route proof");
    decision = updateFresh(controller, inputs, 13250, 0.0F);
    check(decision.state == RunState::DirectionProbe && decision.faultFlags == autonomy::FaultNone,
          "R43/R44 timeout route proof must release the hold without bypassing normal direction confirmation");
    updateFresh(controller, inputs, 13300, 0.0F);
    decision = updateFresh(controller, inputs, 13350, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == CourseDirection::Clockwise &&
              decision.cornerCount == 0 && decision.faultFlags == autonomy::FaultNone,
          "R43/R44 fresh confirmed direction and restored clearance must resume a held first corner");

    Controller timed(autonomy::makeObstacleFallbackConfig(progressPriority));
    Inputs timedInputs;
    startController(timed, timedInputs);
    timedInputs.driveFeedbackAvailable = true;
    timedInputs.appliedDrivePermille = 0;
    timedInputs.front.rangeMm = 550;
    updateFresh(timed, timedInputs, 100, 0.0F);
    decision = updateFresh(timed, timedInputs, 90U + timed.config().maxRunMs, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultRunTimeout) != 0 &&
              decision.speedPermille == 0,
          "R43/R44 stopped holds must never extend the absolute170-second run deadline");
  }
}

void testR44CameraIndependentRouteSkipsOnlyTheStaleTarget() {
  for (const bool redPillar : {true, false}) {
    Controller controller(autonomy::makeObstacleFallbackConfig(true));
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    setStraightGeometry(inputs, 1500);
    for (auto& side : inputs.sides) side.rangeMm = 600;
    auto& oldPillar = redPillar ? inputs.vision.red : inputs.vision.green;
    oldPillar = usableBlob(0, 10000, 3000, 900);
    Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.obstacle == (redPillar ? ObstacleTarget::RedPassRight
                                         : ObstacleTarget::GreenPassLeft),
          "R44 route fixture must first commit a real detected pillar");
    inputs.vision.online = false;
    inputs.vision.valid = false;
    decision = updateFresh(controller, inputs, 210, 0.0F);
    check(decision.speedPermille == 0,
          "R44 camera loss must first collect measured route proof instead of immediately dropping a target");
    updateFresh(controller, inputs, 260, 0.0F);
    decision = updateFresh(controller, inputs, 359, 0.0F);
    check(decision.speedPermille == 0,
          "R44 camera-independent target release must respect its150 ms proof span");
    decision = updateFresh(controller, inputs, 360, 0.0F);
    decision = updateFresh(controller, inputs, 370, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::None &&
              decision.speedPermille > 0 && decision.speedPermille <= 400 &&
              decision.cornerCount == 0 && decision.lapCount == 0 &&
              decision.learnedRedSections == 0 && decision.learnedGreenSections == 0,
          "R44 a verified current range route may release stale color steering and move without awarding pass or lap credit");
    inputs.vision.online = true;
    inputs.vision.valid = true;
    inputs.vision.red = {};
    inputs.vision.green = {};
    auto& newPillar = redPillar ? inputs.vision.green : inputs.vision.red;
    newPillar = usableBlob(0, 10000, 3000, 900);
    decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.obstacle == (redPillar ? ObstacleTarget::GreenPassLeft
                                         : ObstacleTarget::RedPassRight) &&
              (redPillar ? decision.steeringPermille < 0
                         : decision.steeringPermille > 0) &&
              decision.cornerCount == 0 && decision.lapCount == 0,
          "R44 skipping the old color must never suppress a newly visible opposite pillar");
  }
  for (const uint8_t rejection : {0U, 1U, 2U, 3U}) {
    Controller controller(autonomy::makeObstacleFallbackConfig(true));
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    inputs.front.rangeMm = rejection == 0U ? 350 : 1500;
    for (auto& side : inputs.sides) side.rangeMm = 600;
    inputs.vision.online = false;
    inputs.vision.valid = false;
    if (rejection == 1U) inputs.sides[FrontLeft].rangeMm = 200;
    if (rejection == 2U) {
      inputs.sides[FrontLeft].valid = false;
      inputs.sides[FrontLeft].rangeStatus = 2;
      inputs.sides[FrontRight].valid = false;
      inputs.sides[FrontRight].rangeStatus = 2;
    }
    if (rejection == 3U) inputs.sides[BackRight].rangeMm = 200;
    Decision decision;
    for (const uint32_t nowMs : {100U, 200U, 400U}) {
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      check(decision.speedPermille == 0 && decision.cornerCount == 0,
            "R44 optional camera must not authorize motion with insufficient numeric front, leading or rear route clearance");
    }
  }
}

void testObstacleBackupSoftCornerHoldCannotBeOverwrittenBySideEscape() {
  for (const bool progressPriority : {false, true}) {
    for (const bool clockwise : {false, true}) {
      Controller controller(autonomy::makeObstacleFallbackConfig(progressPriority));
      Inputs inputs;
      beginObstacleBackupCorner(controller, inputs, clockwise);
      inputs.front.rangeMm = 1000;
      for (auto& side : inputs.sides) {
        side.valid = true;
        side.rangeStatus = 0;
        side.rangeMm = 600;
      }
      inputs.sides[clockwise ? FrontLeft : FrontRight].rangeMm = 185;
      for (const uint32_t nowMs : {5200U, 5250U}) {
        const Decision decision = updateFresh(controller, inputs, nowMs, 0.0F);
        check(decision.state == RunState::Cornering &&
                  decision.speedPermille == 0 &&
                  decision.driveDirection == DriveDirection::Stopped &&
                  ((decision.faultFlags & FaultCornerTimeout) != 0 ||
                   (nowMs > 5200U && decision.wallRecoveryPhase == 1U)) &&
                  decision.cornerTargetHeadingDegrees == (clockwise ? 90.0F : -90.0F) &&
                  decision.cornerCount == 0,
              "R43/R44 forward side escape cannot overwrite the stop; a measured side hazard may begin neutral guarded recovery");
      }
    }
  }
}

void testR44CornerTimeoutPermitsOnlyOneMeasuredRetry() {
  for (const bool progressPriority : {false, true}) {
    Controller controller(autonomy::makeObstacleFallbackConfig(progressPriority));
    Inputs inputs;
    beginObstacleBackupCorner(controller, inputs, true);
    setStraightGeometry(inputs, 1000);
    for (auto& side : inputs.sides) side.rangeMm = 600;
    Decision decision = updateFresh(controller, inputs, 5200, 0.0F);
    check(decision.state == RunState::Cornering && decision.speedPermille == 0 &&
              (decision.faultFlags & FaultCornerTimeout) != 0 &&
              decision.cornerTargetHeadingDegrees == 90.0F,
          "obstacle backup corner timeout must preserve the target in a zero-output hold");
    inputs.appliedDrivePermille = 0;
    updateFresh(controller, inputs, 5300, 0.0F);
    updateFresh(controller, inputs, 5350, 0.0F);
    updateFresh(controller, inputs, 5450, 0.0F);
    decision = updateFresh(controller, inputs, 5500, 0.0F);
    if (!progressPriority) {
      check(decision.speedPermille == 0 && (decision.faultFlags & FaultCornerTimeout) != 0,
            "R43 unchanged corner geometry cannot earn a timeout retry");
      continue;
    }
    check(decision.state == RunState::Cornering && decision.speedPermille > 0 &&
              decision.cornerTargetHeadingDegrees == 90.0F && decision.cornerCount == 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "R44 one fresh measured route proof may retry the same unfinished corner without awarding a count");
    inputs.appliedDrivePermille = 450;
    decision = updateFresh(controller, inputs, 10500, 0.0F);
    check(decision.speedPermille == 0 && (decision.faultFlags & FaultCornerTimeout) != 0,
          "R44 the one permitted corner retry remains bounded by its active timeout");
    inputs.appliedDrivePermille = 0;
    for (const uint32_t nowMs : {10600U, 10700U, 10900U}) {
      decision = updateFresh(controller, inputs, nowMs, 0.0F);
      check(decision.speedPermille == 0 &&
                (decision.faultFlags & FaultCornerTimeout) != 0 &&
                decision.cornerCount == 0 && decision.cornerTargetHeadingDegrees == 90.0F,
            "R44 a second timeout cannot create another retry or a false completed corner");
    }
  }
}

void testR44RouteProofMustRestartAfterCameraOrElectricalGap() {
  for (const bool electricalGap : {false, true}) {
    Controller controller(autonomy::makeObstacleFallbackConfig(true));
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    inputs.front.rangeMm = 1500;
    for (auto& side : inputs.sides) side.rangeMm = 600;
    inputs.vision.online = false;
    inputs.vision.valid = false;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 150, 0.0F);
    Decision decision;
    if (electricalGap) {
      inputs.front.i2cStatus = 1;
      decision = updateFresh(controller, inputs, 200, 0.0F);
      check(decision.speedPermille == 0,
            "R44 a real front I2C gap must stop optional-camera route acquisition");
      inputs.front.i2cStatus = 0;
      decision = updateFresh(controller, inputs, 250, 0.0F);
      check(decision.speedPermille == 0,
            "R44 two route samples before an electrical gap cannot combine with one recovered sample");
      updateFresh(controller, inputs, 300, 0.0F);
      updateFresh(controller, inputs, 400, 0.0F);
      decision = updateFresh(controller, inputs, 410, 0.0F);
      check(decision.speedPermille > 0 && decision.speedPermille <= 400,
            "R44 fresh route proof after electrical recovery must eventually resume measured forward progress");
    } else {
      updateFresh(controller, inputs, 250, 0.0F);
      inputs.vision.online = true;
      inputs.vision.valid = true;
      updateFresh(controller, inputs, 260, 0.0F);
      inputs.vision.online = false;
      inputs.vision.valid = false;
      inputs.nowMs = 270;
      inputs.imu.updatedAtMs = 270;
      inputs.vision.updatedAtMs = 270;
      decision = controller.update(inputs);
      check(decision.speedPermille == 0,
            "R44 a new camera outage must not reuse route proof from the preceding outage");
      inputs.nowMs = 350;
      inputs.imu.updatedAtMs = 350;
      inputs.vision.updatedAtMs = 350;
      decision = controller.update(inputs);
      check(decision.speedPermille == 0,
            "R44 cached range timestamps cannot advance camera-outage route proof");
    }
  }
  for (const bool blindFront : {false, true}) {
    Controller controller(autonomy::makeObstacleFallbackConfig(true));
    Inputs inputs = healthyInputs(0);
    inputs.driveFeedbackAvailable = true;
    inputs.front.rangeMm = 1500;
    for (auto& side : inputs.sides) side.rangeMm = 600;
    inputs.vision.online = false;
    inputs.vision.valid = false;
    if (blindFront) {
      inputs.front.valid = false;
      inputs.front.rangeStatus = 2;
      inputs.front.rangeMm = 0;
    }
    controller.update(inputs);
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 90, 0.0F);
    check(decision.speedPermille == 0,
          "R44 accepting Start without camera must not bypass the measured-route proof gate");
  }
}

void testObstacleBackupHardFaultsRemainMotionBlocking() {
  for (const bool progressPriority : {false, true}) {
    for (const uint8_t failure : {0U, 1U, 2U, 3U, 4U}) {
      Controller controller(autonomy::makeObstacleFallbackConfig(progressPriority));
      Inputs inputs;
      startController(controller, inputs);
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      uint32_t expectedFault = FaultFrontTofUnavailable;
      if (failure == 0U) inputs.front.i2cStatus = 1;
      if (failure == 1U) {
        inputs.imu.online = false;
        expectedFault = autonomy::FaultImuUnavailable;
      }
      if (failure == 2U) {
        inputs.sides[FrontLeft].i2cStatus = 1;
        expectedFault = FaultSideTofUnavailable;
      }
      if (failure == 3U) {
        inputs.front.valid = false;
        inputs.front.rangeStatus = 3;
        inputs.front.rangeMm = 0;
        expectedFault = FaultFrontTooClose;
      }
      if (failure == 4U) expectedFault = FaultImuStale;
      Decision decision;
      for (const uint32_t nowMs : {500U, 1000U, 2000U}) {
        inputs.nowMs = nowMs;
        inputs.imu.updatedAtMs = failure == 4U ? 0U : nowMs;
        inputs.front.updatedAtMs = nowMs;
        for (auto& side : inputs.sides) side.updatedAtMs = nowMs;
        inputs.vision.updatedAtMs = nowMs;
        decision = controller.update(inputs);
        check(decision.speedPermille == 0,
              "R43/R44 I2C, IMU or actual too-close faults must never become motion permission");
      }
      check((failure != 3U || decision.state == RunState::FaultLatched) &&
                (decision.faultFlags & expectedFault) != 0 && decision.speedPermille == 0,
            "R43/R44 electrical and stale IMU faults must remain motion-blocking; sensor-too-close retains its hard latch");
    }
  }
}

void testR19FirstDirectionRejectsNearbyStart() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  Controller controller(config);
  Inputs inputs = healthyInputs(0);
  inputs.front.rangeMm = 267;
  inputs.sides[FrontLeft].rangeMm = 48;
  Decision decision = controller.update(inputs);
  check(decision.state == RunState::WaitingForHealth &&
            (decision.faultFlags & FaultDirectionAmbiguous) != 0 &&
            !decision.startLatched && !decision.motionAllowed,
        "the logged finger-test pose must fail readiness before accepting Start");
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, 40, 0.0F);
  decision = updateFresh(controller, inputs, 90, 0.0F);
  check(decision.state == RunState::WaitingForHealth &&
            !decision.startLatched && decision.speedPermille == 0,
        "pressing Start with front 267 mm must not create an immediately faulted moving run");
  inputs.front.rangeMm = 650;
  inputs.sides[FrontLeft].rangeMm = 300;
  decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::WaitingForHealth &&
            (decision.faultFlags & FaultDirectionAmbiguous) != 0,
        "nearby-start readiness must retain its 650 mm hysteresis boundary");
  inputs.front.rangeMm = 651;
  decision = updateFresh(controller, inputs, 110, 0.0F);
  check(decision.state == RunState::WaitingForStart &&
            !decision.startLatched && decision.speedPermille == 0 &&
            decision.faultFlags == autonomy::FaultNone,
        "fresh clearance beyond 650 mm may restore readiness without accepting the old held Start");
  inputs.buttonPressed = false;
  updateFresh(controller, inputs, 120, 0.0F);
  updateFresh(controller, inputs, 160, 0.0F);
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, 170, 0.0F);
  decision = updateFresh(controller, inputs, 210, 0.0F);
  check(decision.state == RunState::DirectionProbe && decision.startLatched &&
            decision.faultFlags == autonomy::FaultNone,
        "a new deliberate Start may arm after nearby-start clearance recovers");
}

void testR19FirstDirectionOpticalGapsDoNotEraseFreshProof() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  for (const bool clockwise : {true, false}) {
    for (const uint8_t gapKind : {0U, 1U, 2U, 6U, 7U}) {
      const bool frontGap = gapKind == 0U;
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs);
      const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
      const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
      inputs.front.rangeMm = 930;
      inputs.sides[openRay].valid = false;
      inputs.sides[openRay].rangeStatus = 2;
      inputs.sides[closedRay].rangeMm = 792;
      Decision decision = updateFresh(controller, inputs, 100, 0.0F);
      check(decision.speedPermille == 0 &&
                decision.state == RunState::DirectionProbe,
            "first late field-like side acquisition must stop for confirmation");
      inputs.front.rangeMm = 850;
      inputs.sides[closedRay].rangeMm = 761;
      updateFresh(controller, inputs, 130, 0.0F);
      if (frontGap) {
        inputs.front.valid = false;
        inputs.front.rangeMm = 0;
        inputs.front.rangeStatus = 7;
      } else {
        inputs.sides[closedRay].valid = false;
        inputs.sides[closedRay].rangeStatus = gapKind;
      }
      decision = updateFresh(controller, inputs, 160, 0.0F);
      check(decision.state == RunState::DirectionProbe &&
                decision.direction == CourseDirection::Unknown &&
                decision.speedPermille == 0 &&
                decision.faultFlags == autonomy::FaultNone,
            "a short optical gap must neither erase the stopped candidate nor authorize cached direction");
      inputs.front.valid = true;
      inputs.front.rangeStatus = 0;
      inputs.front.rangeMm = 700;
      inputs.sides[closedRay].valid = true;
      inputs.sides[closedRay].rangeStatus = 0;
      inputs.sides[closedRay].rangeMm = 747;
      decision = updateFresh(controller, inputs, 205, 0.0F);
      check(decision.state == RunState::Cornering &&
                decision.direction == (clockwise ? CourseDirection::Clockwise
                                                : CourseDirection::CounterClockwise) &&
                decision.faultFlags == autonomy::FaultNone,
            "three actual samples spanning 105 ms must survive one short optical gap in both directions");
    }
  }

  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.rangeMm = 850;
    inputs.sides[FrontLeft].rangeMm = 730;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 130, 0.0F);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.nowMs = 160;
    inputs.imu.updatedAtMs = 160;
    for (auto& side : inputs.sides) side.updatedAtMs = 160;
    inputs.vision.updatedAtMs = 160;
    controller.update(inputs);
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeStatus = 0;
    inputs.nowMs = 205;
    inputs.imu.updatedAtMs = 205;
    for (auto& side : inputs.sides) side.updatedAtMs = 205;
    inputs.vision.updatedAtMs = 205;
    // Only side data has recovered. The center acquisition remains the same
    // as during the gap; it still cannot satisfy a new synchronized sample.
    inputs.front.updatedAtMs = 130;
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.speedPermille == 0,
          "retained optical-gap proof must not count refreshed sides with an old center acquisition");
    decision = updateFresh(controller, inputs, 225, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == CourseDirection::Clockwise,
          "only a third complete fresh acquisition may finish retained optical-gap proof");
  }

  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.rangeMm = 900;
    inputs.sides[FrontLeft].rangeMm = 600;
    inputs.sides[FrontRight].rangeMm = 900;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 140, 0.0F);
    inputs.sides[FrontRight].rangeMm = 600;
    updateFresh(controller, inputs, 160, 0.0F);
    inputs.sides[FrontRight].rangeMm = 900;
    Decision decision = updateFresh(controller, inputs, 220, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "a fresh numeric tie must erase the old direction instead of receiving optical-gap grace");
    updateFresh(controller, inputs, 270, 0.0F);
    decision = updateFresh(controller, inputs, 320, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == CourseDirection::Clockwise,
          "a new full sample set after numeric tie may confirm direction normally");
  }

  // Numeric contradiction carries real evidence; a retained old direction
  // must not complete on one sample of its opposite, or after an expired gap.
  for (const bool opposite : {false, true}) {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.rangeMm = 900;
    inputs.sides[FrontLeft].rangeMm = 600;
    inputs.sides[FrontRight].rangeMm = 900;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 140, 0.0F);
    if (opposite) {
      inputs.sides[FrontLeft].rangeMm = 900;
      inputs.sides[FrontRight].rangeMm = 600;
    } else {
      inputs.sides[FrontLeft].valid = false;
      inputs.sides[FrontLeft].rangeStatus = 2;
      inputs.sides[FrontRight].valid = false;
      inputs.sides[FrontRight].rangeStatus = 2;
    }
    Decision decision = updateFresh(controller, inputs, 261, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "opposite or expired optical evidence must not complete the old direction");
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeStatus = 0;
    inputs.sides[FrontRight].valid = true;
    inputs.sides[FrontRight].rangeStatus = 0;
    decision = updateFresh(controller, inputs, 270, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "new direction after contradiction or expiry must collect its own full sample set");
  }
}

void testR19FirstTurnThenTwelveCornersAndPermanentStop(bool useOpticalBackup = false) {
  for (const bool clockwise : {true, false}) {
    autonomy::Config config =
        useOpticalBackup ? autonomy::makeOpenFallbackConfig()
                         : autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
    // Keep all production first-turn proof, rearm, and finish parameters. Only
    // collapse the physical turn durations, which a software trace cannot prove.
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    const float sign = clockwise ? 1.0F : -1.0F;
    const uint8_t openRay = clockwise ? FrontRight : FrontLeft;
    const uint8_t closedRay = clockwise ? FrontLeft : FrontRight;
    inputs.front.rangeMm = 900;
    inputs.sides[openRay].valid = false;
    inputs.sides[openRay].rangeStatus = 2;
    inputs.sides[closedRay].rangeMm = 700;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 150, 0.0F);
    Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::Cornering && decision.cornerCount == 0,
          "R19 must begin a production-confirmed mirrored first turn");
    uint32_t nowMs = 200;
    float heading = 0.0F;
    for (uint8_t corner = 0; corner < 12; ++corner) {
      if (corner != 0) {
        nowMs += config.cornerRefractoryMs;
        setStraightGeometry(inputs, config.cornerRearmFrontClearMm);
        updateFresh(controller, inputs, nowMs, wrapYaw(heading));
        nowMs += config.cornerRearmStableMs;
        decision = updateFresh(controller, inputs, nowMs, wrapYaw(heading));
        check(decision.cornerDetectionArmed,
              "R19 must retain production clear-front rearming for every later corner");
        nowMs += 10;
        inputs.front.rangeMm = config.frontCornerTriggerMm;
        decision = updateFresh(controller, inputs, nowMs, wrapYaw(heading));
        check(decision.state == RunState::Cornering &&
                  decision.cornerCount == corner,
              "R19 must enter each later corner exactly once");
      }
      nowMs += 50;
      heading += sign * 45.0F;
      updateFresh(controller, inputs, nowMs, wrapYaw(heading));
      nowMs += 50;
      heading = sign * 90.0F * static_cast<float>(corner + 1U);
      setStraightGeometry(inputs, config.cornerRearmFrontClearMm);
      decision = updateFresh(controller, inputs, nowMs, wrapYaw(heading));
      check(decision.cornerCount == corner + 1U &&
                decision.faultFlags == autonomy::FaultNone,
            "R19 must count exactly one verified cardinal per corner through all three laps");
    }
    check(decision.state == RunState::FinishDelay &&
              decision.cornerCount == 12 && decision.lapCount == 3,
          "R19 must reach three laps after twelve verified mirrored exits");
    nowMs += config.finishDriveMs;
    updateFresh(controller, inputs, nowMs, wrapYaw(heading));
    nowMs += config.finishStableMs;
    decision = updateFresh(controller, inputs, nowMs, wrapYaw(heading));
    check(decision.state == RunState::Finished &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R19 must finish with the existing stable stop after three laps");
    inputs.buttonPressed = false;
    updateFresh(controller, inputs, nowMs + 50, wrapYaw(heading));
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, nowMs + 100, wrapYaw(heading));
    decision = updateFresh(controller, inputs, nowMs + 150, wrapYaw(heading));
    check(decision.state == RunState::Finished &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R19 must retain the permanent three-lap stop despite another Start press");
  }
}

void testProductionObstacleDirectionProbeUsesCurrentForwardSpace() {
  const autonomy::Config obstacle =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);

  // Challenge mode cannot change the chassis: FL/FR are the symmetric
  // forward-space pair, while the wider BL/BR pair has a mounting offset.
  // Deliberately reverse the rear bias on every acquisition and require the
  // decision to continue following only the current FL/FR winner.
  for (const bool clockwise : {true, false}) {
    Controller controller(obstacle);
    Inputs inputs;
    startController(controller, inputs);

    const uint8_t fartherFront = clockwise ? FrontRight : FrontLeft;
    const uint8_t nearerFront = clockwise ? FrontLeft : FrontRight;
    const CourseDirection expected =
        clockwise ? CourseDirection::Clockwise
                  : CourseDirection::CounterClockwise;
    inputs.front.rangeMm = 1001;
    inputs.sides[fartherFront].rangeMm = 750;
    inputs.sides[nearerFront].rangeMm = 550;
    inputs.sides[BackLeft].rangeMm = clockwise ? 1100 : 100;
    inputs.sides[BackRight].rangeMm = clockwise ? 100 : 1100;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "Obstacle FL/FR contrast above 1000 mm must not pre-confirm direction");

    inputs.front.rangeMm = 1000;
    inputs.sides[BackLeft].rangeMm = clockwise ? 100 : 1100;
    inputs.sides[BackRight].rangeMm = clockwise ? 1100 : 100;
    decision = updateFresh(controller, inputs, 101, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.speedPermille == 0,
          "Obstacle current-space sample one must stop for confirmation");

    inputs.front.rangeMm = 900;
    inputs.sides[BackLeft].rangeMm = clockwise ? 1100 : 100;
    inputs.sides[BackRight].rangeMm = clockwise ? 100 : 1100;
    decision = updateFresh(controller, inputs, 151, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "Obstacle current-space sample two must not satisfy the three-sample contract");

    inputs.sides[BackLeft].rangeMm = clockwise ? 100 : 1100;
    inputs.sides[BackRight].rangeMm = clockwise ? 1100 : 100;
    decision = updateFresh(controller, inputs, 201, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == expected &&
              decision.steeringPermille ==
                  (clockwise
                       ? static_cast<int16_t>(obstacle.cornerSteeringPermille)
                       : -static_cast<int16_t>(obstacle.cornerSteeringPermille)) &&
              decision.faultFlags == autonomy::FaultNone,
          "Obstacle direction must follow three FL/FR samples despite contradictory BL/BR bias");
  }

  // The lower edge of the decision window is a hard latest-decision bound.
  // A still-ambiguous obstacle probe may approach to 601 mm, but 600 mm must
  // latch safely rather than guessing a turn.
  Controller ambiguousController(obstacle);
  Inputs ambiguousInputs;
  startController(ambiguousController, ambiguousInputs);
  setStraightGeometry(ambiguousInputs, 601);
  Decision decision = updateFresh(ambiguousController, ambiguousInputs,
                                  100, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.faultFlags == autonomy::FaultNone,
        "ambiguous Obstacle direction must remain bounded but nonfaulting at 601 mm");
  ambiguousInputs.front.rangeMm = 600;
  decision = updateFresh(ambiguousController, ambiguousInputs, 101, 0.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultDirectionAmbiguous) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0,
        "ambiguous Obstacle direction must fault without guessing at 600 mm");
}

void testProductionDirectionProbeIsHeadingOnlyAndUsesFrontEnvelope() {
  autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  // Retain the R18 immediate-latch contract as an explicit legacy regression.
  config.firstDirectionApproachHoldEnabled = false;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  setStraightGeometry(inputs, 1200);
  Decision decision = updateFresh(controller, inputs, 500, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.speedPermille == config.directionProbeSpeedPermille &&
            decision.steeringPermille == 0,
        "a clear centered production probe must reach its configured 600 speed");

  inputs.sides[FrontLeft].rangeMm = 400;
  inputs.sides[BackLeft].rangeMm = 400;
  inputs.sides[FrontRight].rangeMm = 200;
  inputs.sides[BackRight].rangeMm = 1100;
  decision = updateFresh(controller, inputs, 510, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.steeringPermille == 0,
        "diagonal side ranges must not steer a zero-error DirectionProbe");

  inputs.sides[FrontLeft].rangeMm = 180;
  inputs.sides[BackLeft].rangeMm = 1150;
  inputs.sides[FrontRight].rangeMm = 850;
  inputs.sides[BackRight].rangeMm = 100;
  decision = updateFresh(controller, inputs, 520, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.steeringPermille == 0,
        "mirrored diagonal side ranges must not create range-derived probe steering");

  decision = updateFresh(controller, inputs, 530, 5.0F);
  const int16_t headingCorrection = decision.steeringPermille;
  check(headingCorrection < 0,
        "positive probe yaw drift must command heading correction toward the start heading");
  inputs.sides[FrontLeft].rangeMm = 850;
  inputs.sides[BackLeft].rangeMm = 150;
  inputs.sides[FrontRight].rangeMm = 180;
  inputs.sides[BackRight].rangeMm = 1150;
  decision = updateFresh(controller, inputs, 540, 5.0F);
  check(decision.steeringPermille == headingCorrection,
        "changing only diagonal ranges must not change DirectionProbe heading steering");

  setStraightGeometry(inputs, 700);
  decision = updateFresh(controller, inputs, 550, 0.0F);
  check(decision.speedPermille >= config.minimumMovingSpeedPermille &&
            decision.speedPermille < config.directionProbeSpeedPermille,
        "a 700 mm probe front range must enter the stopping envelope");
  inputs.front.rangeMm = 601;
  decision = updateFresh(controller, inputs, 560, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.motionAllowed && decision.faultFlags == autonomy::FaultNone,
        "an ambiguous production Open probe must remain nonfaulting one millimeter above abort");
  inputs.front.rangeMm = 600;
  decision = updateFresh(controller, inputs, 570, 0.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultDirectionAmbiguous) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0,
        "an ambiguous production Open probe must fault immediately at 600 mm");

  Controller farController(config);
  Inputs farInputs = healthyInputs(0);
  farInputs.front.rangeMm = 1200;
  farInputs.sides[FrontLeft].rangeMm = 5000;
  farInputs.sides[BackLeft].rangeMm = 5000;
  farInputs.sides[FrontRight].rangeMm = 300;
  farInputs.sides[BackRight].rangeMm = 300;
  decision = farController.update(farInputs);
  check(decision.state == RunState::WaitingForStart,
        "an asymmetric 5 m Open start must remain a legal starting pose");
  updateFresh(farController, farInputs, 40, 0.0F);
  farInputs.buttonPressed = true;
  updateFresh(farController, farInputs, 50, 0.0F);
  decision = updateFresh(farController, farInputs, 90, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.steeringPermille == 0,
        "a 5 m side return must not command max lock toward the far side");
  farInputs.sides[FrontRight].rangeMm = 400;
  farInputs.sides[BackRight].rangeMm = 400;
  decision = updateFresh(farController, farInputs, 500, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.steeringPermille == 0,
        "a newly visible diagonal pair must not add range-derived probe steering");
}

void testProductionOpenKnownStraightIsHeadingOnly() {
  for (const bool clockwise : {true, false}) {
    autonomy::Config config = legacyOpenFixtureConfig();
    config.openingConfirmationMs = 0;
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    config.cornerRefractoryMs = 0;
    config.cornerRearmStableMs = 0;
    config.postCornerHeadingOnlyMs = 0;
    config.postCornerWallBlendMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (clockwise) {
      setRightOpening(inputs);
    } else {
      setLeftOpening(inputs);
    }
    const float yawSign = clockwise ? 1.0F : -1.0F;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 120, 40.0F * yawSign);
    updateFresh(controller, inputs, 130, 80.0F * yawSign);
    setStraightGeometry(inputs, 1200);
    Decision decision = updateFresh(controller, inputs, 140,
                                    90.0F * yawSign);
    check(decision.state == RunState::Running,
          "heading-only fixture must complete its direction-establishing corner");

    // The physical front pair is narrower than the rear pair and all four rays
    // are diagonal. These deliberately contradictory values therefore cannot
    // be interpreted as lateral offset or wall angle.
    inputs.sides[FrontLeft].rangeMm = 850;
    inputs.sides[BackLeft].rangeMm = 100;
    inputs.sides[FrontRight].rangeMm = 180;
    inputs.sides[BackRight].rangeMm = 1150;
    decision = updateFresh(controller, inputs, 250, 90.0F * yawSign);
    check(decision.state == RunState::Running &&
              decision.steeringPermille == 0,
          "diagonal ranges must not steer a known Open straight at its heading target");

    inputs.sides[FrontLeft].rangeMm = 180;
    inputs.sides[BackLeft].rangeMm = 1150;
    inputs.sides[FrontRight].rangeMm = 850;
    inputs.sides[BackRight].rangeMm = 100;
    decision = updateFresh(controller, inputs, 260, 90.0F * yawSign);
    check(decision.state == RunState::Running &&
              decision.steeringPermille == 0,
          "mirroring only diagonal ranges must leave known-straight steering unchanged");

    const float offsetYaw = 90.0F * yawSign + 5.0F;
    decision = updateFresh(controller, inputs, 270, offsetYaw);
    const int16_t headingCorrection = decision.steeringPermille;
    check(headingCorrection < 0,
          "positive known-straight yaw error must produce heading-only correction");
    inputs.sides[FrontLeft].rangeMm = 1100;
    inputs.sides[BackLeft].rangeMm = 140;
    inputs.sides[FrontRight].rangeMm = 220;
    inputs.sides[BackRight].rangeMm = 1000;
    decision = updateFresh(controller, inputs, 280, offsetYaw);
    check(decision.state == RunState::Running &&
              decision.steeringPermille == headingCorrection,
          "arbitrary diagonal rear offsets must not change known-straight heading correction");
  }
}

void testProductionObstacleDiagonalRearBiasCannotSteer() {
  autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
  config.directionMinimumDistinctSamples = 1;
  config.openingConfirmationMs = 0;
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  config.cornerRefractoryMs = 0;
  config.cornerRearmStableMs = 0;
  config.postCornerHeadingOnlyMs = 0;
  config.postCornerWallBlendMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  setRightOpening(inputs, 900);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::Clockwise,
        "Obstacle rear-bias fixture must establish a clockwise direction");
  updateFresh(controller, inputs, 140, 40.0F);
  updateFresh(controller, inputs, 180, 80.0F);
  setStraightGeometry(inputs, 1200);
  decision = updateFresh(controller, inputs, 200, 90.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1,
        "Obstacle rear-bias fixture must reach a known straight");

  inputs.sides[FrontLeft].rangeMm = 300;
  inputs.sides[FrontRight].rangeMm = 300;
  inputs.sides[BackLeft].rangeMm = 150;
  inputs.sides[BackRight].rangeMm = 1100;
  decision = updateFresh(controller, inputs, 700, 90.0F);
  check(decision.wallLateralOffsetMm == 0.0F &&
            decision.wallAngleErrorMm == 0.0F &&
            decision.steeringPermille == 0,
        "wide diagonal rear bias must not invent Obstacle wall angle or straight steering");

  inputs.sides[BackLeft].rangeMm = 1100;
  inputs.sides[BackRight].rangeMm = 150;
  decision = updateFresh(controller, inputs, 710, 90.0F);
  check(decision.wallLateralOffsetMm == 0.0F &&
            decision.wallAngleErrorMm == 0.0F &&
            decision.steeringPermille == 0,
        "mirroring only BL/BR must leave centered Obstacle steering unchanged");

  decision = updateFresh(controller, inputs, 720, 95.0F);
  const int16_t headingCorrection = decision.steeringPermille;
  check(headingCorrection < 0,
        "Obstacle straight yaw error must still produce heading correction");
  inputs.sides[BackLeft].rangeMm = 150;
  inputs.sides[BackRight].rangeMm = 1100;
  decision = updateFresh(controller, inputs, 730, 95.0F);
  check(decision.wallAngleErrorMm == 0.0F &&
            decision.steeringPermille == headingCorrection,
        "arbitrary diagonal rear offsets must not alter Obstacle heading steering");
}

void testR21DepthRampedMirroredCameraSteering() {
  const auto sampleSteering = [](bool red, uint16_t bottomY,
                                 bool distortFrontPair) {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.accelerationPermillePerSecond = 60000;
    config.decelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (red) {
      inputs.vision.red = usableBlob(0, bottomY, 1800, 900);
    } else {
      inputs.vision.green = usableBlob(0, bottomY, 1800, 900);
    }
    if (distortFrontPair) {
      // This is still outside the independent 140 mm emergency guard. During
      // an active pass it must not overpower the camera with a fabricated lane
      // offset from the two oblique rays.
      inputs.sides[FrontLeft].rangeMm = 900;
      inputs.sides[FrontRight].rangeMm = 150;
    }
    return updateFresh(controller, inputs, 100, 0.0F);
  };

  const Decision farRed = sampleSteering(true, 3000, false);
  const Decision farGreen = sampleSteering(false, 3000, false);
  check(farRed.obstacle == ObstacleTarget::RedPassRight &&
            farGreen.obstacle == ObstacleTarget::GreenPassLeft &&
            farRed.steeringPermille == 0 && farGreen.steeringPermille == 0,
        "R21 must acquire a threshold-depth pillar without snapping off the cardinal straight");

  const Decision closeRed = sampleSteering(true, 5500, false);
  const Decision closeGreen = sampleSteering(false, 5500, false);
  check(closeRed.steeringPermille > 0 && closeGreen.steeringPermille < 0 &&
            closeRed.steeringPermille == -closeGreen.steeringPermille &&
            std::abs(closeRed.steeringPermille) <= 500,
        "R21 close red and green guidance must be mirrored and camera-bounded");

  const Decision distortedRed = sampleSteering(true, 5500, true);
  check(distortedRed.steeringPermille == closeRed.steeringPermille,
        "an active R21 pillar must ignore uncalibrated diagonal FL/FR lane distortion");
}

void testR21SideClearanceArbitration() {
  const auto sample = [](uint16_t leftMm, uint16_t rightMm) {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.accelerationPermillePerSecond = 60000;
    config.decelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[FrontLeft].rangeMm = leftMm;
    inputs.sides[BackLeft].rangeMm = leftMm;
    inputs.sides[FrontRight].rangeMm = rightMm;
    inputs.sides[BackRight].rangeMm = rightMm;
    // Keep this outside direction-probe corner classification so the test
    // isolates the side-clearance arbiter instead of its intentional stop
    // while a left/right opening candidate is being confirmed.
    inputs.front.rangeMm = 1200;
    return updateFresh(controller, inputs, 100, 0.0F);
  };

  const Decision leftClose = sample(140, 300);
  check(leftClose.steeringPermille >= 650 &&
            leftClose.speedPermille <= 300 && leftClose.speedPermille > 0,
        "a close left ray must override toward the right at bounded speed");
  const Decision rightClose = sample(300, 140);
  check(rightClose.steeringPermille <= -650 &&
            rightClose.speedPermille <= 300 && rightClose.speedPermille > 0,
        "a close right ray must mirror the clearance override toward the left");
  const Decision bothClose = sample(140, 140);
  check(bothClose.steeringPermille == 0 && bothClose.speedPermille == 0,
        "two blocked sides must stop instead of choosing an unsafe oscillating escape");
}

void testR21CornerRearmRequiresCardinalHeading() {
  autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
  config.directionMinimumDistinctSamples = 1;
  config.openingConfirmationMs = 0;
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  config.cornerRefractoryMs = 1;
  config.cornerRearmStableMs = 0;
  config.postCornerHeadingOnlyMs = 0;
  config.postCornerWallBlendMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  setRightOpening(inputs, 900);
  updateFresh(controller, inputs, 100, 0.0F);
  updateFresh(controller, inputs, 120, 45.0F);
  setStraightGeometry(inputs, 1200);
  Decision decision = updateFresh(controller, inputs, 140, 90.0F);
  check(decision.state == RunState::Running && !decision.cornerDetectionArmed,
        "R21 heading-rearm fixture must exit its first corner disarmed");

  setStraightGeometry(inputs, 1200);
  decision = updateFresh(controller, inputs, 200, 99.0F);
  check(!decision.cornerDetectionArmed,
        "a nine-degree post-corner yaw error must keep R21 corner detection disarmed");
  decision = updateFresh(controller, inputs, 210, 98.0F);
  check(decision.cornerDetectionArmed,
        "R21 may rearm at the configured eight-degree cardinal-heading boundary");
}

void testOpenHeadingOnlyStraightStartsFullTorqueCorner() {
  autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  config.openingConfirmationMs = 0;
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  config.cornerRefractoryMs = 0;
  config.cornerRearmStableMs = 0;
  config.postCornerHeadingOnlyMs = 0;
  config.postCornerWallBlendMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  setRightOpening(inputs, 1000);
  updateFresh(controller, inputs, 100, 0.0F);
  updateFresh(controller, inputs, 120, 40.0F);
  updateFresh(controller, inputs, 130, 80.0F);
  setStraightGeometry(inputs, 1200);
  updateFresh(controller, inputs, 140, 90.0F);
  Decision decision = updateFresh(controller, inputs, 1000, 90.0F);
  check(decision.state == RunState::Running &&
            decision.speedPermille == config.directionProbeSpeedPermille,
        "heading-only Open straight must stay at the demonstrated 600 command");

  inputs.front.rangeMm = 650;
  decision = updateFresh(controller, inputs, 1010, 90.0F);
  check(decision.state == RunState::Cornering &&
            decision.motionAllowed &&
            decision.speedPermille == config.cornerSpeedPermille &&
            decision.steeringPermille ==
                static_cast<int16_t>(config.cornerSteeringPermille),
        "a known-direction Open corner must start its proven 450 drive and right steering together");

  inputs.front.valid = false;
  inputs.front.rangeStatus = 2;
  decision = updateFresh(controller, inputs, 1109, 95.0F);
  check(decision.state == RunState::Cornering && decision.motionAllowed &&
            decision.speedPermille == config.cornerSpeedPermille &&
            decision.faultFlags == autonomy::FaultNone,
        "a guided Open corner must retain 450 through a strong optical no-target");
  decision = updateFresh(controller, inputs, 1110, 95.0F);
  check(decision.state == RunState::Cornering && decision.motionAllowed &&
            decision.speedPermille == config.cornerSpeedPermille &&
            decision.faultFlags == autonomy::FaultNone,
        "Open corner torque must not depend on the retired 100 ms drive delay");
}

void testProductionObstacleCornerBypassesFrontEnvelope() {
  autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
  config.openingConfirmationMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  setRightOpening(inputs, 650);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::Clockwise &&
            decision.motionAllowed &&
            decision.speedPermille == config.minimumMovingSpeedPermille &&
            decision.speedPermille < config.cornerSpeedPermille &&
            decision.steeringPermille == config.cornerSteeringPermille &&
            decision.faultFlags == autonomy::FaultNone,
        "Obstacle corner entry must begin at the motion floor while acceleration slews up");

  decision = updateFresh(controller, inputs, 200, 5.0F);
  check(decision.state == RunState::Cornering && decision.motionAllowed &&
            decision.speedPermille > config.minimumMovingSpeedPermille &&
            decision.speedPermille <= config.cornerSpeedPermille &&
            decision.faultFlags == autonomy::FaultNone,
        "a confirmed diagonal Obstacle corner must bypass the proportional front envelope");
  decision = updateFresh(controller, inputs, 300, 10.0F);
  check(decision.state == RunState::Cornering && decision.motionAllowed &&
            decision.speedPermille == config.cornerSpeedPermille &&
            decision.steeringPermille == config.cornerSteeringPermille &&
            decision.faultFlags == autonomy::FaultNone,
        "a confirmed Obstacle corner at 650 mm must reach the proven 450/760 command");

  inputs.front.rangeMm = config.frontEmergencyStopMm;
  decision = updateFresh(controller, inputs, 310, 10.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultFrontTooClose) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0 &&
            decision.steeringPermille == 0,
        "Obstacle corner envelope bypass must retain the hard 180 mm stop");
}

void testProductionOpenCornerStallIsBoundedAndCanRecover() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  const auto enterClockwiseCorner = [&config](Controller& controller,
                                               Inputs& inputs) {
    startDirectionProbeWithBaselines(controller, inputs, true, false, false,
                                     595);
    inputs.front.rangeMm = 1000;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "stall replay must stop while the first clockwise sample is unconfirmed");
    decision = updateFresh(controller, inputs, 150, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "stall replay must retain the stopped three-sample direction proof");
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == CourseDirection::Clockwise &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.steeringPermille == config.cornerSteeringPermille,
          "stall replay must enter a powered clockwise corner on sample three");
    return decision;
  };

  {
    Controller controller(config);
    Inputs inputs;
    enterClockwiseCorner(controller, inputs);
    inputs.front.rangeMm = 700;

    updateFresh(controller, inputs, 300, 11.1F);
    Decision decision = updateFresh(controller, inputs, 400, 11.1F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille == config.cornerSpeedPermille &&
              decision.steeringPermille == config.cornerSteeringPermille &&
              decision.faultFlags == autonomy::FaultNone,
          "the logged 11.1 degree plateau must still receive the full 450 Open corner command");

    decision = updateFresh(controller, inputs, 5199, 11.1F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille == config.cornerSpeedPermille &&
              decision.faultFlags == autonomy::FaultNone,
          "a stalled Open corner must remain bounded one millisecond before its hard timeout");
    decision = updateFresh(controller, inputs, 5200, 11.1F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultCornerTimeout) != 0 &&
              !decision.motionAllowed && decision.speedPermille == 0 &&
              decision.steeringPermille == 0,
          "an Open corner stuck at 11.1 degrees must latch 0x4000 at exactly five seconds");
    decision = updateFresh(controller, inputs, 5300, 90.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultCornerTimeout) != 0 &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "late heading recovery must never clear a latched corner timeout");
  }

  {
    Controller controller(config);
    Inputs inputs;
    enterClockwiseCorner(controller, inputs);
    inputs.front.rangeMm = 700;

    updateFresh(controller, inputs, 400, 11.1F);
    Decision decision = updateFresh(controller, inputs, 3000, 11.1F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille == config.cornerSpeedPermille &&
              decision.faultFlags == autonomy::FaultNone,
          "a transient 11.1 degree plateau before the hard deadline must remain recoverable");
    decision = updateFresh(controller, inputs, 4400, 50.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone,
          "renewed clockwise yaw progress must continue the same corner safely");
    decision = updateFresh(controller, inputs, 4900, 88.0F);
    check(decision.state == RunState::Cornering && decision.cornerCount == 0,
          "recovered heading must still satisfy the full 200 ms exit stability interval");
    decision = updateFresh(controller, inputs, 5099, 89.0F);
    check(decision.state == RunState::Cornering && decision.cornerCount == 0,
          "recovered corner exit must not complete at 199 stable milliseconds");
    decision = updateFresh(controller, inputs, 5100, 89.0F);
    check(decision.state == RunState::Running && decision.cornerCount == 1 &&
              decision.faultFlags == autonomy::FaultNone,
          "real heading recovery before five seconds must complete one verified corner");
  }
}

void testProductionOpenUsesMeasuredWrappedFrontRange() {
  autonomy::Config config = legacyOpenFixtureConfig();
  config.openingConfirmationMs = 0;
  config.straightFrontOpticalGraceMs = 100;
  config.accelerationPermillePerSecond = 10000;
  config.decelerationPermillePerSecond = 10000;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  inputs.front.valid = false;
  inputs.front.rangeStatus = 7;
  inputs.front.rangeMm = 900;
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  decision = updateFresh(controller, inputs, 201, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.faultFlags == autonomy::FaultNone &&
            decision.speedPermille > config.cornerBlindSpeedPermille,
        "production Open must use the measured status-7 front range beyond the old grace boundary");

  setLeftOpening(inputs, 600);
  inputs.front.valid = false;
  inputs.front.rangeStatus = 7;
  inputs.front.rangeMm = config.directionProbeFrontMaximumMm;
  decision = updateFresh(controller, inputs, 220, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::CounterClockwise,
        "a measured status-7 front range must participate in Open direction and corner timing");
}

void testOpenBlindFrontFallbackIsStrictlyGated() {
  const auto blindConfig = []() {
    autonomy::Config config = legacyOpenFixtureConfig();
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    config.cornerRefractoryMs = 0;
    config.cornerRearmStableMs = 0;
    config.postCornerHeadingOnlyMs = 0;
    config.postCornerWallBlendMs = 0;
    return config;
  };
  const auto reachKnownStraight = [](Controller& controller, Inputs& inputs,
                                     bool clockwise) {
    startController(controller, inputs);
    if (clockwise) {
      setRightOpening(inputs, 600);
    } else {
      setLeftOpening(inputs, 600);
    }
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe,
          "blind-front fixture must respect first-direction confirmation");
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::Cornering,
          "blind-front fixture must establish a known direction");
    const float yawSign = clockwise ? 1.0F : -1.0F;
    updateFresh(controller, inputs, 220, 40.0F * yawSign);
    updateFresh(controller, inputs, 240, 80.0F * yawSign);
    setStraightGeometry(inputs, 1200);
    decision = updateFresh(controller, inputs, 250, 90.0F * yawSign);
    check(decision.state == RunState::Running &&
              decision.cornerCount == 1,
          "blind-front fixture must reach its baseline-backed straight");
  };

  // Strong front no-target may start a known-direction turn only with the
  // mirrored expected-side baseline rise. Exercise both exact boundaries.
  for (const bool clockwise : {true, false}) {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs;
    reachKnownStraight(controller, inputs, clockwise);
    const float yaw = clockwise ? 90.0F : -90.0F;
    const uint8_t expectedFront = clockwise ? FrontRight : FrontLeft;
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    inputs.sides[expectedFront].rangeMm = 479;
    Decision decision = updateFresh(controller, inputs, 300, yaw);
    check(decision.state == RunState::Running &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "a 179 mm expected-side rise must not start a blind turn and must cap speed to 200");
    inputs.sides[expectedFront].rangeMm = 480;
    decision = updateFresh(controller, inputs, 310, yaw);
    check(decision.state == RunState::Running,
          "the exact 180 mm blind opening must start confirmation, not skip it");
    decision = updateFresh(controller, inputs, 409, yaw);
    check(decision.state == RunState::Running,
          "blind opening confirmation must not finish at 99 ms");
    decision = updateFresh(controller, inputs, 410, yaw);
    check(decision.state == RunState::Cornering &&
              decision.cornerCount == 1,
          "mirrored strong-blind opening must enter at exactly 100 ms");
  }

  // An opening on the wrong side cannot turn the car; persistent blindness
  // remains bounded by the production straight optical grace.
  for (const bool clockwise : {true, false}) {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs;
    reachKnownStraight(controller, inputs, clockwise);
    const float yaw = clockwise ? 90.0F : -90.0F;
    const uint8_t wrongFront = clockwise ? FrontLeft : FrontRight;
    inputs.front.valid = false;
    inputs.front.rangeStatus = 4;
    inputs.sides[wrongFront].rangeMm = 480;
    Decision decision = updateFresh(controller, inputs, 300, yaw);
    decision = updateFresh(controller, inputs, 500, yaw);
    check(decision.state == RunState::Running &&
              decision.cornerCount == 1,
          "a strong blind front plus wrong-side opening must not turn");
    if (clockwise) {
      decision = updateFresh(controller, inputs, 1499, yaw);
      check(decision.state == RunState::Running &&
                decision.speedPermille <= config.cornerBlindSpeedPermille,
            "blind Running must remain capped through 1199 ms of optical grace");
      decision = updateFresh(controller, inputs, 1500, yaw);
      check(decision.state == RunState::FaultLatched &&
                (decision.faultFlags &
                 autonomy::FaultFrontTofUnavailable) != 0 &&
                !decision.motionAllowed && decision.speedPermille == 0,
            "unresolved blind front must latch a full stop at 1200 ms");
    }
  }

  // SigmaFail, a status-6 optical outcome, and wrapped-target status are
  // degraded inputs but never the strong 2/4 evidence needed for a blind turn.
  {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs;
    reachKnownStraight(controller, inputs, true);
    inputs.sides[FrontRight].rangeMm = 480;
    uint32_t nowMs = 300;
    for (const uint8_t weakStatus : {1U, 6U, 7U}) {
      inputs.front.valid = false;
      inputs.front.rangeStatus = weakStatus;
      Decision decision = updateFresh(controller, inputs, nowMs, 90.0F);
      decision = updateFresh(controller, inputs, nowMs + 150U, 90.0F);
      check(decision.state == RunState::Running,
            "front statuses 1, 6, and 7 must never authorize a blind turn");
      inputs.front.valid = true;
      inputs.front.rangeStatus = 0;
      inputs.front.rangeMm = 1200;
      updateFresh(controller, inputs, nowMs + 160U, 90.0F);
      nowMs += 200U;
    }
  }

  // Dropping below the 180 mm side threshold resets the same candidate; its
  // earlier 50 ms cannot be carried into the next confirmation interval.
  {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs;
    reachKnownStraight(controller, inputs, false);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    inputs.sides[FrontLeft].rangeMm = 480;
    updateFresh(controller, inputs, 300, -90.0F);
    inputs.sides[FrontLeft].rangeMm = 479;
    Decision decision = updateFresh(controller, inputs, 350, -90.0F);
    check(decision.state == RunState::Running,
          "a transient blind opening must reset below 180 mm");
    inputs.sides[FrontLeft].rangeMm = 480;
    updateFresh(controller, inputs, 400, -90.0F);
    decision = updateFresh(controller, inputs, 499, -90.0F);
    check(decision.state == RunState::Running,
          "restarted blind confirmation must remain Running at 99 ms");
    decision = updateFresh(controller, inputs, 500, -90.0F);
    check(decision.state == RunState::Cornering,
          "restarted blind confirmation may turn at its own 100 ms boundary");
  }

  // Once direction is known, a numeric close front does not require a side
  // opening or another 100 ms delay in Open Challenge.  Blind evidence above
  // still keeps the full confirmation interval.
  {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs;
    reachKnownStraight(controller, inputs, true);
    setStraightGeometry(inputs, 650);
    Decision decision = updateFresh(controller, inputs, 300, 90.0F);
    check(decision.state == RunState::Cornering,
          "known-direction Open numeric front must enter immediately");
  }

  // A complete open-side snapshot with reference 0 is not a learned local
  // wall. Strong blind front cannot turn from that static start geometry.
  {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart,
          "reference-zero snapshot must remain a legal Open start");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    updateFresh(controller, inputs, 90, 0.0F);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.leftOpeningMm == 0.0F &&
              decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "reference-zero complete snapshot must not choose first direction");
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "reference-zero blind evidence must remain untrusted after 100 ms");
  }

  // When both real wall pairs later appear, a strong blind front may capture
  // nonzero baselines and then use a subsequent expected-side opening.
  {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    for (auto& side : inputs.sides) side.rangeMm = 5000;
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart,
          "later-baseline fixture must arm from far side readings");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    updateFresh(controller, inputs, 90, 0.0F);
    setStraightGeometry(inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "both local pairs may capture baselines while blind at capped speed");
    inputs.sides[FrontLeft].rangeMm = 480;
    updateFresh(controller, inputs, 110, 0.0F);
    decision = updateFresh(controller, inputs, 209, 0.0F);
    check(decision.state == RunState::DirectionProbe,
          "later learned blind direction must not confirm at 99 ms");
    decision = updateFresh(controller, inputs, 210, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "a blind center front must not authorize first direction from an old side baseline");
  }

  // With every side ray blind there is no pair from which to learn a local
  // reference, so no powered direction may be guessed.
  {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart,
          "all-blind Open geometry may arm electrically without inventing a wall");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    updateFresh(controller, inputs, 90, 0.0F);
    decision = updateFresh(controller, inputs, 100, 0.0F);
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "all-blind first direction must remain unknown at capped speed");
  }

  // A blind front plus only one numeric side pair on the button edge is not a
  // safe baseline provenance. Losing that pair's forward ray must not turn;
  // two complete local pairs may later recover baseline learning explicitly.
  {
    const autonomy::Config config = blindConfig();
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart,
          "one numeric side pair plus blind front must remain a legal Open start");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    updateFresh(controller, inputs, 90, 0.0F);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    decision = updateFresh(controller, inputs, 100, 0.0F);
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "blind start may not turn from a baseline captured with only one pair");

    setStraightGeometry(inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    updateFresh(controller, inputs, 210, 0.0F);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    updateFresh(controller, inputs, 220, 0.0F);
    decision = updateFresh(controller, inputs, 319, 0.0F);
    check(decision.state == RunState::DirectionProbe,
          "safely recovered blind baseline must still confirm for 100 ms");
    decision = updateFresh(controller, inputs, 320, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "two-pair recovery still requires a numeric center-front direction gate");
  }
}

void testActiveObstacleBlocksDirectionLearning() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  inputs.vision.red = usableBlob(-3500, 4500, 2000, 700);
  setLeftOpening(inputs, 600);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown &&
            decision.obstacle == ObstacleTarget::RedPassRight,
        "an active starting-straight obstacle must block direction learning");
  decision = updateFresh(controller, inputs, 200, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "corner-like ranges must stay ignored while the obstacle remains active");
}

void testProductionObstacleMaximumCommitTimeout() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.obstacle == ObstacleTarget::RedPassRight,
        "maximum-commit fixture must begin with a production obstacle");

  const uint32_t committedAtMs = 100;
  decision = updateFresh(controller, inputs,
                         committedAtMs + config.obstacleMaximumCommitMs - 1U,
                         0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.obstacle == ObstacleTarget::RedPassRight &&
            (decision.faultFlags & FaultObstaclePassTimeout) == 0,
        "a committed obstacle must remain live at 7999 elapsed milliseconds");
  decision = updateFresh(controller, inputs,
                         committedAtMs + config.obstacleMaximumCommitMs,
                         0.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultObstaclePassTimeout) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0 &&
            decision.steeringPermille == 0,
        "an uncleared obstacle must latch a full stop at 8000 elapsed milliseconds");

  inputs.vision.red = {};
  decision = updateFresh(controller, inputs,
                         committedAtMs + config.obstacleMaximumCommitMs + 1U,
                         0.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultObstaclePassTimeout) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0,
        "healthy input after obstacle timeout must not clear the latched stop");
}

void testCommittedObstaclePassThroughCameraLoss() {
  for (const bool redPillar : {true, false}) {
    Controller controller(fixtureConfig());
    Inputs inputs;
    startController(controller, inputs);

    if (redPillar) {
      inputs.vision.red = usableBlob(-1000, 5200, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(1000, 5200, 2000, 700);
    }
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    const ObstacleTarget expectedTarget =
        redPillar ? ObstacleTarget::RedPassRight : ObstacleTarget::GreenPassLeft;
    const int16_t expectedOffsetSign = redPillar ? 1 : -1;
    check(decision.obstacle == expectedTarget &&
              decision.targetLateralOffsetMm * expectedOffsetSign > 0 &&
              decision.steeringPermille * expectedOffsetSign > 0,
          "a pillar in the starting straight must steer to its required passing side");

    // Vision is optional for motion, but a brief camera dropout must not make
    // the vehicle abandon the avoidance path it already committed to.
    inputs.vision.online = false;
    inputs.vision.valid = false;
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe && decision.motionAllowed &&
              (decision.faultFlags &
               (autonomy::FaultVisionUnavailable | FaultVisionStale)) == 0,
          "temporary optional camera loss must not stop an otherwise healthy run");
    check(decision.obstacle == expectedTarget &&
              decision.targetLateralOffsetMm * expectedOffsetSign > 0,
          "temporary camera loss must retain the committed passing side");

  }
}

void testObstacleMotionFloorAndOverrunRejection() {
  {
    // Mirrors the assembled belt-drive relationship: 20% is the measured
    // movement floor, while moderately uncertain obstacle speed was set below
    // that floor. A valid 70% confidence pillar must still move at the floor.
    autonomy::Config config = fixtureConfig();
    config.directionProbeSpeedPermille = 220;
    config.obstacleSpeedPermille = 200;
    config.obstacleUncertainSpeedPermille = 150;
    config.minimumMovingSpeedPermille = 200;
    config.accelerationPermillePerSecond = 10000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    const Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.speedPermille == config.minimumMovingSpeedPermille &&
              decision.steeringPermille > 0,
          "an accepted obstacle must not deadlock below the physical movement floor");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.accelerationPermillePerSecond = 10000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.pipelineOverrun = true;
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 900);
    const Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.obstacle == ObstacleTarget::None &&
              decision.targetLateralOffsetMm == 0,
          "an overrun-marked camera frame must not create a new steering target");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.obstacleAvoidanceEnabled = false;
    config.requireVisionForMotion = false;
    config.directionProbeSpeedPermille = 240;
    config.accelerationPermillePerSecond = 10000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.online = false;
    inputs.vision.valid = false;
    inputs.vision.pipelineOverrun = true;
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 900);
    const Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::None &&
              decision.speedPermille == config.directionProbeSpeedPermille &&
              decision.steeringPermille == 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "Open profile must remain independent of every vision failure and false pillar");
  }
}

void testRequiredVisionLossStopsThenRecoversOrFaults() {
  autonomy::Config config = fixtureConfig();
  config.requireVisionForMotion = true;
  config.visionLossGraceMs = 100;
  config.visionUncertainSpeedPermille = 100;
  config.minimumMovingSpeedPermille = 140;
  config.accelerationPermillePerSecond = 10000;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  Decision decision = updateFresh(controller, inputs, 200, 0.0F);
  check(decision.motionAllowed && decision.speedPermille > 0,
        "required-vision fixture must move with a healthy camera");

  // CameraValid describes a processed frame, not whether that frame happened
  // to contain a pillar. Repeated fresh frames with no red/green blobs are the
  // normal case between obstacles and must remain legal beyond the loss grace.
  decision = updateFresh(controller, inputs, 800, 0.0F);
  check(decision.state == RunState::DirectionProbe && decision.motionAllowed &&
            decision.speedPermille > 0 &&
            decision.obstacle == ObstacleTarget::None,
        "fresh required-camera no-target frames must never be mistaken for camera loss");

  inputs.vision.online = false;
  inputs.vision.valid = false;
  decision = updateFresh(controller, inputs, 810, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.motionAllowed && decision.speedPermille == 0 &&
            decision.faultFlags == autonomy::FaultNone,
        "brief required-camera loss must hold motion without latching the round");
  decision = updateFresh(controller, inputs, 909, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.speedPermille == 0,
        "required-camera hold must remain recoverable inside its grace interval");

  inputs.vision.online = true;
  inputs.vision.valid = true;
  decision = updateFresh(controller, inputs, 910, 0.0F);
  check(decision.state == RunState::DirectionProbe && decision.motionAllowed &&
            decision.faultFlags == autonomy::FaultNone,
        "required camera recovery must resume the same run without another button press");

  // A live UART heartbeat with an overrun-marked observation is not usable
  // steering evidence either: stop immediately, then fault if it persists.
  inputs.vision.pipelineOverrun = true;
  updateFresh(controller, inputs, 920, 0.0F);
  decision = updateFresh(controller, inputs, 1019, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "a new camera dropout must receive a fresh bounded grace interval");
  decision = updateFresh(controller, inputs, 1020, 0.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & autonomy::FaultVisionUnavailable) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0,
        "persistent required-camera loss must fault exactly at the grace boundary");
}

void testPassClearSidesAndMillisecondAcceleration() {
  for (const bool redPillar : {true, false}) {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);

    if (redPillar) {
      inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 5200, 2000, 700);
    }
    const ObstacleTarget expectedTarget =
        redPillar ? ObstacleTarget::RedPassRight : ObstacleTarget::GreenPassLeft;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.obstacle == expectedTarget,
          "pass-clear fixture must commit the intended pillar color");
    inputs.vision.red = {};
    inputs.vision.green = {};

    // Establish a per-pass minimum on the selected outer pair. Red passes on
    // the right and therefore observes the pillar move along the left pair;
    // green is the exact mirror on the right pair.
    setStraightGeometry(inputs);
    const uint8_t selectedFront = redPillar ? FrontLeft : FrontRight;
    const uint8_t selectedBack = redPillar ? BackLeft : BackRight;
    const uint8_t oppositeFront = redPillar ? FrontRight : FrontLeft;
    const uint8_t oppositeBack = redPillar ? BackRight : BackLeft;
    inputs.sides[selectedFront].rangeMm = 200;
    inputs.sides[selectedBack].rangeMm = 300;
    decision = updateFresh(controller, inputs, 249, 0.0F);
    check(decision.obstacle == expectedTarget,
          "same-color loss must not clear a pass one millisecond early");

    decision = updateFresh(controller, inputs, 250, 0.0F);
    check(decision.obstacle == expectedTarget,
          "blob loss alone must not clear without minimum commit and pair transition");

    // A large transition on the opposite pair is irrelevant.
    inputs.sides[oppositeFront].rangeMm = 600;
    inputs.sides[oppositeBack].rangeMm = 300;
    decision = updateFresh(controller, inputs, 330, 0.0F);
    check(decision.obstacle == expectedTarget,
          "clear geometry on the opposite side must not end a committed pass");

    // Raise the selected-pair delta from -100 mm to +20 mm: exactly the
    // configured 120 mm relative transition. It cannot start confirmation
    // until the production 300 ms minimum commitment has elapsed.
    inputs.sides[selectedFront].rangeMm = 320;
    inputs.sides[selectedBack].rangeMm = 300;
    decision = updateFresh(controller, inputs, 399, 0.0F);
    check(decision.obstacle == expectedTarget,
          "relative pass geometry must respect the minimum commitment time");
    decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.obstacle == expectedTarget,
          "the qualifying pair transition must start, not skip, confirmation");
    decision = updateFresh(controller, inputs, 579, 0.0F);
    check(decision.obstacle == expectedTarget,
          "pass geometry confirmation must not complete one millisecond early");
    decision = updateFresh(controller, inputs, 580, 0.0F);
    const int16_t clearedOffset = decision.targetLateralOffsetMm;
    check(decision.obstacle == ObstacleTarget::None &&
              clearedOffset * (redPillar ? 1 : -1) > 0,
          "close encounter, 150 ms loss, 120 mm rise, and 180 ms stability must clear into recentering");

    // The target returns toward center smoothly and the tail of the same blob
    // cannot immediately recommit the just-cleared color.
    if (redPillar) {
      inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 5200, 2000, 700);
    }
    decision = updateFresh(controller, inputs, 730, 0.0F);
    check(decision.obstacle == ObstacleTarget::None &&
              std::abs(decision.targetLateralOffsetMm) > 0 &&
              std::abs(decision.targetLateralOffsetMm) <
                  std::abs(clearedOffset),
          "recenter must decay the old offset while same-color tail frames remain blocked");

    inputs.vision.red = {};
    inputs.vision.green = {};
    updateFresh(controller, inputs, 731, 0.0F);
    decision = updateFresh(controller, inputs, 1030, 0.0F);
    check(decision.obstacle == ObstacleTarget::None &&
              decision.targetLateralOffsetMm == 0,
          "same-color recommit cooldown must require its full absent interval");
    decision = updateFresh(controller, inputs, 1031, 0.0F);
    if (redPillar) {
      inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 5200, 2000, 700);
    }
    decision = updateFresh(controller, inputs, 1032, 0.0F);
    check(decision.obstacle == expectedTarget,
          "same color may become a new obstacle only after the absent cooldown");

  }

  autonomy::Config config = fixtureConfig();
  config.accelerationPermillePerSecond = 500;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  const uint16_t initialSpeed = controller.decision().speedPermille;
  Decision decision;
  for (uint32_t nowMs = 91; nowMs <= 101; ++nowMs) {
    decision = updateFresh(controller, inputs, nowMs, 0.0F);
  }
  check(initialSpeed == 0 && decision.speedPermille > 0 &&
            decision.speedPermille < controller.config().directionProbeSpeedPermille,
        "one-millisecond updates must eventually advance a bounded acceleration command");
}

Decision completeClosePillarPassForCooldown(Controller& controller,
                                            Inputs& inputs,
                                            bool redPillar) {
  startController(controller, inputs);
  setStraightGeometry(inputs);
  const uint8_t selectedFront = redPillar ? FrontLeft : FrontRight;
  const uint8_t selectedBack = redPillar ? BackLeft : BackRight;
  inputs.sides[selectedFront].rangeMm = 200;
  inputs.sides[selectedBack].rangeMm = 300;
  if (redPillar) {
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
  } else {
    inputs.vision.green = usableBlob(3500, 5200, 2000, 700);
  }
  const ObstacleTarget expectedTarget =
      redPillar ? ObstacleTarget::RedPassRight
                : ObstacleTarget::GreenPassLeft;
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.obstacle == expectedTarget,
        "cooldown fixture must commit its close first pillar");

  inputs.vision.red = {};
  inputs.vision.green = {};
  updateFresh(controller, inputs, 110, 0.0F);
  inputs.sides[selectedFront].rangeMm = 320;
  decision = updateFresh(controller, inputs, 400, 0.0F);
  check(decision.obstacle == expectedTarget,
        "cooldown fixture must begin pass-clear confirmation at 300 ms");
  decision = updateFresh(controller, inputs, 579, 0.0F);
  check(decision.obstacle == expectedTarget,
        "cooldown fixture must retain the first pillar through 179 ms confirmation");
  decision = updateFresh(controller, inputs, 580, 0.0F);
  check(decision.obstacle == ObstacleTarget::None &&
            decision.targetLateralOffsetMm * (redPillar ? 1 : -1) > 0,
        "cooldown fixture must complete one proven mirrored pass");
  return decision;
}

void testConsecutiveSameColorPillarsUseBoundedCooldown() {
  for (const bool redPillar : {true, false}) {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    completeClosePillarPassForCooldown(controller, inputs, redPillar);
    const ObstacleTarget expectedTarget =
        redPillar ? ObstacleTarget::RedPassRight
                  : ObstacleTarget::GreenPassLeft;

    // Official cards can place two red or two green pillars on one straight.
    // A continuously visible second pillar must stay quarantined only for the
    // bounded tail interval; its presence must not restart that interval.
    if (redPillar) {
      inputs.vision.red = usableBlob(-3500, 3800, 1400, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 3800, 1400, 700);
    }
    Decision decision = updateFresh(controller, inputs, 581, 0.0F);
    check(decision.obstacle == ObstacleTarget::None,
          "a same-color tail or second pillar must remain blocked just after clear");
    decision = updateFresh(controller, inputs, 879, 0.0F);
    check(decision.obstacle == ObstacleTarget::None,
          "same-color recommit must remain blocked through 299 ms");
    decision = updateFresh(controller, inputs, 880, 0.0F);
    check(decision.obstacle == expectedTarget &&
              decision.targetLateralOffsetMm * (redPillar ? 1 : -1) > 0,
          "a continuously visible legal second same-color pillar must commit at 300 ms");
  }

  {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    completeClosePillarPassForCooldown(controller, inputs, true);
    inputs.vision.green = usableBlob(3500, 3800, 1400, 700);
    const Decision decision = updateFresh(controller, inputs, 581, 0.0F);
    check(decision.obstacle == ObstacleTarget::GreenPassLeft &&
              decision.targetLateralOffsetMm < 0,
          "a different-color second pillar must remain eligible immediately");
  }

  {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    check(config.visionLossGraceMs > config.obstacleRecommitCooldownMs,
          "camera-recovery cooldown fixture requires a bounded non-faulting hold");
    Controller controller(config);
    Inputs inputs;
    completeClosePillarPassForCooldown(controller, inputs, true);

    inputs.vision.online = false;
    inputs.vision.valid = false;
    updateFresh(controller, inputs, 581, 0.0F);
    Decision decision = updateFresh(controller, inputs, 700, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.motionAllowed && decision.speedPermille == 0,
          "short required-camera loss must hold without faulting the cooldown fixture");

    inputs.vision.online = true;
    inputs.vision.valid = true;
    inputs.vision.red = usableBlob(-3500, 3800, 1400, 700);
    decision = updateFresh(controller, inputs, 880, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.faultFlags == autonomy::FaultNone,
          "camera hold must not restart the proven-pass same-color cooldown");
  }
}

void testPassClearAcceptsOneStrongNoTargetRay() {
  for (const bool redPillar : {true, false}) {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);

    if (redPillar) {
      inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 5200, 2000, 700);
    }
    const uint8_t selectedFront = redPillar ? FrontLeft : FrontRight;
    const uint8_t selectedBack = redPillar ? BackLeft : BackRight;
    inputs.sides[selectedBack].valid = false;
    inputs.sides[selectedBack].rangeStatus = 2;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    const ObstacleTarget expectedTarget =
        redPillar ? ObstacleTarget::RedPassRight : ObstacleTarget::GreenPassLeft;
    check(decision.obstacle == expectedTarget,
          "one blind selected rear ray must not block a mirrored obstacle commitment");

    inputs.vision.red = {};
    inputs.vision.green = {};
    inputs.sides[selectedFront].rangeMm = 300;
    updateFresh(controller, inputs, 110, 0.0F);
    decision = updateFresh(controller, inputs, 399, 0.0F);
    check(decision.obstacle == expectedTarget,
          "one-ray pass proof must still respect minimum commitment time");

    // The rear ray remains a fresh, strong no-target. A 120 mm rise on the
    // selected numeric front ray must provide the same bounded relative proof.
    inputs.sides[selectedFront].rangeMm = 420;
    decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.obstacle == expectedTarget,
          "one-ray pass transition must begin, not skip, clear confirmation");
    decision = updateFresh(controller, inputs, 579, 0.0F);
    check(decision.obstacle == expectedTarget,
          "one-ray pass transition must retain the target through 179 ms");
    decision = updateFresh(controller, inputs, 580, 0.0F);
    check(decision.obstacle == ObstacleTarget::None &&
              decision.targetLateralOffsetMm * (redPillar ? 1 : -1) > 0,
          "one strong no-target plus one numeric ray must clear red and green symmetrically");
  }
}

void testObstacleAbsenceRequiresFreshTrulyBlankFrames() {
  for (const bool redPillar : {true, false}) {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (redPillar) {
      inputs.vision.red = usableBlob(-3500, 4499, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 4499, 2000, 700);
    }
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    const ObstacleTarget expectedTarget =
        redPillar ? ObstacleTarget::RedPassRight : ObstacleTarget::GreenPassLeft;
    check(decision.obstacle == expectedTarget,
          "a mirrored 4499-bottom pillar may commit but is not close-qualified");
    inputs.vision.red = {};
    inputs.vision.green = {};
    updateFresh(controller, inputs, 110, 0.0F);
    decision = updateFresh(controller, inputs, 259, 0.0F);
    check(decision.obstacle == expectedTarget && decision.speedPermille > 0,
          "brief pre-close loss must preserve the pass through 149 ms");
    decision = updateFresh(controller, inputs, 260, 0.0F);
    check(decision.obstacle == expectedTarget && decision.speedPermille == 0,
          "pre-close loss must stop at the ordinary 150 ms blob-loss boundary");
    decision = updateFresh(controller, inputs, 609, 0.0F);
    check(decision.obstacle == expectedTarget && decision.speedPermille == 0,
          "a never-close target must stay stopped through 499 ms of fresh loss");
    decision = updateFresh(controller, inputs, 610, 0.0F);
    check(decision.obstacle == ObstacleTarget::None &&
              decision.targetLateralOffsetMm * (redPillar ? 1 : -1) > 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "500 ms fresh loss must abandon and recenter a never-close target symmetrically");

    // A false/far abort is not a completed pass. If the same color returns it
    // must be eligible immediately instead of waiting for the pass cooldown.
    if (redPillar) {
      inputs.vision.red = usableBlob(-3500, 4499, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 4499, 2000, 700);
    }
    decision = updateFresh(controller, inputs, 611, 0.0F);
    check(decision.obstacle == expectedTarget,
          "an aborted far target must recommit immediately when genuinely reacquired");
  }

  {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[FrontLeft].rangeMm = 220;
    inputs.sides[BackLeft].rangeMm = 300;
    inputs.vision.red = usableBlob(-3500, 4500, 2000, 700);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.vision.red = {};
    updateFresh(controller, inputs, 110, 0.0F);
    inputs.sides[FrontLeft].rangeMm = 340;
    Decision decision = updateFresh(controller, inputs, 399, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "exactly close pillar must retain the real-pass proof at 299 ms");
    updateFresh(controller, inputs, 400, 0.0F);
    decision = updateFresh(controller, inputs, 579, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "close-gated pass confirmation must remain active at 179 ms");
    decision = updateFresh(controller, inputs, 580, 0.0F);
    check(decision.obstacle == ObstacleTarget::None,
          "bottom 4500 plus fresh absence and stable pair transition may clear");
  }

  {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[FrontLeft].rangeMm = 220;
    inputs.sides[BackLeft].rangeMm = 300;
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "fresh-absence fixture must commit a close red pillar");

    // Three hundred milliseconds without camera frames is unknown time, not
    // proof that the pillar disappeared.
    inputs.vision.red = {};
    inputs.vision.online = false;
    inputs.vision.valid = false;
    updateFresh(controller, inputs, 120, 0.0F);
    decision = updateFresh(controller, inputs, 420, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.speedPermille == 0,
          "a bounded camera outage must hold motion and preserve the commitment");

    inputs.vision.online = true;
    inputs.vision.valid = true;
    inputs.sides[FrontLeft].rangeMm = 340;
    decision = updateFresh(controller, inputs, 421, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "one blank recovery frame must not inherit camera-outage wall clock");

    // A raw same-color tail remains presence even when confidence is too low
    // to refresh steering. It must reset the absence interval.
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 100);
    updateFresh(controller, inputs, 422, 0.0F);
    decision = updateFresh(controller, inputs, 700, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "a low-confidence present tail must never count as absence");
    inputs.vision.red = {};
    updateFresh(controller, inputs, 701, 0.0F);
    decision = updateFresh(controller, inputs, 850, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "true fresh absence must last all 150 ms");
    decision = updateFresh(controller, inputs, 851, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "the fresh-absence boundary must start, not skip, clear confirmation");
    decision = updateFresh(controller, inputs, 1030, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "fresh pass-clear confirmation must not finish one millisecond early");
    decision = updateFresh(controller, inputs, 1031, 0.0F);
    check(decision.obstacle == ObstacleTarget::None,
          "only 150 ms truly blank plus 180 ms stable geometry may clear");
  }

  {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[FrontLeft].rangeMm = 220;
    inputs.sides[BackLeft].rangeMm = 300;
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.vision.red = {};
    updateFresh(controller, inputs, 110, 0.0F);
    inputs.sides[FrontLeft].rangeMm = 340;
    Decision decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "qualifying pass geometry must begin its 180 ms confirmation");
    updateFresh(controller, inputs, 500, 0.0F);

    inputs.vision.online = false;
    inputs.vision.valid = false;
    decision = updateFresh(controller, inputs, 501, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.speedPermille == 0,
          "camera loss during pass confirmation must hold motion and reset evidence");
    inputs.vision.online = true;
    inputs.vision.valid = true;
    decision = updateFresh(controller, inputs, 600, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "the first recovered blank frame must restart absence from zero");
    updateFresh(controller, inputs, 750, 0.0F);
    decision = updateFresh(controller, inputs, 929, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "outage-interrupted clear confirmation must earn a full fresh 180 ms again");
    decision = updateFresh(controller, inputs, 930, 0.0F);
    check(decision.obstacle == ObstacleTarget::None,
          "pass may clear only after the restarted fresh confirmation completes");
  }

  {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[FrontLeft].rangeMm = 220;
    inputs.sides[BackLeft].rangeMm = 300;
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.vision.red = {};
    updateFresh(controller, inputs, 110, 0.0F);
    inputs.sides[FrontLeft].rangeMm = 340;
    updateFresh(controller, inputs, 400, 0.0F);

    // Side loss is evaluated before the required-camera hold. When both fail
    // together, that earlier branch must still discard absence and clear time.
    inputs.sides[BackRight].i2cStatus = 4;
    inputs.vision.online = false;
    inputs.vision.valid = false;
    Decision decision = updateFresh(controller, inputs, 450, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.speedPermille == 0,
          "combined side and camera loss must hold a committed pass safely");
    inputs.sides[BackRight].i2cStatus = 0;
    inputs.vision.online = true;
    inputs.vision.valid = true;
    updateFresh(controller, inputs, 451, 0.0F);
    decision = updateFresh(controller, inputs, 600, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "combined-dropout recovery must earn 150 fresh blank milliseconds");
    updateFresh(controller, inputs, 601, 0.0F);
    decision = updateFresh(controller, inputs, 780, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "combined-dropout recovery must also re-earn 180 ms clear stability");
    decision = updateFresh(controller, inputs, 781, 0.0F);
    check(decision.obstacle == ObstacleTarget::None,
          "combined dropout may clear only after both fresh timers restart");
  }
}

void testVisionSafetyCapsAndHeadingRecovery() {
  {
    autonomy::Config config = fixtureConfig();
    config.directionProbeSpeedPermille = 300;
    config.cruiseSpeedPermille = 300;
    config.obstacleSpeedPermille = 300;
    config.visionUncertainSpeedPermille = 100;
    config.obstacleUncertainSpeedPermille = 80;
    config.visionOverrunSpeedPermille = 120;
    config.fullSpeedFrameQuality = 50;
    config.accelerationPermillePerSecond = 60000;
    config.decelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    Decision decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.speedPermille == 300,
          "vision-cap fixture must reach its uncapped direction-probe speed");

    inputs.vision.frameQuality = 18;
    inputs.vision.red = usableBlob(-3500, 5200, 2000, 500);
    decision = updateFresh(controller, inputs, 210, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.speedPermille == 172,
          "low frame quality must immediately combine with obstacle confidence");

    inputs.vision.pipelineOverrun = true;
    decision = updateFresh(controller, inputs, 211, 0.0F);
    check(decision.speedPermille == 0,
          "a safety cap below the physical movement floor must stop, not fake motion");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 120, 40.0F);
    setStraightGeometry(inputs);
    Decision decision = updateFresh(controller, inputs, 140, 90.0F);
    check(decision.state == RunState::Running &&
              decision.direction == CourseDirection::CounterClockwise,
          "heading-recovery fixture must establish a measured straight reference");

    decision = updateFresh(controller, inputs, 150, 100.0F);
    check(decision.steeringPermille > 0,
          "a physical leftward drift from the reference must steer right");
    updateFresh(controller, inputs, 160, 90.0F);
    decision = updateFresh(controller, inputs, 170, 80.0F);
    check(decision.steeringPermille < 0,
          "a physical rightward drift from the reference must steer left");
  }
}

void testLearnedObstacleSectionCapsLaterLapWithoutSteering() {
  autonomy::Config config = fixtureConfig();
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  config.headingSteeringPerDegree = 0.0F;
  config.accelerationPermillePerSecond = 10000;
  config.decelerationPermillePerSecond = 10000;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  inputs.vision.red = usableBlob(-1000, 5200, 2000, 700);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.learnedRedSections == 0x01 &&
            decision.learnedGreenSections == 0x00,
        "first-lap red pillar must set only coarse section zero red memory");
  inputs.vision.red = {};

  // Finish the physical pass before direction/corner logic may resume.
  setStraightGeometry(inputs);
  inputs.sides[FrontLeft].rangeMm = 200;
  inputs.sides[BackLeft].rangeMm = 300;
  updateFresh(controller, inputs, 249, 0.0F);
  updateFresh(controller, inputs, 398, 0.0F);
  updateFresh(controller, inputs, 399, 0.0F);
  inputs.sides[FrontLeft].rangeMm = 320;
  updateFresh(controller, inputs, 400, 0.0F);
  updateFresh(controller, inputs, 579, 0.0F);
  decision = updateFresh(controller, inputs, 580, 0.0F);
  check(decision.obstacle == ObstacleTarget::None,
        "first-lap obstacle must clear only after live pass geometry");
  decision = updateFresh(controller, inputs, 880, 0.0F);
  check(decision.targetLateralOffsetMm == 0,
        "learned-section fixture must finish recentering before corner tests");

  uint32_t nowMs = 900;
  float headingDegrees = 0.0F;
  for (uint8_t corner = 0; corner < 4; ++corner) {
    setLeftOpening(inputs, 300);
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    check(decision.state == RunState::Cornering,
          "learned-section fixture must enter each physical corner");

    nowMs += 20;
    headingDegrees += 45.0F;
    updateFresh(controller, inputs, nowMs, wrapYaw(headingDegrees));
    nowMs += 20;
    headingDegrees += 45.0F;
    setStraightGeometry(inputs);
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    check(decision.state == RunState::Running &&
              decision.cornerCount == static_cast<uint8_t>(corner + 1),
          "learned-section fixture must count each verified corner exit");

    nowMs += 10;
  }

  setStraightGeometry(inputs);
  nowMs += 100;
  decision = updateFresh(controller, inputs, nowMs, wrapYaw(headingDegrees));
  check(decision.state == RunState::Running && decision.lapCount == 1 &&
            decision.cornerCount == 4 && decision.obstacle == ObstacleTarget::None &&
            decision.targetLateralOffsetMm == 0 && decision.steeringPermille == 0 &&
            decision.speedPermille ==
                controller.config().learnedObstacleSectionSpeedPermille,
        "later pass through a learned section must cap speed without steering from memory");
}

void testCornerExitObservationRecordsNewStraightSection() {
  autonomy::Config config = fixtureConfig();
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  setLeftOpening(inputs);
  updateFresh(controller, inputs, 100, 0.0F);
  updateFresh(controller, inputs, 120, 40.0F);

  setStraightGeometry(inputs);
  inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
  const Decision decision = updateFresh(controller, inputs, 140, 90.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1 &&
            decision.obstacle == ObstacleTarget::RedPassRight &&
            decision.learnedRedSections == 0x02 &&
            decision.learnedGreenSections == 0x00,
        "pillar on a verified corner-exit tick must commit to the new straight section");
}

void testCornerExitStabilityIgnoresCommittedObstacleSteering() {
  autonomy::Config config = fixtureConfig();
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 200;
  config.headingSteeringPerDegree = 0.0F;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  setLeftOpening(inputs);
  updateFresh(controller, inputs, 100, 0.0F);
  updateFresh(controller, inputs, 130, 40.0F);
  setStraightGeometry(inputs);
  inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
  Decision decision = updateFresh(controller, inputs, 150, 90.0F);
  check(decision.state == RunState::Cornering &&
            decision.obstacle == ObstacleTarget::None &&
            decision.targetLateralOffsetMm == 0 &&
            decision.steeringPermille == 0,
        "corner-exit stability must ignore a new pillar and align to live walls");
  decision = updateFresh(controller, inputs, 349, 90.0F);
  check(decision.state == RunState::Cornering &&
            decision.obstacle == ObstacleTarget::None,
        "a pillar in the corner must not become a steering target one millisecond before exit");
  decision = updateFresh(controller, inputs, 350, 90.0F);
  check(decision.state == RunState::Running &&
            decision.obstacle == ObstacleTarget::RedPassRight &&
            decision.learnedRedSections == 0x02,
        "the same fresh pillar may commit to the new straight on the verified exit tick");
}

void testCornerBlindGeometryAndImuAccuracyGrace() {
  {
    autonomy::Config config = fixtureConfig();
    config.cornerSideGeometryGraceMs = 100;
    config.accelerationPermillePerSecond = 10000;
    config.decelerationPermillePerSecond = 10000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    updateFresh(controller, inputs, 100, 0.0F);

    inputs.front.rangeMm = 900;
    Decision decision = updateFresh(controller, inputs, 120, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.speedPermille == controller.config().cornerSpeedPermille,
          "blind-corner fixture must first reach the normal corner-speed command");

    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[BackRight].valid = true;
    decision = updateFresh(controller, inputs, 130, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille == controller.config().cornerBlindSpeedPermille,
          "diagonal singleton side readings must immediately use blind-corner speed");

    for (auto& side : inputs.sides) side.valid = true;
    setStraightGeometry(inputs);
    decision = updateFresh(controller, inputs, 180, 0.0F);
    check(decision.state == RunState::Cornering,
          "side-pair recovery within grace must keep the corner alive");

    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[BackRight].valid = true;
    decision = updateFresh(controller, inputs, 190, 0.0F);
    decision = updateFresh(controller, inputs, 289, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille == controller.config().cornerBlindSpeedPermille,
          "recovered side pairs must reset the next blind-corner grace timer");
    decision = updateFresh(controller, inputs, 290, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultSideGeometryUnavailable) != 0,
          "diagonal singleton side readings beyond corner grace must fault safely");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    config.imuAccuracyGraceMs = 100;
    config.minimumRunImuAccuracy = 2;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 120, 40.0F);
    setStraightGeometry(inputs);
    Decision decision = updateFresh(controller, inputs, 140, 80.0F);
    check(decision.state == RunState::Running,
          "IMU-accuracy fixture must first reach straight running");

    inputs.imu.accuracy = 1;
    decision = updateFresh(controller, inputs, 150, 80.0F);
    check(decision.state == RunState::Running && decision.motionAllowed,
          "brief low IMU accuracy must be tolerated during its grace interval");
    inputs.imu.accuracy = 2;
    decision = updateFresh(controller, inputs, 200, 80.0F);
    check(decision.state == RunState::Running,
          "IMU accuracy recovery within grace must clear the pending fault timer");

    inputs.imu.accuracy = 1;
    updateFresh(controller, inputs, 210, 80.0F);
    decision = updateFresh(controller, inputs, 309, 80.0F);
    check(decision.state == RunState::Running && decision.motionAllowed,
          "recovered IMU accuracy must reset the next grace interval");
    decision = updateFresh(controller, inputs, 310, 80.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultImuUncalibrated) != 0,
          "persistently low IMU accuracy must fault at the configured grace boundary");
  }
}

void testOpenChallengeAcceptsFarRearSensorsWithoutLosingSafety() {
  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.sideSensorPairsAreParallel = false;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.sides[BackLeft].valid = false;
    inputs.sides[BackLeft].rangeStatus = 5;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags & FaultSideTofUnavailable) != 0,
          "a side hardware-failure status must not be treated as an open corner");
  }

  autonomy::Config config = fixtureConfig();
  config.openingConfirmationMs = 100;
  config.requireAllSideSensorsFresh = false;
  config.obstacleAvoidanceEnabled = false;
  config.sideSensorPairsAreParallel = false;
  config.minimumMovingSpeedPermille = 200;
  config.cornerBlindSpeedPermille = 200;
  config.sideElectricalLossGraceMs = 100;
  config.accelerationPermillePerSecond = 10000;
  Controller controller(config);
  Inputs inputs = healthyInputs(0);
  inputs.sides[BackLeft].valid = false;
  inputs.sides[BackLeft].rangeStatus = 1;
  inputs.sides[BackRight].valid = false;
  inputs.sides[BackRight].rangeStatus = 2;

  Decision decision = controller.update(inputs);
  check(decision.state == RunState::WaitingForStart &&
            decision.faultFlags == autonomy::FaultNone,
        "fresh rear optical no-target must not block the Open start gate");
  updateFresh(controller, inputs, 40, 0.0F);
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, 50, 0.0F);
  decision = updateFresh(controller, inputs, 90, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "far rear walls must not prevent a debounced start");

  inputs.vision.red = usableBlob(-1000, 6000, 2000, 900);
  decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.obstacle == ObstacleTarget::None &&
            decision.steeringPermille == 0 &&
            decision.speedPermille >= config.minimumMovingSpeedPermille,
        "Open direction probe must ignore false colour and command useful centred motion");

  inputs.front.rangeMm = 600;
  inputs.sides[FrontLeft].valid = false;
  inputs.sides[FrontLeft].rangeStatus = 2;
  decision = updateFresh(controller, inputs, 110, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "one fresh forward no-target sample must wait for confirmation");
  decision = updateFresh(controller, inputs, 210, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::CounterClockwise,
        "persistent left forward no-target must identify the left opening");

  inputs.sides[BackRight].online = false;
  decision = updateFresh(controller, inputs, 220, 0.0F);
  check(decision.state == RunState::Cornering && decision.motionAllowed &&
            decision.speedPermille == 0,
        "side electrical loss must stop immediately during its recovery grace");
  decision = updateFresh(controller, inputs, 319, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.speedPermille == 0,
        "side electrical loss must remain stopped before its grace boundary");
  decision = updateFresh(controller, inputs, 320, 0.0F);
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultSideTofUnavailable) != 0,
        "persistent electrical loss must latch at the bounded grace boundary");
}

void testOpenChallengeKeepsTurningThroughOpticalNoTarget() {
  autonomy::Config config = fixtureConfig();
  config.requireAllSideSensorsFresh = false;
  config.obstacleAvoidanceEnabled = false;
  config.sideSensorPairsAreParallel = false;
  config.minimumMovingSpeedPermille = 200;
  config.directionProbeSpeedPermille = 320;
  config.cruiseSpeedPermille = 480;
  config.cornerSpeedPermille = 380;
  config.cornerBlindSpeedPermille = 340;
  config.straightSideGeometryGraceMs = 50;
  config.cornerSideGeometryGraceMs = 50;
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 100;
  config.cornerRefractoryMs = 200;
  config.openCornerExitFallbackDegrees = 85.0F;
  config.accelerationPermillePerSecond = 10000;
  config.decelerationPermillePerSecond = 10000;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  Decision decision = updateFresh(controller, inputs, 130, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.speedPermille == config.directionProbeSpeedPermille,
        "Open direction probe must preserve its configured staged speed");

  setLeftOpening(inputs, 600);
  decision = updateFresh(controller, inputs, 150, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.speedPermille > 0 &&
            decision.speedPermille <= config.cornerSpeedPermille &&
            decision.steeringPermille < 0,
        "Open first corner must keep moving within its front-limited staged speed");

  for (auto& side : inputs.sides) {
    side.valid = false;
    side.rangeStatus = 2;
  }
  inputs.front.rangeMm = 416;
  decision = updateFresh(controller, inputs, 180, 40.0F);
  check(decision.state == RunState::Cornering && decision.motionAllowed &&
            decision.speedPermille > 0 &&
            decision.speedPermille <= config.cornerSpeedPermille &&
            decision.steeringPermille < 0,
        "fresh Open optical no-target must not stall a front-limited powered turn");

  decision = updateFresh(controller, inputs, 225, 85.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 0,
        "Open IMU fallback must still honor corner-exit stability time");
  decision = updateFresh(controller, inputs, 324, 85.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 0,
        "Open IMU fallback must not count one millisecond early");
  decision = updateFresh(controller, inputs, 325, 85.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1 &&
            decision.steeringPermille < 0,
        "a stable under-turned Open exit must keep steering toward 90 degrees");

  inputs.front.rangeMm = 221;
  decision = updateFresh(controller, inputs, 420, 85.0F);
  check(decision.state == RunState::Running && decision.motionAllowed &&
            decision.speedPermille == config.minimumMovingSpeedPermille,
        "Open non-emergency front slowdown must use the physical movement floor, not zero");

  inputs.front.rangeMm = 900;
  decision = updateFresh(controller, inputs, 620, 85.0F);
  check(decision.state == RunState::Running && decision.motionAllowed &&
            decision.faultFlags == autonomy::FaultNone &&
            decision.speedPermille >= config.minimumMovingSpeedPermille,
        "persistent fresh side no-target must remain drivable in Open after startup");

  inputs.front.rangeMm = 300;
  decision = updateFresh(controller, inputs, 630, 85.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 1,
        "Open must use a confirmed forward wall for the next corner when side baselines are absent");
}

void testOpenCornerExitUnwindsToCardinalHeadingSymmetrically() {
  for (const bool clockwise : {true, false}) {
    autonomy::Config config = legacyOpenFixtureConfig();
    config.openingConfirmationMs = 0;
    config.cornerRefractoryMs = 0;
    config.cornerRearmStableMs = 0;
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 100;
    config.accelerationPermillePerSecond = 10000;
    config.decelerationPermillePerSecond = 10000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (clockwise) {
      setRightOpening(inputs);
    } else {
      setLeftOpening(inputs);
    }
    const float yawSign = clockwise ? 1.0F : -1.0F;
    const int steeringSign = clockwise ? 1 : -1;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.steeringPermille * steeringSign > 0,
          "both Open directions must enter a powered first corner");

    updateFresh(controller, inputs, 120, 40.0F * yawSign);
    updateFresh(controller, inputs, 130, 70.0F * yawSign);
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    inputs.front.rangeMm = config.cornerRearmFrontClearMm;
    decision = updateFresh(controller, inputs, 150, 88.0F * yawSign);
    check(decision.state == RunState::Cornering &&
              decision.steeringPermille * steeringSign > 0 &&
              std::abs(decision.steeringPermille) <
                  config.cornerSteeringPermille,
          "Open steering must taper toward the run-anchored cardinal");
    decision = updateFresh(controller, inputs, 249, 95.0F * yawSign);
    check(decision.state == RunState::Cornering &&
              decision.steeringPermille * steeringSign < 0,
          "bounded Open overshoot must counter-steer before exit");
    decision = updateFresh(controller, inputs, 250, 95.0F * yawSign);
    check(decision.state == RunState::Running && decision.cornerCount == 1 &&
              decision.steeringPermille * steeringSign < 0 &&
              decision.speedPermille <= config.cornerSpeedPermille,
          "corner exit must keep correcting to the exact cardinal at corner speed");

    setStraightGeometry(inputs);
    const uint8_t outerFront = clockwise ? FrontLeft : FrontRight;
    const uint8_t outerBack = clockwise ? BackLeft : BackRight;
    inputs.sides[outerFront].rangeMm = 500;
    inputs.sides[outerBack].rangeMm = 500;
    decision = updateFresh(controller, inputs, 399, 90.0F * yawSign);
    check(decision.steeringPermille == 0 &&
              decision.speedPermille <= config.cornerSpeedPermille,
          "heading-only recovery must ignore the wall through its last millisecond");
    decision = updateFresh(controller, inputs, 400, 90.0F * yawSign);
    check(decision.steeringPermille == 0,
          "outer-wall blend must begin continuously from zero");
    decision = updateFresh(controller, inputs, 525, 90.0F * yawSign);
    check(decision.steeringPermille == 0 &&
              decision.wallAngleErrorMm == 0.0F &&
              !decision.outerWallUsable,
          "diagonal Open ranges must remain diagnostic-only after the recovery window");
    decision = updateFresh(controller, inputs, 650, 90.0F * yawSign);
    check(decision.steeringPermille == 0 &&
              decision.speedPermille <= config.directionProbeSpeedPermille,
          "heading-only Open must not restore uncalibrated wall steering");
  }
}

void testOpenRepeatedCornersStayAnchoredToRunHeading() {
  autonomy::Config config = fixtureConfig();
  config.requireAllSideSensorsFresh = false;
  config.obstacleAvoidanceEnabled = false;
  config.sideSensorPairsAreParallel = false;
  config.clockwiseYawSign = 1;
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  config.cornerRefractoryMs = 0;
  config.openCornerExitFallbackDegrees = 88.0F;
  config.openCornerExitMaximumDegrees = 100.0F;
  config.openCornerTaperStartDegrees = 55.0F;
  config.openCornerMinimumApproachSteerPermille = 220;
  config.headingSteeringPerDegree = 14.0F;
  config.maximumHeadingCorrectionPermille = 520;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  const auto removeSideTargets = [](Inputs& sample) {
    for (auto& side : sample.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    sample.front.rangeMm = 900;
  };

  setRightOpening(inputs);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::Clockwise,
        "cardinal fixture must enter the first clockwise corner");
  removeSideTargets(inputs);
  updateFresh(controller, inputs, 120, 45.0F);
  decision = updateFresh(controller, inputs, 140, 88.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1 &&
            decision.steeringPermille >= 200,
        "the first Open corner must retain physical steering authority at the 88-degree exit threshold");

  // Simulate a small residual under-turn at the next corner entry. The second
  // target must remain the run-start +180 degree cardinal, not 85+90=175.
  inputs.front.rangeMm = 300;
  decision = updateFresh(controller, inputs, 150, 85.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 1,
        "a close front wall must enter the second Open corner");
  inputs.front.rangeMm = 900;
  updateFresh(controller, inputs, 170, 130.0F);
  setStraightGeometry(inputs);
  decision = updateFresh(controller, inputs, 190, 173.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 1 &&
            decision.cornerProgressDegrees < 88.0F &&
            decision.steeringPermille > 0,
        "reacquired walls must not count an 88-degree local turn that is still short of the absolute 180-degree cardinal");
  decision = updateFresh(controller, inputs, 200, 178.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 2,
        "the second corner must count at the run-anchored 178-degree fallback");

  // Repeat the same residual before corner three, crossing the raw IMU wrap.
  inputs.front.rangeMm = 300;
  decision = updateFresh(controller, inputs, 210, 175.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 2,
        "a close front wall must enter the third Open corner");
  inputs.front.rangeMm = 900;
  updateFresh(controller, inputs, 230, -140.0F);  // unwrapped +220
  decision = updateFresh(controller, inputs, 250, -97.0F);  // unwrapped +263
  check(decision.state == RunState::Cornering && decision.cornerCount == 2 &&
            decision.cornerProgressDegrees < 88.0F &&
            decision.steeringPermille > 0,
        "the third corner must not accumulate the prior five-degree under-turn across yaw wrap");
  decision = updateFresh(controller, inputs, 260, -92.0F);  // unwrapped +268
  check(decision.state == RunState::Running && decision.cornerCount == 3,
        "the third corner must count only at the run-anchored 268-degree fallback");
}

void testAllOpenCornersStayAnchoredAcrossThreeLaps() {
  for (const bool clockwise : {true, false}) {
    autonomy::Config config = legacyOpenFixtureConfig();
    // Only physical turn/confirmation durations are collapsed; the production
    // 250 ms + 100 ms clear-front rearm remains active on all later corners.
    config.openingConfirmationMs = 0;
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);

    const float yawSign = clockwise ? 1.0F : -1.0F;
    float unwrappedYaw = 0.0F;
    uint32_t nowMs = 100;
    Decision decision;
    for (uint8_t corner = 0; corner < 12; ++corner) {
      if (corner == 0) {
        if (clockwise) {
          setRightOpening(inputs, 600);
        } else {
          setLeftOpening(inputs, 600);
        }
      } else {
        setStraightGeometry(inputs, config.cornerRearmFrontClearMm);
        updateFresh(controller, inputs, nowMs,
                    wrapYaw(unwrappedYaw));
        nowMs += config.cornerRearmStableMs;
        decision = updateFresh(controller, inputs, nowMs,
                               wrapYaw(unwrappedYaw));
        check(decision.cornerDetectionArmed,
              "every production straight must rearm after stable clear front");
        nowMs += 10;
        inputs.front.rangeMm = config.frontCornerTriggerMm;
      }

      decision = updateFresh(controller, inputs, nowMs,
                             wrapYaw(unwrappedYaw));
      check(decision.state == RunState::Cornering &&
                decision.cornerCount == corner,
            "each armed production wall must start exactly one Open corner");
      const float expectedTarget =
          yawSign * 90.0F * static_cast<float>(corner + 1U);
      checkNear(decision.cornerTargetHeadingDegrees, expectedTarget, 0.001F,
                "every corner target must be a run-start cardinal, never the prior exit heading");

      nowMs += 20;
      unwrappedYaw += yawSign * 45.0F;
      updateFresh(controller, inputs, nowMs, wrapYaw(unwrappedYaw));
      nowMs += 20;
      unwrappedYaw = expectedTarget;
      setStraightGeometry(inputs, config.cornerRearmFrontClearMm);
      decision = updateFresh(controller, inputs, nowMs,
                             wrapYaw(unwrappedYaw));
      check(decision.cornerCount == static_cast<uint8_t>(corner + 1U),
            "a cardinal exit must count one and only one corner");

      // Begin the next production rearm exactly 250 ms after this exit.
      nowMs += config.cornerRefractoryMs;
    }
    check(decision.state == RunState::FinishDelay &&
              decision.cornerCount == 12 && decision.lapCount == 3,
          "both directions must reach the three-lap finish on twelve anchored cardinals");
  }
}

void testFinalOpenCornerKeepsApproachSteeringInFinishDelay() {
  autonomy::Config config = legacyOpenFixtureConfig();
  config.openingConfirmationMs = 0;
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  config.cornerRefractoryMs = 0;
  config.cornerRearmStableMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  float headingDegrees = 0.0F;
  uint32_t nowMs = 100;
  setRightOpening(inputs, 600);
  Decision decision;
  for (uint8_t corner = 0; corner < 12; ++corner) {
    if (corner > 0) {
      inputs.front.rangeMm = config.frontCornerTriggerMm;
    }
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    check(decision.state == RunState::Cornering,
          "final-corner fixture must enter every clockwise corner");

    const float target = 90.0F * static_cast<float>(corner + 1U);
    nowMs += 20;
    headingDegrees += 45.0F;
    updateFresh(controller, inputs, nowMs, wrapYaw(headingDegrees));
    nowMs += 20;
    headingDegrees = corner == 11 ? target - 2.0F : target;
    setStraightGeometry(inputs, config.cornerRearmFrontClearMm);
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    if (corner < 11) {
      check(decision.state == RunState::Running,
            "pre-finish Open cardinal must return to Running");
      nowMs += 10;
    }
  }

  check(decision.state == RunState::FinishDelay &&
            decision.cornerCount == 12 &&
            decision.steeringPermille >=
                config.openCornerMinimumApproachSteerPermille,
        "an 88-degree twelfth exit must carry at least 250 right steer into FinishDelay");
  decision = updateFresh(controller, inputs, nowMs + 149U,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::FinishDelay &&
            decision.steeringPermille >=
                config.openCornerMinimumApproachSteerPermille,
        "final heading-only recovery must retain the 250 approach floor through 149 ms");
}

void testOpenCornerExitStabilityRejectsActiveOvershoot() {
  autonomy::Config config = fixtureConfig();
  config.requireAllSideSensorsFresh = false;
  config.obstacleAvoidanceEnabled = false;
  config.sideSensorPairsAreParallel = false;
  config.clockwiseYawSign = 1;
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 100;
  config.openCornerExitFallbackDegrees = 88.0F;
  config.openCornerExitMaximumDegrees = 100.0F;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  setRightOpening(inputs);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::Cornering,
        "bounded-exit fixture must enter a clockwise corner");
  updateFresh(controller, inputs, 120, 45.0F);
  decision = updateFresh(controller, inputs, 150, 88.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 0,
        "entering the Open exit window must start, not skip, stability time");
  decision = updateFresh(controller, inputs, 250, 101.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 0 &&
            decision.steeringPermille < 0,
        "leaving the exit window while still rotating must reset readiness and counter-steer");
  decision = updateFresh(controller, inputs, 260, 99.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 0,
        "re-entering the exit window must begin a fresh stability interval");
  decision = updateFresh(controller, inputs, 359, 99.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 0,
        "the restarted stability interval must not count one millisecond early");
  decision = updateFresh(controller, inputs, 360, 99.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1,
        "a bounded stable heading may count exactly one Open corner");
}

void testOpenSideElectricalLossUsesStoppedBoundedGrace() {
  const auto openConfig = []() {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.obstacleAvoidanceEnabled = false;
    config.sideSensorPairsAreParallel = false;
    config.sideElectricalLossGraceMs = 100;
    config.minimumMovingSpeedPermille = 200;
    config.accelerationPermillePerSecond = 10000;
    return config;
  };

  {
    Controller controller(openConfig());
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[BackRight].online = false;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed && decision.speedPermille == 0,
          "a side I2C/module loss must stop drive immediately inside grace");
    inputs.sides[BackRight].online = true;
    decision = updateFresh(controller, inputs, 150, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille >=
                  controller.config().minimumMovingSpeedPermille,
          "a brief recovered side electrical loss must resume without latching");
  }

  {
    Controller controller(openConfig());
    Inputs inputs;
    startController(controller, inputs);
    inputs.nowMs = 301;
    inputs.imu.updatedAtMs = 301;
    inputs.front.updatedAtMs = 301;
    inputs.vision.updatedAtMs = 301;
    for (auto& side : inputs.sides) side.updatedAtMs = 301;
    inputs.sides[BackRight].updatedAtMs = 0;
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "stale side telemetry must stop immediately while grace is pending");

    inputs.nowMs = 400;
    inputs.imu.updatedAtMs = 400;
    inputs.front.updatedAtMs = 400;
    inputs.vision.updatedAtMs = 400;
    for (uint8_t i = 0; i < autonomy::SideSensorCount - 1; ++i) {
      inputs.sides[i].updatedAtMs = 400;
    }
    decision = controller.update(inputs);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "persistent stale side telemetry must remain stopped before timeout");

    inputs.nowMs = 401;
    inputs.imu.updatedAtMs = 401;
    inputs.front.updatedAtMs = 401;
    inputs.vision.updatedAtMs = 401;
    for (uint8_t i = 0; i < autonomy::SideSensorCount - 1; ++i) {
      inputs.sides[i].updatedAtMs = 401;
    }
    decision = controller.update(inputs);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultSideTofStale) != 0 &&
              !decision.motionAllowed,
          "persistent stale side telemetry must latch at the grace boundary");
  }
}

void testStartupGeometryUsesAnyCompleteSafeSolution() {
  const auto startupConfig = []() {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.sideSensorPairsAreParallel = false;
    config.obstacleAvoidanceEnabled = false;
    return config;
  };

  {
    Controller controller(startupConfig());
    Inputs inputs = healthyInputs(0);
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone,
          "one complete left pair plus live right no-target samples must arm startup");
  }

  {
    Controller controller(startupConfig());
    Inputs inputs = healthyInputs(0);
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    inputs.sides[BackRight].i2cStatus = 1;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags & FaultSideTofUnavailable) != 0,
          "a complete opposite pair must not hide a side-sensor I2C failure");
  }

  {
    Controller controller(startupConfig());
    Inputs inputs = healthyInputs(0);
    inputs.nowMs = 301;
    inputs.imu.updatedAtMs = 301;
    inputs.front.updatedAtMs = 301;
    inputs.vision.updatedAtMs = 301;
    for (auto& side : inputs.sides) side.updatedAtMs = 301;
    inputs.sides[BackRight].updatedAtMs = 0;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags & autonomy::FaultSideTofStale) != 0,
          "a complete geometric pair must not hide one stale opposite-side module");
  }

  {
    Controller controller(startupConfig());
    Inputs inputs = healthyInputs(0);
    inputs.sides[BackLeft].valid = false;
    inputs.sides[BackLeft].rangeStatus = 2;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone,
          "Open must arm from fresh optical samples even when no numeric side geometry is available");
  }

  {
    const uint8_t opticalNoTargetStatuses[] = {1, 2, 4, 6, 7};
    for (const uint8_t status : opticalNoTargetStatuses) {
      Controller controller(startupConfig());
      Inputs inputs = healthyInputs(0);
      for (auto& side : inputs.sides) {
        side.valid = false;
        side.rangeStatus = status;
      }
      const Decision decision = controller.update(inputs);
      check(decision.state == RunState::WaitingForStart &&
                decision.faultFlags == autonomy::FaultNone,
            "all four fresh Open optical no-target statuses must still arm");
    }
  }

  {
    autonomy::Config config = startupConfig();
    config.obstacleAvoidanceEnabled = true;
    config.requireVisionForMotion = true;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.sides[BackLeft].valid = false;
    inputs.sides[BackLeft].rangeStatus = 2;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.wallLateralOffsetMm == 0.0F &&
              decision.wallAngleErrorMm == 0.0F,
          "Obstacle must accept a local FL+BR cross-pair only as zero-authority two-sided geometry");
  }

  {
    autonomy::Config config = startupConfig();
    config.obstacleAvoidanceEnabled = true;
    config.requireVisionForMotion = true;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.wallLateralOffsetMm == 0.0F &&
              decision.wallAngleErrorMm == 0.0F,
          "Obstacle must mirror the live BL+FR cross-pair without inventing steering");
  }

  {
    autonomy::Config config = startupConfig();
    config.obstacleAvoidanceEnabled = true;
    config.requireVisionForMotion = true;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0,
          "Obstacle must still block when no local numeric ray exists on one side");
  }

  {
    autonomy::Config config = startupConfig();
    config.obstacleAvoidanceEnabled = true;
    config.requireVisionForMotion = true;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    inputs.sides[BackLeft].rangeMm =
        static_cast<uint16_t>(config.openingReferenceMaximumMm + 1.0F);
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0,
          "Obstacle must not treat a non-local cross-pair range as nearby safety geometry");
  }

  {
    autonomy::Config config = startupConfig();
    config.obstacleAvoidanceEnabled = true;
    config.requireVisionForMotion = true;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = config.directionProbeAbortFrontMm;
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0,
          "cross-pair startup must block at the front abort boundary");

    inputs.front.rangeMm = config.directionProbeAbortFrontMm + 1U;
    decision = updateFresh(controller, inputs, 1, 0.0F);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone,
          "cross-pair startup may arm one millimetre beyond the clear-front boundary");
  }

  {
    autonomy::Config config = startupConfig();
    config.obstacleAvoidanceEnabled = true;
    config.requireVisionForMotion = true;
    config.straightSideGeometryGraceMs = 600;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart,
          "cross-pair moving-loss fixture must arm while stopped");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    decision = updateFresh(controller, inputs, 90, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed,
          "cross-pair startup must enter one bounded direction probe");
    inputs.buttonPressed = false;
    decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone,
          "cross-pair must begin, not skip, the moving geometry grace");
    decision = updateFresh(controller, inputs, 699, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone,
          "cross-pair moving geometry loss must remain recoverable at 599 milliseconds");
    decision = updateFresh(controller, inputs, 700, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0 &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "persistent cross-pair-only geometry must stop at the 600 millisecond boundary");
  }
}

void testR23BoundedCalibrationCrossGeometry() {
  const auto setBlFrCross = [](Inputs& inputs) {
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[BackLeft].valid = true;
    inputs.sides[BackLeft].rangeMm = 850;
    inputs.sides[BackLeft].rangeStatus = 0;
    inputs.sides[FrontRight].valid = true;
    inputs.sides[FrontRight].rangeMm = 420;
    inputs.sides[FrontRight].rangeStatus = 0;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
  };
  const auto calibrationConfig = []() {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.stopAfterFirstVerifiedObstaclePass = true;
    config.maxRunMs = 5000;
    config.straightSideGeometryGraceMs = 2000;
    config.obstacleSideClearanceGuardMm = 200;
    return config;
  };
  const auto pressStart = [](Controller& controller, Inputs& inputs) {
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart,
          "R23C cross fixture must arm while stopped");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    decision = updateFresh(controller, inputs, 90, 0.0F);
    inputs.buttonPressed = false;
    return decision;
  };

  {
    Controller controller(calibrationConfig());
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 1850;
    setBlFrCross(inputs);
    check(controller.update(inputs).state == RunState::WaitingForStart,
          "a valid clear front must arm the zero-authority BL+FR startup gate");
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.startupFrontClearRecent = true;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.wallLateralOffsetMm == 0.0F &&
              decision.wallAngleErrorMm == 0.0F,
          "one board-filtered front status-7 sample must not erase a recent clear startup or invent wall steering");

    Controller noHistory(calibrationConfig());
    inputs = healthyInputs(0);
    setBlFrCross(inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.startupFrontClearRecent = false;
    decision = noHistory.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0,
          "front status 7 without prior clear evidence must not arm a cross pair");

    Controller closeFront(calibrationConfig());
    inputs = healthyInputs(0);
    setBlFrCross(inputs);
    inputs.front.rangeMm = closeFront.config().directionProbeAbortFrontMm;
    inputs.startupFrontClearRecent = true;
    decision = closeFront.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0,
          "a current front reading at the abort boundary must override cached clear evidence");
  }

  {
    Controller controller(calibrationConfig());
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 1850;
    setBlFrCross(inputs);
    inputs.vision.red = usableBlob(3500, 5200, 1800, 840);
    Decision decision = pressStart(controller, inputs);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "the start-edge update must remain stopped before the cross guard is evaluated as moving");
    decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.steeringPermille > 0 &&
              decision.wallLateralOffsetMm == 0.0F &&
              decision.wallAngleErrorMm == 0.0F,
          "R23C must use close-red camera steering while BL+FR retain zero wall authority");
    decision = updateFresh(controller, inputs, 700, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0,
          "a close locked target and intact BL+FR guard must remain boundedly drivable past servo settle");

    inputs.sides[BackLeft].valid = false;
    inputs.sides[BackLeft].rangeStatus = 2;
    decision = updateFresh(controller, inputs, 701, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille == 0,
          "loss of the calibration cross guard must stop immediately inside fault grace");
    decision = updateFresh(controller, inputs, 2700, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille == 0,
          "R24C cross-guard loss must remain stopped one millisecond before its calibration-only timeout");
    decision = updateFresh(controller, inputs, 2701, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0 &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R24C cross-guard loss must latch at the 2000 millisecond stationary-proof boundary");
  }

  {
    Controller controller(calibrationConfig());
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 1850;
    setBlFrCross(inputs);
    Decision decision = pressStart(controller, inputs);
    decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "cross-only calibration without a close target must remain stopped");
    decision = updateFresh(controller, inputs, 2099, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "targetless cross-only calibration must stay stopped throughout recovery grace");
    decision = updateFresh(controller, inputs, 2100, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0,
          "targetless R24C calibration must fault at the bounded 2000 millisecond timeout");
  }

  {
    Controller controller(calibrationConfig());
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 1850;
    setBlFrCross(inputs);
    inputs.vision.red = usableBlob(3500, 5200, 1800, 840);
    pressStart(controller, inputs);
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.speedPermille > 0,
          "cross-pair pass fixture must commit and approach red");
    inputs.vision.red = {};
    updateFresh(controller, inputs, 110, 0.0F);
    inputs.sides[BackLeft].rangeMm = 700;
    decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.state != RunState::Finished &&
              decision.obstacle == ObstacleTarget::RedPassRight,
          "close-loss-side transition must begin the strict confirmation interval");
    decision = updateFresh(controller, inputs, 579, 0.0F);
    check(decision.state != RunState::Finished,
          "R23C cross pass must not finish one millisecond early");
    decision = updateFresh(controller, inputs, 580, 0.0F);
    check(decision.state == RunState::Finished &&
              decision.obstacle == ObstacleTarget::None &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R23C cross pass must auto-stop only after close, loss, side transition, and stability proof");
  }

  {
    // Replay the optical end of the physical red run. Once only the two rear
    // rays remain, motor power must drop immediately. The longer R24C latch
    // interval allows a strict FL/BL transition to settle while stationary;
    // a one-frame both-open flicker still cannot count as proof.
    Controller controller(calibrationConfig());
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 1850;
    setBlFrCross(inputs);
    inputs.vision.red = usableBlob(3500, 5200, 1800, 840);
    pressStart(controller, inputs);
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              decision.speedPermille > 0,
          "R24C physical replay must commit the close red pillar");

    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeStatus = 0;
    inputs.sides[FrontLeft].rangeMm = 54;
    inputs.sides[BackLeft].valid = false;
    inputs.sides[BackLeft].rangeStatus = 2;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    inputs.sides[BackRight].valid = true;
    inputs.sides[BackRight].rangeStatus = 0;
    inputs.sides[BackRight].rangeMm = 452;
    updateFresh(controller, inputs, 300, 0.0F);

    inputs.vision.red = {};
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[BackLeft].valid = true;
    inputs.sides[BackLeft].rangeStatus = 0;
    inputs.sides[BackLeft].rangeMm = 963;
    decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "R24C rear-ray geometry must remove drive immediately without pre-empting strict pass proof");
    decision = updateFresh(controller, inputs, 550, 0.0F);
    check(decision.state != RunState::Finished,
          "fresh loss and transition must begin rather than skip confirmation");

    inputs.sides[BackLeft].valid = false;
    inputs.sides[BackLeft].rangeStatus = 2;
    decision = updateFresh(controller, inputs, 600, 0.0F);
    check(decision.state != RunState::Finished,
          "two blind selected rays must reset strict pass confirmation");
    inputs.sides[BackLeft].valid = true;
    inputs.sides[BackLeft].rangeStatus = 0;
    inputs.sides[BackLeft].rangeMm = 977;
    decision = updateFresh(controller, inputs, 700, 0.0F);
    check(decision.state != RunState::Finished,
          "recovered selected-side proof must restart its full interval");
    decision = updateFresh(controller, inputs, 879, 0.0F);
    check(decision.state != RunState::Finished,
          "stationary recovered proof must not finish one millisecond early");
    decision = updateFresh(controller, inputs, 880, 0.0F);
    check(decision.state == RunState::Finished &&
              decision.obstacle == ObstacleTarget::None &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R24C must finish from strict stationary red pass evidence before its 2000 millisecond fault boundary");
  }
}

void testR24CalibrationClosePassNeverCounterSteersTowardPillar() {
  for (const bool redPillar : {true, false}) {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.stopAfterFirstVerifiedObstaclePass = true;
    config.maxRunMs = 5000;
    config.straightSideGeometryGraceMs = 2000;
    // Isolate the heading term that helped reverse R23C toward the pillar.
    config.obstacleImageSteeringPerUnit = 0.0F;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (redPillar) {
      inputs.vision.red = usableBlob(-4608, 6360, 1800, 840);
    } else {
      inputs.vision.green = usableBlob(4608, 6360, 1800, 840);
    }
    const float escapeYaw = redPillar ? 13.13F : -13.13F;
    Decision decision = updateFresh(controller, inputs, 100, escapeYaw);
    check(decision.obstacle ==
                  (redPillar ? ObstacleTarget::RedPassRight
                             : ObstacleTarget::GreenPassLeft) &&
              decision.steeringPermille == 0,
          "R24C must clamp heading counter-steer toward a still-visible close pillar to centre");

    inputs.vision.red = {};
    inputs.vision.green = {};
    decision = updateFresh(controller, inputs, 101, escapeYaw);
    check((redPillar && decision.steeringPermille >= 0) ||
              (!redPillar && decision.steeringPermille <= 0),
          "a one-frame camera blank must not reverse toward the remembered pillar");
    decision = updateFresh(controller, inputs, 250, escapeYaw);
    check((redPillar && decision.steeringPermille >= 0) ||
              (!redPillar && decision.steeringPermille <= 0),
          "the non-reversal hold must cover the full 149 millisecond blank window");
  }

  // Production remains unchanged until the bounded red and green maneuvers
  // have physical evidence. With the calibration switch off, ordinary heading
  // recovery is still free to counter-steer.
  for (const bool redPillar : {true, false}) {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.obstacleImageSteeringPerUnit = 0.0F;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (redPillar) {
      inputs.vision.red = usableBlob(-4608, 6360, 1800, 840);
    } else {
      inputs.vision.green = usableBlob(4608, 6360, 1800, 840);
    }
    const Decision decision = updateFresh(
        controller, inputs, 100, redPillar ? 13.13F : -13.13F);
    check((redPillar && decision.steeringPermille < 0) ||
              (!redPillar && decision.steeringPermille > 0),
          "production Obstacle must not silently inherit the unproven R24C steering clamp");
  }
}

void testR27CalibrationUsesMeasuredCenteredEscapeHeading() {
  const auto r27Config = []() {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.stopAfterFirstVerifiedObstaclePass = true;
    config.maxRunMs = 5000;
    config.straightSideGeometryGraceMs = 2000;
    config.obstacleDesiredImageOffset = 7000;
    config.obstacleSideClearanceGuardMm = 200;
    config.obstacleBodyClearOppositeSideGuardMm = 140;
    config.obstacleSinglePassBodyClearMs = 500;
    config.obstacleCenteredEscapeInitialAbsX = 4000;
    config.obstacleCenteredEscapeHeadingDegrees = 22.0F;
    config.obstacleCenteredEscapeDeadlineMs = 2500;
    config.obstacleCenteredEscapeHeadingStableMs = 80;
    return config;
  };

  const autonomy::Config production =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
  check(production.obstacleCenteredEscapeInitialAbsX == 0 &&
            production.obstacleCenteredEscapeHeadingDegrees == 0.0F &&
            production.obstacleCenteredEscapeDeadlineMs == 0U &&
            production.obstacleCenteredEscapeHeadingStableMs == 0U,
        "R27 measured escape must remain disabled in production before physical proof");

  {
    autonomy::Config invalid = r27Config();
    invalid.obstacleCenteredEscapeDeadlineMs = 0;
    Controller controller(invalid);
    Inputs inputs = healthyInputs(0);
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags & FaultConfiguration) != 0U &&
              !decision.motionAllowed,
          "R27 incomplete centred-escape configuration must fail closed");
  }

  // Both colours hold equal mirrored steering until BNO proves 22 degrees,
  // then centre instead of steering back toward the pillar.
  for (const bool redPillar : {true, false}) {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (redPillar) {
      inputs.vision.red = usableBlob(-2038, 5439, 1800, 830);
    } else {
      inputs.vision.green = usableBlob(2038, 5439, 1800, 830);
    }
    const int16_t expectedSteering =
        redPillar ? config.obstacleMaximumImageSteeringPermille
                  : static_cast<int16_t>(
                        -config.obstacleMaximumImageSteeringPermille);
    const float yawSign = redPillar ? 1.0F : -1.0F;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle ==
                  (redPillar ? ObstacleTarget::RedPassRight
                             : ObstacleTarget::GreenPassLeft) &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.speedPermille <=
                  config.obstacleSideClearanceSpeedPermille &&
              decision.steeringPermille == expectedSteering,
          "R27 centred red and green must begin equal mirrored measured escapes");
    decision = updateFresh(controller, inputs, 300, yawSign * 21.9F);
    check(decision.speedPermille > 0 &&
              decision.speedPermille <=
                  config.obstacleSideClearanceSpeedPermille &&
              decision.steeringPermille == expectedSteering,
          "R27 must hold full mirrored steering one tenth degree before target");
    decision = updateFresh(controller, inputs, 320, yawSign * 22.0F);
    check(decision.motionAllowed && decision.speedPermille > 0 &&
              decision.speedPermille <=
                  config.obstacleSideClearanceSpeedPermille &&
              decision.steeringPermille == expectedSteering,
          "R27 must not centre from only one target-sized BNO sample");
    decision = updateFresh(controller, inputs, 400, yawSign * 22.0F);
    check(decision.motionAllowed && decision.speedPermille > 0 &&
              decision.speedPermille <=
                  config.obstacleSideClearanceSpeedPermille &&
              decision.steeringPermille == 0,
          "R27 must centre only after two fresh BNO samples prove the escape");
  }

  // A single legal-sized BNO spike must not latch turn completion, and a
  // repeated controller tick over the same IMU acquisition is not a second
  // sample. Require two consecutive, distinct target samples for both colours.
  for (const bool redPillar : {true, false}) {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (redPillar) {
      inputs.vision.red = usableBlob(-2038, 5439, 1800, 830);
    } else {
      inputs.vision.green = usableBlob(2038, 5439, 1800, 830);
    }
    const int16_t expectedSteering =
        redPillar ? config.obstacleMaximumImageSteeringPermille
                  : static_cast<int16_t>(
                        -config.obstacleMaximumImageSteeringPermille);
    const float yawSign = redPillar ? 1.0F : -1.0F;
    updateFresh(controller, inputs, 100, 0.0F);
    Decision decision =
        updateFresh(controller, inputs, 120, yawSign * 22.0F);
    check(decision.speedPermille > 0 &&
              decision.steeringPermille == expectedSteering,
          "R27 first target BNO sample must retain escape steering");
    inputs.nowMs = 121;
    decision = controller.update(inputs);
    check(decision.speedPermille > 0 &&
              decision.steeringPermille == expectedSteering,
          "R27 repeated control tick must not count one IMU sample twice");
    decision = updateFresh(controller, inputs, 140, 0.0F);
    check(decision.speedPermille > 0 &&
              decision.steeringPermille == expectedSteering,
          "R27 return below target must reset heading confirmation");
    decision = updateFresh(controller, inputs, 160, yawSign * 22.0F);
    check(decision.speedPermille > 0 &&
              decision.steeringPermille == expectedSteering,
          "R27 first clean target sample must still retain escape steering");
    decision = updateFresh(controller, inputs, 239, yawSign * 22.0F);
    check(decision.speedPermille > 0 &&
              decision.steeringPermille == expectedSteering,
          "R27 heading confirmation must not complete one millisecond early");
    decision = updateFresh(controller, inputs, 240, yawSign * 22.0F);
    check(decision.speedPermille > 0 && decision.steeringPermille == 0,
          "R27 second distinct clean target sample may centre steering");
  }

  // A qualifying heading and a pillar-side guard breach on the same sample
  // must fail closed before turn completion is considered.
  for (const bool redPillar : {true, false}) {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (redPillar) {
      inputs.vision.red = usableBlob(-2038, 5439, 1800, 830);
    } else {
      inputs.vision.green = usableBlob(2038, 5439, 1800, 830);
    }
    const float yawSign = redPillar ? 1.0F : -1.0F;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 200, yawSign * 22.0F);
    const uint8_t pillarSide = redPillar ? FrontLeft : FrontRight;
    inputs.sides[pillarSide].rangeMm =
        config.obstacleSideClearanceGuardMm;
    Decision decision =
        updateFresh(controller, inputs, 280, yawSign * 22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0 &&
              decision.steeringPermille == 0,
          "R27 same-sample guard breach must beat heading completion");
    inputs.sides[pillarSide].rangeMm = 400;
    decision = updateFresh(controller, inputs, 400, yawSign * 24.0F);
    check(decision.speedPermille == 0 && decision.steeringPermille == 0,
          "R27 guard breach must remain latched after clearance recovers");
  }

  // A stable target may complete exactly at the bounded deadline, but never
  // one millisecond after it.
  {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.green = usableBlob(2038, 5439, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 2520, -22.0F);
    const Decision atDeadline =
        updateFresh(controller, inputs, 2600, -22.0F);
    check(atDeadline.speedPermille > 0 &&
              atDeadline.steeringPermille == 0,
          "R27 stable heading may complete at its exact deadline");
  }
  {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.green = usableBlob(2038, 5439, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 2521, -22.0F);
    const Decision afterDeadline =
        updateFresh(controller, inputs, 2601, -22.0F);
    check(afterDeadline.speedPermille == 0 &&
              afterDeadline.steeringPermille == 0,
          "R27 stable heading must not complete after its deadline");
  }

  // Replay the exact R26 green image/yaw progression before contact. R26
  // cancelled at -10.2 degrees; R27 must hold -500 through the last blob.
  {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    const int16_t greenX[] = {2038, 2257, 2884, 3762, 4953, 6176,
                              7210, 7837, 8119, 8276, 8464, 8558,
                              8621, 8746, 8871, 9060, 9342};
    const uint16_t greenBottom[] = {5439, 5523, 5649, 5774, 6025, 6318,
                                    6653, 7071, 7448, 7908, 8410, 8996,
                                    9623, 9958, 10000, 10000, 10000};
    const float yawDegrees[] = {0.0F, -0.9F, -2.8F, -5.0F, -7.2F, -9.1F,
                                -10.2F, -10.5F, -10.5F, -10.4F, -10.4F,
                                -10.3F, -10.2F, -10.1F, -10.0F, -9.9F,
                                -9.9F};
    Decision decision;
    for (size_t index = 0; index < sizeof(greenX) / sizeof(greenX[0]); ++index) {
      inputs.vision.green =
          usableBlob(greenX[index], greenBottom[index], 1800, 830);
      decision = updateFresh(controller, inputs,
                             100U + static_cast<uint32_t>(index) * 100U,
                             yawDegrees[index]);
      check(decision.state == RunState::DirectionProbe &&
                decision.obstacle == ObstacleTarget::GreenPassLeft &&
                decision.motionAllowed && decision.speedPermille > 0 &&
                decision.speedPermille <=
                    config.obstacleSideClearanceSpeedPermille &&
                decision.steeringPermille ==
                    -config.obstacleMaximumImageSteeringPermille,
            "R27 must not repeat the R26 image and heading cancellation trace");
    }

    inputs.vision.green = {};
    decision = updateFresh(controller, inputs, 2599, -9.9F);
    check(decision.speedPermille > 0 &&
              decision.steeringPermille ==
                  -config.obstacleMaximumImageSteeringPermille,
          "R27 must retain the escape through the last millisecond before its deadline");
    decision = updateFresh(controller, inputs, 2600, -9.9F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::GreenPassLeft &&
              decision.motionAllowed && decision.speedPermille == 0 &&
              decision.steeringPermille == 0,
          "R27 must stop rather than continue a failed escape past 2500 ms");
  }

  // The old FR=23 contact geometry could imitate a pass. Require the measured
  // turn and a clear selected-front side ray before ordinary proof may start.
  {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[FrontRight].rangeMm = 220;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeMm = 0;
    inputs.sides[BackRight].rangeStatus = 2;
    inputs.vision.green = usableBlob(2038, 6000, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.vision.green = {};
    updateFresh(controller, inputs, 110, -5.0F);
    inputs.sides[FrontRight].rangeMm = 23;
    inputs.sides[BackRight].valid = true;
    inputs.sides[BackRight].rangeStatus = 0;
    inputs.sides[BackRight].rangeMm = 767;
    Decision decision = updateFresh(controller, inputs, 400, -22.0F);
    decision = updateFresh(controller, inputs, 600, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::GreenPassLeft &&
              decision.speedPermille == 0 &&
              decision.steeringPermille == 0,
          "R27 contact-close selected front must fail and stop the escape");

    inputs.sides[FrontRight].rangeMm = 360;
    inputs.sides[BackRight].rangeMm = 340;
    decision = updateFresh(controller, inputs, 1000, -26.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::GreenPassLeft &&
              decision.speedPermille == 0 &&
              decision.steeringPermille == 0,
          "R27 later contact-induced yaw and clearance recovery must not reauthorize motion");
  }

  // A clean measured escape can still use the existing strict pair transition
  // and bounded body-clear tail after the pillar-side front is genuinely clear.
  {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[FrontRight].rangeMm = 220;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeMm = 0;
    inputs.sides[BackRight].rangeStatus = 2;
    inputs.vision.green = usableBlob(2038, 6000, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.vision.green = {};
    updateFresh(controller, inputs, 110, -5.0F);
    inputs.sides[FrontRight].rangeMm = 360;
    inputs.sides[BackRight].valid = true;
    inputs.sides[BackRight].rangeStatus = 0;
    inputs.sides[BackRight].rangeMm = 340;
    updateFresh(controller, inputs, 390, -22.0F);
    Decision decision = updateFresh(controller, inputs, 470, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::GreenPassLeft &&
              decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R27 clean measured escape must hold its diagonal while proof settles");
    decision = updateFresh(controller, inputs, 649, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::GreenPassLeft,
          "R27 strict pass confirmation must not complete one millisecond early");
    decision = updateFresh(controller, inputs, 650, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::GreenPassLeft &&
              decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R27 proven escape may enter the existing bounded body-clear phase");
  }

  // Either side crossing the 200 mm envelope before the measured turn is a
  // failed centred attempt. Free-side proximity must never reverse steering
  // toward the pillar, and pillar-side contact risk must never be called a pass.
  for (const bool redPillar : {true, false}) {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (redPillar) {
      inputs.vision.red = usableBlob(-2038, 5439, 1800, 830);
    } else {
      inputs.vision.green = usableBlob(2038, 5439, 1800, 830);
    }
    updateFresh(controller, inputs, 100, 0.0F);
    const uint8_t freeSide = redPillar ? FrontRight : FrontLeft;
    inputs.sides[freeSide].rangeMm = 200;
    Decision decision = updateFresh(controller, inputs, 120, 0.0F);
    check(decision.speedPermille == 0 && decision.steeringPermille == 0,
          "R27 close free side must stop instead of steering toward the pillar");

    inputs.sides[freeSide].rangeMm = 300;
    const uint8_t pillarSide = redPillar ? FrontLeft : FrontRight;
    inputs.sides[pillarSide].rangeMm = 200;
    decision = updateFresh(controller, inputs, 140, 0.0F);
    check(decision.speedPermille == 0 && decision.steeringPermille == 0,
          "R27 close pillar side before turn completion must fail and stop");
  }

  // The physically proven red entry at x=+7743 is outside the centred trigger.
  // Enabling R27 must leave its legacy decision bit-for-bit equal.
  {
    autonomy::Config legacy = r27Config();
    legacy.obstacleCenteredEscapeInitialAbsX = 0;
    legacy.obstacleCenteredEscapeHeadingDegrees = 0.0F;
    legacy.obstacleCenteredEscapeDeadlineMs = 0;
    legacy.obstacleCenteredEscapeHeadingStableMs = 0;
    Controller legacyController(legacy);
    Controller r27Controller(r27Config());
    Inputs legacyInputs;
    Inputs r27Inputs;
    startController(legacyController, legacyInputs);
    startController(r27Controller, r27Inputs);
    legacyInputs.vision.red = usableBlob(7743, 5439, 1800, 830);
    r27Inputs.vision.red = legacyInputs.vision.red;
    const Decision legacyDecision =
        updateFresh(legacyController, legacyInputs, 100, 0.0F);
    const Decision r27Decision =
        updateFresh(r27Controller, r27Inputs, 100, 0.0F);
    check(legacyDecision.state == r27Decision.state &&
              legacyDecision.obstacle == r27Decision.obstacle &&
              legacyDecision.speedPermille == r27Decision.speedPermille &&
              legacyDecision.steeringPermille ==
                  r27Decision.steeringPermille &&
              legacyDecision.targetLateralOffsetMm ==
                  r27Decision.targetLateralOffsetMm,
          "R27 must not alter the proven off-centre red entry path");
  }

  {
    const autonomy::Config config = r27Config();
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.green = usableBlob(2038, 10000, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.front.rangeMm = 180;
    const Decision emergencyStop =
        updateFresh(controller, inputs, 120, -1.0F);
    check(emergencyStop.state == RunState::FaultLatched &&
              (emergencyStop.faultFlags & FaultFrontTooClose) != 0 &&
              !emergencyStop.motionAllowed &&
              emergencyStop.speedPermille == 0,
          "R27 must retain the exact 180 mm centre-front emergency stop");
  }
}

void testR28CenteredBodyClearUsesFreshWrapRange() {
  const auto r28Config = []() {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.stopAfterFirstVerifiedObstaclePass = true;
    config.maxRunMs = 5000;
    config.straightSideGeometryGraceMs = 2000;
    config.obstacleDesiredImageOffset = 7000;
    config.obstacleSideClearanceGuardMm = 200;
    config.obstacleBodyClearOppositeSideGuardMm = 140;
    config.obstacleSinglePassBodyClearMs = 500;
    config.obstacleCenteredEscapeInitialAbsX = 4000;
    config.obstacleCenteredEscapeHeadingDegrees = 22.0F;
    config.obstacleCenteredEscapeDeadlineMs = 2500;
    config.obstacleCenteredEscapeHeadingStableMs = 80;
    return config;
  };

  const auto enterCenteredGreenBodyClear = [](Controller& controller,
                                               Inputs& inputs) {
    startController(controller, inputs);
    inputs.sides[FrontRight].rangeMm = 220;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeMm = 0;
    inputs.sides[BackRight].rangeStatus = 2;
    inputs.vision.green = usableBlob(2038, 6000, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.vision.green = {};
    updateFresh(controller, inputs, 110, -5.0F);
    inputs.sides[FrontRight].rangeMm = 360;
    inputs.sides[BackRight].valid = true;
    inputs.sides[BackRight].rangeStatus = 0;
    inputs.sides[BackRight].rangeMm = 340;
    updateFresh(controller, inputs, 390, -22.0F);
    updateFresh(controller, inputs, 470, -22.0F);
    updateFresh(controller, inputs, 649, -22.0F);
    const Decision decision = updateFresh(controller, inputs, 650, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == ObstacleTarget::GreenPassLeft &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R28 fixture must enter the proven centred-green body-clear tail");
  };

  const auto setGreenCrossOnlyGeometry = [](Inputs& inputs) {
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeMm = 0;
      side.rangeStatus = 2;
      side.i2cStatus = 0;
    }
    // Reproduce the R27 stop: only the physical FL+BR diagonal cross remains,
    // both readings are beyond every configured side guard.
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeStatus = 0;
    inputs.sides[FrontLeft].rangeMm = 296;
    inputs.sides[BackRight].valid = true;
    inputs.sides[BackRight].rangeStatus = 0;
    inputs.sides[BackRight].rangeMm = 601;
  };

  // Exact R27 failure class: after strict green proof and the stable 22-degree
  // escape, a fresh/I2C-good status-7 sample at the logged 867 mm must keep the
  // guarded translation continuous through its existing 500 ms bound.
  {
    Controller controller(r28Config());
    Inputs inputs;
    enterCenteredGreenBodyClear(controller, inputs);
    setGreenCrossOnlyGeometry(inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.front.rangeMm = 867;
    Decision decision = updateFresh(controller, inputs, 651, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R28 fresh 867 mm wrap range must continue the proven body-clear tail");
    decision = updateFresh(controller, inputs, 1150, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0,
          "R28 guarded body-clear must not finish one millisecond early");
    decision = updateFresh(controller, inputs, 1151, -22.0F);
    check(decision.state == RunState::Finished &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.obstacle == ObstacleTarget::None &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R28 wrap-range replay must finish permanently at the 500 ms bound");
  }

  // The numeric abort boundary remains strict even for accepted status 7.
  // A sample at the boundary cannot authorize cross-only translation; one
  // millimetre beyond it may recover, but must start a fresh full interval.
  {
    Controller controller(r28Config());
    Inputs inputs;
    enterCenteredGreenBodyClear(controller, inputs);
    setGreenCrossOnlyGeometry(inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.front.rangeMm = controller.config().directionProbeAbortFrontMm;
    Decision decision = updateFresh(controller, inputs, 651, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "R28 status-7 range at the abort boundary must not authorize motion");
    inputs.front.rangeMm = static_cast<uint16_t>(
        controller.config().directionProbeAbortFrontMm + 1U);
    decision = updateFresh(controller, inputs, 700, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0,
          "R28 status-7 range above the abort boundary may recover guarded motion");
    updateFresh(controller, inputs, 701, -22.0F);
    decision = updateFresh(controller, inputs, 1200, -22.0F);
    check(decision.state == RunState::DirectionProbe,
          "R28 recovered body-clear must retain its full 500 ms interval");
    decision = updateFresh(controller, inputs, 1201, -22.0F);
    check(decision.state == RunState::Finished,
          "R28 recovered body-clear may finish at its new exact bound");
  }

  // Converting the measured status-7 sample into a usable numeric front range
  // must preserve the ordinary hard emergency stop with no grace.
  {
    Controller controller(r28Config());
    Inputs inputs;
    enterCenteredGreenBodyClear(controller, inputs);
    setGreenCrossOnlyGeometry(inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.front.rangeMm = controller.config().frontEmergencyStopMm;
    const Decision decision = updateFresh(controller, inputs, 651, -22.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTooClose) != 0U &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R28 accepted wrap range must retain the exact front emergency stop");
  }

  // Merely being a centred calibration target is insufficient. Before strict
  // pass proof enters body-clear, status 7 retains the existing bounded-loss
  // behavior and must still end in FaultFrontTofUnavailable.
  {
    Controller controller(r28Config());
    Inputs inputs;
    startController(controller, inputs);
    inputs.vision.green = usableBlob(2038, 6000, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.front.rangeMm = 867;
    updateFresh(controller, inputs, 101, -1.0F);
    const Decision decision = updateFresh(controller, inputs, 1301, -10.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTofUnavailable) != 0U,
          "R28 status 7 before strict body-clear proof must retain bounded faulting");
  }

  const auto r29Config = [&]() {
    autonomy::Config config = r28Config();
    config.obstacleBodyClearFrontGuardMm = 320;
    return config;
  };

  const auto setR29StoppedGeometry = [&](Inputs& inputs) {
    setGreenCrossOnlyGeometry(inputs);
    inputs.sides[FrontLeft].rangeMm = 247;
    inputs.sides[BackRight].rangeMm = 632;
    inputs.front.valid = true;
    inputs.front.rangeStatus = 0;
    inputs.front.rangeMm = 540;
  };

  // R28 stopped at the initial-probe 600 mm gate despite a proven pass and
  // clear FL+BR rays. The body-clear-only guard must accept its actual 540 mm
  // return, including a fresh WrapTargetFail, for exactly the bounded tail.
  {
    Controller controller(r29Config());
    Inputs inputs;
    enterCenteredGreenBodyClear(controller, inputs);
    setR29StoppedGeometry(inputs);
    Decision decision = updateFresh(controller, inputs, 651, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R29 proven green tail must drive with numeric front 540 and FL247 BR632");
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    decision = updateFresh(controller, inputs, 700, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R29 proven green tail must continue through fresh wrap front 540");
    decision = updateFresh(controller, inputs, 1150, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0,
          "R29 measured geometry must not finish one millisecond early");
    decision = updateFresh(controller, inputs, 1151, -22.0F);
    check(decision.state == RunState::Finished &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.obstacle == ObstacleTarget::None &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R29 measured geometry must finish at its exact 500 ms bound");
  }

  // Complete side geometry cannot bypass the independent body-clear front
  // hold: this path does not rely on calibrationCrossOnlyHold to stop drive.
  {
    Controller controller(r29Config());
    Inputs inputs;
    enterCenteredGreenBodyClear(controller, inputs);
    inputs.front.rangeMm = 320;
    Decision decision = updateFresh(controller, inputs, 651, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "R29 complete side geometry must still stop at body-clear front 320");
    inputs.front.rangeMm = 321;
    decision = updateFresh(controller, inputs, 700, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0,
          "R29 complete side geometry may resume above the body-clear front guard");
  }

  for (const uint8_t frontStatus : {uint8_t{0}, uint8_t{7}}) {
    Controller controller(r29Config());
    Inputs inputs;
    enterCenteredGreenBodyClear(controller, inputs);
    setR29StoppedGeometry(inputs);
    inputs.front.valid = frontStatus == 0;
    inputs.front.rangeStatus = frontStatus;
    inputs.front.rangeMm = 320;
    Decision decision = updateFresh(controller, inputs, 651, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "R29 numeric and wrap body-clear must stop at the exact 320 mm guard");
    inputs.front.rangeMm = 321;
    decision = updateFresh(controller, inputs, 700, -22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0,
          "R29 numeric and wrap body-clear may recover at 321 mm");
    updateFresh(controller, inputs, 701, -22.0F);
    decision = updateFresh(controller, inputs, 1200, -22.0F);
    check(decision.state == RunState::DirectionProbe,
          "R29 guard recovery must restart the full body-clear interval");
    decision = updateFresh(controller, inputs, 1201, -22.0F);
    check(decision.state == RunState::Finished,
          "R29 recovered tail must finish at its new exact 500 ms bound");
  }

  for (const uint8_t frontStatus : {uint8_t{0}, uint8_t{7}}) {
    Controller controller(r29Config());
    Inputs inputs;
    enterCenteredGreenBodyClear(controller, inputs);
    setR29StoppedGeometry(inputs);
    inputs.front.valid = frontStatus == 0;
    inputs.front.rangeStatus = frontStatus;
    inputs.front.rangeMm = 180;
    const Decision decision = updateFresh(controller, inputs, 651, -22.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTooClose) != 0U &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R29 numeric and wrap ranges must preserve the exact 180 mm emergency stop");
  }

  // Front/side bus failures and stale receipts still remove drive; the lower
  // body-clear distance grants no exception to electrical health or freshness.
  for (int failure = 0; failure < 4; ++failure) {
    Controller controller(r29Config());
    Inputs inputs;
    enterCenteredGreenBodyClear(controller, inputs);
    setR29StoppedGeometry(inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    updateFresh(controller, inputs, 651, -22.0F);
    inputs.nowMs = 652;
    inputs.imu.updatedAtMs = 652;
    inputs.vision.updatedAtMs = 652;
    inputs.front.updatedAtMs = 652;
    for (auto& side : inputs.sides) side.updatedAtMs = 652;
    if (failure == 0) inputs.front.i2cStatus = 4;
    if (failure == 1) inputs.sides[FrontLeft].i2cStatus = 4;
    if (failure == 2) inputs.front.updatedAtMs = 0;
    if (failure == 3) inputs.sides[FrontLeft].updatedAtMs = 0;
    const Decision decision = controller.update(inputs);
    check(decision.state != RunState::Finished && decision.speedPermille == 0,
          "R29 body-clear must stop for front or side electrical/stale failures");
  }

  // Production Obstacle has no single-pass escape state and therefore keeps
  // its original status-7 rejection unchanged.
  {
    const autonomy::Config production =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(production);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.front.rangeMm = 867;
    updateFresh(controller, inputs, 100, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 1300, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTofUnavailable) != 0U,
          "R28 must not change production Obstacle wrap-target policy");
  }
}

void testR30CenteredEscapeRecoversBeforeFreshPassProof() {
  for (const bool fullRun : {false, true}) {
  for (const int successorScenario : {0, 1, 2}) {
  const bool continuousSameColour = successorScenario != 0;
  if (continuousSameColour && !fullRun) continue;
  for (const bool redPillar : {true, false}) {
    autonomy::Config config =
        fullRun ? autonomy::makeGuardedObstacleConfig()
                : autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.stopAfterFirstVerifiedObstaclePass = !fullRun;
    config.preferForwardObstacleProgress = false;
    // This fixture isolates the pass proof; R37 separately covers wall
    // backout preempting a committed pass at its350mm front boundary.
    config.wallRecoveryEnabled = false;
    config.maxRunMs = 8000;
    config.straightSideGeometryGraceMs = 2000;
    config.obstacleDesiredImageOffset = 7000;
    config.obstacleSideClearanceGuardMm = 200;
    config.obstacleBodyClearOppositeSideGuardMm = 140;
    config.obstacleBodyClearFrontGuardMm = 320;
    config.obstacleSinglePassBodyClearMs = 500;
    config.obstacleCenteredEscapeInitialAbsX = 4000;
    config.obstacleCenteredEscapeHeadingDegrees = 22.0F;
    config.obstacleCenteredEscapeDeadlineMs = 2500;
    config.obstacleCenteredEscapeHeadingStableMs = 80;
    config.recoverCenteredEscapeBeforePass = true;
    config.obstaclePassProofMinimumBottomY = 7500;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    const uint8_t selectedFront = redPillar ? FrontLeft : FrontRight;
    const uint8_t selectedBack = redPillar ? BackLeft : BackRight;
    const uint8_t freeFront = redPillar ? FrontRight : FrontLeft;
    const uint8_t freeBack = redPillar ? BackRight : BackLeft;
    const float yawSign = redPillar ? 1.0F : -1.0F;
    const auto setBlob = [&](uint16_t bottom) {
      inputs.vision.red = {};
      inputs.vision.green = {};
      if (redPillar) {
        inputs.vision.red = usableBlob(-2038, bottom, 1800, 830);
      } else {
        inputs.vision.green = usableBlob(2038, bottom, 1800, 830);
      }
    };
    const auto clearBlob = [&]() {
      inputs.vision.red = {};
      inputs.vision.green = {};
    };
    const auto setCompleteSides = [&]() {
      for (auto& side : inputs.sides) {
        side.valid = true;
        side.rangeStatus = 0;
        side.rangeMm = 300;
        side.i2cStatus = 0;
      }
    };

    inputs.sides[selectedFront].rangeMm = 220;
    inputs.sides[selectedBack].valid = false;
    inputs.sides[selectedBack].rangeStatus = 2;
    inputs.sides[selectedBack].rangeMm = 0;
    // Even an old image that already met the near-base threshold cannot
    // count after steering the pillar out of view and returning to heading.
    setBlob(8000);
    updateFresh(controller, inputs, 100, 0.0F);
    clearBlob();
    updateFresh(controller, inputs, 110, yawSign * 5.0F);
    inputs.sides[selectedFront].rangeMm = 360;
    inputs.sides[selectedBack].valid = true;
    inputs.sides[selectedBack].rangeStatus = 0;
    inputs.sides[selectedBack].rangeMm = 340;
    updateFresh(controller, inputs, 390, yawSign * 22.0F);
    Decision decision = updateFresh(controller, inputs, 470, yawSign * 22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == (redPillar ? ObstacleTarget::RedPassRight
                                              : ObstacleTarget::GreenPassLeft) &&
              decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille < 0
                         : decision.steeringPermille > 0),
          "R30 stable 22 degree escape must countersteer toward original heading for both colours");

    inputs.sides[selectedFront].valid = false;
    inputs.sides[selectedFront].rangeStatus = 2;
    inputs.sides[selectedFront].rangeMm = 0;
    inputs.sides[freeBack].valid = false;
    inputs.sides[freeBack].rangeStatus = 2;
    inputs.sides[freeBack].rangeMm = 0;
    inputs.sides[freeFront].rangeMm = 247;
    inputs.sides[selectedBack].rangeMm = 632;
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    inputs.front.rangeMm = 667;
    decision = updateFresh(controller, inputs, 471, yawSign * 22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille < 0
                         : decision.steeringPermille > 0),
          "R30 mirrored return must move with fresh wrap667 and the measured diagonal cross pair");
    if (fullRun) {
      inputs.front.rangeMm = 320;
      decision = updateFresh(controller, inputs, 472, yawSign * 22.0F);
      check(decision.state == RunState::DirectionProbe &&
                decision.speedPermille > 0 &&
                (redPillar ? decision.steeringPermille < 0
                           : decision.steeringPermille > 0),
            "R31 heading recovery must not inherit the body-tail320mm hold");
    }
    updateFresh(controller, inputs, 650, yawSign * 14.0F);
    decision = updateFresh(controller, inputs, 750, yawSign * 7.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle != ObstacleTarget::None &&
              (redPillar ? decision.steeringPermille < 0
                         : decision.steeringPermille > 0),
          "R30 old camera loss and wall transition must not finish during heading return");

    inputs.front.valid = true;
    inputs.front.rangeStatus = 0;
    inputs.front.rangeMm = 900;
    setCompleteSides();
    updateFresh(controller, inputs, 850, yawSign * 3.0F);
    decision = updateFresh(controller, inputs, 929, yawSign * 3.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle != ObstacleTarget::None,
          "R30 first aligned samples cannot finish the pillar attempt");
    updateFresh(controller, inputs, 930, yawSign * 3.0F);
    updateFresh(controller, inputs, 940, 0.0F);
    inputs.sides[selectedFront].rangeMm = 600;
    inputs.sides[selectedBack].rangeMm = 300;
    updateFresh(controller, inputs, 1110, 0.0F);
    decision = updateFresh(controller, inputs, 1350, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.obstacle != ObstacleTarget::None,
          "R30 restored heading must discard old close image loss and side proof");

    inputs.sides[selectedFront].rangeMm = 220;
    inputs.sides[selectedBack].rangeMm = 500;
    setBlob(7400);
    updateFresh(controller, inputs, 1400, 0.0F);
    clearBlob();
    updateFresh(controller, inputs, 1410, 0.0F);
    inputs.sides[selectedFront].rangeMm = 600;
    inputs.sides[selectedBack].rangeMm = 300;
    updateFresh(controller, inputs, 1700, 0.0F);
    decision = updateFresh(controller, inputs, 2000, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle != ObstacleTarget::None,
          "R30 fresh image below7500 cannot satisfy pass proof even with later loss and side transition");

    inputs.sides[selectedFront].rangeMm = 220;
    inputs.sides[selectedBack].rangeMm = 500;
    setBlob(continuousSameColour ? 8500 : 7500);
    inputs.front.rangeMm = 1200;
    decision = updateFresh(controller, inputs, 2100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle != ObstacleTarget::None &&
              (fullRun ? (redPillar ? decision.steeringPermille > 0
                                    : decision.steeringPermille < 0)
                       : decision.steeringPermille == 0),
          "R31 must steer away from a fresh centered pillar while legacy R30 preserves heading hold");
    if (fullRun) {
      decision = updateFresh(controller, inputs, 2101, yawSign * 22.0F);
      check(decision.state == RunState::DirectionProbe &&
                (redPillar ? decision.steeringPermille < 0
                           : decision.steeringPermille > 0),
            "R31 at22 degrees must suppress additional image escape and return toward the entry heading");
      updateFresh(controller, inputs, 2102, 0.0F);
      clearBlob();
      decision = updateFresh(controller, inputs, 2103, yawSign * 3.0F);
      check(decision.state == RunState::DirectionProbe &&
                (redPillar ? decision.steeringPermille < 0
                           : decision.steeringPermille > 0),
            "R31 absent target must use heading return without remembered image steering");
      setBlob(continuousSameColour ? 8500 : 7500);
      updateFresh(controller, inputs, 2104, 0.0F);
    }
    if (continuousSameColour) {
      setBlob(6500);
      if (successorScenario == 1) {
        inputs.sides[selectedFront].rangeMm = 600;
        inputs.sides[selectedBack].rangeMm = 300;
      }
      decision = updateFresh(controller, inputs, 2110, 0.0F);
      if (successorScenario == 1) {
        check(decision.steeringPermille == 0,
              "R31 farther successor candidate must not restart image escape for the old pillar");
      }
      updateFresh(controller, inputs, 2190, 0.0F);
    } else {
      clearBlob();
      updateFresh(controller, inputs, 2110, 0.0F);
    }
    if (successorScenario != 2) {
      inputs.sides[selectedFront].rangeMm = 600;
      inputs.sides[selectedBack].rangeMm = 300;
    }
    updateFresh(controller, inputs, 2400, 0.0F);
    updateFresh(controller, inputs, 2580, 0.0F);
    updateFresh(controller, inputs, 2581, 0.0F);
    decision = updateFresh(controller, inputs, 3080, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0,
          "R30 fresh post-alignment proof must retain its full500ms body-clear tail");
    decision = updateFresh(controller, inputs, 3081, 0.0F);
    if (successorScenario == 2) {
      check(decision.state == RunState::DirectionProbe &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.obstacle == (redPillar ? ObstacleTarget::RedPassRight
                                                : ObstacleTarget::GreenPassLeft),
            "R31 a farther same-colour image without a matching side transition must not clear the current pillar");
      continue;
    }
    if (!fullRun) {
      check(decision.state == RunState::Finished &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.obstacle == ObstacleTarget::None &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R30 fresh post-alignment near image loss and side transition may finish at the bounded tail");
    } else {
      if (decision.state != RunState::DirectionProbe ||
          decision.faultFlags != autonomy::FaultNone ||
          decision.obstacle != ObstacleTarget::None ||
          !decision.motionAllowed || decision.speedPermille == 0 ||
          decision.steeringPermille != 0) {
        std::cerr << "R31 post-pass state=" << static_cast<int>(decision.state)
                  << " fault=" << decision.faultFlags
                  << " obstacle=" << static_cast<int>(decision.obstacle)
                  << " motion=" << decision.motionAllowed
                  << " speed=" << decision.speedPermille
                  << " steer=" << decision.steeringPermille << '\n';
      }
      check(decision.state == RunState::DirectionProbe &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.obstacle == ObstacleTarget::None &&
                decision.motionAllowed && decision.speedPermille > 0 &&
                decision.steeringPermille == 0,
            "R31 full run must continue straight after a verified pillar body-clear tail");
      setCompleteSides();
      inputs.front.rangeMm = 1200;
      if (continuousSameColour) {
        setBlob(6500);
        decision = updateFresh(controller, inputs, 3100, 0.0F);
        check(decision.obstacle == ObstacleTarget::None,
              "R31 continuous same-colour successor must retain the completed pillar cooldown");
        decision = updateFresh(controller, inputs, 3382, 0.0F);
        check(decision.state == RunState::DirectionProbe &&
                  decision.obstacle == (redPillar ? ObstacleTarget::RedPassRight
                                                  : ObstacleTarget::GreenPassLeft) &&
                  decision.speedPermille > 0,
              "R31 farther same-colour successor must commit after the bounded cooldown without a camera blank");
      } else if (redPillar) {
        inputs.vision.green = usableBlob(2038, 8000, 1800, 830);
      } else {
        inputs.vision.red = usableBlob(-2038, 8000, 1800, 830);
      }
      if (!continuousSameColour) {
        decision = updateFresh(controller, inputs, 3100, 0.0F);
        check(decision.state == RunState::DirectionProbe &&
                decision.obstacle == (redPillar ? ObstacleTarget::GreenPassLeft
                                                : ObstacleTarget::RedPassRight) &&
                decision.speedPermille > 0,
            "R31 full run must commit the next colour after completing the previous pass");
      }
    }
  }
  }
  }
}

void testR31IndependentDiagonalObservations() {
  autonomy::Config config = autonomy::makeGuardedObstacleConfig();
  config.wallRecoveryEnabled = false;
  check(config.guardedObstaclePassEnabled &&
            !config.stopAfterFirstVerifiedObstaclePass &&
            !config.parkingEnabled && config.maxRunMs > 8000U,
        "R31 shared profile must select a full guarded obstacle run without parking");
  const auto startOneRayEscape = [&](Controller& controller, Inputs& inputs) {
    startController(controller, inputs);
    inputs.front.rangeMm = 1734;
    inputs.sides[FrontLeft].rangeMm = 614;
    inputs.sides[BackLeft].rangeMm = 710;
    inputs.sides[FrontRight].rangeMm = 520;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeMm = 0;
    inputs.sides[BackRight].rangeStatus = 2;
    inputs.vision.green = usableBlob(157, 6653, 1800, 690);
    updateFresh(controller, inputs, 100, 0.0F);
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
      side.rangeMm = 0;
      side.i2cStatus = 0;
    }
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeStatus = 0;
    inputs.sides[FrontLeft].rangeMm = 498;
    inputs.front.rangeMm = 320;
    inputs.vision.green = usableBlob(2947, 8870, 1800, 960);
    return updateFresh(controller, inputs, 600, -8.0F);
  };

  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.rangeMm = 1970;
    inputs.sides[FrontLeft].rangeMm = 650;
    inputs.sides[FrontRight].rangeMm = 530;
    inputs.sides[BackLeft].rangeMm = 710;
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    inputs.sides[BackRight].rangeMm = 0;
    const Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 && decision.steeringPermille == 0,
          "R31 unequal independent diagonal rays must not create the old minus170 steering before a pillar");
  }

  {
    Controller controller(config);
    Inputs inputs;
    Decision decision = startOneRayEscape(controller, inputs);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              decision.steeringPermille < 0,
          "R31 exact R30 front320 FL498 live green observation must continue its left escape");
    decision = updateFresh(controller, inputs, 2601, -8.0F);
    check(decision.state != RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultSideGeometryUnavailable) == 0U,
          "R31 fresh single-ray camera guidance must not acquire the old two-second geometry fault");
  }

  {
    Controller controller(config);
    Inputs inputs;
    startOneRayEscape(controller, inputs);
    inputs.front.rangeMm = 180;
    const Decision decision = updateFresh(controller, inputs, 601, -8.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTooClose) != 0U &&
              decision.speedPermille == 0,
          "R31 single-ray guidance must preserve the exact front180 emergency stop");
  }

  for (int failure = 0; failure < 4; ++failure) {
    Controller controller(config);
    Inputs inputs;
    startOneRayEscape(controller, inputs);
    inputs.nowMs = 601;
    inputs.imu.updatedAtMs = 601;
    inputs.vision.updatedAtMs = 601;
    inputs.front.updatedAtMs = 601;
    for (auto& side : inputs.sides) side.updatedAtMs = 601;
    if (failure == 0) inputs.sides[FrontLeft].i2cStatus = 4;
    if (failure == 1) inputs.sides[BackRight].i2cStatus = 4;
    if (failure == 2) inputs.sides[FrontLeft].updatedAtMs = 0;
    if (failure == 3) inputs.sides[BackRight].updatedAtMs = 0;
    const Decision decision = controller.update(inputs);
    check(decision.speedPermille == 0 && decision.state != RunState::Finished,
          "R31 numeric and no-return side rays must both stop motion on electrical or stale input");
  }
}

void testR32DelayedDrivetrainStillCompletesEscape() {
  autonomy::Config config = autonomy::makeGuardedObstacleConfig();
  // Preserve the strict proof mode's powered and overall deadline contracts.
  config.preferForwardObstacleProgress = false;
  check(config.obstacleCenteredEscapeDeadlineMs == 4000U,
        "R32 full profile must allow four seconds for a measured centered escape");
  check(config.obstacleMaximumCommitMs == 12000U &&
            config.maxRunMs == 170000U,
        "R32 must bound the complete pillar maneuver to12s inside the170s course limit");
  for (const bool redPillar : {true, false}) {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.rangeMm = 1500;
    if (redPillar) {
      inputs.vision.red = usableBlob(-2038, 6000, 1800, 830);
    } else {
      inputs.vision.green = usableBlob(2038, 6000, 1800, 830);
    }
    const float yawSign = redPillar ? 1.0F : -1.0F;
    updateFresh(controller, inputs, 100, 0.0F);
    // The field drivetrain did not begin turning until about533ms after
    // camera commitment because its startup servo-settle interval remained.
    updateFresh(controller, inputs, 633, 0.0F);
    updateFresh(controller, inputs, 1700, yawSign * 12.0F);
    Decision decision = updateFresh(controller, inputs, 2601, yawSign * 21.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille > 0
                         : decision.steeringPermille < 0),
          "R32 delayed escape at21 degrees after2501ms must keep moving toward its measured target");
    decision = updateFresh(controller, inputs, 2800, yawSign * 22.0F);
    check(decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille > 0
                         : decision.steeringPermille < 0),
          "R32 first late22-degree sample must still wait for independent heading confirmation");
    decision = updateFresh(controller, inputs, 2880, yawSign * 22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.motionAllowed && decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille < 0
                         : decision.steeringPermille > 0),
          "R32 stable22-degree heading after2780ms must start mirrored recovery instead of a dead stop");
  }
  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.rangeMm = 1500;
    inputs.vision.green = usableBlob(2038, 6000, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    Decision decision = updateFresh(controller, inputs, 4099, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 && decision.steeringPermille < 0,
          "R32 no-heading escape may run until the last millisecond of its4s bound");
    decision = updateFresh(controller, inputs, 4100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0 && decision.steeringPermille == 0,
          "R32 no-heading response must stop exactly at the4s escape deadline");
    decision = updateFresh(controller, inputs, 12100, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultObstaclePassTimeout) != 0U &&
              decision.speedPermille == 0,
          "R32 an unfinished pillar maneuver must still fault at its12s commitment limit");
  }
}

void testR33EscapeDeadlineUsesAppliedDriveTime() {
  autonomy::Config config = autonomy::makeGuardedObstacleConfig();
  config.preferForwardObstacleProgress = false;
  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs, 49900);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 300;
    inputs.front.rangeMm = 1500;
    inputs.vision.green = usableBlob(2038, 6200, 1800, 830);
    updateFresh(controller, inputs, 50000, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 50010, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.obstacle == ObstacleTarget::GreenPassLeft &&
              decision.speedPermille > 0 && decision.steeringPermille < 0,
          "R35 a new pillar at50s uptime must count only10ms applied drive instead of the whole boot uptime");
  }
  for (const bool redPillar : {true, false}) {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    inputs.front.rangeMm = 1500;
    if (redPillar) {
      inputs.vision.red = usableBlob(-2038, 6200, 1800, 830);
    } else {
      inputs.vision.green = usableBlob(2038, 6200, 1800, 830);
    }
    const float sign = redPillar ? 1.0F : -1.0F;
    updateFresh(controller, inputs, 100, 0.0F);
    inputs.vision.red = {};
    inputs.vision.green = {};
    updateFresh(controller, inputs, 800, 0.0F);  // startup:700ms unpowered
    inputs.appliedDrivePermille = 300;
    Decision decision = updateFresh(controller, inputs, 1400, sign * 8.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle != ObstacleTarget::None &&
              decision.speedPermille > 0,
          "R33 bounded centered escape must not stop for fresh-camera target absence below the6500 close threshold");
    inputs.appliedDrivePermille = 0;
    updateFresh(controller, inputs, 2400, sign * 8.0F);  // pause:1000ms
    inputs.appliedDrivePermille = 300;
    updateFresh(controller, inputs, 3600, sign * 20.0F);
    decision = updateFresh(controller, inputs, 4100, sign * 22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille > 0
                         : decision.steeringPermille < 0),
          "R33 first22-degree sample after4s wall time must remain eligible with only2300ms actual drive");
    decision = updateFresh(controller, inputs, 4180, sign * 22.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0 &&
              (redPillar ? decision.steeringPermille < 0
                         : decision.steeringPermille > 0),
          "R33 stable22-degree heading beyond4s wall time must start recovery when powered time is below4s");
  }
  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    inputs.front.rangeMm = 1500;
    inputs.vision.green = usableBlob(2038, 6000, 1800, 830);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 1100, 0.0F);
    inputs.appliedDrivePermille = 300;
    updateFresh(controller, inputs, 3100, 0.0F);  // powered2000ms
    inputs.appliedDrivePermille = 0;
    updateFresh(controller, inputs, 4100, 0.0F);
    inputs.appliedDrivePermille = 300;
    Decision decision = updateFresh(controller, inputs, 6099, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 && decision.steeringPermille < 0,
          "R33 no-heading escape must remain eligible at3999ms applied drive despite5999ms wall time");
    decision = updateFresh(controller, inputs, 6100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0 && decision.steeringPermille == 0,
          "R33 no-heading escape must still stop exactly at4000ms applied drive");
    inputs.appliedDrivePermille = 0;
    decision = updateFresh(controller, inputs, 12100, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultObstaclePassTimeout) != 0U &&
              decision.speedPermille == 0,
          "R33 paused drive must not extend the12s overall pillar commitment limit");
  }
}

void testR34FarPreviewDoesNotBlockFirstCorner() {
  const autonomy::Config config = autonomy::makeGuardedObstacleConfig();
  check(config.obstacleMinimumBottomY == 6000 &&
            config.obstacleCloseBottomY == 6500 &&
            config.obstacleFullSteeringBottomY == 7500,
        "R34 must separate nearby pillar commitment from the measured far red preview");
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  inputs.front.rangeMm = 1160;
  inputs.sides[FrontLeft].rangeMm = 600;
  inputs.sides[BackLeft].rangeMm = 650;
  for (const uint8_t index : {uint8_t{FrontRight}, uint8_t{BackRight}}) {
    inputs.sides[index].valid = false;
    inputs.sides[index].rangeStatus = 2;
    inputs.sides[index].rangeMm = 0;
  }
  inputs.vision.red = usableBlob(1160, 4500, 625, 730);
  inputs.vision.red.width = 188;
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.obstacle == ObstacleTarget::None &&
            decision.speedPermille > 0 && decision.steeringPermille == 0,
        "R34 measured tiny red preview at4500 must not initiate an escape or suppress first-corner navigation");
  inputs.front.rangeMm = 1000;
  inputs.vision.red.bottomY = 4854;
  decision = updateFresh(controller, inputs, 200, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.obstacle == ObstacleTarget::None,
        "R34 measured maximum far red base4854 must stay uncommitted while a right opening confirms");
  updateFresh(controller, inputs, 250, 0.0F);
  decision = updateFresh(controller, inputs, 300, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::Clockwise &&
            decision.obstacle == ObstacleTarget::None &&
            decision.faultFlags == autonomy::FaultNone &&
            decision.speedPermille > 0 && decision.steeringPermille > 0,
        "R34 front1000 FL600 and fresh right no-return must confirm the first clockwise corner despite distant red");
}

void testR36EarlyOpticalOpeningSurvivesLaterNumericWallView() {
  for (const bool clockwise : {false, true}) {
    for (const bool confirmedOptical : {false, true}) {
      Controller controller(autonomy::makeGuardedObstacleConfig());
      Inputs inputs;
      startController(controller, inputs);
      inputs.front.rangeMm = 1400;
      inputs.sides[FrontLeft].rangeMm = clockwise ? 599 : 0;
      inputs.sides[FrontRight].rangeMm = clockwise ? 0 : 599;
      const uint8_t openSide = clockwise ? FrontRight : FrontLeft;
      inputs.sides[openSide].valid = false;
      inputs.sides[openSide].rangeStatus = 2;
      Decision decision = updateFresh(controller, inputs, 100, 0.0F);
      check(decision.state == RunState::DirectionProbe &&
                decision.direction == CourseDirection::Unknown &&
                decision.steeringPermille == 0,
            "R36 early optical opening atfront1400 must only gather evidence and never start the turn");
      if (!confirmedOptical) {
        inputs.sides[openSide].valid = true;
        inputs.sides[openSide].rangeStatus = 0;
        inputs.sides[openSide].rangeMm = 400;
      }
      updateFresh(controller, inputs, 150, 0.0F);
      decision = updateFresh(controller, inputs, 200, 0.0F);
      check(decision.state == RunState::DirectionProbe &&
                decision.direction == CourseDirection::Unknown &&
                decision.steeringPermille == 0,
            "R36 three early optical samples must preserve straight motion until the first-corner front window");
      inputs.front.rangeMm = 900;
      inputs.sides[FrontLeft].valid = true;
      inputs.sides[FrontLeft].rangeStatus = 0;
      inputs.sides[FrontLeft].rangeMm = clockwise ? 750 : 440;
      inputs.sides[FrontRight].valid = true;
      inputs.sides[FrontRight].rangeStatus = 0;
      inputs.sides[FrontRight].rangeMm = clockwise ? 440 : 750;
      decision = updateFresh(controller, inputs, 300, 0.0F);
      check(decision.state == RunState::DirectionProbe,
            "R36 cached opening must still require current first-corner confirmation");
      updateFresh(controller, inputs, 350, 0.0F);
      decision = updateFresh(controller, inputs, 400, 0.0F);
      const bool expectedClockwise = confirmedOptical ? clockwise : !clockwise;
      const CourseDirection expectedDirection = expectedClockwise
          ? CourseDirection::Clockwise : CourseDirection::CounterClockwise;
      check(decision.state == RunState::Cornering &&
                decision.direction == expectedDirection &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.speedPermille > 0 &&
                (expectedClockwise ? decision.steeringPermille > 0
                                   : decision.steeringPermille < 0),
            confirmedOptical
                ? "R36 confirmed early optical opening must beat a misleading later numeric wall comparison in either direction"
                : "R36 one isolated early no-return must not override the later confirmed numeric direction");
    }
  }
}

void testR36CornerWallReversePreservesThePendingTurn() {
  for (const bool clockwise : {false, true}) {
    autonomy::Config config = autonomy::makeGuardedObstacleConfig();
    config.openingConfirmationMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    if (clockwise) setRightOpening(inputs, 650);
    else setLeftOpening(inputs, 650);
    inputs.sides[BackLeft].rangeMm = 500;
    inputs.sides[BackRight].rangeMm = 500;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    const CourseDirection expectedDirection = clockwise
        ? CourseDirection::Clockwise : CourseDirection::CounterClockwise;
    const float sign = clockwise ? 1.0F : -1.0F;
    check(decision.state == RunState::Cornering &&
              decision.direction == expectedDirection,
          "R36 corner reverse fixture must start the observed mirrored turn");
    updateFresh(controller, inputs, 200, sign * 40.0F);
    inputs.front.rangeMm = 250;
    if (clockwise) inputs.vision.green = usableBlob(0, 7000, 1800, 830);
    else inputs.vision.red = usableBlob(0, 7000, 1800, 830);
    decision = updateFresh(controller, inputs, 210, sign * 40.0F);
    check(decision.state == RunState::Cornering &&
              decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
          "R36 a wall during a40degree corner must begin neutral recovery even with a next-straight pillar visible");
    decision = updateFresh(controller, inputs, 509, sign * 40.0F);
    check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
          "R36 corner backout must preserve the full300ms entry neutral");
    decision = updateFresh(controller, inputs, 510, sign * 40.0F);
    check(decision.wallRecoveryPhase == 2 &&
              decision.driveDirection == DriveDirection::Reverse &&
              decision.speedPermille == 300 &&
              (clockwise ? decision.steeringPermille > 0
                         : decision.steeringPermille < 0),
          "R36 reverse must keep the original mirrored corner steering sign to unwind heading");
    decision = updateFresh(controller, inputs, 600, sign * 20.0F);
    check(decision.wallRecoveryPhase == 2 &&
              decision.driveDirection == DriveDirection::Reverse &&
              decision.cornerCount == 0 && decision.direction == expectedDirection,
          "R36 decreasing corner progress while reversing must preserve the pending turn and count");
    inputs.front.rangeMm = 650;
    decision = updateFresh(controller, inputs, 700, sign * 2.0F);
    check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
          "R36 recovered front650 must stop the corner reverse into exit neutral");
    decision = updateFresh(controller, inputs, 999, sign * 2.0F);
    check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
          "R36 corner backout must preserve the full300ms reverse-to-forward neutral");
    updateFresh(controller, inputs, 1000, sign * 2.0F);
    decision = updateFresh(controller, inputs, 1010, sign * 2.0F);
    check(decision.state == RunState::Cornering &&
              decision.wallRecoveryPhase == 0 &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.direction == expectedDirection &&
              decision.cornerCount == 0 && decision.lapCount == 0 &&
              std::fabs(decision.cornerTargetHeadingDegrees - sign * 90.0F) < 0.01F &&
              decision.driveDirection == DriveDirection::Forward &&
              decision.speedPermille > 0 &&
              (clockwise ? decision.steeringPermille > 0
                         : decision.steeringPermille < 0),
          "R36 after corner backout the same90degree target and direction must resume without awarding a corner");
  }
}

void testR37CommittedPillarWallBackoutReacquiresCurrentTarget() {
  for (const bool redPillar : {false, true}) {
    for (const bool retainSameColor : {false, true}) {
      for (const uint16_t recoveryFront : {uint16_t{350}, uint16_t{319}}) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      // Strict mode retains and retries the old target; R42 progress mode
      // separately verifies its authorized post-backoff one-second skip.
      config.preferForwardObstacleProgress = false;
      config.openingConfirmationMs = 0;
      config.cornerMinimumDurationMs = 0;
      config.cornerExitStableMs = 0;
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs, 50000);
      const float sign = redPillar ? 1.0F : -1.0F;
      const ObstacleTarget original = redPillar
          ? ObstacleTarget::RedPassRight : ObstacleTarget::GreenPassLeft;
      const ObstacleTarget opposite = redPillar
          ? ObstacleTarget::GreenPassLeft : ObstacleTarget::RedPassRight;
      if (redPillar) setRightOpening(inputs, 650);
      else setLeftOpening(inputs, 650);
      updateFresh(controller, inputs, 50100, 0.0F);
      updateFresh(controller, inputs, 50200, wrapYaw(sign * 45.0F));
      setStraightGeometry(inputs, 1200);
      Decision decision = updateFresh(controller, inputs, 50300,
                                      wrapYaw(sign * 90.0F));
      check(decision.state == RunState::Running && decision.cornerCount == 1,
            "R37 physical replay must begin after one verified mirrored corner");
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 300;
      auto& oldBlob = redPillar ? inputs.vision.red : inputs.vision.green;
      auto& nextBlob = redPillar ? inputs.vision.green : inputs.vision.red;
      oldBlob = usableBlob(redPillar ? 1693 : -1693, 6360, 1800, 830);
      decision = updateFresh(controller, inputs, 50400,
                             wrapYaw(sign * 93.56F));
      check(decision.obstacle == original,
            "R37 physical replay must commit the original near pillar at the corner exit heading");
      oldBlob = usableBlob(redPillar ? -8683 : 8683, 10000, 4000, 830);
      updateFresh(controller, inputs, 50410, wrapYaw(sign * 104.5F));
      oldBlob.present = false;
      nextBlob = usableBlob(redPillar ? 5361 : -5361, 9000, 3000, 830);
      inputs.front.rangeMm = 351;
      inputs.sides[FrontLeft].rangeMm = redPillar ? 370 : 413;
      inputs.sides[BackLeft].rangeMm = redPillar ? 600 : 508;
      inputs.sides[FrontRight].rangeMm = redPillar ? 413 : 370;
      inputs.sides[BackRight].rangeMm = redPillar ? 508 : 600;
      decision = updateFresh(controller, inputs, 50440,
                             wrapYaw(sign * 101.37F));
      check(decision.wallRecoveryPhase == 0 && decision.obstacle == original,
            "R37 a close committed target must remain above the350mm recovery boundary");
      inputs.front.rangeMm = recoveryFront;
      decision = updateFresh(controller, inputs, 50450,
                             wrapYaw(sign * 101.37F));
      check(decision.state == RunState::Running &&
                decision.wallRecoveryPhase == 1 &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.speedPermille == 0 && decision.cornerCount == 1,
            "R37 front350 boundary and observed319 must start backout despite close old commitment, visible opposite pillar, and11degree heading error");
      inputs.appliedDrivePermille = 0;
      decision = updateFresh(controller, inputs, 50749,
                             wrapYaw(sign * 101.37F));
      check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
            "R37 committed-target backout must preserve the300ms neutral bound");
      decision = updateFresh(controller, inputs, 50750,
                             wrapYaw(sign * 101.37F));
      check(decision.wallRecoveryPhase == 2 &&
                decision.driveDirection == DriveDirection::Reverse &&
                decision.speedPermille == 300 && decision.steeringPermille == 0,
            "R37 off-axis pillar recovery must reverse straight from its current pose");
      inputs.appliedDrivePermille = 300;
      inputs.front.rangeMm = 650;
      if (retainSameColor) {
        oldBlob = usableBlob(0, 8000, 2500, 830);
        nextBlob.present = false;
      }
      decision = updateFresh(controller, inputs, 50850,
                             wrapYaw(sign * 101.37F));
      check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
            "R37 front650 must stop the committed-target backout into exit neutral");
      inputs.appliedDrivePermille = 0;
      decision = updateFresh(controller, inputs, 51149,
                             wrapYaw(sign * 101.37F));
      check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0 &&
                decision.driveDirection != DriveDirection::Forward,
            "R37 current-target backout must retain its full300ms exit neutral before any forward request");
      decision = updateFresh(controller, inputs, 51150,
                             wrapYaw(sign * 101.37F));
      check(decision.wallRecoveryPhase == 0 && decision.speedPermille == 0 &&
                decision.cornerCount == 1 && decision.lapCount == 0 &&
                decision.obstacle == (retainSameColor ? original : ObstacleTarget::None),
            "R37 reverse recovery must release neutral without awarding a pass or corner");
      inputs.appliedDrivePermille = 300;
      decision = updateFresh(controller, inputs, 51160,
                             wrapYaw(sign * 101.37F));
      check(decision.state == RunState::Running &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.obstacle == (retainSameColor ? original : opposite) &&
                decision.speedPermille > 0 && decision.cornerCount == 1 &&
                decision.lapCount == 0 &&
                std::fabs(decision.cornerTargetHeadingDegrees - sign * 90.0F) < 0.01F,
            retainSameColor
                ? "R37 still-visible original pillar must restart its pass from fresh evidence and a fresh powered timer"
                : "R37 absent old pillar must be abandoned even when the opposite color is visible, allowing fresh opposite commitment");
      }
    }
  }
}

void testR38LeadingSideHazardPreemptsCornerMotion() {
  // R37 saw green at x8934..9498, bottom4812..5439, height1833..2167;
  // the blob then left the frame. The leading ray still measured the hazard
  // continuously, so this test deliberately does not require a camera blob.
  for (const bool clockwise : {false, true}) {
    for (const uint8_t frontStatus : {uint8_t{0}, uint8_t{2},
                                      uint8_t{4}, uint8_t{7}}) {
      for (const bool delayedAppliedReverse : {false, true}) {
        autonomy::Config config = autonomy::makeGuardedObstacleConfig();
        config.openingConfirmationMs = 0;
        Controller controller(config);
        Inputs inputs;
        startController(controller, inputs);
        const float sign = clockwise ? 1.0F : -1.0F;
        const uint8_t hazardIndex = clockwise ? FrontRight : FrontLeft;
        const uint8_t clearRearIndex = clockwise ? BackLeft : BackRight;
        const uint8_t openRearIndex = clockwise ? BackRight : BackLeft;
        const uint8_t expectedMask = clockwise ? 4U : 2U;
        if (clockwise) setRightOpening(inputs, 650);
        else setLeftOpening(inputs, 650);
        updateFresh(controller, inputs, 100, 0.0F);
        inputs.front.rangeMm = 825;
        inputs.sides[hazardIndex].rangeMm = 202;
        inputs.sides[clearRearIndex].rangeMm = 768;
        inputs.sides[openRearIndex].valid = false;
        inputs.sides[openRearIndex].rangeStatus = 2;
        inputs.sides[openRearIndex].rangeMm = 0;
        Decision decision = updateFresh(controller, inputs, 200,
                                        wrapYaw(sign * 16.0F));
        check(decision.state == RunState::Cornering &&
                  decision.wallRecoveryPhase == 0 && decision.speedPermille > 0,
              "R38 physical corner replay must remain above the200mm leading-side boundary at202mm");
        inputs.front.valid = frontStatus == 0;
        inputs.front.rangeStatus = frontStatus;
        inputs.front.rangeMm = (frontStatus == 2 || frontStatus == 4) ? 0 : 790;
        inputs.sides[hazardIndex].rangeMm = 144;
        inputs.driveFeedbackAvailable = true;
        inputs.appliedDrivePermille = 0;
        decision = updateFresh(controller, inputs, 210,
                               wrapYaw(sign * 20.0F));
        check(decision.state == RunState::Cornering &&
                  decision.wallRecoveryPhase == 1 &&
                  decision.wallRecoveryHazards == expectedMask &&
                  decision.speedPermille == 0 && decision.steeringPermille == 0 &&
                  decision.driveDirection != DriveDirection::Forward,
              "R38 fresh leading-side144 must preempt corner drive and center steering even with far or optically open front");
        decision = updateFresh(controller, inputs, 509,
                               wrapYaw(sign * 20.0F));
        check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
              "R38 side-hazard backout must preserve the300ms entry neutral");
        decision = updateFresh(controller, inputs, 510,
                               wrapYaw(sign * 20.0F));
        check(decision.wallRecoveryPhase == 2 &&
                  decision.driveDirection == DriveDirection::Reverse &&
                  decision.speedPermille == 300 && decision.steeringPermille == 0,
              "R38 side-hazard backout must reverse straight with fresh numeric or optical front evidence");
        uint32_t exitStartedAt = 0;
        if (delayedAppliedReverse) {
          inputs.sides[hazardIndex].rangeMm = 301;
          updateFresh(controller, inputs, 610, wrapYaw(sign * 20.0F));
          updateFresh(controller, inputs, 660, wrapYaw(sign * 20.0F));
          decision = updateFresh(controller, inputs, 710,
                                 wrapYaw(sign * 20.0F));
          check(decision.wallRecoveryPhase == 2 &&
                    decision.driveDirection == DriveDirection::Reverse,
                "R38 stable side clearance alone must not finish a reverse that has applied zero drive");
          inputs.appliedDrivePermille = 300;
          updateFresh(controller, inputs, 810, wrapYaw(sign * 20.0F));
          updateFresh(controller, inputs, 910, wrapYaw(sign * 20.0F));
          decision = updateFresh(controller, inputs, 959,
                                 wrapYaw(sign * 20.0F));
          check(decision.wallRecoveryPhase == 2 && decision.speedPermille > 0,
                "R38 side clearance must retain reverse through249ms actual applied drive");
          exitStartedAt = 960;
        } else {
          inputs.appliedDrivePermille = 300;
          inputs.front.valid = true;
          inputs.front.rangeStatus = 0;
          inputs.front.rangeMm = 1400;
          inputs.sides[hazardIndex].rangeMm = 99;
          updateFresh(controller, inputs, 610, wrapYaw(sign * 20.0F));
          decision = updateFresh(controller, inputs, 760,
                                 wrapYaw(sign * 20.0F));
          check(decision.wallRecoveryPhase == 2 &&
                    decision.speedPermille > 0 && decision.steeringPermille == 0,
                "R38 front1400 must not falsely clear the initiating side hazard while its ray remains99mm");
          inputs.sides[hazardIndex].rangeMm = 300;
          decision = updateFresh(controller, inputs, 770,
                                 wrapYaw(sign * 20.0F));
          check(decision.wallRecoveryPhase == 2,
                "R38 side-hazard clearance requires more than the exact300mm boundary");
          inputs.sides[hazardIndex].rangeMm = 301;
          updateFresh(controller, inputs, 780, wrapYaw(sign * 20.0F));
          updateFresh(controller, inputs, 790, wrapYaw(sign * 20.0F));
          updateFresh(controller, inputs, 800, wrapYaw(sign * 20.0F));
          inputs.nowMs = 879;
          inputs.imu.updatedAtMs = 879;
          inputs.front.updatedAtMs = 879;
          inputs.vision.updatedAtMs = 879;
          for (auto& side : inputs.sides) side.updatedAtMs = 879;
          inputs.sides[hazardIndex].updatedAtMs = 800;
          decision = controller.update(inputs);
          check(decision.wallRecoveryPhase == 2,
                "R38 three fast clear samples plus unchanged receipt must not mature100ms confirmation on wall-clock time alone");
          exitStartedAt = 880;
        }
        decision = updateFresh(controller, inputs, exitStartedAt,
                               wrapYaw(sign * 20.0F));
        check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
              "R38 adequate powered reverse and stable initiating-ray clearance must enter exit neutral");
        inputs.appliedDrivePermille = 0;
        decision = updateFresh(controller, inputs, exitStartedAt + 299,
                               wrapYaw(sign * 20.0F));
        check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
              "R38 side-hazard backout must preserve the300ms exit neutral");
        updateFresh(controller, inputs, exitStartedAt + 300,
                    wrapYaw(sign * 20.0F));
        decision = updateFresh(controller, inputs, exitStartedAt + 310,
                               wrapYaw(sign * 20.0F));
        check(decision.state == RunState::Cornering &&
                  decision.wallRecoveryPhase == 0 &&
                  decision.faultFlags == autonomy::FaultNone &&
                  decision.cornerCount == 0 && decision.lapCount == 0 &&
                  std::fabs(decision.cornerTargetHeadingDegrees - sign * 90.0F) < 0.01F &&
                  decision.driveDirection == DriveDirection::Forward &&
                  decision.speedPermille > 0,
              "R38 side backout must resume the existing cardinal corner without awarding progress");
      }
    }
  }
}

void testR38RearOnlyStartupAndReverseGuards() {
  for (const bool leftRearClose : {false, true}) {
    Controller controller(autonomy::makeGuardedObstacleConfig());
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 1905;
    inputs.sides[FrontLeft].rangeMm = 596;
    inputs.sides[FrontRight].rangeMm = 550;
    const uint8_t closeRear = leftRearClose ? BackLeft : BackRight;
    const uint8_t openRear = leftRearClose ? BackRight : BackLeft;
    inputs.sides[closeRear].rangeMm = 75;
    inputs.sides[openRear].valid = false;
    inputs.sides[openRear].rangeStatus = 2;
    inputs.sides[openRear].rangeMm = 0;
    controller.update(inputs);
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 90, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.steeringPermille == 0,
          "R38 isolated rear75 with clear leading rays must not force a650 forward turn at startup");
    const Decision noisyHeading = updateFresh(controller, inputs, 100,
                                              leftRearClose ? -0.06F : 0.06F);
    check(noisyHeading.state == RunState::DirectionProbe &&
              noisyHeading.speedPermille > 0 &&
              std::abs(noisyHeading.steeringPermille) <= 10,
          "R39 ordinary startup with rear75 and tiny heading noise must not be stalled by a recovery-only tail guard");
  }

  for (const bool clockwise : {false, true}) {
    for (const bool staleRear : {false, true}) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      config.openingConfirmationMs = 0;
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs);
      if (clockwise) setRightOpening(inputs, 650);
      else setLeftOpening(inputs, 650);
      updateFresh(controller, inputs, 100, 0.0F);
      const uint8_t hazard = clockwise ? FrontRight : FrontLeft;
      const uint8_t rear = clockwise ? BackRight : BackLeft;
      inputs.front.rangeMm = 790;
      inputs.sides[hazard].rangeMm = 144;
      inputs.nowMs = 500;
      inputs.imu.yawDegrees = wrapYaw(clockwise ? 20.0F : -20.0F);
      inputs.imu.updatedAtMs = 500;
      inputs.front.updatedAtMs = 500;
      inputs.vision.updatedAtMs = 500;
      for (auto& side : inputs.sides) side.updatedAtMs = 500;
      if (staleRear) inputs.sides[rear].updatedAtMs = 0;
      else inputs.sides[rear].rangeMm = 200;
      const Decision decision = controller.update(inputs);
      check(decision.speedPermille == 0 && !decision.motionAllowed &&
                decision.driveDirection != DriveDirection::Reverse &&
                decision.driveDirection != DriveDirection::Forward,
            "R38 leading-side hazard with blocked or stale rear must remove drive without attempting reverse");
    }
  }
}

void testR38LeadingHazardWinsAtStraightAndCornerExitBoundaries() {
  for (const bool clockwise : {false, true}) {
    for (int phase = 0; phase < 3; ++phase) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      // R38's original140mm trigger remains a contract of strict mode.
      config.preferForwardObstacleProgress = false;
      config.openingConfirmationMs = 0;
      config.cornerMinimumDurationMs = 0;
      config.cornerExitStableMs = 0;
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs);
      const float sign = clockwise ? 1.0F : -1.0F;
      const uint8_t hazard = clockwise ? FrontRight : FrontLeft;
      if (phase != 0) {
        if (clockwise) setRightOpening(inputs, 650);
        else setLeftOpening(inputs, 650);
        updateFresh(controller, inputs, 100, 0.0F);
        updateFresh(controller, inputs, 200, wrapYaw(sign * 45.0F));
      }
      setStraightGeometry(inputs, 1200);
      inputs.sides[BackLeft].rangeMm = 500;
      inputs.sides[BackRight].rangeMm = 500;
      if (phase == 2) {
        inputs.sides[hazard].rangeMm = 144;
        const Decision decision = updateFresh(controller, inputs, 300,
                                               wrapYaw(sign * 90.0F));
        check(decision.state == RunState::Cornering &&
                  decision.cornerCount == 0 && decision.lapCount == 0 &&
                  decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
              "R38 a leading hazard on the corner-exit tick must stop before counting or starting the next straight");
        continue;
      }
      const float yaw = phase == 1 ? sign * 90.0F : 0.0F;
      if (phase == 1) updateFresh(controller, inputs, 300, wrapYaw(yaw));
      inputs.sides[hazard].rangeMm = 141;
      Decision decision = updateFresh(controller, inputs, 400, wrapYaw(yaw));
      check(decision.wallRecoveryPhase == 0,
            "R38 straight leading-side protection must preserve the exact140mm boundary");
      inputs.sides[hazard].rangeMm = 140;
      decision = updateFresh(controller, inputs, 410, wrapYaw(yaw));
      check(decision.state == (phase == 1 ? RunState::Running
                                         : RunState::DirectionProbe) &&
                decision.wallRecoveryPhase == 1 && decision.speedPermille == 0 &&
                decision.steeringPermille == 0 &&
                decision.cornerCount == (phase == 1 ? 1 : 0),
            "R38 leading-side140 must preempt both direction probing and ordinary straight motion");
    }
  }
}

void testR38AttemptCapRequiresVerifiedSectionProgressToReset() {
  for (const bool clockwise : {false, true}) {
    for (const bool completeCorner : {false, true}) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      config.openingConfirmationMs = 0;
      config.cornerMinimumDurationMs = 0;
      config.cornerExitStableMs = 0;
      Controller controller(config);
      Inputs inputs;
      startController(controller, inputs);
      const float sign = clockwise ? 1.0F : -1.0F;
      const uint8_t hazard = clockwise ? FrontRight : FrontLeft;
      if (clockwise) setRightOpening(inputs, 650);
      else setLeftOpening(inputs, 650);
      inputs.sides[BackLeft].rangeMm = 500;
      inputs.sides[BackRight].rangeMm = 500;
      updateFresh(controller, inputs, 100, 0.0F);
      updateFresh(controller, inputs, 200, wrapYaw(sign * 40.0F));
      uint32_t nowMs = 210;
      Decision decision;
      for (int attempt = 0; attempt < 3; ++attempt) {
        inputs.front.rangeMm = 900;
        inputs.sides[hazard].rangeMm = 100;
        decision = updateFresh(controller, inputs, nowMs,
                               wrapYaw(sign * 40.0F));
        check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
              "R38 the first three attempts within one section may enter neutral recovery");
        updateFresh(controller, inputs, nowMs + 300, wrapYaw(sign * 40.0F));
        inputs.sides[hazard].rangeMm = 400;
        updateFresh(controller, inputs, nowMs + 450, wrapYaw(sign * 40.0F));
        updateFresh(controller, inputs, nowMs + 500, wrapYaw(sign * 40.0F));
        decision = updateFresh(controller, inputs, nowMs + 550,
                               wrapYaw(sign * 40.0F));
        check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
              "R38 each bounded attempt must stop after powered reverse and stable initiating-ray clearance");
        updateFresh(controller, inputs, nowMs + 850, wrapYaw(sign * 40.0F));
        decision = updateFresh(controller, inputs, nowMs + 860,
                               wrapYaw(sign * 40.0F));
        check(decision.state == RunState::Cornering && decision.cornerCount == 0,
              "R38 successful reverse alone must never reset the attempt cap by awarding a corner");
        nowMs += 900;
      }
      float yaw = sign * 40.0F;
      if (completeCorner) {
        setStraightGeometry(inputs, 1200);
        inputs.sides[BackLeft].rangeMm = 500;
        inputs.sides[BackRight].rangeMm = 500;
        updateFresh(controller, inputs, nowMs, wrapYaw(sign * 65.0F));
        nowMs += 100;
        decision = updateFresh(controller, inputs, nowMs, wrapYaw(sign * 90.0F));
        check(decision.state == RunState::Running && decision.cornerCount == 1,
              "R38 resetting retries must require an independently verified corner exit");
        yaw = sign * 90.0F;
        nowMs += 100;
      }
      inputs.sides[hazard].rangeMm = 100;
      decision = updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      check(completeCorner
                ? decision.wallRecoveryPhase == 1 &&
                      decision.faultFlags == autonomy::FaultNone
                : decision.state == RunState::FaultLatched &&
                      decision.speedPermille == 0,
            completeCorner
                ? "R38 a verified new section must restore the bounded recovery attempts"
                : "R38 a fourth recovery in the same section must latch stopped rather than retry indefinitely");
    }
  }
}

void enterR39OffCenterPillarAfterCorner(Controller& controller, Inputs& inputs,
                                     bool redPillar) {
  startController(controller, inputs);
  const float sign = redPillar ? 1.0F : -1.0F;
  if (redPillar) setRightOpening(inputs, 650);
  else setLeftOpening(inputs, 650);
  updateFresh(controller, inputs, 100, 0.0F);
  updateFresh(controller, inputs, 200, wrapYaw(sign * 45.0F));
  updateFresh(controller, inputs, 280, wrapYaw(sign * 85.0F));
  inputs.front.rangeMm = 434;
  inputs.sides[FrontLeft].rangeMm = redPillar ? 707 : 484;
  inputs.sides[BackLeft].rangeMm = redPillar ? 598 : 243;
  inputs.sides[FrontRight].rangeMm = redPillar ? 484 : 707;
  inputs.sides[BackRight].rangeMm = redPillar ? 243 : 598;
  auto& pillar = redPillar ? inputs.vision.red : inputs.vision.green;
  pillar = usableBlob(redPillar ? 5987 : -5987, 7490, 3375, 940);
  pillar.width = 1219;
  Decision decision = updateFresh(controller, inputs, 300,
                                  wrapYaw(sign * 93.87F));
  const ObstacleTarget target = redPillar ? ObstacleTarget::RedPassRight
                                        : ObstacleTarget::GreenPassLeft;
  check(decision.state == RunState::Running && decision.cornerCount == 1 &&
            decision.obstacle == target && decision.speedPermille > 0 &&
            (redPillar ? decision.steeringPermille > 0
                       : decision.steeringPermille < 0),
        "R39 newly committed off-center pillar must own away steering on the corner-exit tick");
  inputs.front.rangeMm = 370;
  pillar = usableBlob(redPillar ? 5329 : -5329, 8033, 3750, 940);
  pillar.width = 1313;
  decision = updateFresh(controller, inputs, 403,
                         wrapYaw(sign * 94.69F));
  check(decision.state == RunState::Running && decision.cornerCount == 1 &&
            decision.obstacle == target && decision.speedPermille > 0 &&
            (redPillar ? decision.steeringPermille > 0
                       : decision.steeringPermille < 0),
        "R39 previous-corner counterpulse must not overwrite live red-right or green-left avoidance during its150ms window");
}

void testR39RearTravelLimitAllowsGuardedForwardReplanning() {
  for (const bool redPillar : {false, true}) {
    for (int departureCase = 0; departureCase < 11; ++departureCase) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      config.preferForwardObstacleProgress = false;
      config.openingConfirmationMs = 0;
      config.cornerMinimumDurationMs = 0;
      config.cornerExitStableMs = 0;
      Controller controller(config);
      Inputs inputs;
      enterR39OffCenterPillarAfterCorner(controller, inputs, redPillar);
      const float sign = redPillar ? 1.0F : -1.0F;
      const uint8_t limitingRear = redPillar ? BackRight : BackLeft;
      const uint8_t oppositeRear = redPillar ? BackLeft : BackRight;
      const uint8_t leadingSide = redPillar ? FrontRight : FrontLeft;
      inputs.sides[limitingRear].valid = false;
      inputs.sides[limitingRear].rangeStatus = 2;
      inputs.sides[limitingRear].rangeMm = 0;
      inputs.front.rangeMm = 336;
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      Decision decision = updateFresh(controller, inputs, 480,
                                      wrapYaw(sign * 94.69F));
      check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
            "R39 observed front336 must begin neutral center recovery with a retained close pillar");
      decision = updateFresh(controller, inputs, 779, wrapYaw(sign * 94.69F));
      check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
            "R39 rear-limited replay must preserve the300ms entry neutral");
      inputs.front.rangeMm = 261;
      decision = updateFresh(controller, inputs, 780, wrapYaw(sign * 94.69F));
      check(decision.wallRecoveryPhase == 2 &&
                decision.driveDirection == DriveDirection::Reverse &&
                decision.speedPermille == 300,
            "R39 rear-limited replay must actually enter reverse");
      inputs.appliedDrivePermille = 300;
      inputs.front.rangeMm = 358;
      inputs.sides[limitingRear].valid = true;
      inputs.sides[limitingRear].rangeStatus = 0;
      inputs.sides[limitingRear].rangeMm = 388;
      decision = updateFresh(controller, inputs, 1030, wrapYaw(sign * 94.69F));
      check(decision.wallRecoveryPhase == 2 && decision.speedPermille > 0,
            "R39 fresh rear388 must permit reverse while the front recovers");
      inputs.front.rangeMm = 399;
      inputs.sides[limitingRear].rangeMm = 273;
      decision = updateFresh(controller, inputs, 1130, wrapYaw(sign * 94.56F));
      check(decision.wallRecoveryPhase == 3 &&
                decision.wallRecoveryExitReason == 2 &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.speedPermille == 0,
            "R39 observed rear273 must brake reverse early into neutral instead of waiting for rear198 and a geometry fault");
      inputs.appliedDrivePermille = 0;
      inputs.front.rangeMm = departureCase == 1 ? 399 : 471;
      inputs.sides[limitingRear].rangeMm = departureCase >= 6 ? 273 : 198;
      if (departureCase == 2) inputs.sides[leadingSide].rangeMm = 200;
      if (departureCase == 5) inputs.sides[oppositeRear].rangeMm = 198;
      if (departureCase >= 6) inputs.sides[oppositeRear].rangeMm = 273;
      if (departureCase == 10) {
        inputs.vision.red.present = false;
        inputs.vision.green.present = false;
      }
      decision = updateFresh(controller, inputs, 1230, wrapYaw(sign * 94.25F));
      check(decision.wallRecoveryPhase == 3 &&
                decision.faultFlags == autonomy::FaultNone && decision.speedPermille == 0,
            "R39 fresh rear198 during exit neutral must constrain reverse without falsely becoming unavailable geometry");
      if (departureCase == 3 || departureCase == 4) {
        inputs.nowMs = 1300;
        inputs.imu.updatedAtMs = 1300;
        inputs.front.updatedAtMs = 1300;
        inputs.vision.updatedAtMs = 1300;
        for (auto& side : inputs.sides) side.updatedAtMs = 1300;
        if (departureCase == 3) inputs.sides[limitingRear].updatedAtMs = 0;
        else inputs.sides[limitingRear].i2cStatus = 4;
        decision = controller.update(inputs);
        check(decision.speedPermille == 0 && !decision.motionAllowed &&
                  decision.driveDirection == DriveDirection::Stopped,
              "R39 stale or electrical rear failure must still prevent either departure direction");
        continue;
      }
      decision = updateFresh(controller, inputs, 1429, wrapYaw(sign * 94.25F));
      check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
            "R39 rear-limit forward replanning must preserve the full300ms exit neutral");
      decision = updateFresh(controller, inputs, 1430, wrapYaw(sign * 94.25F));
      uint32_t forwardCheckAt = 1450;
      if (departureCase == 1 || departureCase == 2 || departureCase == 10) {
        check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
              "R39 front399, leading200, or front471 with only an absent stale commitment must keep rear-limited departure neutral");
        inputs.front.rangeMm = departureCase == 10 ? 650 : 400;
        inputs.sides[leadingSide].rangeMm = 201;
        if (departureCase == 10) {
          inputs.sides[BackLeft].rangeMm = 500;
          inputs.sides[BackRight].rangeMm = 500;
        }
        updateFresh(controller, inputs, 1440, wrapYaw(sign * 94.25F));
        updateFresh(controller, inputs, 1490, wrapYaw(sign * 94.25F));
        updateFresh(controller, inputs, 1540, wrapYaw(sign * 94.25F));
        forwardCheckAt = 1550;
      }
      if (departureCase == 6) inputs.sides[oppositeRear].rangeMm = 198;
      if (departureCase >= 7 && departureCase < 10) {
        inputs.sides[oppositeRear].valid = false;
        inputs.sides[oppositeRear].rangeMm = 0;
        inputs.sides[oppositeRear].rangeStatus =
            departureCase == 7 ? 1 : departureCase == 8 ? 6 : 7;
      }
      decision = updateFresh(controller, inputs, forwardCheckAt,
                             wrapYaw(sign * 94.25F));
      check(decision.state == RunState::Running &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.obstacle == (departureCase == 10 ? ObstacleTarget::None
                    : redPillar ? ObstacleTarget::RedPassRight
                                : ObstacleTarget::GreenPassLeft) &&
                decision.cornerCount == 1 && decision.lapCount == 0 &&
                std::fabs(decision.cornerTargetHeadingDegrees - sign * 90.0F) < 0.01F,
            "R39 rear-limit departure must retain the current pillar and cardinal progress without a fake pass or corner");
      if (departureCase >= 5 && departureCase < 10) {
        check(decision.speedPermille == 0 &&
                  decision.driveDirection == DriveDirection::Stopped,
              "R39 constrained opposite rear becoming198 or weak1/6/7 must prevent the otherwise desired away turn");
      } else {
        check(decision.speedPermille > 0 &&
                  decision.driveDirection == DriveDirection::Forward &&
                  (departureCase == 10 ||
                   (redPillar ? decision.steeringPermille > 0
                              : decision.steeringPermille < 0)),
              "R39 forward-clear rear-limited recovery must resume the same pillar with mirrored away steering");
      }
    }
  }
}

void testR39UnplannedForwardDepartureRemainsBounded() {
  autonomy::Config config = autonomy::makeGuardedObstacleConfig();
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  inputs.sides[BackLeft].rangeMm = 500;
  inputs.sides[BackRight].rangeMm = 500;
  inputs.front.rangeMm = 300;
  inputs.driveFeedbackAvailable = true;
  inputs.appliedDrivePermille = 0;
  updateFresh(controller, inputs, 100, 0.0F);
  updateFresh(controller, inputs, 400, 0.0F);
  inputs.front.rangeMm = 471;
  inputs.sides[BackRight].rangeMm = 273;
  Decision decision = updateFresh(controller, inputs, 650, 0.0F);
  check(decision.wallRecoveryPhase == 3 && decision.wallRecoveryExitReason == 2 &&
            decision.speedPermille == 0,
        "R39 rear travel limit must stop reverse even if the drivetrain has applied no motion");
  inputs.appliedDrivePermille = 300;
  decision = updateFresh(controller, inputs, 950, 0.0F);
  check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
        "R39 unplanned direction probing must not depart with only471mm center clearance");
  decision = updateFresh(controller, inputs, 3100, 0.0F);
  check(decision.state == RunState::FaultLatched && decision.speedPermille == 0,
        "R39 unavailable forward-route neutral hold must remain bounded by the recovery wall-clock limit");
}

void testR39ReverseRequiresStrongRearEvidenceAndHonorsHardLimit() {
  for (const uint8_t rear : {uint8_t{BackLeft}, uint8_t{BackRight}}) {
    for (const uint8_t weakStatus : {uint8_t{1}, uint8_t{6}, uint8_t{7}}) {
      Controller controller(autonomy::makeGuardedObstacleConfig());
      Inputs inputs;
      startController(controller, inputs);
      inputs.sides[BackLeft].rangeMm = 500;
      inputs.sides[BackRight].rangeMm = 500;
      inputs.front.rangeMm = 300;
      updateFresh(controller, inputs, 100, 0.0F);
      updateFresh(controller, inputs, 400, 0.0F);
      inputs.sides[rear].valid = false;
      inputs.sides[rear].rangeStatus = weakStatus;
      inputs.sides[rear].rangeMm = 0;
      const Decision decision = updateFresh(controller, inputs, 500, 0.0F);
      check(decision.speedPermille == 0 &&
                decision.driveDirection != DriveDirection::Reverse,
            "R39 reverse must stop when either rear becomes a fresh but weak optical1/6/7 observation");
    }
  }
  for (const bool rightHazard : {false, true}) {
    Controller controller(autonomy::makeGuardedObstacleConfig());
    Inputs inputs;
    startController(controller, inputs);
    const uint8_t leading = rightHazard ? FrontRight : FrontLeft;
    const uint8_t rear = rightHazard ? BackLeft : BackRight;
    inputs.sides[BackLeft].rangeMm = 500;
    inputs.sides[BackRight].rangeMm = 500;
    inputs.sides[leading].rangeMm = 100;
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 400, 0.0F);
    inputs.appliedDrivePermille = 300;
    inputs.front.rangeMm = 399;
    inputs.sides[leading].rangeMm = 400;
    inputs.sides[rear].rangeMm = 273;
    Decision decision = updateFresh(controller, inputs, 500, 0.0F);
    check(decision.wallRecoveryPhase == 3 && decision.wallRecoveryExitReason == 2 &&
              decision.speedPermille == 0,
          "R39 a rear travel limit after only100ms reverse must brake immediately");
    inputs.appliedDrivePermille = 0;
    inputs.front.rangeMm = 650;
    inputs.sides[rear].rangeMm = 198;
    updateFresh(controller, inputs, 550, 0.0F);
    updateFresh(controller, inputs, 600, 0.0F);
    decision = updateFresh(controller, inputs, 799, 0.0F);
    check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
          "R39 hard rear departure must still honor the full300ms neutral interval");
    decision = updateFresh(controller, inputs, 800, 0.0F);
    check(decision.wallRecoveryPhase == 0 && decision.faultFlags == autonomy::FaultNone &&
              decision.cornerCount == 0 && decision.lapCount == 0,
          "R39 hardrear198 with front650 and freshly confirmed forward and flank clearance may replan without a minimum reverse duration");
  }
}

void testR39TimeLimitedReverseReplansOnlyAnExistingManeuver() {
  for (const bool redPillar : {false, true}) {
    for (int maneuver = 0; maneuver < 4; ++maneuver) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      // This legacy variant resumes the same pillar after reverse. R42's
      // progress mode instead skips it; its rear-limited departure has a
      // separate production-profile regression below.
      if (maneuver == 0) config.preferForwardObstacleProgress = false;
      config.openingConfirmationMs = 0;
      config.cornerMinimumDurationMs = 0;
      config.cornerExitStableMs = 0;
      Controller controller(config);
      Inputs inputs;
      const float sign = redPillar ? 1.0F : -1.0F;
      const uint8_t leading = redPillar ? FrontRight : FrontLeft;
      uint32_t triggerAt = 210;
      float yaw = sign * 20.0F;
      if (maneuver == 0) {
        enterR39OffCenterPillarAfterCorner(controller, inputs, redPillar);
        triggerAt = 480;
        yaw = sign * 94.69F;
        inputs.front.rangeMm = 336;
      } else {
        startController(controller, inputs);
        if (maneuver == 3) {
          triggerAt = 100;
          yaw = 0.0F;
          inputs.front.rangeMm = 300;
        } else {
          if (redPillar) setRightOpening(inputs, 650);
          else setLeftOpening(inputs, 650);
          updateFresh(controller, inputs, 100, 0.0F);
          updateFresh(controller, inputs, 200, wrapYaw(yaw));
          inputs.front.rangeMm = maneuver == 1 ? 250 : 790;
          if (maneuver == 2) inputs.sides[leading].rangeMm = 144;
        }
      }
      inputs.sides[BackLeft].valid = true;
      inputs.sides[BackLeft].rangeStatus = 0;
      inputs.sides[BackLeft].rangeMm = 500;
      inputs.sides[BackRight].valid = true;
      inputs.sides[BackRight].rangeStatus = 0;
      inputs.sides[BackRight].rangeMm = 500;
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      updateFresh(controller, inputs, triggerAt, wrapYaw(yaw));
      updateFresh(controller, inputs, triggerAt + 300, wrapYaw(yaw));
      inputs.appliedDrivePermille = 300;
      inputs.front.rangeMm = 471;
      Decision decision = updateFresh(controller, inputs, triggerAt + 1799,
                                      wrapYaw(yaw));
      check(decision.wallRecoveryPhase == 2 &&
                decision.driveDirection == DriveDirection::Reverse &&
                decision.speedPermille > 0,
            "R39 time-limited retry must retain reverse through1499ms actual drive when its original hazard is not fully cleared");
      decision = updateFresh(controller, inputs, triggerAt + 1800, wrapYaw(yaw));
      check(decision.wallRecoveryPhase == 3 &&
                decision.wallRecoveryExitReason == 3 && decision.speedPermille == 0,
            "R39 exact1500ms powered limit must stop into a distinct time-limit exit reason");
      inputs.appliedDrivePermille = 0;
      decision = updateFresh(controller, inputs, triggerAt + 2099, wrapYaw(yaw));
      check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
            "R39 time-limit forward retry must preserve the entire300ms neutral interval");
      decision = updateFresh(controller, inputs, triggerAt + 2100, wrapYaw(yaw));
      if (maneuver == 3) {
        check(decision.state == RunState::FaultLatched && decision.speedPermille == 0,
              "R39 ordinary direction probing without a planned pass or corner must retain the full650mm center-clear requirement");
        continue;
      }
      uint32_t resumeAt = triggerAt + 2110;
      if (maneuver == 2) {
        check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
              "R39 time-limit replanning must not discard the original uncleared side hazard");
        inputs.sides[leading].rangeMm = 301;
        updateFresh(controller, inputs, triggerAt + 2110, wrapYaw(yaw));
        updateFresh(controller, inputs, triggerAt + 2160, wrapYaw(yaw));
        updateFresh(controller, inputs, triggerAt + 2210, wrapYaw(yaw));
        resumeAt = triggerAt + 2220;
      }
      decision = updateFresh(controller, inputs, resumeAt, wrapYaw(yaw));
      check(decision.wallRecoveryPhase == 0 &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.state == (maneuver == 0 ? RunState::Running
                                                 : RunState::Cornering) &&
                decision.cornerCount == (maneuver == 0 ? 1 : 0) &&
                decision.lapCount == 0 &&
                decision.driveDirection == DriveDirection::Forward &&
                decision.speedPermille > 0 &&
                std::fabs(decision.cornerTargetHeadingDegrees - sign * 90.0F) < 0.01F,
            "R39 front471 and a verified forward route may resume an existing pillar or corner after bounded reverse without inventing progress");
    }
  }
}

void testR39ShortRearBrakeRequiresFreshForwardRouteConfirmation() {
  for (const bool redPillar : {false, true}) {
    for (const uint32_t poweredMs : {1U, 100U, 249U}) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      config.openingConfirmationMs = 0;
      config.cornerMinimumDurationMs = 0;
      config.cornerExitStableMs = 0;
      Controller controller(config);
      Inputs inputs;
      enterR39OffCenterPillarAfterCorner(controller, inputs, redPillar);
      const float yaw = redPillar ? 94.69F : -94.69F;
      const uint8_t limitingRear = redPillar ? BackRight : BackLeft;
      inputs.sides[BackLeft].rangeMm = 500;
      inputs.sides[BackRight].rangeMm = 500;
      inputs.front.rangeMm = 336;
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      updateFresh(controller, inputs, 480, wrapYaw(yaw));
      updateFresh(controller, inputs, 780, wrapYaw(yaw));
      const uint32_t rearBrakeAt = 780 + poweredMs;
      inputs.appliedDrivePermille = 300;
      inputs.front.rangeMm = 399;
      inputs.sides[limitingRear].rangeMm = 273;
      Decision decision = updateFresh(controller, inputs, rearBrakeAt, wrapYaw(yaw));
      check(decision.wallRecoveryPhase == 3 && decision.wallRecoveryExitReason == 2 &&
                decision.speedPermille == 0,
            "R39 rear brake after1to249ms must stop promptly without requiring more reverse travel");
      inputs.appliedDrivePermille = 0;
      inputs.front.rangeMm = 471;
      updateFresh(controller, inputs, rearBrakeAt + 50, wrapYaw(yaw));
      updateFresh(controller, inputs, rearBrakeAt + 60, wrapYaw(yaw));
      updateFresh(controller, inputs, rearBrakeAt + 70, wrapYaw(yaw));
      inputs.nowMs = rearBrakeAt + 300;
      inputs.imu.updatedAtMs = inputs.nowMs;
      inputs.vision.updatedAtMs = inputs.nowMs;
      inputs.sides[BackLeft].updatedAtMs = inputs.nowMs;
      inputs.sides[BackRight].updatedAtMs = inputs.nowMs;
      decision = controller.update(inputs);
      check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
            "R39 three fast route samples plus neutral wall time must not fake100ms fresh forward-route confirmation");
      decision = updateFresh(controller, inputs, rearBrakeAt + 310, wrapYaw(yaw));
      check(decision.wallRecoveryPhase == 0 && decision.speedPermille == 0,
            "R39 a newly confirmed forward route may release neutral after short reverse without a250ms dead end");
      decision = updateFresh(controller, inputs, rearBrakeAt + 320, wrapYaw(yaw));
      check(decision.state == RunState::Running &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.speedPermille > 0 &&
                decision.driveDirection == DriveDirection::Forward &&
                decision.cornerCount == 1 && decision.lapCount == 0,
            "R39 short rear-limited recovery must resume the retained maneuver only after confirmed forward-route evidence");
    }
  }
}

void testR42SkippedPillarCanLeaveRearLimitAlongKnownStraight() {
  for (const bool redPillar : {false, true}) {
    for (int routeCase = 0; routeCase < 7; ++routeCase) {
      autonomy::Config config = autonomy::makeGuardedObstacleConfig();
      config.openingConfirmationMs = 0;
      config.cornerMinimumDurationMs = 0;
      config.cornerExitStableMs = 0;
      Controller controller(config);
      Inputs inputs;
      enterR39OffCenterPillarAfterCorner(controller, inputs, redPillar);
      const float sign = redPillar ? 1.0F : -1.0F;
      const float yaw = sign * (routeCase == 3 ? 100.0F : 94.0F);
      const uint8_t limitingRear = redPillar ? BackRight : BackLeft;
      inputs.sides[FrontLeft].rangeMm = 600;
      inputs.sides[FrontRight].rangeMm = 600;
      inputs.sides[BackLeft].rangeMm = 500;
      inputs.sides[BackRight].rangeMm = 500;
      inputs.front.rangeMm = 336;
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      updateFresh(controller, inputs, 480, yaw);
      updateFresh(controller, inputs, 780, yaw);
      inputs.appliedDrivePermille = 300;
      inputs.front.rangeMm = 399;
      inputs.sides[limitingRear].rangeMm = 273;
      Decision decision = updateFresh(controller, inputs, 880, yaw);
      check(decision.wallRecoveryPhase == 3 &&
                decision.wallRecoveryExitReason == 2 && decision.speedPermille == 0,
            "R42 partial skip route must first brake actual reverse at the rear travel limit");
      inputs.appliedDrivePermille = 0;
      inputs.front.rangeMm = 471;
      inputs.sides[limitingRear].rangeMm = 198;
      if (routeCase == 4) {
        auto& otherPillar = redPillar ? inputs.vision.green : inputs.vision.red;
        otherPillar = usableBlob(redPillar ? -5000 : 5000, 8000, 3000, 950);
      }
      updateFresh(controller, inputs, 930, yaw);
      updateFresh(controller, inputs, 980, yaw);
      updateFresh(controller, inputs, 1080, yaw);
      decision = updateFresh(controller, inputs, 1179, yaw);
      check(decision.speedPermille == 0,
            "R42 known-straight partial departure must preserve the entire300ms exit neutral");
      updateFresh(controller, inputs, 1180, yaw);
      decision = updateFresh(controller, inputs, 1190, yaw);
      if (routeCase == 3 || routeCase == 4) {
        check(!(decision.driveDirection == DriveDirection::Forward &&
                decision.speedPermille > 0),
              "R42 heading beyond8degrees or a fresh opposite pillar with blocked tail must not use the skipped-target straight departure exception");
        continue;
      }
      check(decision.state == RunState::Running &&
                decision.faultFlags == autonomy::FaultNone &&
                decision.obstacle == ObstacleTarget::None &&
                decision.driveDirection == DriveDirection::Forward &&
                decision.speedPermille >= 300 && decision.speedPermille <= 400 &&
                decision.steeringPermille == 0 &&
                decision.cornerCount == 1 && decision.lapCount == 0,
            "R42 after skipping the old pillar front471 and rear198 must permit bounded straight departure on the known near-cardinal course without fake progress");
      if (routeCase == 0) {
        inputs.sides[limitingRear].rangeMm = 500;
        decision = updateFresh(controller, inputs, 1290, yaw);
        check(decision.state == RunState::Running &&
                  decision.speedPermille > 0 &&
                  (redPillar ? decision.steeringPermille < 0
                             : decision.steeringPermille > 0) &&
                  decision.cornerCount == 1,
              "R42 once the tail clears the bounded straight departure must return to ordinary heading correction");
      } else if (routeCase == 1) {
        inputs.appliedDrivePermille = 400;
        decision = updateFresh(controller, inputs, 1689, yaw);
        check(decision.driveDirection == DriveDirection::Forward &&
                  decision.speedPermille == 400 && decision.steeringPermille == 0,
              "R42 constrained-tail straight departure may continue through499ms actual drive");
        decision = updateFresh(controller, inputs, 1690, yaw);
        check(decision.speedPermille == 0,
              "R42 an uncleared tail must stop its straight departure exactly at500ms applied drive");
      } else if (routeCase == 2) {
        inputs.vision.red = {};
        inputs.vision.green = {};
        decision = updateFresh(controller, inputs, 2689, yaw);
        check(decision.driveDirection == DriveDirection::Forward &&
                  decision.speedPermille == 400,
              "R42 unpowered departure waiting must not spend the applied-drive budget");
        decision = updateFresh(controller, inputs, 2690, yaw);
        check(decision.speedPermille == 0,
              "R42 constrained-tail straight departure remains bounded by1500ms wall time when no drive is applied");
      } else if (routeCase == 5) {
        inputs.front.rangeMm = 350;
        decision = updateFresh(controller, inputs, 1290, yaw);
        check(!(decision.driveDirection == DriveDirection::Forward &&
                decision.speedPermille > 0),
              "R42 a newly close front350 must immediately stop the otherwise permitted straight tail departure");
      } else {
        auto& otherPillar = redPillar ? inputs.vision.green : inputs.vision.red;
        otherPillar = usableBlob(redPillar ? -5000 : 5000, 8000, 3000, 950);
        decision = updateFresh(controller, inputs, 1290, yaw);
        check(decision.obstacle == (redPillar ? ObstacleTarget::GreenPassLeft
                                             : ObstacleTarget::RedPassRight) &&
                  !(decision.driveDirection == DriveDirection::Forward &&
                    decision.speedPermille > 0),
              "R42 a fresh other pillar appearing after departure must disable straight creep when its required turn would sweep into the blocked rear");
      }
    }
  }
}

void testR35WallReverseRecoveryIsBoundedAndGuarded() {
  const autonomy::Config config = autonomy::makeGuardedObstacleConfig();
  check(config.wallRecoveryEnabled,
        "R35 full profile must explicitly enable bounded wall reverse recovery");
  {
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 149;
    inputs.sides[BackLeft].rangeMm = 500;
    inputs.sides[BackRight].rangeMm = 500;
    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart &&
              decision.speedPermille == 0 &&
              decision.driveDirection != DriveDirection::Forward,
          "R35 front149 with clear rear observations may arm without any forward pulse");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    decision = updateFresh(controller, inputs, 90, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.wallRecoveryPhase == 1 &&
              decision.speedPermille == 0 &&
              decision.driveDirection != DriveDirection::Forward,
          "R35 near-wall start must enter neutral recovery before requesting motion");
    decision = updateFresh(controller, inputs, 389, 0.0F);
    check(decision.wallRecoveryPhase == 1 && decision.speedPermille == 0,
          "R35 entry neutral must last its full300ms");
    decision = updateFresh(controller, inputs, 390, 0.0F);
    check(decision.wallRecoveryPhase == 2 &&
              decision.driveDirection == DriveDirection::Reverse &&
              decision.speedPermille == 300,
          "R35 rear-clear recovery may reverse at300 only after its exact neutral bound");
    inputs.front.rangeMm = 650;
    decision = updateFresh(controller, inputs, 500, 0.0F);
    check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
          "R35 front650 must stop reverse and begin exit neutral");
    decision = updateFresh(controller, inputs, 799, 0.0F);
    check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
          "R35 reverse-to-forward neutral must last its full300ms");
    decision = updateFresh(controller, inputs, 800, 0.0F);
    check(decision.wallRecoveryPhase == 0 && decision.speedPermille == 0,
          "R35 completed exit neutral must release recovery without a forward transition pulse");
    decision = updateFresh(controller, inputs, 810, 0.0F);
    check(decision.wallRecoveryPhase == 0 &&
              decision.state == RunState::DirectionProbe &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.driveDirection == DriveDirection::Forward &&
              decision.speedPermille > 0,
          "R35 rear-clear recovery must resume ordinary navigation after exit neutral");
  }

  {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[BackLeft].rangeMm = 500;
    inputs.sides[BackRight].rangeMm = 500;
    inputs.front.rangeMm = 300;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.wallRecoveryPhase == 1 &&
              decision.speedPermille == 0 &&
              decision.faultFlags == autonomy::FaultNone,
          "R35 moving approach atfront300 must enter neutral wall recovery before a terminal navigation fault");
    inputs.driveFeedbackAvailable = true;
    inputs.appliedDrivePermille = 0;
    decision = updateFresh(controller, inputs, 400, 0.0F);
    check(decision.wallRecoveryPhase == 2 &&
              decision.driveDirection == DriveDirection::Reverse,
          "R35 wall recovery must request reverse after entry neutral");
    inputs.appliedDrivePermille = 300;
    decision = updateFresh(controller, inputs, 1899, 0.0F);
    check(decision.wallRecoveryPhase == 2 &&
              decision.driveDirection == DriveDirection::Reverse &&
              decision.speedPermille > 0,
          "R35 reverse may continue through1499ms applied drive while all guards remain clear");
    decision = updateFresh(controller, inputs, 1900, 0.0F);
    check(decision.wallRecoveryPhase == 3 &&
              decision.speedPermille == 0 &&
              decision.driveDirection != DriveDirection::Forward,
          "R35 reverse must stop at1500ms powered time without a hopeful forward retry if front remains blocked");
    inputs.appliedDrivePermille = 0;
    decision = updateFresh(controller, inputs, 2199, 0.0F);
    check(decision.wallRecoveryPhase == 3 && decision.speedPermille == 0,
          "R35 exhausted reverse must remain neutral through the exit interlock");
    decision = updateFresh(controller, inputs, 2200, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              decision.speedPermille == 0 &&
              decision.driveDirection != DriveDirection::Forward,
          "R35 exhausted reverse must latch after exit neutral when the wall remains close");
  }

  for (int failure = 0; failure < 3; ++failure) {
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[BackLeft].rangeMm = 500;
    inputs.sides[BackRight].rangeMm = 500;
    inputs.front.rangeMm = 300;
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 400, 0.0F);
    inputs.nowMs = 401;
    inputs.imu.updatedAtMs = 401;
    inputs.vision.updatedAtMs = 401;
    inputs.front.updatedAtMs = 401;
    for (auto& side : inputs.sides) side.updatedAtMs = 401;
    if (failure == 0) inputs.sides[BackLeft].rangeMm = 200;
    if (failure == 1) inputs.sides[BackRight].updatedAtMs = 0;
    if (failure == 2) inputs.sides[BackRight].i2cStatus = 4;
    const Decision decision = controller.update(inputs);
    check(decision.speedPermille == 0 &&
              decision.driveDirection != DriveDirection::Forward,
          "R35 reverse must stop immediately for rear200, stale rear, or electrical failure");
  }

  {
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.rangeMm = 149;
    inputs.sides[BackLeft].rangeMm = 200;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              decision.speedPermille == 0,
          "R35 a rear-blocked near-wall pose must not gain the reverse-start arming exception");
  }

  {
    autonomy::Config retryConfig = config;
    retryConfig.openingConfirmationMs = 0;
    retryConfig.cornerMinimumDurationMs = 0;
    retryConfig.cornerExitStableMs = 0;
    Controller controller(retryConfig);
    Inputs inputs;
    startController(controller, inputs);
    setRightOpening(inputs, 650);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 200, 45.0F);
    setStraightGeometry(inputs, 1200);
    updateFresh(controller, inputs, 300, 90.0F);
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
      side.rangeMm = 0;
    }
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeStatus = 0;
    inputs.sides[FrontLeft].rangeMm = 500;
    Decision decision = updateFresh(controller, inputs, 400, 90.0F);
    check(decision.state == RunState::Running && decision.speedPermille > 0,
          "R35 transient-retry fixture must start with one-local-ray running guidance");
    const uint8_t cornerCount = decision.cornerCount;
    const uint8_t lapCount = decision.lapCount;
    const auto updateWithoutCamera = [&](uint32_t nowMs) {
      inputs.nowMs = nowMs;
      inputs.imu.updatedAtMs = nowMs;
      inputs.front.updatedAtMs = nowMs;
      for (auto& side : inputs.sides) side.updatedAtMs = nowMs;
      inputs.vision.valid = false;
      inputs.vision.updatedAtMs = 0;
      return controller.update(inputs);
    };
    updateWithoutCamera(500);
    decision = updateWithoutCamera(2601);
    check(decision.state == RunState::Running &&
              decision.speedPermille == 0 && !decision.motionAllowed &&
              (decision.faultFlags & autonomy::FaultVisionUnavailable) != 0U &&
              (decision.faultFlags & autonomy::FaultSideGeometryUnavailable) != 0U,
          "R35 camera loss beyond geometry grace must hold zero without terminally latching combined vision and geometry faults");
    inputs.vision.valid = true;
    decision = updateFresh(controller, inputs, 2602, 90.0F);
    check(decision.state == RunState::Running &&
              decision.faultFlags == autonomy::FaultNone,
          "R35 fresh camera must immediately clear the transient health fault without another button");
    decision = updateFresh(controller, inputs, 2611, 90.0F);
    if (decision.faultFlags != autonomy::FaultNone || decision.speedPermille == 0) {
      std::cerr << "R35 retry state=" << static_cast<int>(decision.state)
                << " fault=" << decision.faultFlags
                << " speed=" << decision.speedPermille
                << " corners=" << static_cast<int>(decision.cornerCount)
                << " laps=" << static_cast<int>(decision.lapCount) << '\n';
    }
    check(decision.state == RunState::Running &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille > 0 && decision.motionAllowed &&
              decision.cornerCount == cornerCount && decision.lapCount == lapCount,
          "R35 fresh camera must resume one-ray running without another button or changing course progress");
    updateWithoutCamera(2700);
    decision = updateWithoutCamera(170090);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultRunTimeout) != 0U &&
              decision.speedPermille == 0,
          "R35 transient sensor waiting must still obey the170second overall run limit");
  }
}

void testR31FullProfileStillFinishesAfterTwelveCorners(uint8_t recoveryPolicy = 0U) {
  for (const bool clockwise : {true, false}) {
    autonomy::Config config = recoveryPolicy == 0U
        ? autonomy::makeGuardedObstacleConfig()
        : autonomy::makeObstacleFallbackConfig(recoveryPolicy == 2U);
    // Collapse only observation/turn waiting in this course-state fixture.
    // The real bounded turn and sensor-freshness timings have dedicated tests.
    config.openingConfirmationMs = 0;
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    config.cornerRefractoryMs = 0;
    config.cornerRearmStableMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    uint32_t nowMs = 100;
    float yaw = 0.0F;
    const float sign = clockwise ? 1.0F : -1.0F;
    Decision decision;
    for (uint8_t corner = 0; corner < 12; ++corner) {
      if (corner > 0) {
        setStraightGeometry(inputs, 1200);
        updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
        nowMs += 10;
        updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
        nowMs += 10;
      }
      if (clockwise) {
        setRightOpening(inputs, 650);
      } else {
        setLeftOpening(inputs, 650);
      }
      decision = updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      check(decision.state == RunState::Cornering &&
                decision.cornerCount == corner,
            "R31 full profile must enter each observed course corner once");
      nowMs += 100;
      yaw += sign * 45.0F;
      updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      nowMs += 100;
      yaw += sign * 45.0F;
      setStraightGeometry(inputs, 1200);
      decision = updateFresh(controller, inputs, nowMs, wrapYaw(yaw));
      check(decision.cornerCount == static_cast<uint8_t>(corner + 1) &&
                decision.faultFlags == autonomy::FaultNone,
            "R31 full profile must count each aligned corner without a geometry fault");
      nowMs += 100;
    }
    check(decision.state == RunState::FinishDelay &&
              decision.cornerCount == 12 && decision.lapCount == 3 &&
              decision.parkingPhase == ParkingPhase::Disabled,
          "R31 full profile must preserve the twelve-corner three-lap finish with parking disabled");
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
      side.rangeMm = 0;
    }
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeStatus = 0;
    inputs.sides[FrontLeft].rangeMm = 300;
    const uint32_t finishEnteredAt = nowMs - 100;
    if (recoveryPolicy != 0U) {
      inputs.driveFeedbackAvailable = true;
      inputs.appliedDrivePermille = 0;
      if (recoveryPolicy == 1U) {
        inputs.vision.online = false;
        inputs.vision.valid = false;
      } else {
        inputs.front.valid = false;
        inputs.front.rangeStatus = 7;
        inputs.front.rangeMm = 0;
      }
      for (const uint32_t offset : {50U, 500U, 2500U}) {
        decision = updateFresh(controller, inputs, finishEnteredAt + offset, wrapYaw(yaw));
        check(decision.state == RunState::FinishDelay &&
                  decision.speedPermille == 0 && decision.cornerCount == 12 &&
                  decision.lapCount == 3,
              "R43/R44 stopped finish health holds must neither expire nor falsely finish the verified three-lap run");
      }
      inputs.vision.online = true;
      inputs.vision.valid = true;
      inputs.front.valid = true;
      inputs.front.rangeStatus = 0;
      uint32_t resumedAt = finishEnteredAt + 2600U;
      if (recoveryPolicy == 2U) {
        inputs.front.rangeMm = 500;
        for (const uint32_t offset : {2600U, 2700U, 2800U}) {
          decision = updateFresh(controller, inputs, finishEnteredAt + offset, wrapYaw(yaw));
          check(decision.state == RunState::FinishDelay &&
                    decision.speedPermille == 0 && decision.cornerCount == 12 &&
                    decision.lapCount == 3,
                "R44 optical finish recovery must retain its hold when numeric500 mm cannot prove the finish straight");
        }
        resumedAt = finishEnteredAt + 2900U;
      }
      inputs.front.rangeMm = 1200;
      decision = updateFresh(controller, inputs, resumedAt, wrapYaw(yaw));
      check(decision.state == RunState::FinishDelay &&
                decision.cornerCount == 12 && decision.lapCount == 3,
            "R43/R44 recovered finish sensing must rebuild finish proof instead of crediting stopped wall time");
      for (uint32_t offset = 100; offset <= 1100 && decision.state != RunState::Finished; offset += 50) {
        decision = updateFresh(controller, inputs, resumedAt + offset, wrapYaw(yaw));
      }
      check(decision.state == RunState::Finished &&
                decision.speedPermille == 0 && !decision.motionAllowed &&
                decision.cornerCount == 12 && decision.lapCount == 3,
            "R43/R44 fresh finish progress after a paused health hold must complete exactly three laps");
      inputs.buttonPressed = false;
      updateFresh(controller, inputs, resumedAt + 1200U, wrapYaw(yaw));
      inputs.buttonPressed = true;
      updateFresh(controller, inputs, resumedAt + 1250U, wrapYaw(yaw));
      decision = updateFresh(controller, inputs, resumedAt + 1300U, wrapYaw(yaw));
      check(decision.state == RunState::Finished && decision.speedPermille == 0,
            "R43/R44 the final three-lap stop must remain permanent after another Start press");
      continue;
    }
    updateFresh(controller, inputs,
                finishEnteredAt + config.finishDriveMs, wrapYaw(yaw));
    decision = updateFresh(controller, inputs,
                           finishEnteredAt + config.finishDriveMs +
                               config.finishStableMs,
                           wrapYaw(yaw));
    if (decision.state != RunState::Finished) {
      std::cerr << "R31 one-ray finish clockwise=" << clockwise
                << " state=" << static_cast<int>(decision.state)
                << " fault=" << decision.faultFlags << '\n';
    }
    check(decision.state == RunState::Finished &&
              decision.faultFlags == autonomy::FaultNone &&
              !decision.motionAllowed && decision.speedPermille == 0,
          "R31 full profile must finish all twelve corners with one fresh local ray and aligned heading");
  }
}

void testR25CalibrationBodyClearIsBoundedAndGuarded() {
  {
    autonomy::Config invalid =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    invalid.obstacleSinglePassBodyClearMs = 500;
    Controller controller(invalid);
    Inputs inputs = healthyInputs(0);
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags & FaultConfiguration) != 0U &&
              !decision.motionAllowed,
          "R25C body-clear time must fail closed outside single-pass calibration");
  }

  for (const bool redPillar : {true, false}) {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.stopAfterFirstVerifiedObstaclePass = true;
    config.obstacleSinglePassBodyClearMs = 500;
    config.maxRunMs = 5000;
    config.straightSideGeometryGraceMs = 2000;
    config.obstacleSideClearanceGuardMm = 200;
    config.obstacleBodyClearOppositeSideGuardMm = 140;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setStraightGeometry(inputs, 1200);

    const uint8_t selectedFront = redPillar ? FrontLeft : FrontRight;
    const uint8_t selectedBack = redPillar ? BackLeft : BackRight;
    inputs.sides[selectedFront].rangeMm = 220;
    inputs.sides[selectedBack].rangeMm = 340;
    // A centred last observation deliberately requests still more steering
    // away from either pillar. Once strict proof starts body-clear, that old
    // image bearing must no longer curve the chassis toward the free-side wall.
    if (redPillar) {
      inputs.vision.red = usableBlob(0, 6360, 1800, 840);
    } else {
      inputs.vision.green = usableBlob(0, 6360, 1800, 840);
    }
    const ObstacleTarget expected =
        redPillar ? ObstacleTarget::RedPassRight
                  : ObstacleTarget::GreenPassLeft;
    const float escapeYaw = redPillar ? 13.13F : -13.13F;
    Decision decision = updateFresh(controller, inputs, 100, escapeYaw);
    check(decision.obstacle == expected && decision.speedPermille > 0 &&
              ((redPillar && decision.steeringPermille > 0) ||
               (!redPillar && decision.steeringPermille < 0)),
          "R25E fixture must prove its last live blob requests additional away steering");

    inputs.vision.red = {};
    inputs.vision.green = {};
    updateFresh(controller, inputs, 110, escapeYaw);
    inputs.sides[selectedFront].rangeMm = 360;
    decision = updateFresh(controller, inputs, 400, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == expected,
          "R25C strict pass transition must begin its ordinary confirmation interval");
    decision = updateFresh(controller, inputs, 579, escapeYaw);
    check(decision.state != RunState::Finished,
          "R25C strict pass proof must not complete one millisecond early");
    decision = updateFresh(controller, inputs, 580, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == expected && decision.motionAllowed &&
              decision.speedPermille > 0,
          "R25C strict proof must enter body-clear instead of stopping beside the pillar");

    // Remove both ordinary same-side pairs while retaining the physical
    // diagonal cross pair seen in the R24C run. It has no steering authority,
    // but fresh clear front plus a close committed target may keep this one
    // bounded calibration maneuver moving.
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    const uint8_t crossLeft = redPillar ? BackLeft : FrontLeft;
    const uint8_t crossRight = redPillar ? FrontRight : BackRight;
    inputs.sides[crossLeft].valid = true;
    inputs.sides[crossLeft].rangeStatus = 0;
    inputs.sides[crossLeft].rangeMm = 800;
    inputs.sides[crossRight].valid = true;
    inputs.sides[crossRight].rangeStatus = 0;
    inputs.sides[crossRight].rangeMm = 250;
    decision = updateFresh(controller, inputs, 581, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R25E body-clear must discard stale image steering and translate straight");

    // Straight is only the base command. A live close return on the known
    // pillar side must retain authority to steer away at the reduced guard
    // speed, mirrored exactly for red and green.
    const uint8_t pillarSideRay = redPillar ? crossLeft : crossRight;
    const uint16_t pillarSideClearRange =
        inputs.sides[pillarSideRay].rangeMm;
    inputs.sides[pillarSideRay].rangeMm = 200;
    decision = updateFresh(controller, inputs, 600, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 &&
              decision.steeringPermille ==
                  (redPillar
                       ? config.obstacleSideClearanceSteeringPermille
                       : -config.obstacleSideClearanceSteeringPermille),
          "R25E pillar-side guard must override straight body-clear steering away from the pillar");
    inputs.sides[pillarSideRay].rangeMm = pillarSideClearRange;
    decision = updateFresh(controller, inputs, 620, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R25E body-clear must return to straight after pillar-side clearance recovers");

    // The last physical red proof had its free-side ray at 203-204 mm. A 200
    // mm sample must therefore keep moving without applying the normal escape
    // back toward the pillar. The separate 140 mm hard bound must stop and
    // reset the continuous body-clear timer.
    const uint8_t oppositeSideRay = redPillar ? crossRight : crossLeft;
    inputs.sides[oppositeSideRay].rangeMm = 200;
    decision = updateFresh(controller, inputs, 650, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0 &&
              decision.steeringPermille == 0,
          "R25E must ignore only the early 200 mm free-side escape that would steer back toward the pillar");
    inputs.sides[oppositeSideRay].rangeMm = 140;
    decision = updateFresh(controller, inputs, 651, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0 &&
              decision.steeringPermille == 0,
          "R25E 140 mm free-side hard guard must command a full stop");
    decision = updateFresh(controller, inputs, 1200, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "R25E must never finish while an opposite-side guard keeps the car stationary past 500 ms");
    inputs.sides[oppositeSideRay].rangeMm =
        oppositeSideRay == crossLeft ? 800 : 250;
    decision = updateFresh(controller, inputs, 1250, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0,
          "R25C body-clear must recover only after opposite-side clearance returns");
    updateFresh(controller, inputs, 1251, escapeYaw);

    // A transient unusable centre-front ray must also remove drive and reset
    // the timer; restoring it begins a new full 500 ms safe interval.
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    decision = updateFresh(controller, inputs, 1400, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == 0,
          "R25C body-clear must stop immediately when numeric front clearance disappears");
    inputs.front.valid = true;
    inputs.front.rangeStatus = 0;
    inputs.front.rangeMm = 1200;
    decision = updateFresh(controller, inputs, 1450, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille > 0,
          "R25C body-clear may restart after guarded front and side geometry recover");
    updateFresh(controller, inputs, 1451, escapeYaw);
    decision = updateFresh(controller, inputs, 1950, escapeYaw);
    check(decision.state == RunState::DirectionProbe &&
              decision.obstacle == expected && decision.speedPermille > 0,
          "R25C body-clear must not stop one millisecond before the restarted 500 ms bound");
    decision = updateFresh(controller, inputs, 1951, escapeYaw);
    check(decision.state == RunState::Finished &&
              decision.obstacle == ObstacleTarget::None &&
              !decision.motionAllowed && decision.speedPermille == 0 &&
              decision.steeringPermille == 0 &&
              decision.driveDirection == DriveDirection::Stopped,
          "R25C red and green body-clear must finish permanently at the guarded 500 ms bound");
  }
}

[[maybe_unused]] void testOpenFarStartLearnsDirectionAfterArming() {
  autonomy::Config config = fixtureConfig();
  config.requireAllSideSensorsFresh = false;
  config.sideSensorPairsAreParallel = false;
  config.obstacleAvoidanceEnabled = false;
  config.openingReferenceMaximumMm = 1200;
  Controller controller(config);
  Inputs inputs = healthyInputs(0);
  for (auto& side : inputs.sides) side.rangeMm = 5000;

  Decision decision = controller.update(inputs);
  check(decision.state == RunState::WaitingForStart &&
            decision.faultFlags == autonomy::FaultNone,
        "a fresh 5 m Open start must arm without pretending that range is a nearby wall");
  updateFresh(controller, inputs, 40, 0.0F);
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, 50, 0.0F);
  decision = updateFresh(controller, inputs, 90, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "a far Open start must begin moving without guessing direction");

  inputs.sides[FrontLeft].rangeMm = 300;
  decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "the probe must learn a close wall that enters range after a far start");
  inputs.front.rangeMm = 600;
  inputs.sides[FrontLeft].valid = false;
  inputs.sides[FrontLeft].rangeStatus = 2;
  decision = updateFresh(controller, inputs, 120, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::CounterClockwise,
        "the newly learned wall baseline must identify the first opening");

  Controller decisionZoneController(config);
  Inputs decisionZoneInputs = healthyInputs(0);
  for (auto& side : decisionZoneInputs.sides) side.rangeMm = 5000;
  decision = decisionZoneController.update(decisionZoneInputs);
  check(decision.state == RunState::WaitingForStart,
        "a second far Open start must arm for the decision-zone regression");
  updateFresh(decisionZoneController, decisionZoneInputs, 40, 0.0F);
  decisionZoneInputs.buttonPressed = true;
  updateFresh(decisionZoneController, decisionZoneInputs, 50, 0.0F);
  updateFresh(decisionZoneController, decisionZoneInputs, 90, 0.0F);
  decisionZoneInputs.front.rangeMm = 600;
  decisionZoneInputs.sides[FrontLeft].rangeMm = 550;
  decision = updateFresh(decisionZoneController, decisionZoneInputs, 100,
                         0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "baseline-less cross-sensor contrast must not choose an Open direction");
  decisionZoneInputs.sides[FrontLeft].rangeMm = 800;
  decision = updateFresh(decisionZoneController, decisionZoneInputs, 120,
                         0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "larger baseline-less contrast must remain non-authorizing");

  Controller oneBlindController(config);
  Inputs oneBlindInputs = healthyInputs(0);
  oneBlindInputs.sides[FrontLeft].valid = false;
  oneBlindInputs.sides[FrontLeft].rangeStatus = 2;
  oneBlindInputs.sides[BackLeft].valid = false;
  oneBlindInputs.sides[BackLeft].rangeStatus = 2;
  decision = oneBlindController.update(oneBlindInputs);
  check(decision.state == RunState::WaitingForStart,
        "a legal Open pose with one blind side must still arm");
  updateFresh(oneBlindController, oneBlindInputs, 40, 0.0F);
  oneBlindInputs.buttonPressed = true;
  updateFresh(oneBlindController, oneBlindInputs, 50, 0.0F);
  updateFresh(oneBlindController, oneBlindInputs, 90, 0.0F);
  oneBlindInputs.front.valid = false;
  oneBlindInputs.front.rangeStatus = 2;
  decision = updateFresh(oneBlindController, oneBlindInputs, 110, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "a blind front ray must never be treated as a nearby corner");
  oneBlindInputs.front.valid = true;
  oneBlindInputs.front.rangeStatus = 0;
  oneBlindInputs.front.rangeMm = 600;
  decision = updateFresh(oneBlindController, oneBlindInputs, 120, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "a reference-zero complete snapshot must not choose an Open direction");

  Controller singleBlindController(config);
  Inputs singleBlindInputs = healthyInputs(0);
  singleBlindInputs.sides[FrontLeft].valid = false;
  singleBlindInputs.sides[FrontLeft].rangeStatus = 2;
  decision = singleBlindController.update(singleBlindInputs);
  check(decision.state == RunState::WaitingForStart,
        "one blind forward side ray must not block Open arming");
  updateFresh(singleBlindController, singleBlindInputs, 40, 0.0F);
  singleBlindInputs.buttonPressed = true;
  updateFresh(singleBlindController, singleBlindInputs, 50, 0.0F);
  updateFresh(singleBlindController, singleBlindInputs, 90, 0.0F);
  singleBlindInputs.front.rangeMm = 600;
  decision = updateFresh(singleBlindController, singleBlindInputs, 120, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown,
        "one static blind side ray must never guess a powered turn");
}

[[maybe_unused]] void testProductionOpenDirectionProbeRejectsBaselineLessGeometry() {
  const autonomy::Config open =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);

  // A close numeric front does not make cross-sensor range contrast meaningful:
  // FL/FR are separate outward-looking rays with no shared reference. Exercise
  // both signs with a 700 mm contrast held longer than confirmation.
  for (const bool clockwiseContrast : {false, true}) {
    Controller controller(open);
    Inputs inputs;
    startDirectionProbeWithBaselines(controller, inputs, false, false, false);
    inputs.front.rangeMm = 600;
    inputs.sides[FrontLeft].valid = true;
    inputs.sides[FrontLeft].rangeStatus = 0;
    inputs.sides[FrontRight].valid = true;
    inputs.sides[FrontRight].rangeStatus = 0;
    inputs.sides[FrontLeft].rangeMm = clockwiseContrast ? 250 : 950;
    inputs.sides[FrontRight].rangeMm = clockwiseContrast ? 950 : 250;
    inputs.sides[BackLeft].valid = true;
    inputs.sides[BackLeft].rangeStatus = 0;
    inputs.sides[BackLeft].rangeMm = 120;
    inputs.sides[BackRight].valid = true;
    inputs.sides[BackRight].rangeStatus = 0;
    inputs.sides[BackRight].rangeMm = 1100;

    updateFresh(controller, inputs, 200, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 301, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              decision.leftOpeningMm == 0.0F &&
              decision.rightOpeningMm == 0.0F &&
              decision.faultFlags == autonomy::FaultNone,
          "baseline-less FL-vs-FR contrast must never authorize Open direction");
  }

  // Reference-zero complete snapshots are also mounting geometry, not proof
  // that either forward ray rose from its own captured wall baseline.
  for (const bool clockwiseSnapshot : {false, true}) {
    Controller controller(open);
    Inputs inputs;
    startDirectionProbeWithBaselines(controller, inputs, false, false, false);
    inputs.front.rangeMm = 600;
    for (auto& side : inputs.sides) {
      side.valid = true;
      side.rangeStatus = 0;
      side.rangeMm = 300;
    }
    const uint8_t openFront =
        clockwiseSnapshot ? FrontRight : FrontLeft;
    const uint8_t openRear =
        clockwiseSnapshot ? BackRight : BackLeft;
    inputs.sides[openFront].valid = false;
    inputs.sides[openFront].rangeStatus = 2;
    inputs.sides[openRear].valid = false;
    inputs.sides[openRear].rangeStatus = 2;

    updateFresh(controller, inputs, 200, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 301, 0.0F);
    const float selectedOpening =
        clockwiseSnapshot ? decision.rightOpeningMm : decision.leftOpeningMm;
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown &&
              selectedOpening == 0.0F,
          "reference-zero complete snapshots must not become Open direction evidence");
  }

  // Once a forward ray owns a nonzero baseline, only that ray's rise matters.
  // Swap extreme rear offsets during confirmation to prove the diagonal rear
  // pair cannot create, cancel, or reverse the result.
  for (const bool clockwise : {false, true}) {
    Controller controller(open);
    Inputs inputs;
    startDirectionProbeWithBaselines(controller, inputs, true, true, true, 628);
    const uint8_t selected = clockwise ? FrontRight : FrontLeft;
    inputs.front.rangeMm = 682;
    inputs.sides[selected].rangeMm = 669;
    inputs.sides[BackLeft].rangeMm = 120;
    inputs.sides[BackRight].rangeMm = 1100;
    updateFresh(controller, inputs, 200, 0.0F);

    inputs.front.rangeMm = 631;
    inputs.sides[selected].rangeMm = 681;
    inputs.sides[BackLeft].rangeMm = 1100;
    inputs.sides[BackRight].rangeMm = 120;
    Decision decision = updateFresh(controller, inputs, 299, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "arbitrary rear offsets must not bypass same-ray confirmation");
    decision = updateFresh(controller, inputs, 300, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction ==
                  (clockwise ? CourseDirection::Clockwise
                             : CourseDirection::CounterClockwise) &&
              decision.faultFlags == autonomy::FaultNone,
          "arbitrary rear offsets must not alter a confirmed same-ray direction");
  }
}

void testWrappedTargetIsDegradedButNeverOpeningEvidence() {
  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.obstacleAvoidanceEnabled = false;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.valid = true;
    inputs.front.rangeMm = 5000;
    inputs.front.rangeStatus = 6;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone,
          "front status 6 with a numeric long range must remain usable downstream");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.obstacleAvoidanceEnabled = false;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 7;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart &&
              decision.faultFlags == autonomy::FaultNone,
          "fresh front wrapped-target status 7 must be accepted as optical degradation at startup");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.sideSensorPairsAreParallel = false;
    config.obstacleAvoidanceEnabled = false;
    config.openingConfirmationMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.rangeMm = 600;
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 7;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    decision = updateFresh(controller, inputs, 300, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "wrapped-target status 7 must not be promoted into strong corner-opening evidence");
  }
}

void testProductionOpenFrontClearRearm() {
  const autonomy::Config config =
      autonomy::makeControllerConfig(autonomy::ChallengeProfile::Open);
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);

  setRightOpening(inputs, 1000);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "production opening confirmation must not choose direction early");
  decision = updateFresh(controller, inputs, 150, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "production direction must still wait after only two acquisitions");
  decision = updateFresh(controller, inputs, 200, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::Clockwise,
        "production Open must confirm the first clockwise corner");
  updateFresh(controller, inputs, 240, 40.0F);
  updateFresh(controller, inputs, 280, 80.0F);
  setStraightGeometry(inputs, 900);
  decision = updateFresh(controller, inputs, 550, 90.0F);
  check(decision.state == RunState::Cornering &&
            decision.cornerCount == 0,
        "production exit geometry must start its 200 ms stability interval");
  decision = updateFresh(controller, inputs, 749, 90.0F);
  check(decision.state == RunState::Cornering,
        "production corner exit must not count one millisecond early");
  decision = updateFresh(controller, inputs, 750, 90.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1 &&
            !decision.cornerDetectionArmed,
        "a completed production corner must leave detection disarmed");

  setStraightGeometry(inputs, 1200);
  decision = updateFresh(controller, inputs, 999, 90.0F);
  check(!decision.cornerDetectionArmed,
        "front-clear rearm must respect the full 250 ms minimum");
  decision = updateFresh(controller, inputs, 1000, 90.0F);
  check(!decision.cornerDetectionArmed,
        "the exact refractory boundary must start, not skip, clear stability");
  inputs.front.rangeMm = 1199;
  decision = updateFresh(controller, inputs, 1050, 90.0F);
  check(!decision.cornerDetectionArmed,
        "a one-millimeter clear-front miss must reset rearm stability");

  // A positive WrapTargetFail range is usable in Open, so it must obey the
  // same raw-distance hysteresis. It cannot simultaneously mean optically
  // clear for rearm and close enough to start the next corner.
  inputs.front.valid = false;
  inputs.front.rangeStatus = 7;
  inputs.front.rangeMm = 1000;
  updateFresh(controller, inputs, 1060, 90.0F);
  decision = updateFresh(controller, inputs, 1160, 90.0F);
  check(decision.state == RunState::Running &&
            !decision.cornerDetectionArmed,
        "a near wrapped-target range must not rearm and trigger on the same frame");

  inputs.front.rangeMm = 1423;
  updateFresh(controller, inputs, 1170, 90.0F);
  decision = updateFresh(controller, inputs, 1269, 90.0F);
  check(!decision.cornerDetectionArmed,
        "a clear wrapped-target range must not rearm one millisecond early");
  decision = updateFresh(controller, inputs, 1270, 90.0F);
  check(decision.cornerDetectionArmed,
        "a clear wrapped-target range for 100 ms must rearm the next corner");

  // Reproduce R12's accepted WrapTargetFail approach after the first corner.
  // The raw range still tracks the wall and is intentionally usable in Open.
  inputs.front.rangeMm = 1001;
  decision = updateFresh(controller, inputs, 1280, 90.0F);
  check(decision.state == RunState::Running,
        "front at 1001 mm must remain outside the 1000 mm trigger");
  inputs.front.rangeMm = 1000;
  decision = updateFresh(controller, inputs, 1290, 90.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 1,
        "known-direction Open must turn on the first fresh accepted trigger sample");
}

void testProductionObstacleRefreshesBaselineAndBlocksRearmDuringPass() {
  const auto exitFirstCornerWithHighLeftRay =
      [](Controller& controller, Inputs& inputs) {
        startController(controller, inputs);
        setLeftOpening(inputs, 900);
        Decision decision = updateFresh(controller, inputs, 100, 0.0F);
        check(decision.state == RunState::DirectionProbe,
              "production Obstacle direction needs its full confirmation");
        decision = updateFresh(controller, inputs, 150, 0.0F);
        check(decision.state == RunState::DirectionProbe,
              "production Obstacle direction must retain the three-sample contract");
        decision = updateFresh(controller, inputs, 200, 0.0F);
        check(decision.state == RunState::Cornering &&
                  decision.direction == CourseDirection::CounterClockwise,
              "production Obstacle fixture must enter its first left corner");
        updateFresh(controller, inputs, 240, -40.0F);
        updateFresh(controller, inputs, 280, -80.0F);
        setStraightGeometry(inputs, 900);
        // Both left rays can still see obliquely through the old opening. This
        // exit snapshot must not become the permanent next-straight baseline.
        inputs.sides[FrontLeft].rangeMm = 900;
        inputs.sides[BackLeft].rangeMm = 900;
        decision = updateFresh(controller, inputs, 550, -90.0F);
        check(decision.state == RunState::Cornering,
              "production Obstacle exit must begin 200 ms stability");
        decision = updateFresh(controller, inputs, 750, -90.0F);
        check(decision.state == RunState::Running &&
                  decision.cornerCount == 1 &&
                  !decision.cornerDetectionArmed,
              "first production Obstacle exit must be counted and disarmed");
      };

  {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    exitFirstCornerWithHighLeftRay(controller, inputs);

    // The ray settles from 900 to 300 while detection is disarmed. Repeated
    // straight frames must replace the oblique exit snapshot before arming.
    setStraightGeometry(inputs, 1100);
    Decision decision = updateFresh(controller, inputs, 800, -90.0F);
    check(!decision.cornerDetectionArmed,
          "a settled wall sample before refractory expiry must stay disarmed");
    updateFresh(controller, inputs, 1000, -90.0F);
    decision = updateFresh(controller, inputs, 1099, -90.0F);
    check(!decision.cornerDetectionArmed,
          "production Obstacle clear-front stability must not finish early");
    decision = updateFresh(controller, inputs, 1100, -90.0F);
    check(decision.cornerDetectionArmed,
          "settled production Obstacle geometry must arm after 250 plus 100 ms");

    inputs.front.rangeMm = 650;
    inputs.sides[FrontLeft].rangeMm = 480;
    inputs.sides[BackLeft].rangeMm = 300;
    decision = updateFresh(controller, inputs, 1110, -90.0F);
    check(decision.state == RunState::Running &&
              decision.leftOpeningMm >= 180.0F,
          "the next 180 mm rise must compare against the settled 300 mm baseline");
    decision = updateFresh(controller, inputs, 1209, -90.0F);
    check(decision.state == RunState::Running,
          "production Obstacle opening confirmation must not finish early");
    decision = updateFresh(controller, inputs, 1210, -90.0F);
    check(decision.state == RunState::Cornering &&
              decision.cornerCount == 1,
          "settled baseline plus close front must start production corner two");
  }

  {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    exitFirstCornerWithHighLeftRay(controller, inputs);
    setStraightGeometry(inputs, 1100);
    updateFresh(controller, inputs, 800, -90.0F);

    inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    Decision decision = updateFresh(controller, inputs, 810, -90.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              !decision.cornerDetectionArmed,
          "an active production obstacle must keep corner detection disarmed");
    inputs.vision.red = {};
    inputs.sides[FrontLeft].rangeMm = 200;
    inputs.sides[BackLeft].rangeMm = 300;
    updateFresh(controller, inputs, 820, -90.0F);
    decision = updateFresh(controller, inputs, 1100, -90.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              !decision.cornerDetectionArmed,
          "elapsed refractory and clear front must not arm during a pass");
    inputs.sides[FrontLeft].rangeMm = 320;
    decision = updateFresh(controller, inputs, 1109, -90.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight,
          "production minimum commit must hold through its last millisecond");
    updateFresh(controller, inputs, 1110, -90.0F);
    decision = updateFresh(controller, inputs, 1289, -90.0F);
    check(decision.obstacle == ObstacleTarget::RedPassRight &&
              !decision.cornerDetectionArmed,
          "pass-clear stability must preserve both commitment and disarm");
    decision = updateFresh(controller, inputs, 1290, -90.0F);
    check(decision.obstacle == ObstacleTarget::None &&
              decision.targetLateralOffsetMm > 0 &&
              !decision.cornerDetectionArmed,
          "a proven pass must begin recentering without arming a corner");

    setStraightGeometry(inputs, 1100);
    decision = updateFresh(controller, inputs, 1589, -90.0F);
    check(!decision.cornerDetectionArmed,
          "recenter must keep corner detection disarmed through its last millisecond");
    decision = updateFresh(controller, inputs, 1590, -90.0F);
    check(!decision.cornerDetectionArmed,
          "recenter completion must start, not skip, a fresh clear-front timer");
    decision = updateFresh(controller, inputs, 1689, -90.0F);
    check(!decision.cornerDetectionArmed,
          "post-pass rearm must not finish one millisecond early");
    decision = updateFresh(controller, inputs, 1690, -90.0F);
    check(decision.cornerDetectionArmed,
          "corner detection may arm only after pass, recenter, and 100 ms clear stability");
  }
}

void testArmedCornerDetectionDisarmsForMirroredObstaclePasses() {
  for (const bool redCounterClockwise : {true, false}) {
    const autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);

    if (redCounterClockwise) {
      setLeftOpening(inputs, 900);
    } else {
      setRightOpening(inputs, 900);
    }
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe,
          "mirrored armed-pass fixture must confirm its first direction");
    decision = updateFresh(controller, inputs, 150, 0.0F);
    check(decision.state == RunState::DirectionProbe,
          "mirrored armed-pass fixture must retain its three-sample contract");
    decision = updateFresh(controller, inputs, 200, 0.0F);
    const CourseDirection expectedDirection =
        redCounterClockwise ? CourseDirection::CounterClockwise
                            : CourseDirection::Clockwise;
    check(decision.state == RunState::Cornering &&
              decision.direction == expectedDirection,
          "mirrored armed-pass fixture must enter its first corner");

    const float halfTurnYaw = redCounterClockwise ? -40.0F : 40.0F;
    const float nearTurnYaw = redCounterClockwise ? -80.0F : 80.0F;
    const float straightYaw = redCounterClockwise ? -90.0F : 90.0F;
    updateFresh(controller, inputs, 240, halfTurnYaw);
    updateFresh(controller, inputs, 280, nearTurnYaw);
    setStraightGeometry(inputs, 900);
    updateFresh(controller, inputs, 550, straightYaw);
    decision = updateFresh(controller, inputs, 750, straightYaw);
    check(decision.state == RunState::Running &&
              !decision.cornerDetectionArmed,
          "first corner exit must leave the mirrored fixture disarmed");

    setStraightGeometry(inputs, 1100);
    updateFresh(controller, inputs, 1000, straightYaw);
    decision = updateFresh(controller, inputs, 1100, straightYaw);
    check(decision.cornerDetectionArmed,
          "clear straight must arm before the obstacle appears");

    if (redCounterClockwise) {
      inputs.vision.red = usableBlob(-3500, 5200, 2000, 700);
    } else {
      inputs.vision.green = usableBlob(3500, 5200, 2000, 700);
    }
    decision = updateFresh(controller, inputs, 1110, straightYaw);
    const ObstacleTarget expectedTarget =
        redCounterClockwise ? ObstacleTarget::RedPassRight
                            : ObstacleTarget::GreenPassLeft;
    check(decision.obstacle == expectedTarget &&
              !decision.cornerDetectionArmed,
          "a red or green commitment must immediately disarm an already-armed corner detector");

    inputs.vision.red = {};
    inputs.vision.green = {};
    const uint8_t selectedFront =
        redCounterClockwise ? FrontLeft : FrontRight;
    const uint8_t selectedBack = redCounterClockwise ? BackLeft : BackRight;
    inputs.sides[selectedFront].rangeMm = 200;
    inputs.sides[selectedBack].rangeMm = 300;
    updateFresh(controller, inputs, 1120, straightYaw);
    inputs.sides[selectedFront].rangeMm = 320;
    updateFresh(controller, inputs, 1410, straightYaw);
    decision = updateFresh(controller, inputs, 1590, straightYaw);
    check(decision.obstacle == ObstacleTarget::None &&
              !decision.cornerDetectionArmed,
          "a proven mirrored pass must clear into a still-disarmed recenter");

    // Recreate exactly the dangerous overlap: the pass-side range rise also
    // resembles the expected course opening while the front is close. It must
    // not arm while the recenter is still active.
    if (redCounterClockwise) {
      setLeftOpening(inputs, 650);
    } else {
      setRightOpening(inputs, 650);
    }
    decision = updateFresh(controller, inputs, 1889, straightYaw);
    check(decision.state == RunState::Running &&
              !decision.cornerDetectionArmed,
          "pass-side opening evidence must stay disarmed through the last recenter millisecond");

    // The legal last seat can leave less than 1.0 m before the corner. Finish
    // recentering on close straight geometry, then present a fresh empty-camera
    // directional opening; preserving the pre-pass baseline is what lets this
    // later evidence recover the detector.
    setStraightGeometry(inputs, 950);
    decision = updateFresh(controller, inputs, 1890, straightYaw);
    check(decision.state == RunState::Running &&
              !decision.cornerDetectionArmed,
          "recenter completion without directional evidence must stay disarmed");
    if (redCounterClockwise) {
      setLeftOpening(inputs, 650);
    } else {
      setRightOpening(inputs, 650);
    }
    decision = updateFresh(controller, inputs, 1900, straightYaw);
    check(decision.state == RunState::Running &&
              decision.cornerDetectionArmed,
          "fresh empty-camera last-seat evidence must recover the mirrored corner detector below 1.0 m");
    decision = updateFresh(controller, inputs, 1999, straightYaw);
    check(decision.state == RunState::Running &&
              decision.cornerDetectionArmed,
          "recovered last-seat evidence must still obey the full corner confirmation interval");
    decision = updateFresh(controller, inputs, 2000, straightYaw);
    check(decision.state == RunState::Cornering &&
              decision.cornerCount == 1,
          "mirrored red and green last-seat passes must enter the proven corner after confirmation");
  }
}

void testElectricalRecoveryRestartsCornerAndOpeningStability() {
  autonomy::Config config = legacyOpenFixtureConfig();
  config.openingConfirmationMs = 0;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  setRightOpening(inputs, 600);
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::Cornering,
        "recovery-timer fixture must enter a clockwise corner");
  updateFresh(controller, inputs, 140, 40.0F);
  updateFresh(controller, inputs, 180, 80.0F);
  setStraightGeometry(inputs, 900);
  decision = updateFresh(controller, inputs, 450, 90.0F);
  check(decision.state == RunState::Cornering,
        "production corner-exit stability must begin after minimum duration");
  updateFresh(controller, inputs, 549, 90.0F);

  inputs.sides[FrontLeft].i2cStatus = 4;
  decision = updateFresh(controller, inputs, 550, 90.0F);
  check(decision.state == RunState::Cornering &&
            decision.speedPermille == 0,
        "side electrical loss must hold motion during corner-exit stability");
  inputs.sides[FrontLeft].i2cStatus = 0;
  decision = updateFresh(controller, inputs, 551, 90.0F);
  check(decision.state == RunState::Cornering && decision.cornerCount == 0,
        "first recovered side sample must restart, not complete, corner-exit stability");
  decision = updateFresh(controller, inputs, 750, 90.0F);
  check(decision.state == RunState::Cornering,
        "restarted 200 ms corner stability must not finish early");
  decision = updateFresh(controller, inputs, 751, 90.0F);
  check(decision.state == RunState::Running && decision.cornerCount == 1,
        "corner may exit only after 200 fresh recovered milliseconds");

  setStraightGeometry(inputs, config.cornerRearmFrontClearMm);
  updateFresh(controller, inputs, 1001, 90.0F);
  decision = updateFresh(controller, inputs, 1101, 90.0F);
  check(decision.cornerDetectionArmed,
        "recovery fixture must rearm the next production Open corner");
  inputs.front.rangeMm = config.frontCornerTriggerMm;
  decision = updateFresh(controller, inputs, 1110, 90.0F);
  check(decision.state == RunState::Cornering,
        "zero-confirmation fixture must enter immediately once rearmed");

  // Exercise the real 100 ms opening timer on a second controller while a
  // transient electrical hold interrupts it.
  config.openingConfirmationMs = 100;
  config.cornerMinimumDurationMs = 0;
  config.cornerExitStableMs = 0;
  config.cornerRefractoryMs = 0;
  config.cornerRearmStableMs = 0;
  Controller openingController(config);
  Inputs openingInputs;
  startController(openingController, openingInputs);
  setRightOpening(openingInputs, 600);
  decision = updateFresh(openingController, openingInputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "opening recovery fixture must start its 100 ms confirmation");
  updateFresh(openingController, openingInputs, 150, 0.0F);
  openingInputs.sides[FrontLeft].i2cStatus = 4;
  decision = updateFresh(openingController, openingInputs, 160, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.speedPermille == 0,
        "side loss must hold motion and discard partial opening confirmation");
  openingInputs.sides[FrontLeft].i2cStatus = 0;
  decision = updateFresh(openingController, openingInputs, 161, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "first recovered opening sample must restart confirmation");
  decision = updateFresh(openingController, openingInputs, 260, 0.0F);
  check(decision.state == RunState::DirectionProbe,
        "restarted opening confirmation must not finish one millisecond early");
  decision = updateFresh(openingController, openingInputs, 261, 0.0F);
  check(decision.state == RunState::Cornering,
        "opening may commit only after 100 fresh recovered milliseconds");
}

void testRequiredSensorFaults() {
  {
    Controller controller(fixtureConfig());
    Inputs inputs;
    startController(controller, inputs);
    inputs.nowMs = 400;
    inputs.front.updatedAtMs = 400;
    for (auto& side : inputs.sides) side.updatedAtMs = 400;
    inputs.vision.updatedAtMs = 400;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultImuStale) != 0,
          "stale BNO data during motion must latch a stop");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.sideElectricalLossGraceMs = 100;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.sides[BackRight].valid = false;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed && decision.speedPermille == 0,
          "an invalid required side ToF must stop during its recovery grace");
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultSideTofUnavailable) != 0,
          "an invalid required side ToF must fail closed at the grace boundary");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.requireVisionForMotion = true;
    config.visionLossGraceMs = 0;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.nowMs = 500;
    inputs.imu.updatedAtMs = 500;
    inputs.front.updatedAtMs = 500;
    for (auto& side : inputs.sides) side.updatedAtMs = 500;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultVisionStale) != 0,
          "required stale camera telemetry must fail closed");
  }
}

void testOpticalNoReturnDegradesWithoutPretendingItIsClear() {
  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 5;
    const Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags &
               autonomy::FaultFrontTofUnavailable) != 0,
          "a front hardware-failure status must stop instead of receiving optical grace");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.cornerBlindSpeedPermille = 200;
    config.accelerationPermillePerSecond = 10000;
    config.decelerationPermillePerSecond = 10000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);

    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;  // live sensor, no usable optical target
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "fresh front no-return must continue only at degraded speed");

    inputs.front.i2cStatus = 1;
    decision = updateFresh(controller, inputs, 110, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultFrontTofUnavailable) != 0,
          "front I2C failure must stop immediately instead of using optical grace");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 3;
    const Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTooClose) != 0,
          "front status 3 must remain an immediate close-hazard stop");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.straightSideGeometryGraceMs = 100;
    config.cornerBlindSpeedPermille = 200;
    config.accelerationPermillePerSecond = 10000;
    config.decelerationPermillePerSecond = 10000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);

    inputs.sides[BackLeft].valid = false;
    inputs.sides[BackLeft].rangeStatus = 2;
    inputs.sides[FrontRight].valid = false;
    inputs.sides[FrontRight].rangeStatus = 2;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "brief side geometry loss must enter bounded degraded motion");
    decision = updateFresh(controller, inputs, 199, 0.0F);
    check(decision.state == RunState::DirectionProbe && decision.motionAllowed,
          "side geometry loss inside grace must not abruptly stop");

    inputs.sides[BackLeft].valid = true;
    inputs.sides[FrontRight].valid = true;
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::DirectionProbe && decision.motionAllowed,
          "side geometry recovery must clear the pending stop timer");

    inputs.sides[BackLeft].valid = false;
    inputs.sides[FrontRight].valid = false;
    updateFresh(controller, inputs, 210, 0.0F);
    decision = updateFresh(controller, inputs, 309, 0.0F);
    check(decision.state == RunState::DirectionProbe && decision.motionAllowed,
          "a recovered side geometry timer must grant a fresh grace interval");
    decision = updateFresh(controller, inputs, 310, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags &
               autonomy::FaultSideGeometryUnavailable) != 0,
          "persistent loss of a complete side geometry solution must stop");
  }
}

void testFrontOpticalNoTargetHasBoundedStateAwareGrace() {
  const uint8_t opticalNoTargetStatuses[] = {1, 2, 4, 6, 7};
  for (const uint8_t status : opticalNoTargetStatuses) {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.straightFrontOpticalGraceMs = 100;
    config.directionProbeSpeedPermille = 400;
    config.cruiseSpeedPermille = 500;
    config.cornerBlindSpeedPermille = 200;
    config.accelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = status;
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    const bool strongFarStatus = status == 2U || status == 4U;
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed &&
              (strongFarStatus
                   ? decision.speedPermille ==
                         config.directionProbeSpeedPermille
                   : decision.speedPermille <=
                         config.cornerBlindSpeedPermille),
          "only strong far no-target may retain useful direction-probe speed");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.obstacleAvoidanceEnabled = false;
    config.directionProbeSpeedPermille = 400;
    config.cruiseSpeedPermille = 500;
    config.cornerSpeedPermille = 380;
    config.cornerBlindSpeedPermille = 340;
    config.accelerationPermillePerSecond = 60000;
    config.decelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    const Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed &&
              decision.speedPermille == config.directionProbeSpeedPermille,
          "Open far-start probing must not be reduced to drivetrain breakaway speed");
  }

  {
    autonomy::Config config = legacyOpenFixtureConfig();
    config.accelerationPermillePerSecond = 60000;
    config.decelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    Decision decision = updateFresh(controller, inputs, 1300, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille == config.directionProbeSpeedPermille,
          "production Open must keep probing after 1.2 s of healthy far no-target");

    setRightOpening(inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    decision = updateFresh(controller, inputs, 1400, 0.0F);
    check(decision.state == RunState::DirectionProbe,
          "far-start opening must still honor production confirmation time");
    decision = updateFresh(controller, inputs, 1500, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.direction == CourseDirection::Unknown,
          "a blind center-front ray must not authorize first direction from side history");
  }

  {
    // Reproduce the R8 start at ms9015: front, FL, BL, and BR were fresh
    // status-2 no-target while FR remained a fresh numeric ~507 mm. This is a
    // live one-ray Open probe, not the total blindness bounded below.
    autonomy::Config config = legacyOpenFixtureConfig();
    config.accelerationPermillePerSecond = 60000;
    config.decelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    inputs.sides[FrontRight].valid = true;
    inputs.sides[FrontRight].rangeStatus = 0;
    inputs.sides[FrontRight].rangeMm = 507;

    Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForStart,
          "one-ray Open R8 fixture must arm while every sensor stays electrical");
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    decision = updateFresh(controller, inputs, 90, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone,
          "one-ray Open start must enter a healthy moving probe");
    decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille == config.directionProbeSpeedPermille,
          "one-ray Open probe must keep its configured 600 command");
    decision = updateFresh(controller, inputs, 1300, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille == config.directionProbeSpeedPermille,
          "one-ray Open probe must not fault at the 1.2 second optical boundary");
    decision = updateFresh(controller, inputs, 12089, 0.0F);
    check(decision.state == RunState::DirectionProbe && decision.motionAllowed,
          "one-ray Open probe must remain bounded one millisecond before direction timeout");
    decision = updateFresh(controller, inputs, 12090, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultDirectionTimeout) != 0 &&
              (decision.faultFlags &
               autonomy::FaultFrontTofUnavailable) == 0,
          "one-ray Open probe must stop at 12 seconds, not front optical grace");
  }

  {
    autonomy::Config config = legacyOpenFixtureConfig();
    config.accelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    controller.update(inputs);
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    Decision decision = updateFresh(controller, inputs, 90, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "zero-ray Open strong no-target must stay at bounded blind speed");
    updateFresh(controller, inputs, 100, 0.0F);
    decision = updateFresh(controller, inputs, 1299, 0.0F);
    check(decision.state == RunState::DirectionProbe && decision.motionAllowed,
          "zero-ray Open probe must retain grace before 1.2 seconds");
    decision = updateFresh(controller, inputs, 1300, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags &
               autonomy::FaultFrontTofUnavailable) != 0,
          "zero-ray Open probe must fault at the optical grace boundary");
  }

  {
    autonomy::Config config = legacyOpenFixtureConfig();
    config.accelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 1;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    inputs.sides[FrontRight].valid = true;
    inputs.sides[FrontRight].rangeStatus = 0;
    inputs.sides[FrontRight].rangeMm = 507;
    controller.update(inputs);
    updateFresh(controller, inputs, 40, 0.0F);
    inputs.buttonPressed = true;
    updateFresh(controller, inputs, 50, 0.0F);
    Decision decision = updateFresh(controller, inputs, 90, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "weak front status must not inherit the one-ray Open exception");
    updateFresh(controller, inputs, 100, 0.0F);
    decision = updateFresh(controller, inputs, 1300, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags &
               autonomy::FaultFrontTofUnavailable) != 0,
          "weak one-ray Open front must still fault at bounded grace");
  }

  {
    autonomy::Config config =
        autonomy::makeControllerConfig(autonomy::ChallengeProfile::Obstacle);
    config.accelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    inputs.sides[FrontRight].valid = true;
    inputs.sides[FrontRight].rangeStatus = 0;
    inputs.sides[FrontRight].rangeMm = 507;
    const Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe &&
              decision.motionAllowed &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "one-ray Obstacle probe must not inherit the Open full-speed exception");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.cornerMinimumDurationMs = 0;
    config.cornerExitStableMs = 0;
    config.straightFrontOpticalGraceMs = 100;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    updateFresh(controller, inputs, 100, 0.0F);
    setStraightGeometry(inputs);
    updateFresh(controller, inputs, 120, 40.0F);
    Decision decision = updateFresh(controller, inputs, 140, 80.0F);
    check(decision.state == RunState::Running,
          "front-blindness fixture must first reach a straight");

    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    decision = updateFresh(controller, inputs, 150, 80.0F);
    check(decision.state == RunState::Running && decision.motionAllowed,
          "a fresh straight front no-target must start bounded grace");
    decision = updateFresh(controller, inputs, 249, 80.0F);
    check(decision.state == RunState::Running && decision.motionAllowed,
          "straight front blindness must remain degraded before its boundary");
    decision = updateFresh(controller, inputs, 250, 80.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultFrontTofUnavailable) != 0 &&
              !decision.motionAllowed,
          "persistent straight front blindness must latch a stop at its boundary");
  }

  {
    // Reproduce autonomousRun-20260904-221814.txt: the first left turn had a
    // fresh status-2 front no-target for more than 2.5 s while the right outer
    // wall remained numerically observed and the IMU stayed healthy. That is
    // valid Open-corner geometry, not a failed front module.
    autonomy::Config config = legacyOpenFixtureConfig();
    config.accelerationPermillePerSecond = 60000;
    config.decelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs, 600);
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::DirectionProbe,
          "logged Open corner fixture must honor direction confirmation");
    decision = updateFresh(controller, inputs, 200, 0.0F);
    check(decision.state == RunState::Cornering &&
              decision.direction == CourseDirection::CounterClockwise,
          "logged Open corner fixture must enter its first left turn");

    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    inputs.sides[FrontLeft].valid = false;
    inputs.sides[FrontLeft].rangeStatus = 2;
    inputs.sides[BackLeft].rangeMm = 200;
    inputs.sides[FrontRight].rangeMm = 510;
    inputs.sides[BackRight].rangeMm = 390;
    decision = updateFresh(controller, inputs, 210, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille == config.cornerSpeedPermille,
          "guided Open status-2 corner must retain usable turning speed");

    // The rear outer ray briefly looked over the 100 mm wall in the live turn.
    // The still-numeric forward outer ray plus IMU must retain useful speed;
    // requiring a complete pair recreated the physical 200-command stall.
    inputs.sides[BackRight].valid = false;
    inputs.sides[BackRight].rangeStatus = 2;
    decision = updateFresh(controller, inputs, 1000, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille == config.cornerSpeedPermille,
          "one numeric outer ray plus IMU must retain usable Open corner speed");
    inputs.sides[BackRight].valid = true;
    inputs.sides[BackRight].rangeStatus = 0;
    decision = updateFresh(controller, inputs, 2000, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille == config.cornerSpeedPermille,
          "restored Open corner pair must preserve usable turning speed");
    decision = updateFresh(controller, inputs, 2710, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.faultFlags == autonomy::FaultNone &&
              decision.speedPermille == config.cornerSpeedPermille,
          "guided Open status-2 corner must not fault at 2.5 seconds");
    decision = updateFresh(controller, inputs, 5199, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed,
          "guided Open corner must remain bounded by its turn timeout");
    decision = updateFresh(controller, inputs, 5200, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultCornerTimeout) != 0 &&
              (decision.faultFlags &
               autonomy::FaultFrontTofUnavailable) == 0,
          "guided Open no-target must end at corner timeout, not front optical grace");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.obstacleAvoidanceEnabled = true;
    config.straightFrontOpticalGraceMs = 100;
    config.cornerFrontOpticalGraceMs = 250;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::Cornering,
          "corner front-grace fixture must enter a physical corner");
    inputs.front.valid = false;
    inputs.front.rangeStatus = 4;
    updateFresh(controller, inputs, 110, 0.0F);
    decision = updateFresh(controller, inputs, 210, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed,
          "Obstacle corner front no-target must use bounded corner grace");
    decision = updateFresh(controller, inputs, 359, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed,
          "Obstacle corner blindness must remain degraded before its boundary");
    decision = updateFresh(controller, inputs, 360, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultFrontTofUnavailable) != 0,
          "Obstacle corner front blindness must still latch after bounded grace");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.obstacleAvoidanceEnabled = false;
    config.cornerFrontOpticalGraceMs = 250;
    config.cornerBlindSpeedPermille = 140;
    config.accelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::Cornering,
          "weak-front Open fixture must enter a physical corner");
    inputs.front.valid = false;
    inputs.front.rangeStatus = 1;
    decision = updateFresh(controller, inputs, 110, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "weak Open no-target must not receive the guided-corner exception");
    decision = updateFresh(controller, inputs, 359, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed,
          "weak Open no-target must retain only its bounded grace");
    decision = updateFresh(controller, inputs, 360, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultFrontTofUnavailable) != 0,
          "persistent weak Open no-target must fault at the grace boundary");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.obstacleAvoidanceEnabled = false;
    config.cornerFrontOpticalGraceMs = 250;
    config.cornerBlindSpeedPermille = 140;
    config.accelerationPermillePerSecond = 60000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.state == RunState::Cornering,
          "unguided Open fixture must enter a physical corner");
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    for (auto& side : inputs.sides) {
      side.valid = false;
      side.rangeStatus = 2;
    }
    decision = updateFresh(controller, inputs, 110, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed &&
              decision.speedPermille <= config.cornerBlindSpeedPermille,
          "strong Open no-target without independent side geometry must stay degraded");
    decision = updateFresh(controller, inputs, 359, 0.0F);
    check(decision.state == RunState::Cornering && decision.motionAllowed,
          "unguided strong Open no-target must receive only bounded grace");
    decision = updateFresh(controller, inputs, 360, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & autonomy::FaultFrontTofUnavailable) != 0,
          "unguided strong Open no-target must fault at the grace boundary");
  }

  {
    autonomy::Config config = fixtureConfig();
    config.requireAllSideSensorsFresh = false;
    config.straightFrontOpticalGraceMs = 1000;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    inputs.front.valid = false;
    inputs.front.rangeStatus = 2;
    updateFresh(controller, inputs, 100, 0.0F);

    // Refresh every required source except the front sensor. Its old optical
    // sample must become a stale electrical-data fault immediately, without
    // inheriting the much longer optical grace interval.
    inputs.nowMs = 301;
    inputs.imu.updatedAtMs = 301;
    for (auto& side : inputs.sides) side.updatedAtMs = 301;
    inputs.vision.updatedAtMs = 301;
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultFrontTofStale) != 0,
          "stale front data must latch immediately during optical grace");
  }
}

void testSigmaFailRequiresValidOpeningCorroboration() {
  autonomy::Config config = fixtureConfig();
  config.requireAllSideSensorsFresh = false;
  config.obstacleAvoidanceEnabled = false;
  Controller controller(config);
  Inputs inputs;
  startController(controller, inputs);
  inputs.front.rangeMm = 600;
  inputs.sides[FrontLeft].valid = false;
  inputs.sides[FrontLeft].rangeStatus = 1;
  inputs.sides[FrontRight].valid = false;
  inputs.sides[FrontRight].rangeStatus = 1;
  Decision decision = updateFresh(controller, inputs, 100, 0.0F);
  check(decision.state == RunState::DirectionProbe &&
            decision.direction == CourseDirection::Unknown &&
            decision.leftOpeningMm < config.cornerOpeningDeltaMm &&
            decision.rightOpeningMm < config.cornerOpeningDeltaMm,
        "SigmaFail alone must not claim an opening or choose a direction");

  inputs.sides[FrontRight].valid = true;
  inputs.sides[FrontRight].rangeStatus = 0;
  inputs.sides[FrontRight].rangeMm = 300;
  decision = updateFresh(controller, inputs, 110, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::CounterClockwise,
        "SigmaFail may indicate an opening only when the opposite wall has a valid closed return");

  Controller strongController(config);
  Inputs strongInputs;
  startController(strongController, strongInputs);
  strongInputs.front.rangeMm = 600;
  strongInputs.sides[FrontLeft].valid = false;
  strongInputs.sides[FrontLeft].rangeStatus = 2;
  strongInputs.sides[FrontRight].valid = false;
  strongInputs.sides[FrontRight].rangeStatus = 1;
  decision = updateFresh(strongController, strongInputs, 100, 0.0F);
  check(decision.state == RunState::Cornering &&
            decision.direction == CourseDirection::CounterClockwise,
        "SignalFail status 2 must remain strong opening evidence without numeric corroboration");

  autonomy::Config exitConfig = fixtureConfig();
  exitConfig.requireAllSideSensorsFresh = false;
  exitConfig.cornerMinimumDurationMs = 0;
  exitConfig.cornerExitStableMs = 0;
  Controller exitController(exitConfig);
  Inputs exitInputs;
  startController(exitController, exitInputs);
  setLeftOpening(exitInputs);
  updateFresh(exitController, exitInputs, 100, 0.0F);
  setStraightGeometry(exitInputs);
  exitInputs.front.valid = false;
  exitInputs.front.rangeStatus = 1;
  updateFresh(exitController, exitInputs, 120, 40.0F);
  decision = updateFresh(exitController, exitInputs, 140, 80.0F);
  check(decision.state == RunState::Cornering,
        "front SigmaFail must not independently claim a clear corner exit");
  exitInputs.front.rangeStatus = 2;
  decision = updateFresh(exitController, exitInputs, 150, 80.0F);
  check(decision.state == RunState::Running,
        "strong front no-target status 2 may confirm clear corner-exit geometry");
}

void testDirectionAndRunFaults() {
  {
    autonomy::Config config = fixtureConfig();
    config.directionTimeoutMs = 100;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    const Decision decision = updateFresh(controller, inputs, 190, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultDirectionTimeout) != 0,
          "lack of first-turn evidence must time out safely");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.directionTimeoutMs = 100;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs, 600);
    const Decision decision = updateFresh(controller, inputs, 190, 0.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultDirectionTimeout) != 0,
          "opening evidence arriving at the direction deadline must not bypass timeout");
  }
  {
    Controller controller(fixtureConfig());
    Inputs inputs;
    startController(controller, inputs);
    const Decision decision = updateFresh(controller, inputs, 100, 60.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultHeadingJump) != 0,
          "physically implausible single-sample heading jump must fault");
  }
  {
    Controller controller(fixtureConfig());
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    Decision decision = updateFresh(controller, inputs, 100, 0.0F);
    check(decision.direction == CourseDirection::CounterClockwise &&
              decision.state == RunState::Cornering,
          "wrong-way test must first enter a physical left corner");
    check(decision.configuredClockwiseYawSign == -1,
          "wrong-way test must use the explicit configured BNO sign");
    decision = updateFresh(controller, inputs, 120, -15.0F);
    check(decision.state == RunState::Cornering,
          "small reverse corner progress must not immediately fault");
    decision = updateFresh(controller, inputs, 140, -25.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultWrongWay) != 0,
          "a reversed servo on the first turn must latch wrong-way");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.clockwiseYawSign = 1;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    updateFresh(controller, inputs, 100, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 120, -25.0F);
    check(decision.state == RunState::Cornering &&
              decision.configuredClockwiseYawSign == 1 &&
              decision.cornerProgressDegrees > 0.0F,
          "an opposite BNO mounting must work only when configured explicitly");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.clockwiseYawSign = 0;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags & FaultConfiguration) != 0 &&
              !decision.motionAllowed,
          "an invalid BNO yaw sign must prohibit starting");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.maxRunMs = 150;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    updateFresh(controller, inputs, 120, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 240, 10.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultRunTimeout) != 0,
          "maximum run time must stop and latch independently of lap count");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.cornerTimeoutMs = 200;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    updateFresh(controller, inputs, 100, 0.0F);
    const Decision decision = updateFresh(controller, inputs, 300, 10.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultCornerTimeout) != 0,
          "a corner that never reaches verified exit geometry must time out");
  }
  {
    autonomy::Config config = fixtureConfig();
    config.cornerMaximumDegrees = 100.0F;
    Controller controller(config);
    Inputs inputs;
    startController(controller, inputs);
    setLeftOpening(inputs);
    updateFresh(controller, inputs, 100, 0.0F);
    updateFresh(controller, inputs, 140, 40.0F);
    updateFresh(controller, inputs, 180, 80.0F);
    const Decision decision = updateFresh(controller, inputs, 220, 120.0F);
    check(decision.state == RunState::FaultLatched &&
              (decision.faultFlags & FaultCornerOvershoot) != 0,
          "corner yaw beyond the configured maximum must stop safely");
  }
}

Decision driveToFinishDelay(Controller& controller, Inputs& inputs,
                            uint32_t& nowMs, float& headingDegrees) {
  startController(controller, inputs);
  nowMs = 100;
  headingDegrees = 0.0F;
  Decision decision;

  for (uint8_t corner = 0; corner < 12; ++corner) {
    if (corner > 0) {
      const uint32_t refractoryMs =
          controller.config().cornerRefractoryMs;
      if (refractoryMs > 0U) {
        // Production timing is covered by testProductionOpenFrontClearRearm.
        // This multi-lap fixture deliberately uses the explicit zero-timing
        // test override and must never underflow now + 0 - 1.
        setLeftOpening(inputs, 300);
        decision = updateFresh(controller, inputs,
                               nowMs + refractoryMs - 1,
                               wrapYaw(headingDegrees));
        check(decision.state == RunState::Running &&
                  decision.cornerCount == corner,
              "corner opening inside refractory time must be ignored");
        nowMs += refractoryMs;
      }
    }

    setLeftOpening(inputs, 300);
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    check(decision.state == RunState::Cornering &&
              decision.direction == CourseDirection::CounterClockwise,
          "each winning left opening must enter one counter-clockwise corner");
    check(decision.cornerCount == corner && decision.motionAllowed &&
              decision.steeringPermille < 0,
          "entering a corner must not count it before physical exit");

    nowMs += 100;
    headingDegrees += 15.0F;
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    nowMs += 100;
    headingDegrees += 35.0F;
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    check(decision.cornerCount == corner,
          "yaw progress alone must not count a corner");

    // Reacquire two straight wall pairs only after the minimum turn duration
    // and enough yaw. The first sample starts, but cannot complete, the
    // stability interval.
    nowMs += 150;
    headingDegrees += 25.0F;
    setStraightGeometry(inputs);
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    check(decision.state == RunState::Cornering &&
              decision.cornerCount == corner,
          "one straight-wall sample must not finish a corner");

    nowMs += controller.config().cornerExitStableMs;
    headingDegrees += 15.0F;
    decision = updateFresh(controller, inputs, nowMs,
                           wrapYaw(headingDegrees));
    check(decision.cornerCount == static_cast<uint8_t>(corner + 1),
          "stable straight-wall reacquisition must count exactly one exit");
    check(decision.lapCount == static_cast<uint8_t>((corner + 1) / 4),
          "four verified corner exits must equal one lap");
  }

  check(decision.cornerCount == 12 && decision.lapCount == 3,
        "twelve verified corner exits must equal exactly three laps");
  check(decision.state == RunState::FinishDelay && decision.motionAllowed &&
            decision.speedPermille == controller.config().finishSpeedPermille &&
            decision.driveDirection == DriveDirection::Forward &&
            decision.parkingPhase == ParkingPhase::Disabled,
        "twelfth verified corner must hold a controlled finish approach before stopping");
  return decision;
}

void testThreeLapsAndPermanentFinish() {
  Controller controller(fixtureConfig());
  Inputs inputs;
  uint32_t nowMs = 0;
  float headingDegrees = 0.0F;
  Decision decision =
      driveToFinishDelay(controller, inputs, nowMs, headingDegrees);

  const uint32_t finishEnteredAt = nowMs;
  const uint32_t finishDriveMs = controller.config().finishDriveMs;
  const uint32_t finishStableMs = controller.config().finishStableMs;
  decision = updateFresh(controller, inputs,
                         finishEnteredAt + finishDriveMs - 1,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::FinishDelay && decision.motionAllowed &&
            decision.speedPermille == controller.config().finishSpeedPermille,
        "finish delay must not stop one millisecond early");

  inputs.sides[FrontLeft].rangeMm = 450;
  inputs.sides[BackLeft].rangeMm = 300;
  inputs.sides[FrontRight].rangeMm = 300;
  inputs.sides[BackRight].rangeMm = 450;
  decision = updateFresh(controller, inputs, finishEnteredAt + finishDriveMs,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::FinishDelay && decision.motionAllowed,
        "non-parallel wall geometry must keep the 12-corner finish hold moving");

  setStraightGeometry(inputs);
  const uint32_t finishReadyAt = finishEnteredAt + finishDriveMs + 1U;
  decision = updateFresh(controller, inputs, finishReadyAt,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::FinishDelay,
        "aligned finish geometry must start, not skip, stability time");
  decision = updateFresh(controller, inputs,
                         finishReadyAt + finishStableMs - 1,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::FinishDelay,
        "one-pair finish stability must not finish one millisecond early");
  const uint32_t finishedAt = finishReadyAt + finishStableMs;
  decision = updateFresh(controller, inputs, finishedAt,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::Finished && !decision.motionAllowed &&
            decision.speedPermille == 0,
        "aligned, parallel, stable finish must stop completely");

  decision = updateFresh(controller, inputs, finishedAt + 25,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::Finished && !decision.motionAllowed &&
            decision.speedPermille == 0 && decision.steeringPermille == 0,
        "finished state must hold the 12-corner stop on subsequent updates");

  inputs.buttonPressed = false;
  updateFresh(controller, inputs, finishedAt + 50, 0.0F);
  inputs.buttonPressed = true;
  updateFresh(controller, inputs, finishedAt + 100, 0.0F);
  decision = updateFresh(controller, inputs, finishedAt + 150, 0.0F);
  check(decision.state == RunState::Finished && !decision.motionAllowed,
        "a finished run must never restart from the button");
}

void testFinishTimeoutSafety() {
  const autonomy::Config baseline = fixtureConfig();
  const uint32_t minimumValidTimeout =
      baseline.finishDriveMs + baseline.finishStableMs;
  const uint32_t invalidTimeouts[] = {
      minimumValidTimeout,
      minimumValidTimeout - 1U,
  };
  for (const uint32_t invalidTimeout : invalidTimeouts) {
    autonomy::Config config = fixtureConfig();
    config.finishTimeoutMs = invalidTimeout;
    Controller controller(config);
    Inputs inputs = healthyInputs(0);
    const Decision decision = controller.update(inputs);
    check(decision.state == RunState::WaitingForHealth &&
              (decision.faultFlags & FaultConfiguration) != 0 &&
              !decision.startLatched && !decision.motionAllowed &&
              decision.speedPermille == 0,
          "finish timeout without drive-plus-stability margin must prohibit start");
  }

  autonomy::Config config = fixtureConfig();
  Controller controller(config);
  Inputs inputs;
  uint32_t finishEnteredAt = 0;
  float headingDegrees = 0.0F;
  driveToFinishDelay(controller, inputs, finishEnteredAt, headingDegrees);

  // Keep both wall pairs valid but deliberately non-parallel. The 150 mm
  // opening deltas stay below the 180 mm corner-opening threshold, isolating
  // the absolute finish timeout from the separate next-corner guard.
  inputs.sides[FrontLeft].rangeMm = 450;
  inputs.sides[BackLeft].rangeMm = 300;
  inputs.sides[FrontRight].rangeMm = 300;
  inputs.sides[BackRight].rangeMm = 450;

  const uint32_t timeoutMs = controller.config().finishTimeoutMs;
  Decision decision = updateFresh(controller, inputs,
                                  finishEnteredAt + timeoutMs - 1U,
                                  wrapYaw(headingDegrees));
  check(decision.state == RunState::FinishDelay && decision.motionAllowed,
        "non-parallel finish geometry must remain moving one millisecond before timeout");

  decision = updateFresh(controller, inputs, finishEnteredAt + timeoutMs,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultFinishTimeout) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0 &&
            decision.steeringPermille == 0,
        "absolute finish timeout must latch a complete stop at its boundary");

  setStraightGeometry(inputs);
  decision = updateFresh(controller, inputs, finishEnteredAt + timeoutMs + 100U,
                         wrapYaw(headingDegrees));
  check(decision.state == RunState::FaultLatched &&
            (decision.faultFlags & FaultFinishTimeout) != 0 &&
            !decision.motionAllowed && decision.speedPermille == 0,
        "healthy finish geometry must never clear a latched finish timeout");
}

}  // namespace

int main() {
  check(vision::protocolSelfTest(),
        "ESP32 UART protocol golden-vector self-test must pass");
  testAngleWrapping();
  testHeldButtonCannotStart();
  testDirectionOpeningAndHeadingWrap();
  testOpeningConfirmation();
  testWallGuidanceSignsAndClamp();
  testObstacleSideSelection();
  testObstacleRangeCannotBecomeCorner();
  testSpeedEnvelopeAndEmergencyStop();
  testProductionProfileContracts();
  testR40GuardedSpeedsPreserveCornerTorqueWithoutStraightSurge();
  testR41FreshClearRouteReleasesMissingPillarWithoutCourseCredit();
  testR41ForwardEscapeUsesFreshLeadingAndTailClearance();
  testR41CornerCoastCanUseClearForwardRouteWithoutFakeProgress();
  testR41ForwardEscapeIsBoundedByAppliedDriveHeadingAndTime();
  testR41SameColorSuccessorRefreshesPlanWithoutPassCredit();
  testR41ForwardCornerEscapeTapersAndUsesNormalExitProof();
  testR42ObstacleSectionMemoryIsOptionalWithoutChangingLiveSteering();
  testR42OpticalRecoveryHoldReplaysFreshInvalidLeadingRay();
  testR42ReverseReadinessNeverTreatsUnknownRearAsClear();
  testR42SuccessfulBackoffSkipsOnlyOldColorForOneSecond();
  testR20ParkingScaffoldingFailsClosed();
  testR21SinglePassCalibrationStopsOnlyAfterVerifiedClear();
  testDriveDirectionNeutralInterlock();
  testProductionOpenDirectionProbeUsesCurrentForwardSpace();
  testProductionObstacleDirectionProbeUsesCurrentForwardSpace();
  testR19FirstDirectionApproachAndStoppedRetry();
  testR19FirstDirectionRejectsNearbyStart();
  testR20FirstCornerBackoffRestoresMeasuredTurnRoom();
  testR20FirstCornerBackoffRejectsUnprovedRetreat();
  testR20FirstCornerBackoffStopsForRearAndReverseBudget();
  testR20BackoffWallDeadlineAndDistanceCap();
  testR20ApproachOpticalGapRetainsAnchorWithoutTravelCredit();
  testR20AllBlindCasesRemainNonRecoverable();

  testR19FirstDirectionOpticalGapsDoNotEraseFreshProof();
  testR19FirstTurnThenTwelveCornersAndPermanentStop();
  testR19FirstTurnThenTwelveCornersAndPermanentStop(true);
  testR21OpticalFallbackIsSeparateAndReacquiresBothDirections();
  testR21AlternatingNumericFramesCannotEvadeBlindMotionBudget();
  testR21CloseThenBlindCannotCreepAndAbsoluteRunDeadlineRemains();
  testR21OpticalHoldNeverMasksHardFaults();
  testR21DirectionTimeoutReacquiresFreshCandidate();
  testR21OpticalHoldPreservesPendingCornerAndPausesOnlyItsClock();
  testProductionDirectionProbeIsHeadingOnlyAndUsesFrontEnvelope();
  testProductionOpenKnownStraightIsHeadingOnly();
  testProductionObstacleDiagonalRearBiasCannotSteer();
  testR21DepthRampedMirroredCameraSteering();
  testR21SideClearanceArbitration();
  testR21CornerRearmRequiresCardinalHeading();
  testProductionOpenUsesMeasuredWrappedFrontRange();
  testOpenHeadingOnlyStraightStartsFullTorqueCorner();
  testProductionObstacleCornerBypassesFrontEnvelope();
  testProductionOpenCornerStallIsBoundedAndCanRecover();
  testOpenBlindFrontFallbackIsStrictlyGated();
  testActiveObstacleBlocksDirectionLearning();
  testProductionObstacleMaximumCommitTimeout();
  testCommittedObstaclePassThroughCameraLoss();
  testObstacleMotionFloorAndOverrunRejection();
  testRequiredVisionLossStopsThenRecoversOrFaults();
  testPassClearSidesAndMillisecondAcceleration();
  testConsecutiveSameColorPillarsUseBoundedCooldown();
  testPassClearAcceptsOneStrongNoTargetRay();
  testObstacleAbsenceRequiresFreshTrulyBlankFrames();
  testVisionSafetyCapsAndHeadingRecovery();
  testLearnedObstacleSectionCapsLaterLapWithoutSteering();
  testCornerExitObservationRecordsNewStraightSection();
  testCornerExitStabilityIgnoresCommittedObstacleSteering();
  testCornerBlindGeometryAndImuAccuracyGrace();
  testOpenChallengeAcceptsFarRearSensorsWithoutLosingSafety();
  testOpenChallengeKeepsTurningThroughOpticalNoTarget();
  testOpenCornerExitUnwindsToCardinalHeadingSymmetrically();
  testOpenRepeatedCornersStayAnchoredToRunHeading();
  testAllOpenCornersStayAnchoredAcrossThreeLaps();
  testFinalOpenCornerKeepsApproachSteeringInFinishDelay();
  testOpenCornerExitStabilityRejectsActiveOvershoot();
  testOpenSideElectricalLossUsesStoppedBoundedGrace();
  testStartupGeometryUsesAnyCompleteSafeSolution();
  testR23BoundedCalibrationCrossGeometry();
  testR24CalibrationClosePassNeverCounterSteersTowardPillar();
  testR27CalibrationUsesMeasuredCenteredEscapeHeading();
  testR28CenteredBodyClearUsesFreshWrapRange();
  testR30CenteredEscapeRecoversBeforeFreshPassProof();
  testR31IndependentDiagonalObservations();
  testR32DelayedDrivetrainStillCompletesEscape();
  testR33EscapeDeadlineUsesAppliedDriveTime();
  testR34FarPreviewDoesNotBlockFirstCorner();
  testR36EarlyOpticalOpeningSurvivesLaterNumericWallView();
  testR36CornerWallReversePreservesThePendingTurn();
  testR37CommittedPillarWallBackoutReacquiresCurrentTarget();
  testR38LeadingSideHazardPreemptsCornerMotion();
  testR38RearOnlyStartupAndReverseGuards();
  testR38LeadingHazardWinsAtStraightAndCornerExitBoundaries();
  testR38AttemptCapRequiresVerifiedSectionProgressToReset();
  testR39RearTravelLimitAllowsGuardedForwardReplanning();
  testR39UnplannedForwardDepartureRemainsBounded();
  testR39ReverseRequiresStrongRearEvidenceAndHonorsHardLimit();
  testR39TimeLimitedReverseReplansOnlyAnExistingManeuver();
  testR39ShortRearBrakeRequiresFreshForwardRouteConfirmation();
  testR42SkippedPillarCanLeaveRearLimitAlongKnownStraight();
  testR35WallReverseRecoveryIsBoundedAndGuarded();
  testR31FullProfileStillFinishesAfterTwelveCorners();
  testR31FullProfileStillFinishesAfterTwelveCorners(1U);
  testR31FullProfileStillFinishesAfterTwelveCorners(2U);
  testObstacleBackupProfileContractsAndGuidedOpticalCorners();
  testR45FreshOpticalOpenKeepsMovingTurnsAndRecovers();
  testObstacleBackupWeakOrUnguidedOpticalCannotTurnBlind();
  testObstacleBackupStoppedHealthHoldsPreserveCornerAndHeading();
  testObstacleBackupDirectionHoldAndAbsoluteDeadline();
  testR44CameraIndependentRouteSkipsOnlyTheStaleTarget();
  testObstacleBackupHardFaultsRemainMotionBlocking();
  testObstacleBackupSoftCornerHoldCannotBeOverwrittenBySideEscape();
  testR44RouteProofMustRestartAfterCameraOrElectricalGap();
  testR44CornerTimeoutPermitsOnlyOneMeasuredRetry();
  testR25CalibrationBodyClearIsBoundedAndGuarded();
  testWrappedTargetIsDegradedButNeverOpeningEvidence();
  testProductionOpenFrontClearRearm();
  testProductionObstacleRefreshesBaselineAndBlocksRearmDuringPass();
  testArmedCornerDetectionDisarmsForMirroredObstaclePasses();
  testElectricalRecoveryRestartsCornerAndOpeningStability();
  testRequiredSensorFaults();
  testOpticalNoReturnDegradesWithoutPretendingItIsClear();
  testFrontOpticalNoTargetHasBoundedStateAwareGrace();
  testSigmaFailRequiresValidOpeningCorroboration();
  testDirectionAndRunFaults();
  testThreeLapsAndPermanentFinish();
  testFinishTimeoutSafety();

  if (failures != 0) {
    std::cerr << failures << " autonomy host check(s) failed\n";
    return 1;
  }
  std::cout << "autonomy host checks passed\n";
  return 0;
}

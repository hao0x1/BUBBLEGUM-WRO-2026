#include "autonomyController.h"

#include <math.h>

namespace autonomy {

namespace {

constexpr uint8_t kCornersPerLap = 4;
constexpr uint8_t kRequiredLaps = 3;
constexpr uint8_t kRequiredCorners = kCornersPerLap * kRequiredLaps;
constexpr uint32_t kRecoveryObstacleSkipMs = 1000U;
constexpr float kRightAngleDegrees = 90.0F;
// R20 deliberately adds only parking interfaces and safety gates. This guard
// prevents a configuration edit from reaching unfinished motion code.
constexpr bool kParkingStateMachineImplemented = false;

float average(float a, float b) { return (a + b) * 0.5F; }

bool opticalNoTargetStatus(uint8_t status) {
  // ST's normalized 1/2/4/6/7 results are fresh nonfatal optical outcomes.
  // Status 6 carries a numeric front range in the VL53L1X acquisition layer;
  // if a side VL53L4CD reports it, keep the sample electrically healthy but
  // geometrically degraded. Hardware, synchronization, malformed (255), and
  // too-close (3) results must never receive this grace.
  return status == 1U || status == 2U || status == 4U || status == 6U ||
         status == 7U;
}

bool strongOpticalNoTargetStatus(uint8_t status) {
  // SigmaFail (1) is deliberately excluded: it says the returned range is
  // too uncertain, not that the ray positively found open space. SignalFail
  // and phase/out-of-bounds (2/4) are the stronger no-return classes used by
  // this controller as opening evidence.
  return status == 2U || status == 4U;
}

}  // namespace

DriveDirection DriveDirectionInterlock::update(DriveDirection requested,
                                                uint32_t nowMs,
                                                uint32_t neutralMs) {
  if (requested == DriveDirection::Stopped) {
    if (lastNonzeroDirection_ != DriveDirection::Stopped &&
        !neutralActive_) {
      neutralActive_ = true;
      neutralStartedAtMs_ = nowMs;
    }
    return DriveDirection::Stopped;
  }

  if (lastNonzeroDirection_ == DriveDirection::Stopped ||
      requested == lastNonzeroDirection_) {
    lastNonzeroDirection_ = requested;
    neutralActive_ = false;
    return requested;
  }

  if (!neutralActive_) {
    neutralActive_ = true;
    neutralStartedAtMs_ = nowMs;
  }
  if (static_cast<uint32_t>(nowMs - neutralStartedAtMs_) < neutralMs) {
    return DriveDirection::Stopped;
  }

  lastNonzeroDirection_ = requested;
  neutralActive_ = false;
  return requested;
}

void DriveDirectionInterlock::reset() {
  lastNonzeroDirection_ = DriveDirection::Stopped;
  neutralActive_ = false;
  neutralStartedAtMs_ = 0;
}

Controller::Controller(const Config& config) : config_(config) {
  if (config_.clockwiseYawSign != -1 && config_.clockwiseYawSign != 1) {
    configurationValid_ = false;
    // Keep subsequent arithmetic defined even though safety will prohibit a
    // start and report FaultConfiguration.
    config_.clockwiseYawSign = -1;
  }
  if (config_.finishTimeoutMs <=
      config_.finishDriveMs + config_.finishStableMs) {
    configurationValid_ = false;
  }
  if (config_.maximumSteeringPermille < 0) {
    config_.maximumSteeringPermille = 0;
  } else if (config_.maximumSteeringPermille > 1000) {
    config_.maximumSteeringPermille = 1000;
  }
  if (config_.cornerSteeringPermille < 0) {
    config_.cornerSteeringPermille =
        static_cast<int16_t>(-config_.cornerSteeringPermille);
  }
  if (config_.cornerSteeringPermille > config_.maximumSteeringPermille) {
    config_.cornerSteeringPermille = config_.maximumSteeringPermille;
  }
  if (config_.cornerCompletionDegrees < 45.0F ||
      config_.cornerCompletionDegrees > 100.0F) {
    config_.cornerCompletionDegrees = 72.0F;
  }
  if (config_.cornerMaximumDegrees <= config_.cornerCompletionDegrees ||
      config_.cornerMaximumDegrees > 170.0F) {
    config_.cornerMaximumDegrees = 125.0F;
  }
  if (config_.openCornerExitFallbackDegrees <
          config_.cornerCompletionDegrees ||
      config_.openCornerExitFallbackDegrees >= config_.cornerMaximumDegrees) {
    config_.openCornerExitFallbackDegrees =
        (config_.cornerCompletionDegrees + config_.cornerMaximumDegrees) *
        0.5F;
  }
  if (config_.openCornerExitMaximumDegrees <=
          config_.openCornerExitFallbackDegrees ||
      config_.openCornerExitMaximumDegrees >= config_.cornerMaximumDegrees) {
    config_.openCornerExitMaximumDegrees =
        (config_.openCornerExitFallbackDegrees +
         config_.cornerMaximumDegrees) *
        0.5F;
  }
  if (config_.openCornerTaperStartDegrees < 0.0F ||
      config_.openCornerTaperStartDegrees >=
          config_.openCornerExitFallbackDegrees) {
    config_.openCornerTaperStartDegrees =
        config_.openCornerExitFallbackDegrees * 0.65F;
  }
  if (config_.openCornerCounterSteerStartDegrees <= kRightAngleDegrees ||
      config_.openCornerCounterSteerStartDegrees >=
          config_.cornerMaximumDegrees) {
    config_.openCornerCounterSteerStartDegrees = 92.0F;
  }
  if (config_.openCornerCounterSteerRampDegrees <= 0.0F ||
      config_.openCornerCounterSteerRampDegrees > 45.0F) {
    config_.openCornerCounterSteerRampDegrees = 8.0F;
  }
  if (config_.openPostCornerCounterSteerMs >
      config_.postCornerHeadingOnlyMs) {
    config_.openPostCornerCounterSteerMs = config_.postCornerHeadingOnlyMs;
  }
  if (config_.obstacleDesiredImageOffset < 0) {
    config_.obstacleDesiredImageOffset =
        static_cast<int16_t>(-config_.obstacleDesiredImageOffset);
  }
  if (config_.obstacleMaximumOffsetMm < 0) {
    config_.obstacleMaximumOffsetMm =
        static_cast<int16_t>(-config_.obstacleMaximumOffsetMm);
  }
  if (config_.obstacleMinimumOffsetMm < 0) {
    config_.obstacleMinimumOffsetMm =
        static_cast<int16_t>(-config_.obstacleMinimumOffsetMm);
  }
  if (config_.obstacleMinimumOffsetMm > config_.obstacleMaximumOffsetMm) {
    config_.obstacleMinimumOffsetMm = config_.obstacleMaximumOffsetMm;
  }
  if (config_.obstacleMaximumImageSteeringPermille < 0) {
    config_.obstacleMaximumImageSteeringPermille = static_cast<int16_t>(
        -config_.obstacleMaximumImageSteeringPermille);
  }
  if (config_.obstacleMaximumImageSteeringPermille >
      config_.maximumSteeringPermille) {
    config_.obstacleMaximumImageSteeringPermille =
        config_.maximumSteeringPermille;
  }
  if (config_.obstacleCenteredEscapeInitialAbsX < 0) {
    config_.obstacleCenteredEscapeInitialAbsX = static_cast<int16_t>(
        -config_.obstacleCenteredEscapeInitialAbsX);
  }
  if (config_.obstacleSideClearanceSteeringPermille < 0) {
    config_.obstacleSideClearanceSteeringPermille = static_cast<int16_t>(
        -config_.obstacleSideClearanceSteeringPermille);
  }
  if (config_.obstacleSideClearanceSteeringPermille >
      config_.maximumSteeringPermille) {
    config_.obstacleSideClearanceSteeringPermille =
        config_.maximumSteeringPermille;
  }
  if (config_.maximumHeadingCorrectionPermille < 0) {
    config_.maximumHeadingCorrectionPermille = static_cast<int16_t>(
        -config_.maximumHeadingCorrectionPermille);
  }
  if (config_.maximumHeadingCorrectionPermille >
      config_.maximumSteeringPermille) {
    config_.maximumHeadingCorrectionPermille =
        config_.maximumSteeringPermille;
  }
  if (config_.maximumWallCorrectionPermille < 0) {
    config_.maximumWallCorrectionPermille = static_cast<int16_t>(
        -config_.maximumWallCorrectionPermille);
  }
  if (config_.maximumWallCorrectionPermille >
      config_.maximumSteeringPermille) {
    config_.maximumWallCorrectionPermille = config_.maximumSteeringPermille;
  }
  if (config_.openMaximumWallAngleCorrectionPermille < 0) {
    config_.openMaximumWallAngleCorrectionPermille = static_cast<int16_t>(
        -config_.openMaximumWallAngleCorrectionPermille);
  }
  if (config_.openMaximumWallAngleCorrectionPermille >
      config_.maximumWallCorrectionPermille) {
    config_.openMaximumWallAngleCorrectionPermille =
        config_.maximumWallCorrectionPermille;
  }
  if (config_.openCornerMinimumApproachSteerPermille < 0) {
    config_.openCornerMinimumApproachSteerPermille = static_cast<int16_t>(
        -config_.openCornerMinimumApproachSteerPermille);
  }
  if (config_.openCornerMinimumApproachSteerPermille >
      config_.cornerSteeringPermille) {
    config_.openCornerMinimumApproachSteerPermille =
        config_.cornerSteeringPermille;
  }
  if (config_.openCornerCounterSteerPermille < 0) {
    config_.openCornerCounterSteerPermille = static_cast<int16_t>(
        -config_.openCornerCounterSteerPermille);
  }
  if (config_.openCornerCounterSteerPermille >
      config_.maximumSteeringPermille) {
    config_.openCornerCounterSteerPermille =
        config_.maximumSteeringPermille;
  }
  config_.directionProbeSpeedPermille =
      clampSpeed(config_.directionProbeSpeedPermille);
  config_.cruiseSpeedPermille = clampSpeed(config_.cruiseSpeedPermille);
  config_.cornerSpeedPermille = clampSpeed(config_.cornerSpeedPermille);
  config_.cornerBlindSpeedPermille =
      clampSpeed(config_.cornerBlindSpeedPermille);
  config_.obstacleSpeedPermille = clampSpeed(config_.obstacleSpeedPermille);
  config_.visionUncertainSpeedPermille =
      clampSpeed(config_.visionUncertainSpeedPermille);
  config_.visionOverrunSpeedPermille =
      clampSpeed(config_.visionOverrunSpeedPermille);
  config_.obstacleUncertainSpeedPermille =
      clampSpeed(config_.obstacleUncertainSpeedPermille);
  config_.obstacleSideClearanceSpeedPermille =
      clampSpeed(config_.obstacleSideClearanceSpeedPermille);
  config_.finishSpeedPermille = clampSpeed(config_.finishSpeedPermille);
  config_.minimumMovingSpeedPermille =
      clampSpeed(config_.minimumMovingSpeedPermille);
  if (config_.frontStoppingProxyMm == 0U) {
    configurationValid_ = false;
  }
  if (config_.cornerRearmFrontClearMm <= config_.frontCornerTriggerMm ||
      config_.openOuterWallTargetMm == 0U ||
      config_.openInnerWallMinimumMm == 0U ||
      config_.obstacleCloseBottomY < config_.obstacleMinimumBottomY ||
      config_.obstacleFullSteeringBottomY <=
          config_.obstacleMinimumBottomY ||
      config_.obstacleFullSteeringBottomY > 10000U ||
      config_.obstacleImageSteeringPerUnit < 0.0F ||
      config_.obstacleCornerRearmHeadingDegrees <= 0.0F ||
      config_.obstacleCornerRearmHeadingDegrees > 45.0F ||
      config_.obstaclePreCloseLossClearMs <= config_.obstacleBlobLossMs ||
      (config_.obstacleMaximumCommitMs > 0U &&
       (config_.obstacleMaximumCommitMs <= config_.obstacleMinimumCommitMs ||
        config_.obstaclePreCloseLossClearMs >=
            config_.obstacleMaximumCommitMs))) {
    configurationValid_ = false;
  }
  if (config_.minimumRunImuAccuracy > 3U ||
      config_.cornerBlindSpeedPermille > config_.cornerSpeedPermille ||
      config_.openingMinimumDistinctSamples == 0U ||
      config_.directionMinimumDistinctSamples == 0U ||
      config_.cornerDriveDelayMs >= config_.cornerTimeoutMs) {
    configurationValid_ = false;
  }
  if (config_.parkingEnabled &&
      (!kParkingStateMachineImplemented ||
       !config_.parkingCalibrationValidated ||
       !config_.parkingRearClearanceValidated ||
       config_.parkingBaySide == ParkingBaySide::Unknown ||
       config_.parkingReverseNeutralMs <
           kMinimumDriveReversalNeutralMs)) {
    configurationValid_ = false;
  }
  if (guardedPassEnabled() &&
      !config_.obstacleAvoidanceEnabled) {
    configurationValid_ = false;
  }
  if (config_.obstacleBodyClearFrontGuardMm > 0U &&
      (!guardedPassEnabled() ||
       config_.obstacleSinglePassBodyClearMs == 0U ||
       config_.obstacleBodyClearFrontGuardMm <= config_.frontEmergencyStopMm)) {
    configurationValid_ = false;
  }
  if (config_.obstacleSinglePassBodyClearMs > 0U &&
      (!guardedPassEnabled() ||
       config_.obstacleSinglePassBodyClearMs >= config_.maxRunMs ||
       config_.obstacleBodyClearOppositeSideGuardMm == 0U ||
       config_.obstacleBodyClearOppositeSideGuardMm >
           config_.obstacleSideClearanceGuardMm)) {
    configurationValid_ = false;
  }
  const bool anyCenteredEscapeSetting =
      config_.obstacleCenteredEscapeInitialAbsX > 0 ||
      config_.obstacleCenteredEscapeHeadingDegrees > 0.0F ||
      config_.obstacleCenteredEscapeDeadlineMs > 0U ||
      config_.obstacleCenteredEscapeHeadingStableMs > 0U;
  const bool completeCenteredEscapeSetting =
      config_.obstacleCenteredEscapeInitialAbsX > 0 &&
      config_.obstacleCenteredEscapeInitialAbsX <= 10000 &&
      config_.obstacleCenteredEscapeHeadingDegrees > 0.0F &&
      config_.obstacleCenteredEscapeHeadingDegrees <= 45.0F &&
      config_.obstacleCenteredEscapeDeadlineMs > 0U &&
      config_.obstacleCenteredEscapeHeadingStableMs > 0U &&
      config_.obstacleCenteredEscapeHeadingStableMs <
          config_.obstacleCenteredEscapeDeadlineMs &&
      config_.obstacleCenteredEscapeDeadlineMs < config_.maxRunMs &&
      config_.obstacleMaximumImageSteeringPermille > 0 &&
      config_.obstacleSideClearanceGuardMm > 0U &&
      config_.obstacleSideClearanceSpeedPermille > 0U;
  if (anyCenteredEscapeSetting &&
      (!guardedPassEnabled() ||
       !completeCenteredEscapeSetting)) {
    configurationValid_ = false;
  }
  if ((config_.recoverCenteredEscapeBeforePass &&
       (!guardedPassEnabled() ||
        !completeCenteredEscapeSetting ||
        config_.obstacleBodyClearFrontGuardMm == 0U ||
        config_.obstacleBodyClearOppositeSideGuardMm == 0U)) ||
      config_.obstaclePassProofMinimumBottomY > 10000U) {
    configurationValid_ = false;
  }
  reset();
}

void Controller::reset() {
  decision_ = Decision();
  opticalProbeCandidate_ = CourseDirection::Unknown;
  opticalProbeDirection_ = CourseDirection::Unknown;
  opticalProbeSinceMs_ = 0;
  opticalProbeConfirmedAtMs_ = 0;
  opticalProbeLastLeftAtMs_ = 0;
  opticalProbeLastRightAtMs_ = 0;
  opticalProbeSamples_ = 0;
  wallRecoveryAttempts_ = 0;
  rearDepartureConstraintMask_ = 0;
  rearStraightDepartureActive_ = false;
  rearStraightDepartureExhausted_ = false;
  rearStraightDepartureStartedAtMs_ = 0;
  rearStraightDepartureUpdatedAtMs_ = 0;
  rearStraightDeparturePoweredMs_ = 0;
  forwardSideEscapeActive_ = false;
  forwardSideEscapeSuppressed_ = false;
  forwardSideEscapeHazards_ = 0;
  forwardSideEscapeBlockedHazards_ = 0;
  forwardSideEscapeStartedAtMs_ = 0;
  forwardSideEscapeUpdatedAtMs_ = 0;
  forwardSideEscapePoweredMs_ = 0;
  forwardSideEscapeHeadingDegrees_ = 0.0F;
  wallRecoveryStartedAtMs_ = 0;
  wallRecoveryPhaseStartedAtMs_ = 0;
  wallRecoveryUpdatedAtMs_ = 0;
  wallRecoveryPoweredMs_ = 0;
  wallRecoveryHealthHoldActive_ = false;
  wallRecoveryClearSinceMs_ = 0;
  wallRecoveryClearLastEvidenceMs_ = 0;
  wallRecoveryClearLeftSampleMs_ = 0;
  wallRecoveryClearRightSampleMs_ = 0;
  wallRecoveryClearSamples_ = 0;
  wallRecoveryForwardSinceMs_ = 0;
  wallRecoveryForwardEvidenceMs_ = 0;
  wallRecoveryForwardFrontSampleMs_ = 0;
  wallRecoveryForwardLeftSampleMs_ = 0;
  wallRecoveryForwardRightSampleMs_ = 0;
  wallRecoveryForwardSamples_ = 0;
  wallRecoveryHeadingReferenceDegrees_ = 0.0F;
  decision_.configuredClockwiseYawSign = config_.clockwiseYawSign;
  driveDirectionInterlock_.reset();
  rawButtonPressed_ = false;
  stableButtonPressed_ = false;
  buttonArmed_ = false;
  rawButtonChangedAtMs_ = 0;
  resetAtMs_ = 0;

  headingInitialized_ = false;
  lastRawHeadingDegrees_ = 0.0F;
  unwrappedHeadingDegrees_ = 0.0F;
  runStartHeadingDegrees_ = 0.0F;
  cornerStartHeadingDegrees_ = 0.0F;
  cornerTargetHeadingDegrees_ = 0.0F;
  straightHeadingReferenceDegrees_ = 0.0F;
  openingCandidate_ = CourseDirection::Unknown;
  leftOpeningBaselineValid_ = false;
  rightOpeningBaselineValid_ = false;
  leftOpeningBaselineMm_ = 0;
  rightOpeningBaselineMm_ = 0;

  runStartedAtMs_ = 0;
  directionProbeTimerStartedAtMs_ = 0;
  lastCornerAtMs_ = 0;
  cornerRearmClearSinceMs_ = 0;
  cornerDetectionArmed_ = false;
  cornerStartedAtMs_ = 0;
  cornerReadySinceMs_ = 0;
  openingCandidateSinceMs_ = 0;
  openingCandidateLastLeftSampleAtMs_ = 0;
  openingCandidateLastRightSampleAtMs_ = 0;
  openingCandidateLastFrontSampleAtMs_ = 0;
  openingCandidateLastEvidenceAtMs_ = 0;
  openingCandidateDistinctSamples_ = 0;
  directionDecisionHoldSinceMs_ = 0;
  directionDecisionHoldActive_ = false;
  finishEnteredAtMs_ = 0;
  finishReadySinceMs_ = 0;
  straightSideGeometryLostSinceMs_ = 0;
  cornerSideGeometryLostSinceMs_ = 0;
  sideElectricalLostSinceMs_ = 0;
  frontOpticalLostSinceMs_ = 0;
  straightSideGeometryLossActive_ = false;
  cornerSideGeometryLossActive_ = false;
  sideElectricalLossActive_ = false;
  frontOpticalLossActive_ = false;
  frontOpticallyDegraded_ = false;
  sideGeometryDegraded_ = false;
  calibrationCrossGuardActive_ = false;
  imuAccuracyLostSinceMs_ = 0;
  visionLostSinceMs_ = 0;
  visionLossActive_ = false;
  committedObstacle_ = ObstacleTarget::None;
  committedObstacleOffsetMm_ = 0;
  committedObstacleCenterX_ = 0;
  committedObstacleConfidence_ = 0;
  obstacleCommittedAtMs_ = 0;
  obstacleClearSinceMs_ = 0;
  obstacleBodyClearStartedAtMs_ = 0;
  obstacleBodyClearActive_ = false;
  obstacleCenteredEscapeActive_ = false;
  obstacleCenteredEscapeTurnComplete_ = false;
  obstacleCenteredEscapeDrivenMs_ = 0;
  obstacleCenteredEscapeDriveUpdatedAtMs_ = 0;
  obstacleCenteredEscapeRecovered_ = false;
  obstacleCenteredEscapeRecoveryCandidateActive_ = false;
  obstacleCenteredEscapeRecoveryCandidateSampleAtMs_ = 0;
  obstacleCenteredEscapeRecoveredVisionSampleAtMs_ = 0;
  obstacleCenteredEscapePostRecoveryObserved_ = false;
  obstacleSuccessorCandidateActive_ = false;
  obstacleSuccessorCandidateSampleAtMs_ = 0;
  obstacleSuccessorRollover_ = false;
  obstacleCenteredEscapeClearanceViolated_ = false;
  obstacleCenteredEscapeHeadingCandidateActive_ = false;
  obstacleCenteredEscapeHeadingCandidateSampleAtMs_ = 0;
  obstacleCenteredEscapeHeadingReferenceDegrees_ = 0.0F;
  obstacleAbsentSinceMs_ = 0;
  obstaclePeakBottomY_ = 0;
  obstaclePairDeltaInitialized_ = false;
  obstacleMinimumPairDeltaMm_ = 0.0F;
  lastClearedObstacle_ = ObstacleTarget::None;
  obstacleRecommitBlockedSinceMs_ = 0;
  recoverySkippedObstacle_ = ObstacleTarget::None;
  recoverySkippedObstacleAtMs_ = 0;
  obstacleRecenterStartedAtMs_ = 0;
  obstacleRecenterStartOffsetMm_ = 0;
  obstaclePassCornerRecoveryPending_ = false;
  lastCommandedSpeedPermille_ = 0;
  lastSpeedCommandAtMs_ = 0;
  latchedFaultFlags_ = FaultNone;
}

float Controller::shortestAngleDeltaDegrees(float fromDegrees,
                                             float toDegrees) {
  float delta = toDegrees - fromDegrees;
  while (delta <= -180.0F) delta += 360.0F;
  while (delta > 180.0F) delta -= 360.0F;
  return delta;
}

bool Controller::elapsedAtLeast(uint32_t nowMs, uint32_t sinceMs,
                                uint32_t intervalMs) {
  return static_cast<uint32_t>(nowMs - sinceMs) >= intervalMs;
}

bool Controller::readingFresh(uint32_t nowMs, uint32_t updatedAtMs,
                              uint32_t maximumAgeMs) {
  return static_cast<uint32_t>(nowMs - updatedAtMs) <= maximumAgeMs;
}

float Controller::absoluteValue(float value) {
  return value < 0.0F ? -value : value;
}

int16_t Controller::clampSteering(float value, int16_t limit) {
  if (value > static_cast<float>(limit)) return limit;
  if (value < -static_cast<float>(limit)) return -limit;
  return static_cast<int16_t>(lroundf(value));
}

uint16_t Controller::clampSpeed(uint16_t value) {
  return value > 1000U ? 1000U : value;
}

float Controller::headingRemainderDegrees(float unwrappedDegrees) {
  float result = fmodf(unwrappedDegrees, 360.0F);
  if (result <= -180.0F) result += 360.0F;
  if (result > 180.0F) result -= 360.0F;
  return result;
}

bool Controller::updateButton(const Inputs& inputs) {
  if (inputs.buttonPressed != rawButtonPressed_) {
    rawButtonPressed_ = inputs.buttonPressed;
    rawButtonChangedAtMs_ = inputs.nowMs;
  }

  // A released button must be observed before a press can start the car. This
  // prevents a held or shorted button from starting at power-up.
  if (!rawButtonPressed_ && !stableButtonPressed_ &&
      elapsedAtLeast(inputs.nowMs, resetAtMs_, config_.buttonDebounceMs)) {
    buttonArmed_ = true;
  }

  bool startEdge = false;
  if (rawButtonPressed_ != stableButtonPressed_ &&
      elapsedAtLeast(inputs.nowMs, rawButtonChangedAtMs_,
                     config_.buttonDebounceMs)) {
    stableButtonPressed_ = rawButtonPressed_;
    if (!stableButtonPressed_) {
      buttonArmed_ = true;
    } else if (buttonArmed_ &&
               decision_.state == RunState::WaitingForStart) {
      startEdge = true;
      buttonArmed_ = false;
    }
  }
  return startEdge;
}

Controller::WallGuidance Controller::computeWallGuidance(
    const Inputs& inputs) const {
  WallGuidance result;
  bool usable[SideSensorCount] = {};
  for (uint8_t i = 0; i < SideSensorCount; ++i) {
    const RangeReading& sample = inputs.sides[i];
    usable[i] = sample.online && sample.i2cStatus == 0U && sample.valid &&
                readingFresh(inputs.nowMs, sample.updatedAtMs,
                             config_.sensorStaleMs) &&
                sample.rangeMm <= config_.openingReferenceMaximumMm;
  }

  // A valid multi-metre return can help identify which side opened, but it is
  // not a local wall-distance measurement. Opening evidence reads the raw
  // samples separately; wall guidance deliberately excludes those far rays so
  // a legal start cannot command full lock toward a 5 m "wall."

  const bool haveLeftAverage = usable[FrontLeft] && usable[BackLeft];
  const bool haveRightAverage = usable[FrontRight] && usable[BackRight];
  result.completePair = haveLeftAverage || haveRightAverage;
  result.bothPairs = haveLeftAverage && haveRightAverage;
  const bool haveAnyLeft = usable[FrontLeft] || usable[BackLeft];
  const bool haveAnyRight = usable[FrontRight] || usable[BackRight];

  if (!config_.sideSensorPairsAreParallel) {
    if (usable[FrontLeft] && usable[FrontRight]) {
      result.lateralOffsetMm =
          (static_cast<float>(inputs.sides[FrontLeft].rangeMm) -
           static_cast<float>(inputs.sides[FrontRight].rangeMm)) *
          0.5F;
      result.usable = true;
    } else if (result.completePair) {
      // A complete angled pair proves that geometry still exists but cannot
      // tell us a corridor-centre offset by itself.
      result.lateralOffsetMm = 0.0F;
      result.usable = true;
    }
    result.angleErrorMm = 0.0F;
    return result;
  }

  float leftAverage = 0.0F;
  float rightAverage = 0.0F;
  if (haveLeftAverage) {
    leftAverage = average(inputs.sides[FrontLeft].rangeMm,
                          inputs.sides[BackLeft].rangeMm);
  } else if (usable[FrontLeft]) {
    leftAverage = inputs.sides[FrontLeft].rangeMm;
  } else if (usable[BackLeft]) {
    leftAverage = inputs.sides[BackLeft].rangeMm;
  }

  if (haveRightAverage) {
    rightAverage = average(inputs.sides[FrontRight].rangeMm,
                           inputs.sides[BackRight].rangeMm);
  } else if (usable[FrontRight]) {
    rightAverage = inputs.sides[FrontRight].rangeMm;
  } else if (usable[BackRight]) {
    rightAverage = inputs.sides[BackRight].rangeMm;
  }

  // Open Challenge follows only the outside boundary once the course
  // direction is known: left wall clockwise, right wall counter-clockwise.
  // This is independent of corridor width, so a wide track can never pull the
  // car toward the inner square. One outer ray gives distance-only guidance;
  // a complete outer pair additionally gives a wall-angle term.
  if (!config_.obstacleAvoidanceEnabled &&
      decision_.direction != CourseDirection::Unknown) {
    const bool clockwise =
        decision_.direction == CourseDirection::Clockwise;
    const bool outerUsable = clockwise ? haveAnyLeft : haveAnyRight;
    const bool outerPair = clockwise ? haveLeftAverage : haveRightAverage;
    const bool innerUsable = clockwise ? haveAnyRight : haveAnyLeft;
    const float outerDistance = clockwise ? leftAverage : rightAverage;
    const float innerDistance = clockwise ? rightAverage : leftAverage;

    result.outerWallUsable = outerUsable;
    result.outerWallPair = outerPair;
    if (outerUsable) {
      result.outerWallDistanceMm = outerDistance;
      float distanceError =
          outerDistance - static_cast<float>(config_.openOuterWallTargetMm);
      if (absoluteValue(distanceError) <=
          static_cast<float>(config_.openOuterWallDeadbandMm)) {
        distanceError = 0.0F;
      }
      // Positive lateral offset is converted to left steering by the existing
      // wall controller. Mirror it for the right outside wall.
      result.lateralOffsetMm = clockwise ? distanceError : -distanceError;
      result.usable = true;
    }

    // The inner wall is never a nominal follow target. It is only an
    // independent clearance guard which commands back toward the outside.
    if (innerUsable &&
        innerDistance < static_cast<float>(config_.openInnerWallMinimumMm)) {
      const float guardError =
          static_cast<float>(config_.openInnerWallMinimumMm) - innerDistance;
      result.lateralOffsetMm += clockwise ? guardError : -guardError;
      result.innerGuardActive = true;
      result.usable = true;
    }

    if (outerPair) {
      result.angleErrorMm =
          clockwise
              ? static_cast<float>(inputs.sides[FrontLeft].rangeMm) -
                    inputs.sides[BackLeft].rangeMm
              : static_cast<float>(inputs.sides[BackRight].rangeMm) -
                    inputs.sides[FrontRight].rangeMm;
    }
    return result;
  }

  // Positive offset means that the vehicle is right of the corridor centre.
  if (haveAnyLeft && haveAnyRight) {
    result.lateralOffsetMm = (leftAverage - rightAverage) * 0.5F;
    result.usable = true;
  } else if (haveLeftAverage) {
    result.lateralOffsetMm =
        leftAverage - static_cast<float>(config_.nominalSideDistanceMm);
    result.usable = true;
  } else if (haveRightAverage) {
    result.lateralOffsetMm =
        static_cast<float>(config_.nominalSideDistanceMm) - rightAverage;
    result.usable = true;
  }

  uint8_t angleTermCount = 0;
  if (haveLeftAverage) {
    // Positive means the front-left is farther from its wall than the
    // back-left: the nose points right and needs a left correction.
    result.angleErrorMm += static_cast<float>(inputs.sides[FrontLeft].rangeMm) -
                           inputs.sides[BackLeft].rangeMm;
    ++angleTermCount;
  }
  if (haveRightAverage) {
    // The equivalent sign on the right side is back-right - front-right.
    result.angleErrorMm += static_cast<float>(inputs.sides[BackRight].rangeMm) -
                           inputs.sides[FrontRight].rangeMm;
    ++angleTermCount;
  }
  if (angleTermCount > 1) result.angleErrorMm *= 0.5F;
  return result;
}

Controller::OpeningEvidence Controller::computeOpeningEvidence(
    const Inputs& inputs) const {
  OpeningEvidence result;
  const RangeReading& frontLeft = inputs.sides[FrontLeft];
  const RangeReading& backLeft = inputs.sides[BackLeft];
  const RangeReading& frontRight = inputs.sides[FrontRight];
  const RangeReading& backRight = inputs.sides[BackRight];

  // Detect openings from each forward ray versus its preceding-straight
  // baseline. This avoids confusing ordinary front/rear wall-angle residual
  // with an opening. A fresh strong no-target (2/4) is evidence; SigmaFail (1)
  // additionally needs a valid, closed opposite wall. Electrical loss or
  // stale data is never interpreted as open space.
  const bool leftNumeric = sideReadingUsable(frontLeft, inputs.nowMs);
  const bool rightNumeric = sideReadingUsable(frontRight, inputs.nowMs);
  const bool leftClosedCorroboration =
      leftOpeningBaselineValid_ && leftNumeric &&
      static_cast<float>(frontLeft.rangeMm) - leftOpeningBaselineMm_ <
          config_.cornerOpeningDeltaMm;
  const bool rightClosedCorroboration =
      rightOpeningBaselineValid_ && rightNumeric &&
      static_cast<float>(frontRight.rangeMm) - rightOpeningBaselineMm_ <
          config_.cornerOpeningDeltaMm;
  const bool leftOpen =
      sideReadingIsOpen(frontLeft, inputs.nowMs) ||
      (sideReadingIsSigmaFail(frontLeft, inputs.nowMs) &&
       rightClosedCorroboration);
  const bool rightOpen =
      sideReadingIsOpen(frontRight, inputs.nowMs) ||
      (sideReadingIsSigmaFail(frontRight, inputs.nowMs) &&
       leftClosedCorroboration);

  // The legacy parallel-pair profile can also use a complete side snapshot or
  // numeric cross-side contrast. The physical Open chassis disables this path:
  // its front pair is narrower than its rear pair and all four rays point
  // outward at about 45 degrees, so only same-forward-ray temporal evidence is
  // allowed to choose Open direction.
  const bool leftOpenFromCompleteSnapshot =
      config_.sideSensorPairsAreParallel && !leftOpeningBaselineValid_ &&
      sideReadingIsOpen(frontLeft, inputs.nowMs) &&
      sideReadingIsOpen(backLeft, inputs.nowMs) &&
      sideReadingUsable(frontRight, inputs.nowMs) &&
      sideReadingUsable(backRight, inputs.nowMs);
  const bool rightOpenFromCompleteSnapshot =
      config_.sideSensorPairsAreParallel && !rightOpeningBaselineValid_ &&
      sideReadingIsOpen(frontRight, inputs.nowMs) &&
      sideReadingIsOpen(backRight, inputs.nowMs) &&
      sideReadingUsable(frontLeft, inputs.nowMs) &&
      sideReadingUsable(backLeft, inputs.nowMs);
  const bool openDirectionProbeForwardContrast =
      config_.sideSensorPairsAreParallel &&
      !config_.obstacleAvoidanceEnabled &&
      decision_.state == RunState::DirectionProbe &&
      frontReadingUsable(inputs.front, inputs.nowMs) &&
      inputs.front.rangeMm <= config_.directionProbeFrontMaximumMm;
  const bool leftOpenFromNumericContrast =
      (!leftOpeningBaselineValid_ || openDirectionProbeForwardContrast) &&
      config_.sideSensorPairsAreParallel && leftNumeric && rightNumeric &&
      (sideReadingUsable(backRight, inputs.nowMs) ||
       openDirectionProbeForwardContrast) &&
      static_cast<float>(frontLeft.rangeMm) >=
          static_cast<float>(frontRight.rangeMm) +
              config_.cornerOpeningDeltaMm +
              config_.directionOpeningWinnerMarginMm;
  const bool rightOpenFromNumericContrast =
      (!rightOpeningBaselineValid_ || openDirectionProbeForwardContrast) &&
      config_.sideSensorPairsAreParallel && rightNumeric && leftNumeric &&
      (sideReadingUsable(backLeft, inputs.nowMs) ||
       openDirectionProbeForwardContrast) &&
      static_cast<float>(frontRight.rangeMm) >=
          static_cast<float>(frontLeft.rangeMm) +
              config_.cornerOpeningDeltaMm +
              config_.directionOpeningWinnerMarginMm;
  result.leftUsable =
      (leftOpeningBaselineValid_ && (leftNumeric || leftOpen)) ||
      leftOpenFromCompleteSnapshot || leftOpenFromNumericContrast;
  result.rightUsable =
      (rightOpeningBaselineValid_ && (rightNumeric || rightOpen)) ||
      rightOpenFromCompleteSnapshot || rightOpenFromNumericContrast;
  if (result.leftUsable) {
    result.leftDeltaMm = (leftOpen || leftOpenFromNumericContrast)
                             ? config_.cornerOpeningDeltaMm +
                                   config_.directionOpeningWinnerMarginMm + 1.0F
                             : static_cast<float>(frontLeft.rangeMm) -
                                   static_cast<float>(leftOpeningBaselineMm_);
    result.leftReferenceMm =
        leftOpeningBaselineValid_ ? leftOpeningBaselineMm_ : 0U;
  }
  if (result.rightUsable) {
    result.rightDeltaMm = (rightOpen || rightOpenFromNumericContrast)
                              ? config_.cornerOpeningDeltaMm +
                                    config_.directionOpeningWinnerMarginMm + 1.0F
                              : static_cast<float>(frontRight.rangeMm) -
                                    static_cast<float>(rightOpeningBaselineMm_);
    result.rightReferenceMm =
        rightOpeningBaselineValid_ ? rightOpeningBaselineMm_ : 0U;
  }
  return result;
}

bool Controller::sideReadingUsable(const RangeReading& reading,
                                   uint32_t nowMs) const {
  return reading.online && reading.i2cStatus == 0U && reading.valid &&
         readingFresh(nowMs, reading.updatedAtMs, config_.sensorStaleMs);
}

bool Controller::frontReadingUsable(const RangeReading& reading,
                                    uint32_t nowMs) const {
  if (!reading.online || reading.i2cStatus != 0U ||
      !readingFresh(nowMs, reading.updatedAtMs, config_.sensorStaleMs)) {
    return false;
  }
  if (reading.valid) return true;
  // The assembled front VL53L1X can return WrapTargetFail (7) with a stable,
  // useful numeric range while looking diagonally across the white mat. Open
  // accepts that measured behavior through its profile flag. Obstacle keeps
  // rejecting it everywhere except the final, strictly proven 500 ms tail of
  // a centred single-pillar calibration: by then the camera close/loss
  // sequence, selected side transition, stable BNO escape heading, and side
  // clearance guards have all succeeded. Keeping this exception here makes
  // the front fault timer, cross-geometry gate, body-clear timer, and speed
  // envelope agree about the same sample. A zero range is never accepted, and
  // the ordinary <=180 mm emergency stop still evaluates any accepted range.
  const bool provenCenteredSinglePassBodyClear =
      guardedPassEnabled() &&
      obstacleCenteredEscapeActive_ && obstacleCenteredEscapeTurnComplete_ &&
      !obstacleCenteredEscapeClearanceViolated_ &&
      (obstacleBodyClearActive_ || centeredEscapePostTurnActive());
  return (config_.acceptFrontWrapTargetRange ||
          provenCenteredSinglePassBodyClear) &&
         reading.rangeStatus == 7U && reading.rangeMm > 0U;
}

CourseDirection Controller::forwardSpaceDirectionCandidate(
    const Inputs& inputs) const {
  const RangeReading& frontLeft = inputs.sides[FrontLeft];
  const RangeReading& frontRight = inputs.sides[FrontRight];
  const bool leftNumeric =
      sideReadingUsable(frontLeft, inputs.nowMs) &&
      frontLeft.rangeMm <= config_.openingReferenceMaximumMm;
  const bool rightNumeric =
      sideReadingUsable(frontRight, inputs.nowMs) &&
      frontRight.rangeMm <= config_.openingReferenceMaximumMm;
  const bool leftOpen = sideReadingIsOpen(frontLeft, inputs.nowMs);
  const bool rightOpen = sideReadingIsOpen(frontRight, inputs.nowMs);

  // FL and FR are the only symmetric pair on the assembled chassis. BL/BR
  // sit farther apart and all four rays point diagonally, so never compare a
  // front ray with a rear ray here. A strong 2/4 no-target is farther/open;
  // weak invalid statuses remain unknown.
  if (leftOpen && rightNumeric && !rightOpen) {
    return CourseDirection::CounterClockwise;
  }
  if (rightOpen && leftNumeric && !leftOpen) {
    return CourseDirection::Clockwise;
  }
  if (!leftNumeric || !rightNumeric) return CourseDirection::Unknown;

  const float margin = config_.directionOpeningWinnerMarginMm;
  if (static_cast<float>(frontLeft.rangeMm) >=
      static_cast<float>(frontRight.rangeMm) + margin) {
    return CourseDirection::CounterClockwise;
  }
  if (static_cast<float>(frontRight.rangeMm) >=
      static_cast<float>(frontLeft.rangeMm) + margin) {
    return CourseDirection::Clockwise;
  }
  return CourseDirection::Unknown;
}

bool Controller::sideReadingIsOpen(const RangeReading& reading,
                                   uint32_t nowMs) const {
  return reading.online && reading.i2cStatus == 0U && !reading.valid &&
         strongOpticalNoTargetStatus(reading.rangeStatus) &&
         readingFresh(nowMs, reading.updatedAtMs, config_.sensorStaleMs);
}

bool Controller::sideReadingIsSigmaFail(const RangeReading& reading,
                                        uint32_t nowMs) const {
  return reading.online && reading.i2cStatus == 0U && !reading.valid &&
         reading.rangeStatus == 1U &&
         readingFresh(nowMs, reading.updatedAtMs, config_.sensorStaleMs);
}

bool Controller::straightSideGeometryUsable(
    const Inputs& inputs, const WallGuidance& wall) const {
  // Either one same-side A/B or C/D pair, or the two forward A/C rays, is
  // sufficient optical geometry. Every module must still remain electrically
  // online and fresh; evaluateSafety() enforces that separately.
  return wall.completePair ||
         (sideReadingUsable(inputs.sides[FrontLeft], inputs.nowMs) &&
          sideReadingUsable(inputs.sides[FrontRight], inputs.nowMs));
}

bool Controller::anyNumericSideReadingUsable(const Inputs& inputs) const {
  for (uint8_t i = 0; i < SideSensorCount; ++i) {
    if (sideReadingUsable(inputs.sides[i], inputs.nowMs)) return true;
  }
  return false;
}

bool Controller::anyLocalSideReadingUsable(const Inputs& inputs) const {
  for (uint8_t i = 0; i < SideSensorCount; ++i) {
    if (sideReadingUsable(inputs.sides[i], inputs.nowMs) &&
        inputs.sides[i].rangeMm <= config_.openingReferenceMaximumMm) {
      return true;
    }
  }
  return false;
}

void Controller::captureOpeningBaselines(const Inputs& inputs,
                                         bool resetFirst) {
  if (resetFirst) {
    leftOpeningBaselineValid_ = false;
    rightOpeningBaselineValid_ = false;
  }
  if (!leftOpeningBaselineValid_ &&
      sideReadingUsable(inputs.sides[FrontLeft], inputs.nowMs) &&
      inputs.sides[FrontLeft].rangeMm <=
          config_.openingReferenceMaximumMm) {
    leftOpeningBaselineMm_ = inputs.sides[FrontLeft].rangeMm;
    leftOpeningBaselineValid_ = true;
  }
  if (!rightOpeningBaselineValid_ &&
      sideReadingUsable(inputs.sides[FrontRight], inputs.nowMs) &&
      inputs.sides[FrontRight].rangeMm <=
          config_.openingReferenceMaximumMm) {
    rightOpeningBaselineMm_ = inputs.sides[FrontRight].rangeMm;
    rightOpeningBaselineValid_ = true;
  }
}

uint32_t Controller::evaluateSafety(const Inputs& inputs,
                                    WallGuidance& wall) {
  uint32_t faults = FaultNone;

  if (!configurationValid_) faults |= FaultConfiguration;

  if (!inputs.imu.online || !inputs.imu.valid ||
      !isfinite(inputs.imu.yawDegrees)) {
    faults |= FaultImuUnavailable;
  } else if (!readingFresh(inputs.nowMs, inputs.imu.updatedAtMs,
                           config_.sensorStaleMs)) {
    faults |= FaultImuStale;
  }
  const bool movingState =
      decision_.state == RunState::DirectionProbe ||
      decision_.state == RunState::Running ||
      decision_.state == RunState::Cornering ||
      decision_.state == RunState::FinishDelay;
  if (movingState && inputs.imu.accuracy < config_.minimumRunImuAccuracy) {
    if (imuAccuracyLostSinceMs_ == 0U) {
      imuAccuracyLostSinceMs_ = inputs.nowMs;
    } else if (elapsedAtLeast(inputs.nowMs, imuAccuracyLostSinceMs_,
                              config_.imuAccuracyGraceMs)) {
      faults |= FaultImuUncalibrated;
    }
  } else {
    imuAccuracyLostSinceMs_ = 0;
  }

  // Compute side geometry before classifying a far/no-target front sample.
  // Extending the startup probe is safe only while independent side guidance
  // remains available.
  wall = computeWallGuidance(inputs);
  frontOpticallyDegraded_ = false;
  bool freshFrontOpticalNoTarget = false;
  if (!inputs.front.online || inputs.front.i2cStatus != 0U) {
    faults |= FaultFrontTofUnavailable;
  } else if (!readingFresh(inputs.nowMs, inputs.front.updatedAtMs,
                           config_.sensorStaleMs)) {
    faults |= FaultFrontTofStale;
  } else if (!frontReadingUsable(inputs.front, inputs.nowMs)) {
    // Range status 3 means the target is below the sensor's minimum range. It
    // is a close hazard, not open space. Other fresh optical failures mean
    // unknown/far: keep driving below the degraded-speed cap and rely on the
    // paired side ToFs, IMU, and vision until a valid front return comes back.
    if (inputs.front.rangeStatus == 3U) {
      faults |= FaultFrontTooClose;
    } else if (opticalNoTargetStatus(inputs.front.rangeStatus)) {
      frontOpticallyDegraded_ = true;
      freshFrontOpticalNoTarget = true;
    } else {
      faults |= FaultFrontTofUnavailable;
    }
  } else if (inputs.front.rangeMm <= config_.frontEmergencyStopMm) {
    faults |= FaultFrontTooClose;
  }

  const bool strongFreshFrontNoTarget =
      freshFrontOpticalNoTarget &&
      sideReadingIsOpen(inputs.front, inputs.nowMs);
  const bool healthyImuGuidance =
      inputs.imu.online && inputs.imu.valid &&
      isfinite(inputs.imu.yawDegrees) &&
      readingFresh(inputs.nowMs, inputs.imu.updatedAtMs,
                   config_.sensorStaleMs) &&
      inputs.imu.accuracy >= config_.minimumRunImuAccuracy;
  const bool guidedFarDirectionProbe =
      strongFreshFrontNoTarget &&
      decision_.state == RunState::DirectionProbe &&
      (config_.obstacleAvoidanceEnabled
           ? straightSideGeometryUsable(inputs, wall)
           : healthyImuGuidance && anyNumericSideReadingUsable(inputs));
  const bool guidedOpenCorner =
      strongFreshFrontNoTarget && !config_.obstacleAvoidanceEnabled &&
      decision_.state == RunState::Cornering && healthyImuGuidance &&
      (wall.outerWallUsable ||
       (!config_.sideSensorPairsAreParallel &&
        anyLocalSideReadingUsable(inputs)));
  if (!freshFrontOpticalNoTarget || !movingState ||
      guidedFarDirectionProbe || guidedOpenCorner) {
    // A legal start can be several metres from the first wall, outside the
    // front VL53L1X optical range. DirectionProbe has its own hard timeout and
    // an Open corner has its own five-second turn timeout. Both still require
    // a fresh, online, I2C-healthy front sensor on every update; the corner
    // exception additionally requires live IMU and either a calibrated outer
    // wall ray or, for the physical diagonal layout, at least one numeric side
    // ray. The five-second corner timeout still bounds this IMU-only case.
    // Do not let servo settling consume a short straight-driving grace before
    // the wheels have even moved.
    frontOpticalLostSinceMs_ = 0;
    frontOpticalLossActive_ = false;
  } else if (!frontOpticalLossActive_) {
    frontOpticalLostSinceMs_ = inputs.nowMs;
    frontOpticalLossActive_ = true;
  } else {
    const uint32_t graceMs = decision_.state == RunState::Cornering
                                 ? config_.cornerFrontOpticalGraceMs
                                 : config_.straightFrontOpticalGraceMs;
    if (elapsedAtLeast(inputs.nowMs, frontOpticalLostSinceMs_, graceMs)) {
      faults |= FaultFrontTofUnavailable;
    }
  }

  bool anySideUnavailable = false;
  bool anySideInvalid = false;
  bool anySideStale = false;
  uint8_t validSideCount = 0;
  for (uint8_t i = 0; i < SideSensorCount; ++i) {
    const RangeReading& side = inputs.sides[i];
    if (!side.online || side.i2cStatus != 0U) {
      anySideUnavailable = true;
    } else if (!readingFresh(inputs.nowMs, side.updatedAtMs,
                             config_.sensorStaleMs)) {
      anySideStale = true;
    } else if (!side.valid) {
      if (opticalNoTargetStatus(side.rangeStatus)) {
        anySideInvalid = true;
      } else {
        anySideUnavailable = true;
      }
    } else {
      ++validSideCount;
    }
  }
  // A live ToF may report no optical target while its ray looks through a
  // corner or briefly above a 100 mm wall. A real I2C/staleness failure is
  // different: before start it blocks arming immediately; after start it holds
  // drive at zero for a short recovery window, then latches the exact fault.
  // This filters one missed poll without letting the car drive blind.
  const bool sideElectricalFault = anySideUnavailable || anySideStale;
  if (!sideElectricalFault) {
    sideElectricalLostSinceMs_ = 0;
    sideElectricalLossActive_ = false;
  } else if (!movingState || config_.sideElectricalLossGraceMs == 0U) {
    if (anySideUnavailable) faults |= FaultSideTofUnavailable;
    if (anySideStale) faults |= FaultSideTofStale;
    sideElectricalLostSinceMs_ = 0;
    sideElectricalLossActive_ = false;
  } else if (!sideElectricalLossActive_) {
    sideElectricalLostSinceMs_ = inputs.nowMs;
    sideElectricalLossActive_ = true;
  } else if (elapsedAtLeast(inputs.nowMs, sideElectricalLostSinceMs_,
                            config_.sideElectricalLossGraceMs)) {
    if (anySideUnavailable) faults |= FaultSideTofUnavailable;
    if (anySideStale) faults |= FaultSideTofStale;
  }
  if (config_.requireAllSideSensorsFresh &&
      decision_.state != RunState::Cornering && anySideInvalid) {
    faults |= FaultSideTofUnavailable;
  }
  const bool straightGeometryAvailable =
      straightSideGeometryUsable(inputs, wall);
  const auto localNumericSide = [&](uint8_t index) {
    return sideReadingUsable(inputs.sides[index], inputs.nowMs) &&
           inputs.sides[index].rangeMm <=
               config_.openingReferenceMaximumMm;
  };
  const uint16_t movingFrontGuardMm =
      (obstacleBodyClearActive_ || centeredEscapePostTurnActive()) &&
              config_.obstacleBodyClearFrontGuardMm > 0U
          ? config_.obstacleBodyClearFrontGuardMm
          : config_.directionProbeAbortFrontMm;
  const bool frontClearNow =
      frontReadingUsable(inputs.front, inputs.nowMs) &&
      inputs.front.rangeMm > movingFrontGuardMm;
  const bool localCrossPair =
      (localNumericSide(FrontLeft) && localNumericSide(BackRight)) ||
      (localNumericSide(BackLeft) && localNumericSide(FrontRight));
  const bool liveCloseObstacle =
      visionFreshAndValid(inputs) &&
      ((blobUsable(inputs.vision.red) &&
        inputs.vision.red.bottomY >= config_.obstacleCloseBottomY) ||
       (blobUsable(inputs.vision.green) &&
        inputs.vision.green.bottomY >= config_.obstacleCloseBottomY));
  const bool committedCloseObstacle =
      committedObstacle_ != ObstacleTarget::None &&
      obstaclePeakBottomY_ >= config_.obstacleCloseBottomY;
  const bool recentFrontClearStillEligible =
      inputs.startupFrontClearRecent && inputs.front.online &&
      inputs.front.i2cStatus == 0U && !inputs.front.valid &&
      inputs.front.rangeStatus == 7U &&
      readingFresh(inputs.nowMs, inputs.front.updatedAtMs,
                   config_.sensorStaleMs);
  // Independent diagonal rays are proximity observations, not a parallel-wall
  // model. Production can maintain IMU/camera guidance with one local ray,
  // while electrical freshness, front clearance and side guards still apply.
  const bool guardedHeadingGeometry =
      config_.guardedObstaclePassEnabled &&
      config_.obstacleAvoidanceEnabled &&
      !config_.sideSensorPairsAreParallel && !sideElectricalFault &&
      healthyImuGuidance && visionFreshAndValid(inputs) &&
      anyLocalSideReadingUsable(inputs) &&
      ((frontReadingUsable(inputs.front, inputs.nowMs) &&
        (inputs.front.rangeMm > config_.frontEmergencyStopMm ||
         config_.wallRecoveryEnabled)) ||
       strongFreshFrontNoTarget);
  calibrationCrossGuardActive_ =
      guardedHeadingGeometry ||
      (guardedPassEnabled() &&
      config_.obstacleAvoidanceEnabled &&
      !config_.sideSensorPairsAreParallel &&
      (decision_.state == RunState::DirectionProbe ||
       decision_.state == RunState::Running) &&
      !sideElectricalFault && frontClearNow && localCrossPair &&
      (committedCloseObstacle || liveCloseObstacle ||
       centeredEscapePostTurnActive()));
  const bool coarseTwoSidedStartupGeometry =
      !movingState && config_.obstacleAvoidanceEnabled &&
      !config_.sideSensorPairsAreParallel &&
      (frontClearNow || recentFrontClearStillEligible) &&
      (localNumericSide(FrontLeft) || localNumericSide(BackLeft)) &&
      (localNumericSide(FrontRight) || localNumericSide(BackRight));
  const bool startupGeometryAvailable =
      straightGeometryAvailable || coarseTwoSidedStartupGeometry ||
      guardedHeadingGeometry;
  sideGeometryDegraded_ = config_.requireAllSideSensorsFresh
                              ? (validSideCount < SideSensorCount ||
                                 !wall.bothPairs)
                              : !straightGeometryAvailable;
  if (!config_.requireAllSideSensorsFresh && !movingState &&
      config_.obstacleAvoidanceEnabled) {
    // Startup normally needs one real geometry solution: a complete A/B or
    // C/D side pair, or both forward A/C rays. The physical diagonal layout
    // may instead expose only BL+FR (or FL+BR) in a legal open-space pose. If
    // the centre-front ray is numerically clear beyond the direction abort
    // boundary, one local ray on each side is enough to arm while stopped.
    // It carries zero steering authority and does not count as sustained
    // moving geometry; the normal bounded loss grace starts after launch.
    // Every module still has to be electrically online and fresh above.
    if (!startupGeometryAvailable) {
      faults |= FaultSideGeometryUnavailable;
    }
    straightSideGeometryLostSinceMs_ = 0;
    cornerSideGeometryLostSinceMs_ = 0;
    straightSideGeometryLossActive_ = false;
    cornerSideGeometryLossActive_ = false;
  } else if (!config_.requireAllSideSensorsFresh &&
             !config_.obstacleAvoidanceEnabled) {
    // Open Challenge contains no pillars and its 100 mm walls legitimately
    // disappear from several oblique rays, including at a legal start pose.
    // Fresh optical no-target samples therefore degrade wall guidance but do
    // not block arming or end the run. Electrical freshness remains mandatory
    // through the independent check and bounded moving hold above.
    straightSideGeometryLostSinceMs_ = 0;
    cornerSideGeometryLostSinceMs_ = 0;
    straightSideGeometryLossActive_ = false;
    cornerSideGeometryLossActive_ = false;
  } else if (decision_.state == RunState::Cornering) {
    straightSideGeometryLostSinceMs_ = 0;
    straightSideGeometryLossActive_ = false;
    if (straightGeometryAvailable || guardedHeadingGeometry) {
      cornerSideGeometryLostSinceMs_ = 0;
      cornerSideGeometryLossActive_ = false;
    } else if (!cornerSideGeometryLossActive_) {
      cornerSideGeometryLostSinceMs_ = inputs.nowMs;
      cornerSideGeometryLossActive_ = true;
    } else if (elapsedAtLeast(inputs.nowMs,
                              cornerSideGeometryLostSinceMs_,
                              config_.cornerSideGeometryGraceMs)) {
      faults |= FaultSideGeometryUnavailable;
    }
  } else if (!config_.requireAllSideSensorsFresh) {
    cornerSideGeometryLostSinceMs_ = 0;
    cornerSideGeometryLossActive_ = false;
    const bool geometryLost =
        !(straightGeometryAvailable || calibrationCrossGuardActive_);
    if (!geometryLost) {
      straightSideGeometryLostSinceMs_ = 0;
      straightSideGeometryLossActive_ = false;
    } else if (!straightSideGeometryLossActive_) {
      straightSideGeometryLostSinceMs_ = inputs.nowMs;
      straightSideGeometryLossActive_ = true;
    } else if (elapsedAtLeast(inputs.nowMs,
                              straightSideGeometryLostSinceMs_,
                              config_.straightSideGeometryGraceMs)) {
      faults |= FaultSideGeometryUnavailable;
    }
  } else {
    straightSideGeometryLostSinceMs_ = 0;
    cornerSideGeometryLostSinceMs_ = 0;
    straightSideGeometryLossActive_ = false;
    cornerSideGeometryLossActive_ = false;
    if (!wall.usable) faults |= FaultSideGeometryUnavailable;
  }

  if (config_.requireVisionForMotion) {
    uint32_t visionFault = FaultNone;
    if (!inputs.vision.online || !inputs.vision.valid ||
        inputs.vision.pipelineOverrun) {
      visionFault = FaultVisionUnavailable;
    } else if (!readingFresh(inputs.nowMs, inputs.vision.updatedAtMs,
                             config_.visionStaleMs)) {
      visionFault = FaultVisionStale;
    }

    if (visionFault == FaultNone) {
      visionLostSinceMs_ = 0;
      visionLossActive_ = false;
    } else if (!movingState || config_.visionLossGraceMs == 0U) {
      // The car may arm only with live vision. The grace interval applies
      // after motion starts, where the speed planner holds a required-vision
      // vehicle stopped until the link either recovers or times out.
      faults |= visionFault;
      visionLostSinceMs_ = 0;
      visionLossActive_ = false;
    } else if (!visionLossActive_) {
      visionLostSinceMs_ = inputs.nowMs;
      visionLossActive_ = true;
    } else if (elapsedAtLeast(inputs.nowMs, visionLostSinceMs_,
                              config_.visionLossGraceMs)) {
      faults |= visionFault;
    }
  } else {
    visionLostSinceMs_ = 0;
    visionLossActive_ = false;
  }

  return faults;
}

void Controller::startRun(const Inputs& inputs) {
  decision_.startLatched = true;
  decision_.state =
      config_.parkingEnabled ? RunState::Parking : RunState::DirectionProbe;
  decision_.parkingPhase = config_.parkingEnabled
                               ? ParkingPhase::ExitAcquire
                               : ParkingPhase::Disabled;
  decision_.direction = CourseDirection::Unknown;
  decision_.cornerCount = 0;
  decision_.lapCount = 0;
  decision_.learnedRedSections = 0;
  decision_.learnedGreenSections = 0;
  decision_.obstacle = ObstacleTarget::None;
  decision_.configuredClockwiseYawSign = config_.clockwiseYawSign;
  decision_.cornerProgressDegrees = 0.0F;

  runStartedAtMs_ = inputs.nowMs;
  directionProbeTimerStartedAtMs_ = inputs.nowMs;
  lastCornerAtMs_ = inputs.nowMs;
  cornerRearmClearSinceMs_ = 0;
  cornerDetectionArmed_ = false;
  cornerStartedAtMs_ = 0;
  cornerReadySinceMs_ = 0;
  finishEnteredAtMs_ = 0;
  finishReadySinceMs_ = 0;
  straightSideGeometryLostSinceMs_ = 0;
  cornerSideGeometryLostSinceMs_ = 0;
  sideElectricalLostSinceMs_ = 0;
  frontOpticalLostSinceMs_ = 0;
  straightSideGeometryLossActive_ = false;
  cornerSideGeometryLossActive_ = false;
  sideElectricalLossActive_ = false;
  frontOpticalLossActive_ = false;
  frontOpticallyDegraded_ = false;
  sideGeometryDegraded_ = false;
  imuAccuracyLostSinceMs_ = 0;
  visionLostSinceMs_ = 0;
  visionLossActive_ = false;
  committedObstacle_ = ObstacleTarget::None;
  committedObstacleOffsetMm_ = 0;
  committedObstacleCenterX_ = 0;
  committedObstacleConfidence_ = 0;
  obstacleCommittedAtMs_ = 0;
  obstacleClearSinceMs_ = 0;
  obstacleBodyClearStartedAtMs_ = 0;
  obstacleBodyClearActive_ = false;
  obstacleCenteredEscapeActive_ = false;
  obstacleCenteredEscapeTurnComplete_ = false;
  obstacleCenteredEscapeDrivenMs_ = 0;
  obstacleCenteredEscapeDriveUpdatedAtMs_ = 0;
  obstacleCenteredEscapeRecovered_ = false;
  obstacleCenteredEscapeRecoveryCandidateActive_ = false;
  obstacleCenteredEscapeRecoveryCandidateSampleAtMs_ = 0;
  obstacleCenteredEscapeRecoveredVisionSampleAtMs_ = 0;
  obstacleCenteredEscapePostRecoveryObserved_ = false;
  obstacleSuccessorCandidateActive_ = false;
  obstacleSuccessorCandidateSampleAtMs_ = 0;
  obstacleSuccessorRollover_ = false;
  obstacleCenteredEscapeClearanceViolated_ = false;
  obstacleCenteredEscapeHeadingCandidateActive_ = false;
  obstacleCenteredEscapeHeadingCandidateSampleAtMs_ = 0;
  obstacleCenteredEscapeHeadingReferenceDegrees_ = 0.0F;
  obstacleAbsentSinceMs_ = 0;
  obstaclePeakBottomY_ = 0;
  obstaclePairDeltaInitialized_ = false;
  obstacleMinimumPairDeltaMm_ = 0.0F;
  lastClearedObstacle_ = ObstacleTarget::None;
  obstacleRecommitBlockedSinceMs_ = 0;
  recoverySkippedObstacle_ = ObstacleTarget::None;
  recoverySkippedObstacleAtMs_ = 0;
  obstacleRecenterStartedAtMs_ = 0;
  obstacleRecenterStartOffsetMm_ = 0;
  obstaclePassCornerRecoveryPending_ = false;
  lastCommandedSpeedPermille_ = 0;
  lastSpeedCommandAtMs_ = inputs.nowMs;

  headingInitialized_ = true;
  lastRawHeadingDegrees_ = inputs.imu.yawDegrees;
  unwrappedHeadingDegrees_ = 0.0F;
  runStartHeadingDegrees_ = 0.0F;
  cornerStartHeadingDegrees_ = 0.0F;
  cornerTargetHeadingDegrees_ = 0.0F;
  straightHeadingReferenceDegrees_ = 0.0F;
  openingCandidate_ = CourseDirection::Unknown;
  openingCandidateSinceMs_ = 0;
  directionDecisionHoldSinceMs_ = 0;
  directionDecisionHoldActive_ = false;
  leftOpeningBaselineValid_ = false;
  rightOpeningBaselineValid_ = false;
  const WallGuidance startWall = computeWallGuidance(inputs);
  const bool startFrontAllowsBaseline =
      frontReadingUsable(inputs.front, inputs.nowMs) ||
      (sideReadingIsOpen(inputs.front, inputs.nowMs) && startWall.bothPairs);
  if (startFrontAllowsBaseline) {
    captureOpeningBaselines(inputs, false);
  }
}

bool Controller::updateHeading(const Inputs& inputs) {
  if (!headingInitialized_) {
    headingInitialized_ = true;
    lastRawHeadingDegrees_ = inputs.imu.yawDegrees;
    unwrappedHeadingDegrees_ = 0.0F;
    return true;
  }

  const float delta = shortestAngleDeltaDegrees(lastRawHeadingDegrees_,
                                                 inputs.imu.yawDegrees);
  lastRawHeadingDegrees_ = inputs.imu.yawDegrees;
  if (!isfinite(delta) || absoluteValue(delta) > config_.maxHeadingStepDegrees) {
    return false;
  }
  unwrappedHeadingDegrees_ += delta;
  return true;
}

float Controller::courseProgressDegrees() const {
  if (decision_.direction == CourseDirection::Unknown) {
    return 0.0F;
  }
  const float directionYawSign =
      decision_.direction == CourseDirection::Clockwise
          ? static_cast<float>(config_.clockwiseYawSign)
          : -static_cast<float>(config_.clockwiseYawSign);
  return (unwrappedHeadingDegrees_ - runStartHeadingDegrees_) *
         directionYawSign;
}

bool Controller::expectedOpeningPresent(
    const OpeningEvidence& opening) const {
  if (decision_.direction == CourseDirection::CounterClockwise) {
    return opening.leftUsable &&
           opening.leftReferenceMm <= config_.openingReferenceMaximumMm &&
           opening.leftDeltaMm >= config_.cornerOpeningDeltaMm;
  }
  if (decision_.direction == CourseDirection::Clockwise) {
    return opening.rightUsable &&
           opening.rightReferenceMm <= config_.openingReferenceMaximumMm &&
           opening.rightDeltaMm >= config_.cornerOpeningDeltaMm;
  }
  return false;
}

bool Controller::openingConfirmed(CourseDirection candidate,
                                  const Inputs& inputs) {
  if (candidate == CourseDirection::Unknown) {
    openingCandidate_ = CourseDirection::Unknown;
    openingCandidateSinceMs_ = 0;
    openingCandidateLastLeftSampleAtMs_ = 0;
    openingCandidateLastRightSampleAtMs_ = 0;
    openingCandidateLastFrontSampleAtMs_ = 0;
    openingCandidateLastEvidenceAtMs_ = 0;
    openingCandidateDistinctSamples_ = 0;
    return false;
  }
  if (candidate != openingCandidate_) {
    openingCandidate_ = candidate;
    openingCandidateSinceMs_ = inputs.nowMs;
    openingCandidateLastLeftSampleAtMs_ =
        inputs.sides[FrontLeft].updatedAtMs;
    openingCandidateLastRightSampleAtMs_ =
        inputs.sides[FrontRight].updatedAtMs;
    openingCandidateLastFrontSampleAtMs_ = inputs.front.updatedAtMs;
    openingCandidateLastEvidenceAtMs_ = inputs.nowMs;
    openingCandidateDistinctSamples_ = 1U;
  } else if (inputs.sides[FrontLeft].updatedAtMs !=
                 openingCandidateLastLeftSampleAtMs_ &&
             inputs.sides[FrontRight].updatedAtMs !=
                 openingCandidateLastRightSampleAtMs_ &&
             inputs.front.updatedAtMs !=
                 openingCandidateLastFrontSampleAtMs_) {
    openingCandidateLastLeftSampleAtMs_ =
        inputs.sides[FrontLeft].updatedAtMs;
    openingCandidateLastRightSampleAtMs_ =
        inputs.sides[FrontRight].updatedAtMs;
    openingCandidateLastFrontSampleAtMs_ = inputs.front.updatedAtMs;
    openingCandidateLastEvidenceAtMs_ = inputs.nowMs;
    if (openingCandidateDistinctSamples_ < 255U) {
      ++openingCandidateDistinctSamples_;
    }
  }
  if (config_.openingConfirmationMs == 0U) return true;
  const uint8_t requiredDistinctSamples =
      decision_.state == RunState::DirectionProbe
          ? config_.directionMinimumDistinctSamples
          : config_.openingMinimumDistinctSamples;
  return openingCandidateDistinctSamples_ >= requiredDistinctSamples &&
         elapsedAtLeast(openingCandidateLastEvidenceAtMs_,
                        openingCandidateSinceMs_,
                        config_.openingConfirmationMs);
}

void Controller::enterCorner(const Inputs& inputs) {
  decision_.state = RunState::Cornering;
  decision_.cornerProgressDegrees = 0.0F;
  decision_.cornerDetectionArmed = false;
  cornerDetectionArmed_ = false;
  cornerRearmClearSinceMs_ = 0;
  obstaclePassCornerRecoveryPending_ = false;
  cornerStartHeadingDegrees_ = unwrappedHeadingDegrees_;
  const float expectedYawSign =
      decision_.direction == CourseDirection::Clockwise
          ? static_cast<float>(config_.clockwiseYawSign)
          : -static_cast<float>(config_.clockwiseYawSign);
  // Every turn targets an absolute cardinal from the run-start heading.
  // Basing later targets on imperfect corner-entry/exit headings compounds a
  // small under-turn until the car points into a wall.
  cornerTargetHeadingDegrees_ =
      runStartHeadingDegrees_ + expectedYawSign * kRightAngleDegrees *
                                    static_cast<float>(decision_.cornerCount + 1U);
  decision_.cornerTargetHeadingDegrees = cornerTargetHeadingDegrees_;
  cornerStartedAtMs_ = inputs.nowMs;
  cornerReadySinceMs_ = 0;
  straightSideGeometryLostSinceMs_ = 0;
  cornerSideGeometryLostSinceMs_ = 0;
  straightSideGeometryLossActive_ = false;
  cornerSideGeometryLossActive_ = false;
  // A front ray that disappeared on the corner approach receives a fresh,
  // still-bounded corner interval while the chassis rotates.
  frontOpticalLostSinceMs_ = 0;
  frontOpticalLossActive_ = false;
  openingCandidate_ = CourseDirection::Unknown;
  openingCandidateSinceMs_ = 0;
  openingCandidateLastLeftSampleAtMs_ = 0;
  openingCandidateLastRightSampleAtMs_ = 0;
  openingCandidateLastFrontSampleAtMs_ = 0;
  openingCandidateLastEvidenceAtMs_ = 0;
  openingCandidateDistinctSamples_ = 0;
  directionDecisionHoldSinceMs_ = 0;
  directionDecisionHoldActive_ = false;
}

void Controller::updateCorner(const Inputs& inputs,
                              const WallGuidance& wall,
                              const OpeningEvidence& opening) {
  (void)opening;
  const float rawCornerDelta =
      unwrappedHeadingDegrees_ - cornerStartHeadingDegrees_;

  const float expectedYawSign =
      decision_.direction == CourseDirection::Clockwise
          ? static_cast<float>(config_.clockwiseYawSign)
          : -static_cast<float>(config_.clockwiseYawSign);
  const float physicalCornerProgress = rawCornerDelta * expectedYawSign;
  if (physicalCornerProgress < -config_.maxReverseProgressDegrees) {
    latchFault(FaultWrongWay);
    return;
  }
  const float targetAnchoredProgress =
      kRightAngleDegrees -
      (cornerTargetHeadingDegrees_ - unwrappedHeadingDegrees_) *
          expectedYawSign;
  const float cornerProgress = targetAnchoredProgress;
  decision_.cornerProgressDegrees = cornerProgress;
  decision_.directedProgressDegrees = courseProgressDegrees();

  if (physicalCornerProgress > config_.cornerMaximumDegrees) {
    latchFault(FaultCornerOvershoot);
    return;
  }
  if (elapsedAtLeast(inputs.nowMs, cornerStartedAtMs_,
                     config_.cornerTimeoutMs)) {
    latchFault(FaultCornerTimeout);
    return;
  }

  const bool twoPairExit =
      config_.sideSensorPairsAreParallel && wall.bothPairs &&
      absoluteValue(wall.angleErrorMm) <=
          config_.cornerExitMaximumWallAngleErrorMm &&
      cornerProgress >= config_.cornerCompletionDegrees;
  // If a rear ray sees beyond the low wall, require the IMU to be nearer a
  // complete 90-degree turn before accepting one-pair/forward-pair geometry.
  const bool onePairExit =
      straightSideGeometryUsable(inputs, wall) &&
      cornerProgress >= config_.cornerCompletionDegrees + 10.0F;
  const bool frontClearForExit =
      (frontReadingUsable(inputs.front, inputs.nowMs) &&
       inputs.front.rangeMm > config_.frontCornerTriggerMm) ||
      sideReadingIsOpen(inputs.front, inputs.nowMs);
  const bool sensorVerifiedExit =
      (twoPairExit || onePairExit) && frontClearForExit;
  // The physical Open run proved that requiring wall reacquisition can leave
  // the steering at full lock until FaultCornerTimeout even after a real turn.
  // The same nonparallel chassis is used for Obstacle, and official pillars
  // exist only on straights. A stable near-90-degree IMU turn is therefore an
  // independent corner-exit proof in either profile. The emergency front
  // threshold and electrical hold still apply.
  const bool diagonalHeadingInExitWindow =
      cornerProgress >= config_.openCornerExitFallbackDegrees &&
      cornerProgress <= config_.openCornerExitMaximumDegrees;
  const bool diagonalHeadingExit =
      !config_.sideSensorPairsAreParallel && !sideElectricalLossActive_ &&
      diagonalHeadingInExitWindow;
  const bool diagonalHeadingGate =
      config_.sideSensorPairsAreParallel || diagonalHeadingInExitWindow;
  const bool clearStraightGeometry =
      elapsedAtLeast(inputs.nowMs, cornerStartedAtMs_,
                     config_.cornerMinimumDurationMs) &&
      diagonalHeadingGate && (sensorVerifiedExit || diagonalHeadingExit);
  if (!clearStraightGeometry) {
    cornerReadySinceMs_ = 0;
    return;
  }

  if (cornerReadySinceMs_ == 0) cornerReadySinceMs_ = inputs.nowMs;
  if (!elapsedAtLeast(inputs.nowMs, cornerReadySinceMs_,
                      config_.cornerExitStableMs)) {
    return;
  }

  if (decision_.cornerCount < kRequiredCorners) ++decision_.cornerCount;
  // Recovery is bounded per completed section, as well as by the run clock.
  wallRecoveryAttempts_ = 0;
  decision_.lapCount = decision_.cornerCount / kCornersPerLap;
  lastCornerAtMs_ = inputs.nowMs;
  // Never bless exit error as the next straight reference. Both profiles keep
  // returning to the exact run-anchored cardinal.
  straightHeadingReferenceDegrees_ = cornerTargetHeadingDegrees_;
  cornerReadySinceMs_ = 0;
  decision_.cornerProgressDegrees = 0.0F;
  // Pillar seats exist in straight sections, never in corners. A target that
  // survived camera occlusion all the way through a verified corner is now
  // behind the car and must not bias the next straight.
  clearCommittedObstacle(inputs.nowMs, false);
  captureOpeningBaselines(inputs, true);
  // Returning to a straight starts the shorter straight-blindness interval;
  // a no-target that persists after this transition will latch a stop.
  frontOpticalLostSinceMs_ = 0;
  frontOpticalLossActive_ = false;

  if (decision_.cornerCount >= kRequiredCorners) {
    if (config_.parkingEnabled) {
      // Unreachable while the R20 implementation guard is false. Keeping the
      // branch explicit prevents a future parking build from silently using
      // the legacy finish coast instead of searching for the bay.
      decision_.state = RunState::Parking;
      decision_.parkingPhase = ParkingPhase::Search;
    } else {
      decision_.state = RunState::FinishDelay;
      finishEnteredAtMs_ = inputs.nowMs;
      finishReadySinceMs_ = 0;
    }
  } else {
    decision_.state = RunState::Running;
    // Zero timing is an explicit deterministic-test/calibration override.
    // Production profiles use a non-zero clear-front re-arm sequence.
    if (config_.cornerRefractoryMs == 0U &&
        config_.cornerRearmStableMs == 0U) {
      cornerDetectionArmed_ = true;
      decision_.cornerDetectionArmed = true;
    }
  }
}

void Controller::updateCornerDetectionRearm(
    const Inputs& inputs, const OpeningEvidence& opening) {
  decision_.cornerDetectionArmed = cornerDetectionArmed_;
  if (decision_.state != RunState::Running || cornerDetectionArmed_) return;
  if (config_.obstacleAvoidanceEnabled &&
      (committedObstacle_ != ObstacleTarget::None ||
       obstacleRecenterStartOffsetMm_ != 0)) {
    // A pillar pass can change both apparent side ranges. Do not freeze a
    // corner baseline or arm the next turn until avoidance and recentering are
    // complete on the straight.
    cornerRearmClearSinceMs_ = 0;
    return;
  }
  if (!elapsedAtLeast(inputs.nowMs, lastCornerAtMs_,
                      config_.cornerRefractoryMs)) {
    cornerRearmClearSinceMs_ = 0;
    return;
  }

  // A wide car must finish the avoidance S-curve and recover its cardinal
  // heading before a nearby end-seat pillar is allowed to expose the corner.
  // Otherwise one side-ray transition can start the 90-degree turn while the
  // chassis is still yawed across the lane.
  if (config_.obstacleAvoidanceEnabled &&
      decision_.direction != CourseDirection::Unknown &&
      absoluteValue(unwrappedHeadingDegrees_ -
                    straightHeadingReferenceDegrees_) >
          config_.obstacleCornerRearmHeadingDegrees) {
    cornerRearmClearSinceMs_ = 0;
    return;
  }

  // An official edge-seat pillar can finish only a short distance before the
  // next corner. In that legal geometry the front wall can already be below
  // the ordinary clear-front rearm threshold when avoidance has fully
  // recentered. Recover only from a new, direction-correct opening sample with
  // a fresh empty camera; openingConfirmed() still supplies the normal
  // multi-sample/time confirmation before steering begins.
  const bool postPassCornerEvidence =
      config_.obstacleAvoidanceEnabled &&
      obstaclePassCornerRecoveryPending_ &&
      visionFreshAndValid(inputs) && !blobUsable(inputs.vision.red) &&
      !blobUsable(inputs.vision.green) &&
      frontReadingUsable(inputs.front, inputs.nowMs) &&
      inputs.front.rangeMm <= config_.frontCornerTriggerMm &&
      expectedOpeningPresent(opening);
  if (postPassCornerEvidence) {
    cornerDetectionArmed_ = true;
    decision_.cornerDetectionArmed = true;
    cornerRearmClearSinceMs_ = 0;
    return;
  }

  const bool acceptedWrappedFrontRange =
      config_.acceptFrontWrapTargetRange && !inputs.front.valid &&
      inputs.front.rangeStatus == 7U && inputs.front.rangeMm > 0U &&
      frontReadingUsable(inputs.front, inputs.nowMs);
  const bool clearFront =
      (frontReadingUsable(inputs.front, inputs.nowMs) &&
       inputs.front.rangeMm >= config_.cornerRearmFrontClearMm) ||
      (sideReadingIsOpen(inputs.front, inputs.nowMs) &&
       !acceptedWrappedFrontRange);
  if (!clearFront) {
    cornerRearmClearSinceMs_ = 0;
    return;
  }
  if (cornerRearmClearSinceMs_ == 0U) {
    cornerRearmClearSinceMs_ = inputs.nowMs;
  }
  if (elapsedAtLeast(inputs.nowMs, cornerRearmClearSinceMs_,
                     config_.cornerRearmStableMs)) {
    cornerDetectionArmed_ = true;
    decision_.cornerDetectionArmed = true;
    cornerRearmClearSinceMs_ = 0;
    obstaclePassCornerRecoveryPending_ = false;
  }
}

void Controller::updateNavigation(const Inputs& inputs,
                                  const WallGuidance& wall,
                                  const OpeningEvidence& opening) {
  if (decision_.state == RunState::DirectionProbe) {
    // A starting-straight pillar is not a wall opening. Finish the committed
    // red/green pass first, then resume direction learning and its ambiguity
    // boundary with fresh evidence.
    if (committedObstacle_ != ObstacleTarget::None ||
        obstacleRecenterStartOffsetMm_ != 0) {
      // Give direction learning its full bounded window after a starting
      // pillar pass instead of counting that avoidance time as ambiguity.
      directionProbeTimerStartedAtMs_ = inputs.nowMs;
      openingConfirmed(CourseDirection::Unknown, inputs);
      directionDecisionHoldSinceMs_ = 0;
      directionDecisionHoldActive_ = false;
      return;
    }
    // An invalid/no-target front ray is unknown, not numeric evidence that the
    // wall is near. A strong live no-target can corroborate a real side-wall
    // opening, while directionTimeoutMs bounds the complete probe.
    const bool closeEnoughToFirstCorner =
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm <= config_.directionProbeFrontMaximumMm;
    const bool strongBlindFront =
        sideReadingIsOpen(inputs.front, inputs.nowMs);
    // Observe the opening while approaching: diagonal rays can see the
    // opposite wall again by the time the front reaches the turn window.
    // Keep only a short-lived, independently repeated optical observation.
    if (config_.guardedObstaclePassEnabled) {
      CourseDirection optical = CourseDirection::Unknown;
      const RangeReading& left = inputs.sides[FrontLeft];
      const RangeReading& right = inputs.sides[FrontRight];
      if (absoluteValue(unwrappedHeadingDegrees_ - runStartHeadingDegrees_) <=
              8.0F && visionFreshAndValid(inputs)) {
        const bool leftLocal = sideReadingUsable(left, inputs.nowMs) &&
            left.rangeMm <= config_.openingReferenceMaximumMm;
        const bool rightLocal = sideReadingUsable(right, inputs.nowMs) &&
            right.rangeMm <= config_.openingReferenceMaximumMm;
        if (sideReadingIsOpen(left, inputs.nowMs) && rightLocal) {
          optical = CourseDirection::CounterClockwise;
        } else if (sideReadingIsOpen(right, inputs.nowMs) && leftLocal) {
          optical = CourseDirection::Clockwise;
        }
      }
      if (optical == CourseDirection::Unknown) {
        opticalProbeCandidate_ = CourseDirection::Unknown;
        opticalProbeSamples_ = 0;
      } else if (optical != opticalProbeCandidate_) {
        opticalProbeCandidate_ = optical;
        opticalProbeSinceMs_ = inputs.nowMs;
        opticalProbeLastLeftAtMs_ = left.updatedAtMs;
        opticalProbeLastRightAtMs_ = right.updatedAtMs;
        opticalProbeSamples_ = 1;
      } else if (left.updatedAtMs != opticalProbeLastLeftAtMs_ &&
                 right.updatedAtMs != opticalProbeLastRightAtMs_) {
        opticalProbeLastLeftAtMs_ = left.updatedAtMs;
        opticalProbeLastRightAtMs_ = right.updatedAtMs;
        if (opticalProbeSamples_ < 255U) ++opticalProbeSamples_;
        if (opticalProbeSamples_ >= 3U &&
            elapsedAtLeast(inputs.nowMs, opticalProbeSinceMs_, 100U)) {
          opticalProbeDirection_ = optical;
          opticalProbeConfirmedAtMs_ = inputs.nowMs;
        }
      }
    }
    // On the physical chassis the current FL/FR forward-space view is the
    // direction signal. R16 used a same-ray rise instead; the mirrored field
    // run proved that the closed-side ray can rise while the opposite side is
    // actually open, producing the wrong turn. Challenge mode cannot change
    // that physical sensor geometry.
    const float directionOpeningThresholdMm =
        closeEnoughToFirstCorner ? config_.directionProbeOpeningDeltaMm
                                 : config_.cornerOpeningDeltaMm;
    const bool baselineRequired = !config_.sideSensorPairsAreParallel;
    const bool leftOpening =
        opening.leftUsable &&
        (!baselineRequired || opening.leftReferenceMm > 0U) &&
        opening.leftReferenceMm <= config_.openingReferenceMaximumMm &&
        opening.leftDeltaMm >= directionOpeningThresholdMm;
    const bool rightOpening =
        opening.rightUsable &&
        (!baselineRequired || opening.rightReferenceMm > 0U) &&
        opening.rightReferenceMm <= config_.openingReferenceMaximumMm &&
        opening.rightDeltaMm >= directionOpeningThresholdMm;
    const bool leftWins =
        leftOpening &&
        (!rightOpening ||
         opening.leftDeltaMm - opening.rightDeltaMm >=
             config_.directionOpeningWinnerMarginMm);
    const bool rightWins =
        rightOpening &&
        (!leftOpening ||
         opening.rightDeltaMm - opening.leftDeltaMm >=
             config_.directionOpeningWinnerMarginMm);
    CourseDirection candidate = CourseDirection::Unknown;
    if (!config_.sideSensorPairsAreParallel) {
      if (closeEnoughToFirstCorner) {
        candidate = forwardSpaceDirectionCandidate(inputs);
      }
    } else if (leftWins &&
               (closeEnoughToFirstCorner ||
                (strongBlindFront && opening.leftReferenceMm > 0U))) {
      candidate = CourseDirection::CounterClockwise;
    } else if (rightWins &&
               (closeEnoughToFirstCorner ||
                (strongBlindFront && opening.rightReferenceMm > 0U))) {
      candidate = CourseDirection::Clockwise;
    }
    if (config_.guardedObstaclePassEnabled && closeEnoughToFirstCorner &&
        opticalProbeDirection_ != CourseDirection::Unknown &&
        inputs.nowMs - opticalProbeConfirmedAtMs_ <= 3000U &&
        absoluteValue(unwrappedHeadingDegrees_ - runStartHeadingDegrees_) <=
            8.0F) {
      candidate = opticalProbeDirection_;
    }
    if (elapsedAtLeast(inputs.nowMs, directionProbeTimerStartedAtMs_,
                       config_.directionTimeoutMs)) {
      latchFault(FaultDirectionTimeout);
      return;
    }
    const bool directionConfirmed = openingConfirmed(candidate, inputs);
    const bool atDirectionAbortBoundary =
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm <= config_.directionProbeAbortFrontMm;
    const bool physicalForwardDirectionProbe =
        !config_.sideSensorPairsAreParallel;
    if (directionConfirmed &&
        (!physicalForwardDirectionProbe || !atDirectionAbortBoundary)) {
      directionDecisionHoldSinceMs_ = 0;
      directionDecisionHoldActive_ = false;
      decision_.direction = candidate;
      enterCorner(inputs);
    } else if (atDirectionAbortBoundary) {
      if (physicalForwardDirectionProbe) {
        // The physical chassis has a wide 1.0 m decision window. Reaching
        // 600 mm without a fully confirmed direction is too late to begin a
        // safe powered turn in either challenge profile.
        latchFault(FaultDirectionAmbiguous);
        return;
      }
      // Stop at the decision boundary while the opening confirmation settles.
      // A real candidate normally completes within openingConfirmationMs; a
      // tied, missing, or flickering candidate gets only this bounded hold.
      if (!directionDecisionHoldActive_) {
        directionDecisionHoldActive_ = true;
        directionDecisionHoldSinceMs_ = inputs.nowMs;
      } else if (elapsedAtLeast(inputs.nowMs,
                                directionDecisionHoldSinceMs_,
                                config_.directionDecisionHoldMs)) {
        latchFault(FaultDirectionAmbiguous);
      }
    } else {
      directionDecisionHoldSinceMs_ = 0;
      directionDecisionHoldActive_ = false;
    }
    return;
  }

  if (decision_.state == RunState::Running) {
    decision_.directedProgressDegrees = courseProgressDegrees();
    updateCornerDetectionRearm(inputs, opening);
    if (!cornerDetectionArmed_) {
      openingConfirmed(CourseDirection::Unknown, inputs);
      return;
    }
    const bool frontSuggestsCorner =
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm <= config_.frontCornerTriggerMm;
    const bool expectedBaselineOpening =
        decision_.direction == CourseDirection::CounterClockwise
            ? opening.leftUsable && opening.leftReferenceMm > 0U &&
                  opening.leftReferenceMm <=
                      config_.openingReferenceMaximumMm &&
                  opening.leftDeltaMm >= config_.cornerOpeningDeltaMm
            : decision_.direction == CourseDirection::Clockwise
                  ? opening.rightUsable && opening.rightReferenceMm > 0U &&
                        opening.rightReferenceMm <=
                            config_.openingReferenceMaximumMm &&
                        opening.rightDeltaMm >= config_.cornerOpeningDeltaMm
                  : false;
    const bool blindFrontSuggestsOpenCorner =
        !config_.obstacleAvoidanceEnabled &&
        sideReadingIsOpen(inputs.front, inputs.nowMs) &&
        expectedBaselineOpening;
    // Open has no pillars, so a close forward wall is sufficient. Obstacle
    // requires that same close wall plus the expected-side opening and no
    // active pass; a far side opening or pillar can never start a turn.
    const bool obstacleCornerEvidence =
        config_.obstacleAvoidanceEnabled && frontSuggestsCorner &&
        committedObstacle_ == ObstacleTarget::None &&
        obstacleRecenterStartOffsetMm_ == 0 &&
        expectedOpeningPresent(opening);
    const bool openCornerEvidence =
        !config_.obstacleAvoidanceEnabled &&
        (frontSuggestsCorner || blindFrontSuggestsOpenCorner);
    // Once Open has learned the course direction, a fresh usable front range
    // at the configured trigger is sufficient to begin the next turn.  The
    // 100 ms opening debounce is still required for the first direction,
    // optically blind Open evidence, and every Obstacle corner.  At full Open
    // speed that redundant delay consumed roughly 160 mm in the field and
    // made the steering reverse from straight-wall correction too late.
    if (!config_.obstacleAvoidanceEnabled && frontSuggestsCorner) {
      enterCorner(inputs);
      return;
    }
    const CourseDirection candidate =
        (openCornerEvidence || obstacleCornerEvidence)
            ? decision_.direction
            : CourseDirection::Unknown;
    if (openingConfirmed(candidate, inputs)) {
      enterCorner(inputs);
    }
    return;
  }

  if (decision_.state == RunState::Cornering) {
    updateCorner(inputs, wall, opening);
    return;
  }

  if (decision_.state == RunState::FinishDelay) {
    if (elapsedAtLeast(inputs.nowMs, finishEnteredAtMs_,
                       config_.finishTimeoutMs)) {
      latchFault(FaultFinishTimeout);
      return;
    }
    const float relativeHeading =
        unwrappedHeadingDegrees_ - runStartHeadingDegrees_;
    const bool guardedStraightFinish =
        config_.guardedObstaclePassEnabled &&
        !config_.sideSensorPairsAreParallel && calibrationCrossGuardActive_ &&
        absoluteValue(headingRemainderDegrees(relativeHeading)) <=
            config_.finishAlignmentDegrees &&
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm > config_.frontCornerTriggerMm;
    if (config_.obstacleAvoidanceEnabled &&
        expectedOpeningPresent(opening) && !guardedStraightFinish) {
      latchFault(FaultFinishTimeout);
      return;
    }
    decision_.directedProgressDegrees = courseProgressDegrees();
    const bool openFinishReady = !config_.obstacleAvoidanceEnabled;
    const bool finishGeometryReady =
        elapsedAtLeast(inputs.nowMs, finishEnteredAtMs_, config_.finishDriveMs) &&
        (openFinishReady ||
         (absoluteValue(headingRemainderDegrees(relativeHeading)) <=
              config_.finishAlignmentDegrees &&
          (straightSideGeometryUsable(inputs, wall) ||
           (config_.guardedObstaclePassEnabled &&
            calibrationCrossGuardActive_)) &&
          absoluteValue(wall.angleErrorMm) <=
              config_.finishMaximumWallAngleErrorMm &&
          ((frontReadingUsable(inputs.front, inputs.nowMs) &&
            inputs.front.rangeMm > config_.frontCornerTriggerMm) ||
           sideReadingIsOpen(inputs.front, inputs.nowMs))));
    if (!finishGeometryReady) {
      finishReadySinceMs_ = 0;
    } else {
      if (finishReadySinceMs_ == 0) finishReadySinceMs_ = inputs.nowMs;
      if (elapsedAtLeast(inputs.nowMs, finishReadySinceMs_,
                         config_.finishStableMs)) {
        decision_.state = RunState::Finished;
      }
    }
  }
}

bool Controller::visionFreshAndValid(const Inputs& inputs) const {
  return inputs.vision.online && inputs.vision.valid &&
         !inputs.vision.pipelineOverrun &&
         readingFresh(inputs.nowMs, inputs.vision.updatedAtMs,
                      config_.visionStaleMs);
}

bool Controller::blobUsable(const BlobReading& blob) const {
  return blob.present && blob.confidence >= config_.obstacleMinimumConfidence &&
         blob.bottomY >= config_.obstacleMinimumBottomY;
}

uint64_t Controller::blobPriority(const BlobReading& blob) {
  // Bottom position dominates (nearer on the floor), then apparent height,
  // then confidence.  This gives a deterministic choice when both colors are
  // present without relying on memory allocation or floating point.
  return static_cast<uint64_t>(blob.bottomY) * 1000000ULL +
         static_cast<uint64_t>(blob.height) * 1000ULL + blob.confidence;
}

void Controller::updateObstacle(const Inputs& inputs) {
  if (!config_.obstacleAvoidanceEnabled) {
    clearCommittedObstacle(inputs.nowMs, false);
    lastClearedObstacle_ = ObstacleTarget::None;
    obstacleRecommitBlockedSinceMs_ = 0;
    obstaclePassCornerRecoveryPending_ = false;
    decision_.obstacle = ObstacleTarget::None;
    decision_.targetLateralOffsetMm = 0;
    return;
  }

  const bool preferProgress = config_.preferForwardObstacleProgress &&
      !config_.stopAfterFirstVerifiedObstaclePass;
  if (recoverySkippedObstacle_ != ObstacleTarget::None &&
      (!preferProgress || elapsedAtLeast(inputs.nowMs,
                                        recoverySkippedObstacleAtMs_,
                                        kRecoveryObstacleSkipMs))) {
    recoverySkippedObstacle_ = ObstacleTarget::None;
    recoverySkippedObstacleAtMs_ = 0;
  }
  const bool freshVision = visionFreshAndValid(inputs);
  if (!freshVision) {
    // Camera downtime is unknown, not proof that a committed pillar is absent.
    // The committed-target blank interval must restart after vision recovers.
    // A completed pass's bounded same-colour quarantine is wall-clock time,
    // however, and must not turn into an indefinite block after a brief fault.
    obstacleAbsentSinceMs_ = 0;
    if (preferProgress) {
      obstacleSuccessorCandidateActive_ = false;
      obstacleSuccessorCandidateSampleAtMs_ = 0;
      obstacleSuccessorRollover_ = false;
    }
  }

  // Quarantine the tail of a physically cleared pillar for a bounded time.
  // Do not require another continuous blank interval: legal layouts can place
  // two pillars of the same colour only 500 mm apart, so the second blob may
  // already be visible when the first pass clears. Fresh vision is still
  // required below before that second target can actually be committed.
  if (lastClearedObstacle_ != ObstacleTarget::None &&
      elapsedAtLeast(inputs.nowMs, obstacleRecommitBlockedSinceMs_,
                     config_.obstacleRecommitCooldownMs)) {
    lastClearedObstacle_ = ObstacleTarget::None;
    obstacleRecommitBlockedSinceMs_ = 0;
  }

  if (committedObstacle_ == ObstacleTarget::None &&
      freshVision) {
    const bool red = blobUsable(inputs.vision.red) &&
                     lastClearedObstacle_ != ObstacleTarget::RedPassRight &&
                     recoverySkippedObstacle_ != ObstacleTarget::RedPassRight;
    const bool green = blobUsable(inputs.vision.green) &&
                       lastClearedObstacle_ != ObstacleTarget::GreenPassLeft &&
                       recoverySkippedObstacle_ != ObstacleTarget::GreenPassLeft;
    if (red || green) {
      const bool chooseRed =
          red && (!green ||
                  blobPriority(inputs.vision.red) >= blobPriority(inputs.vision.green));
      const BlobReading& blob = chooseRed ? inputs.vision.red : inputs.vision.green;
      const ObstacleTarget target = chooseRed ? ObstacleTarget::RedPassRight
                                               : ObstacleTarget::GreenPassLeft;
      const int16_t offset = obstacleOffsetForBlob(target, blob);
      commitObstacle(target, offset, blob.centerX, blob.confidence, blob.bottomY,
                     inputs.nowMs);
    }
  }

  // Once a side is committed it cannot flip because another color appears,
  // but a fresh view of that same pillar still improves the lateral target.
  // This avoids steering from a frozen first-frame position while preserving
  // the occlusion-safe pass decision.
  if (committedObstacle_ != ObstacleTarget::None &&
      freshVision) {
    const BlobReading& committedBlob =
        committedObstacle_ == ObstacleTarget::RedPassRight
            ? inputs.vision.red
            : inputs.vision.green;
    float successorPairDeltaMm = 0.0F;
    const uint8_t successorFrontIndex =
        committedObstacle_ == ObstacleTarget::RedPassRight ? FrontLeft
                                                          : FrontRight;
    const RangeReading& successorFront = inputs.sides[successorFrontIndex];
    const bool successorFrontClear =
        sideReadingIsOpen(successorFront, inputs.nowMs) ||
        (sideReadingUsable(successorFront, inputs.nowMs) &&
         successorFront.rangeMm > config_.obstacleSideClearanceGuardMm);
    const uint16_t successorProofBottomY =
        config_.obstaclePassProofMinimumBottomY > config_.obstacleCloseBottomY
            ? config_.obstaclePassProofMinimumBottomY
            : config_.obstacleCloseBottomY;
    const bool successorRecoveryReady =
        !obstacleCenteredEscapeActive_ ||
        (obstacleCenteredEscapeRecovered_ &&
         obstacleCenteredEscapePostRecoveryObserved_ &&
         !obstacleCenteredEscapeClearanceViolated_);
    const bool freshFartherTarget = blobUsable(committedBlob) &&
        static_cast<uint32_t>(committedBlob.bottomY) + 2000U <=
            obstaclePeakBottomY_;
    const bool strictSuccessorEvidence =
        config_.guardedObstaclePassEnabled && !obstacleBodyClearActive_ &&
        successorRecoveryReady && blobUsable(committedBlob) &&
        obstaclePeakBottomY_ >= successorProofBottomY &&
        freshFartherTarget &&
        absoluteValue(unwrappedHeadingDegrees_ -
                      obstacleCenteredEscapeHeadingReferenceDegrees_) <=
            config_.obstacleCornerRearmHeadingDegrees &&
        obstaclePairDeltaInitialized_ && successorFrontClear &&
        obstaclePassSidePairDelta(inputs, successorPairDeltaMm) &&
        successorPairDeltaMm >=
            obstacleMinimumPairDeltaMm_ + config_.obstaclePassClearDeltaMm;
    const bool successorEvidence = preferProgress ? freshFartherTarget
                                                  : strictSuccessorEvidence;
    if (preferProgress && !freshFartherTarget) {
      obstacleSuccessorRollover_ = false;
    }
    if (successorEvidence && !obstacleSuccessorRollover_) {
      if (!obstacleSuccessorCandidateActive_) {
        obstacleSuccessorCandidateActive_ = true;
        obstacleSuccessorCandidateSampleAtMs_ = inputs.vision.updatedAtMs;
      } else if (inputs.vision.updatedAtMs !=
                     obstacleSuccessorCandidateSampleAtMs_ &&
                 elapsedAtLeast(inputs.vision.updatedAtMs,
                                obstacleSuccessorCandidateSampleAtMs_, 80U)) {
        obstacleSuccessorRollover_ = true;
      }
    } else if (!obstacleSuccessorRollover_) {
      obstacleSuccessorCandidateActive_ = false;
      obstacleSuccessorCandidateSampleAtMs_ = 0;
    }
    // A stable, substantially farther same-color box can be the next pillar.
    // Preserve the old target's near/side evidence until its body tail clears.
    const bool retainedTargetVisible = preferProgress
        ? blobUsable(committedBlob) : committedBlob.present;
    if (retainedTargetVisible && !obstacleSuccessorRollover_ &&
        !successorEvidence) {
      obstacleAbsentSinceMs_ = 0;
      if (blobUsable(committedBlob)) {
        refreshCommittedObstacle(committedBlob);
        const bool postRecoverySample =
            !config_.recoverCenteredEscapeBeforePass ||
            !obstacleCenteredEscapeActive_ ||
            !obstacleCenteredEscapeRecovered_ ||
            inputs.vision.updatedAtMs !=
                obstacleCenteredEscapeRecoveredVisionSampleAtMs_;
        if (obstacleCenteredEscapeRecovered_ && postRecoverySample) {
          obstacleCenteredEscapePostRecoveryObserved_ = true;
        }
        if (postRecoverySample &&
            committedBlob.bottomY > obstaclePeakBottomY_) {
          obstaclePeakBottomY_ = committedBlob.bottomY;
        }
      }
    } else if (obstacleAbsentSinceMs_ == 0U) {
      obstacleAbsentSinceMs_ = inputs.nowMs;
    }
  }

  if (preferProgress && freshVision &&
      committedObstacle_ != ObstacleTarget::None) {
    const auto leadingRayClear = [&](uint8_t index) {
      const RangeReading& ray = inputs.sides[index];
      return (sideReadingUsable(ray, inputs.nowMs) && ray.rangeMm > 200U) ||
             sideReadingIsOpen(ray, inputs.nowMs);
    };
    const bool forwardRouteClear =
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm > 350U && leadingRayClear(FrontLeft) &&
        leadingRayClear(FrontRight);
    const BlobReading& progressBlob =
        committedObstacle_ == ObstacleTarget::RedPassRight
            ? inputs.vision.red : inputs.vision.green;
    const bool targetLost = !blobUsable(progressBlob) &&
        obstacleAbsentSinceMs_ != 0U &&
        elapsedAtLeast(inputs.nowMs, obstacleAbsentSinceMs_,
                       config_.obstacleBlobLossMs);
    const bool commitmentExpired = config_.obstacleMaximumCommitMs > 0U &&
        elapsedAtLeast(inputs.nowMs, obstacleCommittedAtMs_,
                       config_.obstacleMaximumCommitMs);
    const bool escapeFailed = obstacleCenteredEscapeActive_ &&
        (obstacleCenteredEscapeClearanceViolated_ ||
         (!obstacleCenteredEscapeTurnComplete_ &&
          centeredEscapeElapsedMs(inputs.nowMs) >=
              config_.obstacleCenteredEscapeDeadlineMs));
    if (forwardRouteClear &&
        (targetLost || commitmentExpired || escapeFailed ||
         obstacleSuccessorRollover_)) {
      const ObstacleTarget previousTarget = committedObstacle_;
      const BlobReading& currentBlob = progressBlob;
      const bool acceptSuccessor = obstacleSuccessorRollover_ &&
          blobUsable(currentBlob) && !commitmentExpired && !escapeFailed;
      // Releasing an obsolete route is not verified passage. Drop its old
      // steering/proof state and let fresh opening evidence rearm navigation;
      // never advance a corner, lap, or the strict single-pass finish here.
      clearCommittedObstacle(inputs.nowMs, false);
      obstaclePassCornerRecoveryPending_ = true;
      if (acceptSuccessor) {
        commitObstacle(previousTarget,
                       obstacleOffsetForBlob(previousTarget, currentBlob),
                       currentBlob.centerX, currentBlob.confidence,
                       currentBlob.bottomY, inputs.nowMs);
      } else if (commitmentExpired || escapeFailed) {
        // Give navigation a bounded opportunity to leave this failed route
        // instead of immediately recreating it from the same camera sample.
        lastClearedObstacle_ = previousTarget;
        obstacleRecommitBlockedSinceMs_ = inputs.nowMs;
      }
      decision_.obstacle = committedObstacle_;
      decision_.targetLateralOffsetMm = committedObstacleOffsetMm_;
      return;
    }
  }

  if (committedObstacle_ == ObstacleTarget::None) {
    decision_.obstacle = ObstacleTarget::None;
    int16_t recenterOffset = 0;
    if (obstacleRecenterStartOffsetMm_ != 0 &&
        config_.obstacleRecenterMs > 0U &&
        !elapsedAtLeast(inputs.nowMs, obstacleRecenterStartedAtMs_,
                        config_.obstacleRecenterMs)) {
      const uint32_t elapsed =
          static_cast<uint32_t>(inputs.nowMs - obstacleRecenterStartedAtMs_);
      const uint32_t remaining = config_.obstacleRecenterMs - elapsed;
      recenterOffset = static_cast<int16_t>(
          (static_cast<int32_t>(obstacleRecenterStartOffsetMm_) *
           static_cast<int32_t>(remaining)) /
          static_cast<int32_t>(config_.obstacleRecenterMs));
    } else {
      obstacleRecenterStartOffsetMm_ = 0;
      obstacleRecenterStartedAtMs_ = 0;
    }
    decision_.targetLateralOffsetMm = recenterOffset;
    return;
  }

  if (!preferProgress && config_.obstacleMaximumCommitMs > 0U &&
      elapsedAtLeast(inputs.nowMs, obstacleCommittedAtMs_,
                     config_.obstacleMaximumCommitMs)) {
    latchFault(FaultObstaclePassTimeout);
    return;
  }

  if (obstacleCenteredEscapeActive_ &&
      !obstacleCenteredEscapeTurnComplete_) {
    const uint8_t escapeFrontIndex =
        committedObstacle_ == ObstacleTarget::RedPassRight ? FrontLeft
                                                            : FrontRight;
    const RangeReading& escapeFront = inputs.sides[escapeFrontIndex];
    if (sideReadingUsable(escapeFront, inputs.nowMs) &&
        escapeFront.rangeMm <= config_.obstacleSideClearanceGuardMm) {
      // R26 reached FR=85/23 mm before its late second steering burst. Once
      // this envelope is crossed, later contact-induced yaw must never be
      // allowed to convert the attempt into a verified pass.
      obstacleCenteredEscapeClearanceViolated_ = true;
    }
    const uint32_t escapeElapsedMs =
        centeredEscapeElapsedMs(inputs.nowMs);
    const float escapeProgressDegrees =
        obstacleCenteredEscapeProgressDegrees();
    if (obstacleCenteredEscapeClearanceViolated_ ||
        escapeProgressDegrees <
            config_.obstacleCenteredEscapeHeadingDegrees) {
      // One accepted BNO step can still be a transient. Falling below the
      // target resets the candidate so a 0 -> 22 -> 0 degree spike cannot
      // permanently centre the steering early.
      obstacleCenteredEscapeHeadingCandidateActive_ = false;
      obstacleCenteredEscapeHeadingCandidateSampleAtMs_ = 0;
    } else if (escapeElapsedMs <=
                   config_.obstacleCenteredEscapeDeadlineMs &&
               inputs.imu.online && inputs.imu.valid &&
               readingFresh(inputs.nowMs, inputs.imu.updatedAtMs,
                            config_.sensorStaleMs)) {
      if (!obstacleCenteredEscapeHeadingCandidateActive_) {
        obstacleCenteredEscapeHeadingCandidateActive_ = true;
        obstacleCenteredEscapeHeadingCandidateSampleAtMs_ =
            inputs.imu.updatedAtMs;
      } else if (inputs.imu.updatedAtMs !=
                     obstacleCenteredEscapeHeadingCandidateSampleAtMs_ &&
                 elapsedAtLeast(
                     inputs.imu.updatedAtMs,
                     obstacleCenteredEscapeHeadingCandidateSampleAtMs_,
                     config_.obstacleCenteredEscapeHeadingStableMs)) {
        obstacleCenteredEscapeTurnComplete_ = true;
      }
    }
  }

  if (centeredEscapeRecoveryActive()) {
    // The first loss was caused by turning the camera away. Complete the
    // lane change before accepting a new close/loss/pair passage sequence.
    const float headingError = absoluteValue(
        unwrappedHeadingDegrees_ -
        obstacleCenteredEscapeHeadingReferenceDegrees_);
    if (headingError > 4.0F) {
      obstacleCenteredEscapeRecoveryCandidateActive_ = false;
      obstacleCenteredEscapeRecoveryCandidateSampleAtMs_ = 0;
    } else if (!obstacleCenteredEscapeRecoveryCandidateActive_) {
      obstacleCenteredEscapeRecoveryCandidateActive_ = true;
      obstacleCenteredEscapeRecoveryCandidateSampleAtMs_ =
          inputs.imu.updatedAtMs;
    } else if (inputs.imu.updatedAtMs !=
                   obstacleCenteredEscapeRecoveryCandidateSampleAtMs_ &&
               elapsedAtLeast(
                   inputs.imu.updatedAtMs,
                   obstacleCenteredEscapeRecoveryCandidateSampleAtMs_,
                   config_.obstacleCenteredEscapeHeadingStableMs)) {
      obstacleCenteredEscapeRecovered_ = true;
      obstacleCenteredEscapeRecoveredVisionSampleAtMs_ =
          inputs.vision.updatedAtMs;
      obstacleCenteredEscapePostRecoveryObserved_ = false;
  obstacleSuccessorCandidateActive_ = false;
  obstacleSuccessorCandidateSampleAtMs_ = 0;
  obstacleSuccessorRollover_ = false;
      obstaclePeakBottomY_ = 0;
      obstacleAbsentSinceMs_ = 0;
      obstacleClearSinceMs_ = 0;
      obstaclePairDeltaInitialized_ = false;
      obstacleMinimumPairDeltaMm_ = 0.0F;
    }
    decision_.obstacle = committedObstacle_;
    decision_.targetLateralOffsetMm = committedObstacleOffsetMm_;
    return;
  }

  if (obstacleBodyClearActive_) {
    // A close/loss/side sequence is inferred passage evidence, not a measured
    // chassis position. R30 collects that sequence only after heading recovery.
    // The bounded tail continues only while the centre-front ray is
    // numerically clear. A missing/close front resets the continuous interval;
    // the ordinary safety gates remove drive independently.
    const WallGuidance bodyClearWall = computeWallGuidance(inputs);
    const uint16_t bodyClearFrontGuardMm =
        config_.obstacleBodyClearFrontGuardMm > 0U
            ? config_.obstacleBodyClearFrontGuardMm
            : config_.directionProbeAbortFrontMm;
    const bool bodyClearFrontSafe =
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm > bodyClearFrontGuardMm;
    // A numeric centre-front return alone cannot authorize blind travel. Keep
    // the same fresh local-cross or complete-pair geometry gate that allowed
    // the close pass to move; losing it removes drive immediately while the
    // ordinary bounded fault grace decides whether it recovers.
    const bool bodyClearGeometrySafe =
        calibrationCrossGuardActive_ ||
        straightSideGeometryUsable(inputs, bodyClearWall);
    const auto bodyClearSideAtOrBelow = [&](uint8_t index,
                                             uint16_t thresholdMm) {
      return sideReadingUsable(inputs.sides[index], inputs.nowMs) &&
             inputs.sides[index].rangeMm <= thresholdMm;
    };
    const bool bodyClearLeftGloballyTooClose =
        config_.obstacleSideClearanceGuardMm > 0U &&
        (bodyClearSideAtOrBelow(FrontLeft,
                                config_.obstacleSideClearanceGuardMm) ||
         bodyClearSideAtOrBelow(BackLeft,
                                config_.obstacleSideClearanceGuardMm));
    const bool bodyClearRightGloballyTooClose =
        config_.obstacleSideClearanceGuardMm > 0U &&
        (bodyClearSideAtOrBelow(FrontRight,
                                config_.obstacleSideClearanceGuardMm) ||
         bodyClearSideAtOrBelow(BackRight,
                                config_.obstacleSideClearanceGuardMm));
    const bool bodyClearLeftHardStop =
        bodyClearSideAtOrBelow(
            FrontLeft, config_.obstacleBodyClearOppositeSideGuardMm) ||
        bodyClearSideAtOrBelow(
            BackLeft, config_.obstacleBodyClearOppositeSideGuardMm);
    const bool bodyClearRightHardStop =
        bodyClearSideAtOrBelow(
            FrontRight, config_.obstacleBodyClearOppositeSideGuardMm) ||
        bodyClearSideAtOrBelow(
            BackRight, config_.obstacleBodyClearOppositeSideGuardMm);
    const bool bodyClearOppositeSideSafe =
        !(bodyClearLeftGloballyTooClose &&
          bodyClearRightGloballyTooClose &&
          !centeredEscapePostTurnActive()) &&
        !((committedObstacle_ == ObstacleTarget::RedPassRight &&
           bodyClearRightHardStop) ||
          (committedObstacle_ == ObstacleTarget::GreenPassLeft &&
           bodyClearLeftHardStop));
    // Time is only a proxy for the measured translation in this bounded test.
    // Count it from a previously completed tick that actually commanded
    // forward motion; any stopped safety/recovery tick restarts the interval.
    const bool bodyClearTranslationAuthorized =
        decision_.motionAllowed && decision_.speedPermille > 0U &&
        decision_.driveDirection == DriveDirection::Forward;
    if (!bodyClearFrontSafe || !bodyClearGeometrySafe ||
        !bodyClearOppositeSideSafe || !bodyClearTranslationAuthorized) {
      obstacleBodyClearStartedAtMs_ = 0;
    } else if (obstacleBodyClearStartedAtMs_ == 0U) {
      obstacleBodyClearStartedAtMs_ = inputs.nowMs;
    } else if (elapsedAtLeast(inputs.nowMs, obstacleBodyClearStartedAtMs_,
                              config_.obstacleSinglePassBodyClearMs)) {
      clearCommittedObstacle(inputs.nowMs, true);
      obstaclePassCornerRecoveryPending_ = true;
      if (config_.stopAfterFirstVerifiedObstaclePass) {
        decision_.state = RunState::Finished;
      }
    }
    decision_.obstacle = committedObstacle_;
    decision_.targetLateralOffsetMm =
        committedObstacle_ == ObstacleTarget::None
            ? obstacleRecenterStartOffsetMm_
            : committedObstacleOffsetMm_;
    return;
  }

  float passPairDeltaMm = 0.0F;
  const bool passPairValid =
      obstaclePassSidePairDelta(inputs, passPairDeltaMm);
  if (passPairValid) {
    if (!obstaclePairDeltaInitialized_) {
      obstacleMinimumPairDeltaMm_ = passPairDeltaMm;
      obstaclePairDeltaInitialized_ = true;
    } else if (passPairDeltaMm < obstacleMinimumPairDeltaMm_) {
      obstacleMinimumPairDeltaMm_ = passPairDeltaMm;
    }
  }

  const BlobReading& committedBlob =
      committedObstacle_ == ObstacleTarget::RedPassRight
          ? inputs.vision.red
          : inputs.vision.green;
  const bool committedBlobPresent =
      freshVision && committedBlob.present && !obstacleSuccessorRollover_;
  const bool blobAbsentLongEnough =
      freshVision && !committedBlobPresent && obstacleAbsentSinceMs_ != 0U &&
      elapsedAtLeast(inputs.nowMs, obstacleAbsentSinceMs_,
                     config_.obstacleBlobLossMs);
  const uint16_t passProofBottomY =
      config_.obstaclePassProofMinimumBottomY > config_.obstacleCloseBottomY
          ? config_.obstaclePassProofMinimumBottomY
          : config_.obstacleCloseBottomY;
  const bool closeEncounterSeen = obstaclePeakBottomY_ >= passProofBottomY;
  const bool preCloseLossConfirmed =
      !closeEncounterSeen && freshVision && !committedBlobPresent &&
      !(config_.recoverCenteredEscapeBeforePass &&
        obstacleCenteredEscapeActive_) &&
      obstacleAbsentSinceMs_ != 0U &&
      elapsedAtLeast(inputs.nowMs, obstacleAbsentSinceMs_,
                     config_.obstaclePreCloseLossClearMs);
  if (preCloseLossConfirmed) {
    // A far detection that vanished is not a proven physical pass. Recenter
    // smoothly, but allow the same color to recommit immediately if it returns;
    // the post-pass same-color cooldown is only for pillars actually passed.
    clearCommittedObstacle(inputs.nowMs, true);
    lastClearedObstacle_ = ObstacleTarget::None;
    obstacleRecommitBlockedSinceMs_ = 0;
    obstaclePassCornerRecoveryPending_ = false;
    decision_.obstacle = ObstacleTarget::None;
    decision_.targetLateralOffsetMm = obstacleRecenterStartOffsetMm_;
    return;
  }
  const bool relativePairTransition =
      passPairValid && obstaclePairDeltaInitialized_ &&
      passPairDeltaMm >= obstacleMinimumPairDeltaMm_ +
                             static_cast<float>(
                                 config_.obstaclePassClearDeltaMm);
  const uint8_t selectedFrontIndex =
      committedObstacle_ == ObstacleTarget::RedPassRight ? FrontLeft
                                                          : FrontRight;
  const RangeReading& selectedFront = inputs.sides[selectedFrontIndex];
  const bool selectedFrontClear =
      sideReadingIsOpen(selectedFront, inputs.nowMs) ||
      (sideReadingUsable(selectedFront, inputs.nowMs) &&
       selectedFront.rangeMm > config_.obstacleSideClearanceGuardMm);
  const bool centeredEscapeProofReady =
      !obstacleCenteredEscapeActive_ ||
      (obstacleCenteredEscapeTurnComplete_ &&
       !obstacleCenteredEscapeClearanceViolated_ && selectedFrontClear &&
       (!config_.recoverCenteredEscapeBeforePass ||
        (obstacleCenteredEscapeRecovered_ &&
         obstacleCenteredEscapePostRecoveryObserved_ &&
         (!config_.guardedObstaclePassEnabled ||
          absoluteValue(unwrappedHeadingDegrees_ -
                        obstacleCenteredEscapeHeadingReferenceDegrees_) <=
              config_.obstacleCornerRearmHeadingDegrees))));

  // Clearing requires a full temporal sequence, not a raw front/back
  // inequality that a static wall or mounting bias can satisfy: close visual
  // encounter, same-color loss, then a relative pass-side transition.
  if (elapsedAtLeast(inputs.nowMs, obstacleCommittedAtMs_,
                     config_.obstacleMinimumCommitMs) &&
      closeEncounterSeen && blobAbsentLongEnough &&
      relativePairTransition && centeredEscapeProofReady) {
    if (obstacleClearSinceMs_ == 0U) obstacleClearSinceMs_ = inputs.nowMs;
    if (elapsedAtLeast(inputs.nowMs, obstacleClearSinceMs_,
                       config_.obstaclePassClearConfirmationMs)) {
      if (guardedPassEnabled() &&
          config_.obstacleSinglePassBodyClearMs > 0U) {
        obstacleBodyClearActive_ = true;
        obstacleBodyClearStartedAtMs_ = 0;
        obstacleClearSinceMs_ = 0;
      } else {
        clearCommittedObstacle(inputs.nowMs, true);
        obstaclePassCornerRecoveryPending_ = true;
        if (config_.stopAfterFirstVerifiedObstaclePass) {
          decision_.state = RunState::Finished;
        }
      }
    }
  } else {
    obstacleClearSinceMs_ = 0;
  }

  decision_.obstacle = committedObstacle_;
  decision_.targetLateralOffsetMm =
      committedObstacle_ == ObstacleTarget::None
          ? obstacleRecenterStartOffsetMm_
          : committedObstacleOffsetMm_;
}

void Controller::refreshCommittedObstacle(const BlobReading& blob) {
  const int16_t offset = obstacleOffsetForBlob(committedObstacle_, blob);
  // Small integer low-pass filter keeps single-pixel box jitter out of the
  // steering while still responding well within a normal approach.
  committedObstacleOffsetMm_ = static_cast<int16_t>(
      (3 * static_cast<int32_t>(committedObstacleOffsetMm_) + offset) / 4);
  committedObstacleCenterX_ = static_cast<int16_t>(
      (3 * static_cast<int32_t>(committedObstacleCenterX_) + blob.centerX) / 4);
  committedObstacleConfidence_ = static_cast<uint16_t>(
      (3 * static_cast<uint32_t>(committedObstacleConfidence_) +
       blob.confidence) /
      4U);
}

int16_t Controller::obstacleOffsetForBlob(ObstacleTarget target,
                                          const BlobReading& blob) const {
  const bool red = target == ObstacleTarget::RedPassRight;
  const int32_t sideSign = red ? 1 : -1;
  const int32_t desiredImageX =
      sideSign * -static_cast<int32_t>(config_.obstacleDesiredImageOffset);
  int32_t offset = sideSign * static_cast<int32_t>(config_.obstacleOffsetMm);
  if (config_.obstacleDesiredImageOffset > 0) {
    const int32_t imageError =
        static_cast<int32_t>(blob.centerX) - desiredImageX;
    offset += (imageError * config_.obstacleOffsetMm) /
              config_.obstacleDesiredImageOffset;
  }
  if (red) {
    if (offset < config_.obstacleMinimumOffsetMm) {
      offset = config_.obstacleMinimumOffsetMm;
    }
    if (offset > config_.obstacleMaximumOffsetMm) {
      offset = config_.obstacleMaximumOffsetMm;
    }
  } else {
    if (offset > -config_.obstacleMinimumOffsetMm) {
      offset = -config_.obstacleMinimumOffsetMm;
    }
    if (offset < -config_.obstacleMaximumOffsetMm) {
      offset = -config_.obstacleMaximumOffsetMm;
    }
  }
  return static_cast<int16_t>(offset);
}

float Controller::obstacleImageSteeringCorrection() const {
  if (committedObstacle_ == ObstacleTarget::None ||
      config_.obstacleDesiredImageOffset <= 0 ||
      config_.obstacleImageSteeringPerUnit <= 0.0F) {
    return 0.0F;
  }

  // Red must remain to the left of the image while the car passes on its
  // right; green is the exact mirror. The error sign therefore already matches
  // the actuator convention (negative left, positive right) for both colors.
  const int32_t desiredImageX =
      committedObstacle_ == ObstacleTarget::RedPassRight
          ? -static_cast<int32_t>(config_.obstacleDesiredImageOffset)
          : static_cast<int32_t>(config_.obstacleDesiredImageOffset);
  const int32_t imageError =
      static_cast<int32_t>(committedObstacleCenterX_) - desiredImageX;

  float depthScale = 1.0F;
  if (obstaclePeakBottomY_ <= config_.obstacleMinimumBottomY) {
    depthScale = 0.0F;
  } else if (obstaclePeakBottomY_ <
             config_.obstacleFullSteeringBottomY) {
    depthScale =
        static_cast<float>(obstaclePeakBottomY_ -
                           config_.obstacleMinimumBottomY) /
        static_cast<float>(config_.obstacleFullSteeringBottomY -
                           config_.obstacleMinimumBottomY);
    // Smoothstep keeps steering continuous as a far target first becomes
    // actionable, while still reaching full authority before the close pass.
    depthScale = depthScale * depthScale * (3.0F - 2.0F * depthScale);
  }

  const float correction = static_cast<float>(imageError) *
                           config_.obstacleImageSteeringPerUnit * depthScale;
  return static_cast<float>(clampSteering(
      correction, config_.obstacleMaximumImageSteeringPermille));
}

float Controller::obstacleCenteredEscapeProgressDegrees() const {
  if (!obstacleCenteredEscapeActive_ ||
      committedObstacle_ == ObstacleTarget::None) {
    return 0.0F;
  }
  const float steeringSign =
      committedObstacle_ == ObstacleTarget::RedPassRight ? 1.0F : -1.0F;
  const float rawYawSign =
      steeringSign * static_cast<float>(config_.clockwiseYawSign);
  return (unwrappedHeadingDegrees_ -
          obstacleCenteredEscapeHeadingReferenceDegrees_) * rawYawSign;
}

bool Controller::centeredEscapePostTurnActive() const {
  return config_.recoverCenteredEscapeBeforePass &&
         obstacleCenteredEscapeActive_ && obstacleCenteredEscapeTurnComplete_;
}

bool Controller::centeredEscapeRecoveryActive() const {
  return centeredEscapePostTurnActive() && !obstacleCenteredEscapeRecovered_;
}

bool Controller::guardedPassEnabled() const {
  return config_.guardedObstaclePassEnabled ||
         config_.stopAfterFirstVerifiedObstaclePass;
}

uint32_t Controller::centeredEscapeElapsedMs(uint32_t nowMs) const {
  return config_.guardedObstaclePassEnabled
             ? obstacleCenteredEscapeDrivenMs_
             : static_cast<uint32_t>(nowMs - obstacleCommittedAtMs_);
}

void Controller::updateCenteredEscapeDriveTime(const Inputs& inputs) {
  if (!config_.guardedObstaclePassEnabled || !obstacleCenteredEscapeActive_ ||
      obstacleCenteredEscapeTurnComplete_) return;
  if (decision_.wallRecoveryPhase != 0U ||
      decision_.driveDirection == DriveDirection::Reverse) {
    obstacleCenteredEscapeDriveUpdatedAtMs_ = inputs.nowMs;
    return;
  }
  const uint32_t elapsed =
      static_cast<uint32_t>(inputs.nowMs - obstacleCenteredEscapeDriveUpdatedAtMs_);
  obstacleCenteredEscapeDriveUpdatedAtMs_ = inputs.nowMs;
  // Board feedback reports the output applied over the preceding interval.
  // Pure controller callers fall back to that interval's motion decision.
  const bool driveApplied = inputs.driveFeedbackAvailable
                                ? inputs.appliedDrivePermille > 0U
                                : decision_.motionAllowed &&
                                      decision_.speedPermille > 0U &&
                                      decision_.driveDirection ==
                                          DriveDirection::Forward;
  if (driveApplied) {
    const uint32_t remaining = UINT32_MAX - obstacleCenteredEscapeDrivenMs_;
    obstacleCenteredEscapeDrivenMs_ += elapsed > remaining ? remaining : elapsed;
  }
}

void Controller::clearCommittedObstacle(uint32_t nowMs, bool beginRecenter) {
  if (beginRecenter && committedObstacle_ != ObstacleTarget::None) {
    lastClearedObstacle_ = committedObstacle_;
    obstacleRecommitBlockedSinceMs_ = nowMs;
    const bool recoveredGuardedPass =
        config_.guardedObstaclePassEnabled && obstacleCenteredEscapeRecovered_;
    obstacleRecenterStartOffsetMm_ =
        recoveredGuardedPass ? 0 : committedObstacleOffsetMm_;
    obstacleRecenterStartedAtMs_ = recoveredGuardedPass ? 0U : nowMs;
  } else {
    lastClearedObstacle_ = ObstacleTarget::None;
    obstacleRecommitBlockedSinceMs_ = 0;
    obstacleRecenterStartOffsetMm_ = 0;
    obstacleRecenterStartedAtMs_ = 0;
  }
  committedObstacle_ = ObstacleTarget::None;
  committedObstacleOffsetMm_ = 0;
  committedObstacleCenterX_ = 0;
  committedObstacleConfidence_ = 0;
  obstacleCommittedAtMs_ = 0;
  obstacleClearSinceMs_ = 0;
  obstacleBodyClearStartedAtMs_ = 0;
  obstacleBodyClearActive_ = false;
  obstacleCenteredEscapeActive_ = false;
  obstacleCenteredEscapeTurnComplete_ = false;
  obstacleCenteredEscapeDrivenMs_ = 0;
  obstacleCenteredEscapeDriveUpdatedAtMs_ = 0;
  obstacleCenteredEscapeRecovered_ = false;
  obstacleCenteredEscapeRecoveryCandidateActive_ = false;
  obstacleCenteredEscapeRecoveryCandidateSampleAtMs_ = 0;
  obstacleCenteredEscapeRecoveredVisionSampleAtMs_ = 0;
  obstacleCenteredEscapePostRecoveryObserved_ = false;
  obstacleSuccessorCandidateActive_ = false;
  obstacleSuccessorCandidateSampleAtMs_ = 0;
  obstacleSuccessorRollover_ = false;
  obstacleCenteredEscapeClearanceViolated_ = false;
  obstacleCenteredEscapeHeadingCandidateActive_ = false;
  obstacleCenteredEscapeHeadingCandidateSampleAtMs_ = 0;
  obstacleCenteredEscapeHeadingReferenceDegrees_ = 0.0F;
  obstacleAbsentSinceMs_ = 0;
  obstaclePeakBottomY_ = 0;
  obstaclePairDeltaInitialized_ = false;
  obstacleMinimumPairDeltaMm_ = 0.0F;
}

bool Controller::obstaclePassSidePairDelta(const Inputs& inputs,
                                           float& deltaMm) const {
  // Corner geometry intentionally changes both side-pair distances. Do not
  // call that a completed pass; keep the committed avoidance through the turn.
  if (committedObstacle_ == ObstacleTarget::None ||
      (decision_.state != RunState::Running &&
       decision_.state != RunState::DirectionProbe)) {
    return false;
  }
  const uint8_t frontIndex = committedObstacle_ == ObstacleTarget::RedPassRight
                                 ? FrontLeft
                                 : FrontRight;
  const uint8_t backIndex = committedObstacle_ == ObstacleTarget::RedPassRight
                                ? BackLeft
                                : BackRight;
  const RangeReading& frontSide = inputs.sides[frontIndex];
  const RangeReading& backSide = inputs.sides[backIndex];
  const bool frontNumeric = sideReadingUsable(frontSide, inputs.nowMs);
  const bool backNumeric = sideReadingUsable(backSide, inputs.nowMs);
  const bool frontOpen = sideReadingIsOpen(frontSide, inputs.nowMs);
  const bool backOpen = sideReadingIsOpen(backSide, inputs.nowMs);
  if ((!frontNumeric && !frontOpen) || (!backNumeric && !backOpen) ||
      (frontOpen && backOpen)) {
    return false;
  }

  // One selected ray can legitimately look beyond the low wall while the
  // other still tracks the pillar/wall. Give that strong 2/4 no-target a
  // bounded far proxy so the remaining numeric ray can prove the same relative
  // front-to-back transition. Two blind rays remain ambiguous and are rejected.
  const float openProxyMm =
      static_cast<float>(config_.openingReferenceMaximumMm);
  const float frontMm =
      frontOpen
          ? openProxyMm
          : (static_cast<float>(frontSide.rangeMm) < openProxyMm
                 ? static_cast<float>(frontSide.rangeMm)
                 : openProxyMm);
  const float backMm =
      backOpen
          ? openProxyMm
          : (static_cast<float>(backSide.rangeMm) < openProxyMm
                 ? static_cast<float>(backSide.rangeMm)
                 : openProxyMm);
  deltaMm = frontMm - backMm;
  return true;
}

void Controller::commitObstacle(ObstacleTarget target, int16_t lateralOffset,
                                int16_t centerX, uint16_t confidence,
                                uint16_t bottomY,
                                uint32_t nowMs) {
  committedObstacle_ = target;
  committedObstacleOffsetMm_ = lateralOffset;
  committedObstacleCenterX_ = centerX;
  committedObstacleConfidence_ = confidence;
  obstacleCommittedAtMs_ = nowMs;
  obstacleClearSinceMs_ = 0;
  obstacleBodyClearStartedAtMs_ = 0;
  obstacleBodyClearActive_ = false;
  const int32_t absoluteCenterX =
      centerX < 0 ? -static_cast<int32_t>(centerX)
                  : static_cast<int32_t>(centerX);
  obstacleCenteredEscapeActive_ =
      config_.obstacleCenteredEscapeInitialAbsX > 0 &&
      config_.obstacleCenteredEscapeHeadingDegrees > 0.0F &&
      config_.obstacleCenteredEscapeDeadlineMs > 0U &&
      config_.obstacleCenteredEscapeHeadingStableMs > 0U &&
      absoluteCenterX <= config_.obstacleCenteredEscapeInitialAbsX;
  obstacleCenteredEscapeTurnComplete_ = false;
  obstacleCenteredEscapeDrivenMs_ = 0;
  obstacleCenteredEscapeDriveUpdatedAtMs_ = nowMs;
  obstacleCenteredEscapeRecovered_ = false;
  obstacleCenteredEscapeRecoveryCandidateActive_ = false;
  obstacleCenteredEscapeRecoveryCandidateSampleAtMs_ = 0;
  obstacleCenteredEscapeRecoveredVisionSampleAtMs_ = 0;
  obstacleCenteredEscapePostRecoveryObserved_ = false;
  obstacleSuccessorCandidateActive_ = false;
  obstacleSuccessorCandidateSampleAtMs_ = 0;
  obstacleSuccessorRollover_ = false;
  obstacleCenteredEscapeClearanceViolated_ = false;
  obstacleCenteredEscapeHeadingCandidateActive_ = false;
  obstacleCenteredEscapeHeadingCandidateSampleAtMs_ = 0;
  obstacleCenteredEscapeHeadingReferenceDegrees_ = unwrappedHeadingDegrees_;
  obstacleAbsentSinceMs_ = 0;
  obstaclePeakBottomY_ = bottomY;
  obstaclePairDeltaInitialized_ = false;
  obstacleMinimumPairDeltaMm_ = 0.0F;
  obstacleRecenterStartedAtMs_ = 0;
  obstacleRecenterStartOffsetMm_ = 0;
  obstaclePassCornerRecoveryPending_ = false;
  // The selected pass-side ToFs can create the same range delta used to detect
  // a course opening. Require a fresh clear-front rearm after every pass rather
  // than preserving an armed detector through pillar geometry.
  cornerDetectionArmed_ = false;
  decision_.cornerDetectionArmed = false;
  cornerRearmClearSinceMs_ = 0;
  if (config_.rememberObstacleSections &&
      (decision_.state == RunState::DirectionProbe ||
       decision_.state == RunState::Running)) {
    const uint8_t sectionBit =
        static_cast<uint8_t>(1U << (decision_.cornerCount % kCornersPerLap));
    if (target == ObstacleTarget::RedPassRight) {
      decision_.learnedRedSections |= sectionBit;
    } else if (target == ObstacleTarget::GreenPassLeft) {
      decision_.learnedGreenSections |= sectionBit;
    }
  }
}

uint16_t Controller::applySpeedSlew(uint16_t requestedSpeed, uint32_t nowMs,
                                    bool enforceImmediateCap) {
  requestedSpeed = clampSpeed(requestedSpeed);
  if (requestedSpeed > 0U &&
      requestedSpeed < config_.minimumMovingSpeedPermille) {
    // The measured belt-drive breakaway floor is not a useful slow command.
    // A safety cap below it means stop; an ordinary request is raised to the
    // lowest command that physically moves the car.
    requestedSpeed = enforceImmediateCap ? 0U
                                         : config_.minimumMovingSpeedPermille;
  }
  const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - lastSpeedCommandAtMs_);
  const uint16_t previous = lastCommandedSpeedPermille_;
  uint16_t result = previous;
  if (requestedSpeed > previous) {
    const uint32_t rise =
        (static_cast<uint32_t>(config_.accelerationPermillePerSecond) *
         elapsedMs) /
        1000U;
    const uint32_t limited = static_cast<uint32_t>(previous) + rise;
    if (rise > 0U) {
      result = limited < requestedSpeed ? static_cast<uint16_t>(limited)
                                        : requestedSpeed;
    }
  } else if (requestedSpeed < previous && !enforceImmediateCap) {
    const uint32_t fall =
        (static_cast<uint32_t>(config_.decelerationPermillePerSecond) *
         elapsedMs) /
        1000U;
    const uint16_t limited =
        fall >= previous ? 0U : static_cast<uint16_t>(previous - fall);
    if (fall > 0U) result = limited > requestedSpeed ? limited : requestedSpeed;
  } else {
    result = requestedSpeed;
  }
  if (result > 0U && result < config_.minimumMovingSpeedPermille) {
    result = config_.minimumMovingSpeedPermille;
  }
  lastCommandedSpeedPermille_ = result;
  // Keep accumulating sub-permille time when a fast loop produces a zero
  // integer step. Reset the time base only once the command actually changes
  // (or already equals the request), otherwise acceleration could remain zero
  // forever on a sub-2 ms control loop.
  if (result != previous || result == requestedSpeed) {
    lastSpeedCommandAtMs_ = nowMs;
  }
  return result;
}

void Controller::makeMotionDecision(const Inputs& inputs,
                                    const WallGuidance& wall) {
  const bool movingState = decision_.state == RunState::DirectionProbe ||
                           decision_.state == RunState::Running ||
                           decision_.state == RunState::Cornering ||
                           decision_.state == RunState::FinishDelay;
  if (!movingState) {
    decision_.motionAllowed = false;
    decision_.speedPermille = 0;
    decision_.steeringPermille = 0;
    decision_.driveDirection = driveDirectionInterlock_.update(
        DriveDirection::Stopped, inputs.nowMs,
        config_.parkingReverseNeutralMs);
    lastCommandedSpeedPermille_ = 0;
    lastSpeedCommandAtMs_ = inputs.nowMs;
    return;
  }

  // Leading-ray hazards veto every forward command, including the start
  // transition. Recovery runs before navigation on the following update.
  if (leadingObstacleHazards(inputs) != 0U) {
    decision_.motionAllowed = false;
    decision_.speedPermille = 0;
    decision_.steeringPermille = 0;
    decision_.driveDirection = driveDirectionInterlock_.update(
        DriveDirection::Stopped, inputs.nowMs, config_.parkingReverseNeutralMs);
    lastCommandedSpeedPermille_ = 0;
    lastSpeedCommandAtMs_ = inputs.nowMs;
    return;
  }

  decision_.motionAllowed = true;
  bool rangeUncertainCap = false;
  const bool openChallenge = !config_.obstacleAvoidanceEnabled;
  const bool diagonalChassis = !config_.sideSensorPairsAreParallel;
  const bool centeredEscapeDeadlineMissed =
      obstacleCenteredEscapeActive_ &&
      !obstacleCenteredEscapeTurnComplete_ &&
      centeredEscapeElapsedMs(inputs.nowMs) >=
          config_.obstacleCenteredEscapeDeadlineMs;
  const bool centeredEscapeFailed =
      centeredEscapeDeadlineMissed ||
      obstacleCenteredEscapeClearanceViolated_;
  uint16_t speed = config_.cruiseSpeedPermille;
  if (decision_.state == RunState::DirectionProbe) {
    speed = config_.directionProbeSpeedPermille;
  } else if (decision_.state == RunState::Cornering) {
    speed = config_.cornerSpeedPermille;
    if (!openChallenge && !wall.completePair) {
      if (speed > config_.cornerBlindSpeedPermille) {
        speed = config_.cornerBlindSpeedPermille;
      }
      rangeUncertainCap = true;
    }
  } else if (decision_.state == RunState::FinishDelay) {
    speed = config_.finishSpeedPermille;
  } else if (decision_.obstacle != ObstacleTarget::None) {
    speed = config_.obstacleSpeedPermille;
    // Keep the long approach straight and fast, then blend down to the proven
    // obstacle-pass speed as the pillar reaches the close image region.
    if (config_.cruiseSpeedPermille > config_.obstacleSpeedPermille &&
        config_.obstacleCloseBottomY > config_.obstacleMinimumBottomY &&
        obstaclePeakBottomY_ < config_.obstacleCloseBottomY) {
      const uint32_t approachSpan = static_cast<uint32_t>(
          config_.obstacleCloseBottomY - config_.obstacleMinimumBottomY);
      const uint32_t approachProgress =
          obstaclePeakBottomY_ > config_.obstacleMinimumBottomY
              ? static_cast<uint32_t>(obstaclePeakBottomY_ -
                                      config_.obstacleMinimumBottomY)
              : 0U;
      const uint32_t speedSpan = static_cast<uint32_t>(
          config_.cruiseSpeedPermille - config_.obstacleSpeedPermille);
      speed = static_cast<uint16_t>(
          config_.cruiseSpeedPermille -
          (speedSpan * approachProgress) / approachSpan);
    }
  }
  // Each phase keeps its separately tuned speed. Open Challenge only raises a
  // non-zero request that is below the measured drivetrain breakaway floor;
  // it must not silently replace probe/corner/finish staging with cruise.
  if (openChallenge && speed > 0U &&
      speed < config_.minimumMovingSpeedPermille) {
    speed = config_.minimumMovingSpeedPermille;
  }
  const bool directionDecisionHold =
      decision_.state == RunState::DirectionProbe &&
      (directionDecisionHoldActive_ ||
       openingCandidate_ != CourseDirection::Unknown);
  if (directionDecisionHold) speed = 0U;
  const bool cornerSteeringPreposition =
      decision_.state == RunState::Cornering &&
      !elapsedAtLeast(inputs.nowMs, cornerStartedAtMs_,
                      config_.cornerDriveDelayMs);
  if (cornerSteeringPreposition) speed = 0U;
  const bool requiredVisionHold =
      config_.requireVisionForMotion && visionLossActive_;
  if (requiredVisionHold) speed = 0U;
  const bool sideElectricalHold = sideElectricalLossActive_;
  if (sideElectricalHold) speed = 0U;
  const bool calibrationCrossOnlyHold =
      guardedPassEnabled() &&
      !straightSideGeometryUsable(inputs, wall) &&
      !calibrationCrossGuardActive_;
  if (calibrationCrossOnlyHold) speed = 0U;
  const bool bodyClearFrontHold =
      (obstacleBodyClearActive_ ||
       (centeredEscapePostTurnActive() &&
        !config_.guardedObstaclePassEnabled)) &&
      config_.obstacleBodyClearFrontGuardMm > 0U &&
      (!frontReadingUsable(inputs.front, inputs.nowMs) ||
       inputs.front.rangeMm <= config_.obstacleBodyClearFrontGuardMm);
  if (bodyClearFrontHold) speed = 0U;

  // A fresh no-return is not a wiring failure and must not abruptly stop the
  // vehicle. During DirectionProbe it is the expected state of a legal start
  // several metres from the first wall, so retain the separately tuned probe
  // command. Once direction is known, a guided Open corner may use its normal
  // turn command; other phases degrade until numeric front geometry returns.
  const bool healthyImuGuidance =
      inputs.imu.online && inputs.imu.valid &&
      isfinite(inputs.imu.yawDegrees) &&
      readingFresh(inputs.nowMs, inputs.imu.updatedAtMs,
                   config_.sensorStaleMs) &&
      inputs.imu.accuracy >= config_.minimumRunImuAccuracy;
  const bool guidedFarDirectionProbe =
      frontOpticallyDegraded_ &&
      sideReadingIsOpen(inputs.front, inputs.nowMs) &&
      decision_.state == RunState::DirectionProbe &&
      (openChallenge
           ? healthyImuGuidance && anyNumericSideReadingUsable(inputs)
           : !sideGeometryDegraded_);
  const bool guidedOpenCorner =
      frontOpticallyDegraded_ &&
      sideReadingIsOpen(inputs.front, inputs.nowMs) && openChallenge &&
      decision_.state == RunState::Cornering && healthyImuGuidance &&
      (wall.outerWallUsable ||
       (!config_.sideSensorPairsAreParallel &&
        anyLocalSideReadingUsable(inputs)));
  if ((frontOpticallyDegraded_ && !guidedFarDirectionProbe &&
       !guidedOpenCorner) ||
      (!openChallenge && sideGeometryDegraded_)) {
    if (speed > config_.cornerBlindSpeedPermille) {
      speed = config_.cornerBlindSpeedPermille;
    }
    rangeUncertainCap = true;
  }
  if (openChallenge && decision_.state == RunState::Running &&
      decision_.direction != CourseDirection::Unknown) {
    if (!config_.sideSensorPairsAreParallel) {
      // R15 physically demonstrated the 600-permille probe command. Until a
      // full heading-only lap is recorded, do not jump from that evidence to
      // an unmeasured 1000-permille straight with no calibrated wall model.
      if (speed > config_.directionProbeSpeedPermille) {
        speed = config_.directionProbeSpeedPermille;
        rangeUncertainCap = true;
      }
    } else if (!wall.outerWallUsable || wall.innerGuardActive) {
      if (speed > config_.cornerBlindSpeedPermille) {
        speed = config_.cornerBlindSpeedPermille;
      }
      rangeUncertainCap = true;
    }
  }

  // The front range produces a deterministic stopping-distance proxy. It is
  // intentionally a clearance-to-command relationship, not a made-up braking
  // model. The emergency threshold itself is handled by the fault latch.
  bool frontStoppingCap = false;
  uint32_t frontEnvelopeDistance = config_.frontStoppingProxyMm;
  if (config_.frontSlowDistanceMm > config_.frontEmergencyStopMm) {
    const uint32_t legacySlowDistance =
        static_cast<uint32_t>(config_.frontSlowDistanceMm -
                              config_.frontEmergencyStopMm);
    if (legacySlowDistance > frontEnvelopeDistance) {
      frontEnvelopeDistance = legacySlowDistance;
    }
  }
  // Once a corner is confirmed, the centre ray is expected to look at
  // the wall being rounded. R17 let that expected reading reduce the proven
  // 450 turn command to 200, which stalled the assembled drivetrain at full
  // steering lock. Pillars cannot occupy official corner sections, and both
  // profiles require real corner evidence before entering this state. Keep the
  // proportional envelope for straights, but not for a confirmed corner. The
  // hard frontEmergencyStopMm check remains active in evaluateSafety().
  const bool confirmedDiagonalCorner =
      diagonalChassis && decision_.state == RunState::Cornering &&
      decision_.direction != CourseDirection::Unknown;
  const bool useFrontEnvelope =
      !confirmedDiagonalCorner &&
      (decision_.state == RunState::DirectionProbe ||
       decision_.state == RunState::Running ||
       decision_.state == RunState::Cornering ||
       decision_.state == RunState::FinishDelay);
  if (useFrontEnvelope &&
      frontReadingUsable(inputs.front, inputs.nowMs) &&
      inputs.front.rangeMm <
          static_cast<uint32_t>(config_.frontEmergencyStopMm) +
              frontEnvelopeDistance &&
      frontEnvelopeDistance > 0U) {
    const uint32_t clearance =
        inputs.front.rangeMm > config_.frontEmergencyStopMm
            ? static_cast<uint32_t>(inputs.front.rangeMm -
                                    config_.frontEmergencyStopMm)
            : 0U;
    uint32_t cap =
        (static_cast<uint32_t>(speed) * clearance) / frontEnvelopeDistance;
    // The measured belt drive cannot produce a useful command below its
    // breakaway floor. In Open Challenge a non-emergency forward wall is also
    // the next-corner cue, so keep creeping at that floor until the confirmed
    // turn begins; the <= emergency threshold still latches a full stop.
    if (clearance > 0U &&
        cap < config_.minimumMovingSpeedPermille) {
      cap = config_.minimumMovingSpeedPermille;
    }
    if (cap < speed) {
      speed = static_cast<uint16_t>(cap);
      frontStoppingCap = true;
    }
  }

  uint16_t confidenceCap = 1000U;
  bool safetySpeedCap =
      rangeUncertainCap || directionDecisionHold || requiredVisionHold ||
      sideElectricalHold || cornerSteeringPreposition ||
      calibrationCrossOnlyHold;
  if (obstacleCenteredEscapeActive_ && !obstacleBodyClearActive_) {
    if (centeredEscapeFailed) {
      speed = 0U;
    } else if (speed > config_.obstacleSideClearanceSpeedPermille) {
      speed = config_.obstacleSideClearanceSpeedPermille;
    }
    // Apply the calibration cap immediately; a previous faster approach must
    // never coast through the measured escape envelope via comfort slew.
    safetySpeedCap = true;
  }
  const bool unconfirmedObstacleLossHold =
      committedObstacle_ != ObstacleTarget::None &&
      !(config_.guardedObstaclePassEnabled && obstacleCenteredEscapeActive_) &&
      !centeredEscapePostTurnActive() &&
      obstaclePeakBottomY_ < config_.obstacleCloseBottomY &&
      obstacleAbsentSinceMs_ != 0U &&
      elapsedAtLeast(inputs.nowMs, obstacleAbsentSinceMs_,
                     config_.obstacleBlobLossMs);
  if (unconfirmedObstacleLossHold) {
    // Do not continue driving a stale far-only avoidance target. A longer fresh
    // blank interval abandons it in updateObstacle(); reacquisition before then
    // resumes the same committed pass.
    speed = 0U;
    safetySpeedCap = true;
  }
  if (config_.obstacleAvoidanceEnabled) {
    if (!visionFreshAndValid(inputs)) {
      confidenceCap = config_.visionUncertainSpeedPermille;
      safetySpeedCap = true;
    }
    if (inputs.vision.pipelineOverrun &&
        config_.visionOverrunSpeedPermille < confidenceCap) {
      confidenceCap = config_.visionOverrunSpeedPermille;
      safetySpeedCap = true;
    }
    if (config_.fullSpeedFrameQuality > 0U &&
        inputs.vision.frameQuality < config_.fullSpeedFrameQuality) {
      const uint16_t qualityCap = static_cast<uint16_t>(
          (static_cast<uint32_t>(config_.visionUncertainSpeedPermille) *
               (config_.fullSpeedFrameQuality - inputs.vision.frameQuality) +
           static_cast<uint32_t>(config_.cruiseSpeedPermille) *
               inputs.vision.frameQuality) /
          config_.fullSpeedFrameQuality);
      if (qualityCap < confidenceCap) {
        confidenceCap = qualityCap;
        safetySpeedCap = true;
      }
    }
  }
  if (committedObstacle_ != ObstacleTarget::None) {
    // Preserve a margin whenever the committed target was only moderately
    // confident. The camera protocol uses 0..1000 rather than a probability.
    const uint32_t span = config_.obstacleSpeedPermille >
                                  config_.obstacleUncertainSpeedPermille
                              ? config_.obstacleSpeedPermille -
                                    config_.obstacleUncertainSpeedPermille
                              : 0U;
    uint16_t obstacleConfidenceCap = static_cast<uint16_t>(
        config_.obstacleUncertainSpeedPermille +
        (span * committedObstacleConfidence_) / 1000U);
    // A blob that passed the configured acceptance threshold is actionable.
    // If the drivetrain cannot physically move below its measured breakaway
    // floor, confidence should reduce it to that floor rather than silently
    // turn every non-perfect detection into a zero-speed deadlock. Camera
    // loss, overrun, poor frame quality, and range uncertainty remain separate
    // safety caps and can still command an immediate stop below the floor.
    if (obstacleConfidenceCap > 0U &&
        obstacleConfidenceCap < config_.minimumMovingSpeedPermille) {
      obstacleConfidenceCap = config_.minimumMovingSpeedPermille;
    }
    if (obstacleConfidenceCap < confidenceCap) {
      confidenceCap = obstacleConfidenceCap;
      safetySpeedCap = true;
    }
  }
  if (speed > confidenceCap) speed = confidenceCap;

  // First-lap observations become a coarse section-level caution map. This
  // never steers from memory: live camera and ToF data remain the authority.
  // It only avoids entering a previously occupied straight at open-course
  // cruise speed while a pillar is still too far away to reacquire.
  if (config_.rememberObstacleSections &&
      decision_.state == RunState::Running && decision_.lapCount > 0U &&
      committedObstacle_ == ObstacleTarget::None) {
    const uint8_t sectionBit =
        static_cast<uint8_t>(1U << (decision_.cornerCount % kCornersPerLap));
    const uint8_t occupiedSections = static_cast<uint8_t>(
        decision_.learnedRedSections | decision_.learnedGreenSections);
    if ((occupiedSections & sectionBit) != 0U &&
        speed > config_.learnedObstacleSectionSpeedPermille) {
      speed = config_.learnedObstacleSectionSpeedPermille;
      safetySpeedCap = true;
    }
  }

  if (decision_.state == RunState::Cornering && diagonalChassis) {
    // The first physical runs proved that holding full Ackermann lock until
    // 85 degrees stores too much yaw: the car completes the corner but keeps
    // rotating into the next wall. Taper toward centre before 90 degrees. A
    // moderate opposite command is allowed only after measured overshoot, so
    // a car that is still short of the target can never be forced outward.
    // Clockwise/right is positive steering; counter-clockwise/left is negative.
    const int16_t turnSign =
        decision_.direction == CourseDirection::CounterClockwise ? -1 : 1;
    float steeringMagnitude =
        static_cast<float>(config_.cornerSteeringPermille);
    if (decision_.cornerProgressDegrees >=
        config_.openCornerExitFallbackDegrees) {
      const float directedHeadingError =
          decision_.cornerProgressDegrees - kRightAngleDegrees;
      steeringMagnitude =
          -directedHeadingError * config_.headingSteeringPerDegree;
      if (directedHeadingError < 0.0F) {
        // Decay the configured physical steering floor smoothly from the
        // fallback threshold to zero at the exact cardinal. This preserves
        // enough linkage authority to finish an under-turn without creating a
        // steering step at either end of the interval.
        const float finalApproachSpan =
            kRightAngleDegrees - config_.openCornerExitFallbackDegrees;
        float finalApproachScale =
            finalApproachSpan > 0.0F
                ? -directedHeadingError / finalApproachSpan
                : 0.0F;
        if (finalApproachScale < 0.0F) finalApproachScale = 0.0F;
        if (finalApproachScale > 1.0F) finalApproachScale = 1.0F;
        const float physicalApproachSteer =
            static_cast<float>(
                config_.openCornerMinimumApproachSteerPermille) *
            finalApproachScale;
        if (steeringMagnitude < physicalApproachSteer) {
          steeringMagnitude = physicalApproachSteer;
        }
      }
      if (decision_.cornerProgressDegrees >=
              config_.openCornerCounterSteerStartDegrees &&
          config_.openCornerCounterSteerPermille > 0) {
        float counterSteerScale =
            (decision_.cornerProgressDegrees -
             config_.openCornerCounterSteerStartDegrees) /
            config_.openCornerCounterSteerRampDegrees;
        if (counterSteerScale < 0.0F) counterSteerScale = 0.0F;
        if (counterSteerScale > 1.0F) counterSteerScale = 1.0F;
        const float counterSteer =
            static_cast<float>(config_.openCornerCounterSteerPermille) *
            counterSteerScale;
        if (steeringMagnitude > -counterSteer) {
          steeringMagnitude = -counterSteer;
        }
      }
      steeringMagnitude = static_cast<float>(clampSteering(
          steeringMagnitude, config_.maximumHeadingCorrectionPermille));
    } else if (decision_.cornerProgressDegrees >
               config_.openCornerTaperStartDegrees) {
      const float taperSpan = config_.openCornerExitFallbackDegrees -
                              config_.openCornerTaperStartDegrees;
      const float taperRemaining = config_.openCornerExitFallbackDegrees -
                                   decision_.cornerProgressDegrees;
      steeringMagnitude =
          static_cast<float>(config_.openCornerMinimumApproachSteerPermille) +
          (steeringMagnitude -
           static_cast<float>(config_.openCornerMinimumApproachSteerPermille)) *
              taperRemaining / taperSpan;
    }
    decision_.steeringPermille = clampSteering(
        static_cast<float>(turnSign) * steeringMagnitude,
        config_.maximumSteeringPermille);
  } else if (decision_.state == RunState::Cornering &&
             cornerReadySinceMs_ == 0) {
    decision_.steeringPermille =
        decision_.direction == CourseDirection::CounterClockwise
            ? static_cast<int16_t>(-config_.cornerSteeringPermille)
            : config_.cornerSteeringPermille;
  } else {
    const bool diagonalCornerExitUnwind =
        diagonalChassis && decision_.state == RunState::Cornering &&
        cornerReadySinceMs_ != 0U;
    const bool postCornerStraightState =
        decision_.state == RunState::Running ||
        decision_.state == RunState::FinishDelay;
    const bool diagonalPostCornerHeadingOnly =
        diagonalChassis && postCornerStraightState &&
        decision_.cornerCount > 0U && config_.postCornerHeadingOnlyMs > 0U &&
        !elapsedAtLeast(inputs.nowMs, lastCornerAtMs_,
                        config_.postCornerHeadingOnlyMs);
    const uint32_t elapsedSinceCornerMs =
        decision_.cornerCount > 0U
            ? static_cast<uint32_t>(inputs.nowMs - lastCornerAtMs_)
            : 0U;
    const bool diagonalPostCornerWallBlend =
        diagonalChassis && postCornerStraightState &&
        decision_.cornerCount > 0U && config_.postCornerWallBlendMs > 0U &&
        elapsedSinceCornerMs >= config_.postCornerHeadingOnlyMs &&
        elapsedSinceCornerMs < config_.postCornerHeadingOnlyMs +
                                   config_.postCornerWallBlendMs;
    const bool openHeadingOnlyGuidance =
        openChallenge && !config_.sideSensorPairsAreParallel;
    const bool diagonalObstacleCameraGuidance =
        !openChallenge && diagonalChassis &&
        committedObstacle_ != ObstacleTarget::None &&
        decision_.state != RunState::Cornering;
    const bool suppressWallCorrection =
        diagonalCornerExitUnwind || diagonalPostCornerHeadingOnly ||
        openHeadingOnlyGuidance ||
        (config_.guardedObstaclePassEnabled && diagonalChassis &&
         !diagonalObstacleCameraGuidance);
    // A commitment from the previous straight is retained until the corner
    // exit is verified, but it must not pull the car sideways during the
    // exit-stability window. Pillar seats are not in corner sections.
    const int16_t effectiveTargetLateralOffsetMm =
        decision_.state == RunState::Cornering
            ? 0
            : decision_.targetLateralOffsetMm;
    // With incomplete wall geometry, do not invent a corridor width from one
    // ray. Keep the camera's requested pillar offset and IMU heading, but omit
    // uncertain wall-position and wall-angle corrections until geometry heals.
    const bool useOpenOuterWallGuidance =
        openChallenge && config_.sideSensorPairsAreParallel &&
        decision_.direction != CourseDirection::Unknown;
    // Before the first opening identifies CW/CCW, an outside wall does not yet
    // exist. DirectionProbe must therefore use ordinary corridor centering;
    // after direction is known, Open switches permanently to its outside pair.
    const bool wallDistanceUnavailable =
        useOpenOuterWallGuidance ? !wall.outerWallUsable
                                 : sideGeometryDegraded_;
    const bool wallAngleUnavailable =
        useOpenOuterWallGuidance ? !wall.outerWallPair
                                 : sideGeometryDegraded_;
    float lateralCorrection =
        diagonalObstacleCameraGuidance
            ? obstacleImageSteeringCorrection()
            : suppressWallCorrection
            ? 0.0F
            : wallDistanceUnavailable
                  ? config_.lateralSteeringPerMm *
                        effectiveTargetLateralOffsetMm
                  : config_.lateralSteeringPerMm *
                        (static_cast<float>(effectiveTargetLateralOffsetMm) -
                         wall.lateralOffsetMm);
    float angleCorrection =
        suppressWallCorrection || diagonalObstacleCameraGuidance ||
                wallAngleUnavailable
            ? 0.0F
            : -config_.wallAngleSteeringPerMm * wall.angleErrorMm;
    if (openChallenge) {
      angleCorrection = static_cast<float>(clampSteering(
          angleCorrection,
          config_.openMaximumWallAngleCorrectionPermille));
    }
    if (diagonalPostCornerWallBlend) {
      const float blend =
          static_cast<float>(elapsedSinceCornerMs -
                             config_.postCornerHeadingOnlyMs) /
          static_cast<float>(config_.postCornerWallBlendMs);
      lateralCorrection *= blend;
      angleCorrection *= blend;
    }
    float headingCorrection = 0.0F;
    const bool headingControlledDirectionProbe =
        diagonalChassis &&
        decision_.state == RunState::DirectionProbe;
    if (headingControlledDirectionProbe ||
        (decision_.direction != CourseDirection::Unknown &&
         (decision_.state == RunState::Running ||
          decision_.state == RunState::FinishDelay ||
          diagonalCornerExitUnwind))) {
      const float headingReference =
          headingControlledDirectionProbe
              ? runStartHeadingDegrees_
              : diagonalCornerExitUnwind ? cornerTargetHeadingDegrees_
                                         : straightHeadingReferenceDegrees_;
      const float headingError =
          unwrappedHeadingDegrees_ - headingReference;
      // clockwiseYawSign maps a raw yaw change to the car's physical right.
      // Negating it produces the steering needed to return to the reference.
      headingCorrection = -static_cast<float>(config_.clockwiseYawSign) *
                          headingError * config_.headingSteeringPerDegree;
      headingCorrection = static_cast<float>(clampSteering(
          headingCorrection, config_.maximumHeadingCorrectionPermille));
    }
    const float wallCorrection = static_cast<float>(clampSteering(
        lateralCorrection + angleCorrection,
        config_.maximumWallCorrectionPermille));
    decision_.steeringPermille = clampSteering(
        wallCorrection + headingCorrection, config_.maximumSteeringPermille);

    // R26 proved that image error plus original-heading recovery can cancel a
    // valid green escape at only ten degrees. For a calibration target that
    // began near the image centre, use the BNO as the phase boundary instead:
    // hold the exact mirrored away command to the measured 22-degree target,
    // then hold straight wheels on that diagonal until strict pass proof. If
    // the target was not reached by the bounded deadline, centre and stop.
    if (obstacleCenteredEscapeActive_ && !obstacleBodyClearActive_) {
      if (centeredEscapeFailed ||
          obstacleCenteredEscapeTurnComplete_) {
        decision_.steeringPermille = 0;
      } else {
        decision_.steeringPermille =
            committedObstacle_ == ObstacleTarget::RedPassRight
                ? config_.obstacleMaximumImageSteeringPermille
                : static_cast<int16_t>(
                      -config_.obstacleMaximumImageSteeringPermille);
      }
    }

    if (centeredEscapePostTurnActive() && !obstacleBodyClearActive_ &&
        !centeredEscapeFailed) {
      const float rawError = unwrappedHeadingDegrees_ -
                             obstacleCenteredEscapeHeadingReferenceDegrees_;
      const float physicalError =
          rawError * static_cast<float>(config_.clockwiseYawSign);
      if (centeredEscapeRecoveryActive()) {
        const float magnitude = absoluteValue(physicalError);
        float steeringMagnitude = 0.0F;
        if (magnitude > 4.0F) {
          steeringMagnitude = magnitude >= 8.0F
                                  ? 500.0F
                                  : 200.0F + (magnitude - 4.0F) * 75.0F;
        }
        decision_.steeringPermille = clampSteering(
            physicalError > 0.0F ? -steeringMagnitude : steeringMagnitude,
            config_.obstacleMaximumImageSteeringPermille);
      } else {
        // Keep the entry heading, but a live pillar still inside the desired
        // passing bearing can request more steering away. Never chase a stale
        // or successor box, and never add another away turn beyond 22 degrees.
        float headingSteer = -physicalError * 25.0F;
        const BlobReading& currentBlob =
            committedObstacle_ == ObstacleTarget::RedPassRight
                ? inputs.vision.red
                : inputs.vision.green;
        if (config_.guardedObstaclePassEnabled &&
            visionFreshAndValid(inputs) && blobUsable(currentBlob) &&
            !obstacleSuccessorCandidateActive_ && !obstacleSuccessorRollover_ &&
            obstacleCenteredEscapeProgressDegrees() <
                config_.obstacleCenteredEscapeHeadingDegrees) {
          const float imageSteer = obstacleImageSteeringCorrection();
          if ((committedObstacle_ == ObstacleTarget::RedPassRight &&
               imageSteer > 0.0F) ||
              (committedObstacle_ == ObstacleTarget::GreenPassLeft &&
               imageSteer < 0.0F)) {
            headingSteer += imageSteer;
          }
        }
        decision_.steeringPermille = clampSteering(
            headingSteer,
            config_.obstacleMaximumImageSteeringPermille);
      }
    }

    // A close pillar's image bearing can cross its desired bearing long
    // before the chassis has translated far enough to clear it.  R23C then
    // counter-steered toward a still-visible red pillar: the red box moved
    // from the right to the left of the image, but the car had only reached
    // about 14 degrees of escape heading.  In the bounded single-pass build,
    // keep the steering from reversing toward the committed pillar until a
    // continuous fresh-camera loss confirms that it has left the view.  This
    // does not force extra lock; it only clamps the unsafe sign to centre.
    // The independent side-proximity arbitration below can still steer more
    // strongly away, steer back from the opposite wall, or stop completely.
    const bool calibrationClosePassNonReversal =
        guardedPassEnabled() &&
        !centeredEscapePostTurnActive() &&
        committedObstacle_ != ObstacleTarget::None &&
        obstaclePeakBottomY_ >= config_.obstacleCloseBottomY &&
        (obstacleBodyClearActive_ || obstacleAbsentSinceMs_ == 0U ||
         !elapsedAtLeast(inputs.nowMs, obstacleAbsentSinceMs_,
                         config_.obstacleBlobLossMs));
    if (calibrationClosePassNonReversal) {
      if (committedObstacle_ == ObstacleTarget::RedPassRight &&
          decision_.steeringPermille < 0) {
        decision_.steeringPermille = 0;
      } else if (committedObstacle_ == ObstacleTarget::GreenPassLeft &&
                 decision_.steeringPermille > 0) {
        decision_.steeringPermille = 0;
      }
    }

    // Continue the opposite-steer pulse briefly after the state transition,
    // then decay it smoothly to zero. Heading feedback is added throughout,
    // so an under-turn can reduce the pulse while an over-turn strengthens it.
    const bool freshClosePillarNeedsAway =
        config_.guardedObstaclePassEnabled &&
        committedObstacle_ != ObstacleTarget::None &&
        visionFreshAndValid(inputs) &&
        ((committedObstacle_ == ObstacleTarget::RedPassRight &&
          blobUsable(inputs.vision.red) &&
          inputs.vision.red.bottomY >= config_.obstacleCloseBottomY &&
          obstacleImageSteeringCorrection() > 0.0F) ||
         (committedObstacle_ == ObstacleTarget::GreenPassLeft &&
          blobUsable(inputs.vision.green) &&
          inputs.vision.green.bottomY >= config_.obstacleCloseBottomY &&
          obstacleImageSteeringCorrection() < 0.0F));
    if (diagonalPostCornerHeadingOnly && !freshClosePillarNeedsAway) {
      const float directionYawSign =
          decision_.direction == CourseDirection::Clockwise
              ? static_cast<float>(config_.clockwiseYawSign)
              : -static_cast<float>(config_.clockwiseYawSign);
      const float directedHeadingError =
          (unwrappedHeadingDegrees_ - straightHeadingReferenceDegrees_) *
          directionYawSign;
      if (directedHeadingError < 0.0F) {
        const float finalApproachSpan =
            kRightAngleDegrees - config_.openCornerExitFallbackDegrees;
        float finalApproachScale =
            finalApproachSpan > 0.0F
                ? -directedHeadingError / finalApproachSpan
                : 0.0F;
        if (finalApproachScale < 0.0F) finalApproachScale = 0.0F;
        if (finalApproachScale > 1.0F) finalApproachScale = 1.0F;
        const float approachSteer =
            static_cast<float>(
                config_.openCornerMinimumApproachSteerPermille) *
            finalApproachScale;
        const float signedApproachSteer =
            decision_.direction == CourseDirection::Clockwise
                ? approachSteer
                : -approachSteer;
        const bool headingAlreadyStronger =
            (signedApproachSteer < 0.0F &&
             static_cast<float>(decision_.steeringPermille) <=
                 signedApproachSteer) ||
            (signedApproachSteer > 0.0F &&
             static_cast<float>(decision_.steeringPermille) >=
                 signedApproachSteer);
        if (!headingAlreadyStronger) {
          decision_.steeringPermille = clampSteering(
              signedApproachSteer, config_.maximumHeadingCorrectionPermille);
        }
      } else if (config_.openPostCornerCounterSteerMs > 0U &&
                 directedHeadingError >
              config_.openCornerCounterSteerStartDegrees -
                  kRightAngleDegrees &&
          elapsedSinceCornerMs < config_.openPostCornerCounterSteerMs) {
        float overshootScale =
            (directedHeadingError -
             (config_.openCornerCounterSteerStartDegrees -
              kRightAngleDegrees)) /
            config_.openCornerCounterSteerRampDegrees;
        if (overshootScale < 0.0F) overshootScale = 0.0F;
        if (overshootScale > 1.0F) overshootScale = 1.0F;
        const float remaining = static_cast<float>(
            config_.openPostCornerCounterSteerMs - elapsedSinceCornerMs);
        const float counterSteer =
            static_cast<float>(config_.openCornerCounterSteerPermille) *
            overshootScale * remaining /
            static_cast<float>(config_.openPostCornerCounterSteerMs);
        const float signedCounterSteer =
            decision_.direction == CourseDirection::Clockwise
                ? -counterSteer
                : counterSteer;
        const bool headingAlreadyStronger =
            (signedCounterSteer < 0.0F &&
             static_cast<float>(decision_.steeringPermille) <=
                 signedCounterSteer) ||
            (signedCounterSteer > 0.0F &&
             static_cast<float>(decision_.steeringPermille) >=
                 signedCounterSteer);
        if (!headingAlreadyStronger) {
          decision_.steeringPermille = clampSteering(
              signedCounterSteer, config_.maximumHeadingCorrectionPermille);
        }
      }
    }

    // Do not jump from corner speed to full cruise while the chassis is still
    // unwinding.  The physical car can cover a large distance during that
    // short yaw transient, so hold the proven corner speed until the exact
    // cardinal-heading recovery window has ended; normal acceleration slew
    // then releases it to cruise.
    if (diagonalPostCornerHeadingOnly &&
        speed > config_.cornerSpeedPermille) {
      speed = config_.cornerSpeedPermille;
      safetySpeedCap = true;
    }
  }

  if (obstacleBodyClearActive_) {
    // Strict pass proof already requires the close pillar to have left the
    // camera. Its remembered image bearing is therefore stale and must not
    // keep adding steering during the final chassis-length translation. Hold
    // the achieved escape heading with straight wheels; the live proximity
    // arbitration below may still steer away from the pillar side or stop at
    // the tighter free-side wall bound.
    decision_.steeringPermille = 0;
  }

  // Double-X-style arbitration for the sensors this car actually has: a very
  // close side return overrides nominal camera/heading guidance on a straight.
  // These independently angled rays are only used as proximity guards. They
  // never become a fabricated front/rear wall angle.
  if (!openChallenge && decision_.state != RunState::Cornering &&
      config_.obstacleSideClearanceGuardMm > 0U) {
    bool leftTooClose = false;
    bool rightTooClose = false;
    const uint8_t leftIndices[] = {FrontLeft, BackLeft};
    const uint8_t rightIndices[] = {FrontRight, BackRight};
    for (const uint8_t index : leftIndices) {
      const RangeReading& reading = inputs.sides[index];
      leftTooClose =
          leftTooClose ||
          (sideReadingUsable(reading, inputs.nowMs) &&
           reading.rangeMm <= config_.obstacleSideClearanceGuardMm);
    }
    for (const uint8_t index : rightIndices) {
      const RangeReading& reading = inputs.sides[index];
      rightTooClose =
          rightTooClose ||
          (sideReadingUsable(reading, inputs.nowMs) &&
           reading.rangeMm <= config_.obstacleSideClearanceGuardMm);
    }
    // During the short body-clear phase the committed pillar side is known:
    // red is on the left while passing right; green is on the right while
    // passing left. A close return on the opposite/free side cannot safely
    // override the non-reversal clamp and steer back toward the pillar. Stop
    // instead; a close return on the pillar side may still steer away.
    const auto sideAtOrBelow = [&](uint8_t index, uint16_t thresholdMm) {
      return thresholdMm > 0U &&
             sideReadingUsable(inputs.sides[index], inputs.nowMs) &&
             inputs.sides[index].rangeMm <= thresholdMm;
    };
    const bool leftBodyClearHardStop =
        sideAtOrBelow(FrontLeft,
                      config_.obstacleBodyClearOppositeSideGuardMm) ||
        sideAtOrBelow(BackLeft,
                      config_.obstacleBodyClearOppositeSideGuardMm);
    const bool rightBodyClearHardStop =
        sideAtOrBelow(FrontRight,
                      config_.obstacleBodyClearOppositeSideGuardMm) ||
        sideAtOrBelow(BackRight,
                      config_.obstacleBodyClearOppositeSideGuardMm);
    const bool guardedPostTurn =
        obstacleBodyClearActive_ || centeredEscapePostTurnActive();
    const bool bodyClearOppositeSideBlocked =
        guardedPostTurn &&
        ((committedObstacle_ == ObstacleTarget::RedPassRight &&
          rightBodyClearHardStop) ||
         (committedObstacle_ == ObstacleTarget::GreenPassLeft &&
          leftBodyClearHardStop));
    const bool centeredEscapeFreeSideBlocked =
        obstacleCenteredEscapeActive_ && !guardedPostTurn &&
        ((committedObstacle_ == ObstacleTarget::RedPassRight &&
          rightTooClose) ||
         (committedObstacle_ == ObstacleTarget::GreenPassLeft &&
          leftTooClose));
    // Rear rays constrain body clearance and reverse travel, but do not
    // invent a leading obstacle that demands forward steering across the lane.
    bool applyLeftEscape = config_.guardedObstaclePassEnabled
        ? sideAtOrBelow(FrontLeft, config_.obstacleSideClearanceGuardMm)
        : leftTooClose;
    bool applyRightEscape = config_.guardedObstaclePassEnabled
        ? sideAtOrBelow(FrontRight, config_.obstacleSideClearanceGuardMm)
        : rightTooClose;
    if (guardedPostTurn) {
      // Between the ordinary 200 mm pillar guard and the 140 mm hard wall
      // stop, ignore only the free-side ray. Letting it command the normal
      // escape would reverse steering toward the pillar we are still passing.
      if (committedObstacle_ == ObstacleTarget::RedPassRight) {
        applyRightEscape = false;
      } else if (committedObstacle_ == ObstacleTarget::GreenPassLeft) {
        applyLeftEscape = false;
      }
    }
    const bool bothSidesBlocked =
        leftTooClose && rightTooClose && !centeredEscapePostTurnActive();
    if (bothSidesBlocked || centeredEscapeFailed ||
        bodyClearOppositeSideBlocked || centeredEscapeFreeSideBlocked) {
      // There is no unambiguous free side. Stop instead of oscillating between
      // two equally unsafe escape commands.
      speed = 0U;
      decision_.steeringPermille = 0;
      safetySpeedCap = true;
    } else if (applyLeftEscape) {
      if (decision_.steeringPermille <
          config_.obstacleSideClearanceSteeringPermille) {
        decision_.steeringPermille =
            config_.obstacleSideClearanceSteeringPermille;
      }
      if (speed > config_.obstacleSideClearanceSpeedPermille) {
        speed = config_.obstacleSideClearanceSpeedPermille;
      }
      safetySpeedCap = true;
    } else if (applyRightEscape) {
      const int16_t escapeLeft = static_cast<int16_t>(
          -config_.obstacleSideClearanceSteeringPermille);
      if (decision_.steeringPermille > escapeLeft) {
        decision_.steeringPermille = escapeLeft;
      }
      if (speed > config_.obstacleSideClearanceSpeedPermille) {
        speed = config_.obstacleSideClearanceSpeedPermille;
      }
      safetySpeedCap = true;
    }
  }

  // A forward turn swings the tail in the opposite direction. A close rear
  // return cannot command avoidance, but can veto a turn into that rear space.
  const uint8_t departureRearIndices[] = {BackLeft, BackRight};
  for (uint8_t i = 0; i < 2U; ++i) {
    const RangeReading& rear = inputs.sides[departureRearIndices[i]];
    if ((sideReadingUsable(rear, inputs.nowMs) && rear.rangeMm > 350U) ||
        sideReadingIsOpen(rear, inputs.nowMs)) {
      rearDepartureConstraintMask_ &= static_cast<uint8_t>(~(1U << i));
    }
  }
  if (rearDepartureConstraintMask_ != 0U &&
      !rearSweepAllowsSteering(inputs, decision_.steeringPermille)) {
    const bool straightRoute = skippedPillarStraightDepartureReady(
        inputs, rearStraightDepartureActive_);
    if (straightRoute && !rearStraightDepartureActive_) {
      rearStraightDepartureActive_ = true;
      rearStraightDepartureStartedAtMs_ = inputs.nowMs;
      rearStraightDepartureUpdatedAtMs_ = inputs.nowMs;
      rearStraightDeparturePoweredMs_ = 0;
    }
    if (rearStraightDepartureActive_) {
      const uint32_t elapsed = inputs.nowMs - rearStraightDepartureUpdatedAtMs_;
      rearStraightDepartureUpdatedAtMs_ = inputs.nowMs;
      if (decision_.driveDirection == DriveDirection::Forward &&
          (inputs.driveFeedbackAvailable ? inputs.appliedDrivePermille > 0U
                                        : decision_.speedPermille > 0U)) {
        rearStraightDeparturePoweredMs_ += elapsed;
      }
      if (rearStraightDeparturePoweredMs_ >= 500U ||
          elapsedAtLeast(inputs.nowMs, rearStraightDepartureStartedAtMs_, 1500U) ||
          (frontReadingUsable(inputs.front, inputs.nowMs) &&
           inputs.front.rangeMm <= 350U)) {
        rearStraightDepartureExhausted_ = true;
      }
    }
    // Only a skipped-pillar departure can translate straight to clear its
    // known blocked tail. Never turn toward that rear ray or move blindly.
    speed = straightRoute && !rearStraightDepartureExhausted_
        ? config_.cruiseSpeedPermille : 0U;
    decision_.steeringPermille = 0;
    safetySpeedCap = true;
  } else {
    rearStraightDepartureActive_ = false;
    rearStraightDepartureExhausted_ = false;
    rearStraightDeparturePoweredMs_ = 0;
  }

  // Steering severity caps the speed before the actuator sees either command.
  // Full-lock turns are limited to the configured corner-speed envelope.
  if (config_.maximumSteeringPermille > 0) {
    const uint16_t steeringMagnitude = static_cast<uint16_t>(
        absoluteValue(static_cast<float>(decision_.steeringPermille)));
    const uint32_t severity =
        (static_cast<uint32_t>(steeringMagnitude) * 1000U) /
        config_.maximumSteeringPermille;
    uint16_t turnFloor = config_.cornerSpeedPermille;
    if (openChallenge && turnFloor > 0U &&
        turnFloor < config_.minimumMovingSpeedPermille) {
      turnFloor = config_.minimumMovingSpeedPermille;
    }
    // A deliberately slower obstacle straight must not also reduce the
    // separately proven full-lock corner command below its moving floor.
    const uint16_t turnCeiling =
        config_.guardedObstaclePassEnabled &&
                decision_.state == RunState::Cornering &&
                config_.cornerSpeedPermille > config_.cruiseSpeedPermille
            ? config_.cornerSpeedPermille
            : config_.cruiseSpeedPermille;
    const uint16_t turnCap = turnCeiling > turnFloor
                                 ? static_cast<uint16_t>(
                                       turnCeiling -
                                       ((static_cast<uint32_t>(turnCeiling -
                                                               turnFloor) *
                                         severity) /
                                        1000U))
                                 : turnCeiling;
    if (speed > turnCap) {
      speed = turnCap;
      safetySpeedCap = true;
    }
  }

  const DriveDirection requestedDriveDirection =
      speed == 0U ? DriveDirection::Stopped : DriveDirection::Forward;
  const DriveDirection permittedDriveDirection =
      driveDirectionInterlock_.update(requestedDriveDirection, inputs.nowMs,
                                      config_.parkingReverseNeutralMs);
  if (permittedDriveDirection == DriveDirection::Stopped) {
    speed = 0U;
    safetySpeedCap = true;
  }

  // A close front object is a safety envelope, so it bypasses comfort slew.
  // Other changes use the bounded acceleration/deceleration rates above.
  const bool immediatePhaseCap =
      decision_.state == RunState::Cornering ||
      (config_.guardedObstaclePassEnabled &&
       lastCommandedSpeedPermille_ > config_.cruiseSpeedPermille &&
       speed <= config_.cruiseSpeedPermille);
  decision_.speedPermille = applySpeedSlew(
      speed, inputs.nowMs,
      frontStoppingCap || safetySpeedCap || immediatePhaseCap);
  decision_.driveDirection = decision_.speedPermille == 0U
                                 ? DriveDirection::Stopped
                                 : permittedDriveDirection;
}

void Controller::latchFault(uint32_t flags) {
  latchedFaultFlags_ |= flags;
  decision_.faultFlags = latchedFaultFlags_;
  decision_.state = RunState::FaultLatched;
  decision_.motionAllowed = false;
  decision_.speedPermille = 0;
  decision_.driveDirection = DriveDirection::Stopped;
  decision_.steeringPermille = 0;
  lastCommandedSpeedPermille_ = 0;
}

bool Controller::handleForwardSideEscape(const Inputs& inputs,
                                         uint32_t safetyFaults) {
  if (!config_.preferForwardObstacleProgress ||
      !config_.guardedObstaclePassEnabled ||
      config_.stopAfterFirstVerifiedObstaclePass) return false;
  forwardSideEscapeBlockedHazards_ = 0;
  const bool corner = decision_.state == RunState::Cornering;
  if (!corner && decision_.state != RunState::Running &&
      decision_.state != RunState::DirectionProbe &&
      decision_.state != RunState::FinishDelay) return false;
  const auto rayClear = [&](uint8_t index, uint16_t threshold) {
    const RangeReading& ray = inputs.sides[index];
    return (sideReadingUsable(ray, inputs.nowMs) && ray.rangeMm > threshold) ||
        sideReadingIsOpen(ray, inputs.nowMs);
  };
  if (rayClear(FrontLeft, 300U) && rayClear(FrontRight, 300U)) {
    forwardSideEscapeSuppressed_ = false;
  }
  const auto finishEscape = [&]() {
    if (forwardSideEscapeActive_) {
      // This is physical repositioning within the same section, never proof
      // of a completed corner or pillar pass. Keep the absolute course target.
      if (corner) {
        const float yawSign = decision_.direction == CourseDirection::Clockwise
            ? static_cast<float>(config_.clockwiseYawSign)
            : -static_cast<float>(config_.clockwiseYawSign);
        if ((unwrappedHeadingDegrees_ - forwardSideEscapeHeadingDegrees_) *
                yawSign < 0.0F) {
          cornerStartHeadingDegrees_ = unwrappedHeadingDegrees_;
        }
        // A forward arc continues this corner; retain its elapsed time so
        // reaching the cardinal does not start a second minimum-turn delay.
        cornerReadySinceMs_ = 0;
      } else if (decision_.state == RunState::FinishDelay) {
        finishEnteredAtMs_ = inputs.nowMs;
        finishReadySinceMs_ = 0;
      }
    }
    forwardSideEscapeActive_ = false;
  };
  if (safetyFaults != FaultNone || !visionFreshAndValid(inputs) ||
      !wallRecoveryInputsReady(inputs, false, false)) {
    finishEscape();
    return false;
  }
  if (decision_.wallRecoveryPhase == 2U ||
      (decision_.wallRecoveryPhase != 0U &&
       !elapsedAtLeast(inputs.nowMs, wallRecoveryPhaseStartedAtMs_, 300U))) {
    return false;
  }
  if (!updateHeading(inputs)) {
    latchFault(FaultHeadingJump);
    return true;
  }
  if (decision_.wallRecoveryPhase != 0U &&
      absoluteValue(unwrappedHeadingDegrees_ -
                    wallRecoveryHeadingReferenceDegrees_) > 30.0F) return false;

  uint8_t proactiveHazards = 0;
  if (sideReadingUsable(inputs.sides[FrontLeft], inputs.nowMs) &&
      inputs.sides[FrontLeft].rangeMm <= 200U) proactiveHazards |= 2U;
  if (sideReadingUsable(inputs.sides[FrontRight], inputs.nowMs) &&
      inputs.sides[FrontRight].rangeMm <= 200U) proactiveHazards |= 4U;
  const uint8_t hazards = forwardSideEscapeActive_
      ? forwardSideEscapeHazards_ : proactiveHazards;
  if (hazards != 2U && hazards != 4U) {
    finishEscape();
    return false;
  }
  const uint8_t nearIndex = hazards == 2U ? FrontLeft : FrontRight;
  const uint8_t oppositeIndex = hazards == 2U ? FrontRight : FrontLeft;
  const uint8_t sweptRearIndex = hazards == 2U ? BackLeft : BackRight;
  if (forwardSideEscapeActive_ && rayClear(nearIndex, 250U)) {
    finishEscape();
    return false;
  }
  const RangeReading& near = inputs.sides[nearIndex];
  const bool routeClear = frontReadingUsable(inputs.front, inputs.nowMs) &&
      inputs.front.rangeMm > 350U &&
      sideReadingUsable(near, inputs.nowMs) && near.rangeMm > 140U &&
      rayClear(oppositeIndex, 300U) && rayClear(sweptRearIndex, 300U);
  if (!routeClear || forwardSideEscapeSuppressed_) {
    if (!routeClear &&
        (forwardSideEscapeActive_ ||
         frontReadingUsable(inputs.front, inputs.nowMs))) {
      // Once a numeric route can be evaluated, insufficient opposite/tail
      // room must not fall through to the older unqualified soft-side steer.
      // Geometry can improve while stopped; only the powered/time/yaw budget
      // below latches suppression until the old hazard has cleared.
      forwardSideEscapeBlockedHazards_ = hazards;
    }
    finishEscape();
    return false;
  }
  if (!forwardSideEscapeActive_) {
    forwardSideEscapeActive_ = true;
    forwardSideEscapeHazards_ = hazards;
    forwardSideEscapeStartedAtMs_ = inputs.nowMs;
    forwardSideEscapeUpdatedAtMs_ = inputs.nowMs;
    forwardSideEscapePoweredMs_ = 0;
    forwardSideEscapeHeadingDegrees_ = unwrappedHeadingDegrees_;
    // Only stopped entry/exit phases can choose this measured forward route.
    // The independent actuator interlock still enforces physical neutral time.
    decision_.wallRecoveryPhase = 0;
    decision_.wallRecoveryHazards = 0;
    decision_.wallRecoveryExitReason = 0;
  } else {
    const uint32_t elapsed = inputs.nowMs - forwardSideEscapeUpdatedAtMs_;
    forwardSideEscapeUpdatedAtMs_ = inputs.nowMs;
    if (decision_.driveDirection == DriveDirection::Forward &&
        (inputs.driveFeedbackAvailable ? inputs.appliedDrivePermille > 0U
                                      : decision_.speedPermille > 0U)) {
      forwardSideEscapePoweredMs_ += elapsed;
    }
    if (forwardSideEscapePoweredMs_ >= 500U ||
        elapsedAtLeast(inputs.nowMs, forwardSideEscapeStartedAtMs_, 1500U) ||
        absoluteValue(unwrappedHeadingDegrees_ -
                      forwardSideEscapeHeadingDegrees_) >= 20.0F) {
      forwardSideEscapeSuppressed_ = true;
      finishEscape();
      return false;
    }
  }
  const bool turnRight = hazards == 2U;
  const bool followsCorner = corner &&
      ((turnRight && decision_.direction == CourseDirection::Clockwise) ||
       (!turnRight && decision_.direction == CourseDirection::CounterClockwise));
  if (followsCorner) {
    // The measured free route permits continuing this existing corner. Keep
    // its ordinary absolute-heading, timeout and stable-exit proof running;
    // a local escape budget must not bypass the cardinal or count it blindly.
    updateCorner(inputs, computeWallGuidance(inputs), OpeningEvidence{});
    if (decision_.state == RunState::FaultLatched) {
      forwardSideEscapeActive_ = false;
      return true;
    }
    if (decision_.state != RunState::Cornering) {
      forwardSideEscapeActive_ = false;
      return false;
    }
  }
  int16_t magnitude = followsCorner ? config_.cornerSteeringPermille
      : config_.obstacleSideClearanceSteeringPermille;
  if (followsCorner) {
    const float yawSign = decision_.direction == CourseDirection::Clockwise
        ? static_cast<float>(config_.clockwiseYawSign)
        : -static_cast<float>(config_.clockwiseYawSign);
    float remainingScale =
        (cornerTargetHeadingDegrees_ - unwrappedHeadingDegrees_) * yawSign / 20.0F;
    if (remainingScale < 0.0F) remainingScale = 0.0F;
    if (remainingScale > 1.0F) remainingScale = 1.0F;
    magnitude = static_cast<int16_t>(static_cast<float>(magnitude) * remainingScale);
  }
  decision_.steeringPermille = turnRight ? magnitude : -magnitude;
  decision_.driveDirection = driveDirectionInterlock_.update(
      DriveDirection::Forward, inputs.nowMs, config_.parkingReverseNeutralMs);
  decision_.speedPermille = decision_.driveDirection == DriveDirection::Forward
      ? (followsCorner ? config_.cornerSpeedPermille : config_.cruiseSpeedPermille)
      : 0U;
  decision_.motionAllowed = decision_.speedPermille > 0U;
  decision_.faultFlags = FaultNone;
  decision_.unwrappedHeadingDegrees = unwrappedHeadingDegrees_;
  lastCommandedSpeedPermille_ = decision_.speedPermille;
  lastSpeedCommandAtMs_ = inputs.nowMs;
  return true;
}

uint8_t Controller::leadingObstacleHazards(const Inputs& inputs) const {
  if (!config_.guardedObstaclePassEnabled ||
      (decision_.state != RunState::DirectionProbe &&
       decision_.state != RunState::Running &&
       decision_.state != RunState::Cornering &&
       decision_.state != RunState::FinishDelay)) return 0U;
  const bool progressMode = config_.preferForwardObstacleProgress &&
      !config_.stopAfterFirstVerifiedObstaclePass;
  const uint16_t limitMm = decision_.state == RunState::Cornering ? 200U : 140U;
  uint8_t hazards = 0;
  if (sideReadingUsable(inputs.sides[FrontLeft], inputs.nowMs) &&
      inputs.sides[FrontLeft].rangeMm <= limitMm) hazards |= 2U;
  if (sideReadingUsable(inputs.sides[FrontRight], inputs.nowMs) &&
      inputs.sides[FrontRight].rangeMm <= limitMm) hazards |= 4U;
  // If a bounded forward escape could not clear its original ray, hand that
  // same hazard to backout. Ordinary moderate-side handling stays unchanged.
  const uint8_t forcedHazards = forwardSideEscapeBlockedHazards_ |
      (forwardSideEscapeSuppressed_ ? forwardSideEscapeHazards_ : 0U);
  if (progressMode && forcedHazards != 0U) {
    if ((forcedHazards & 2U) != 0U &&
        sideReadingUsable(inputs.sides[FrontLeft], inputs.nowMs) &&
        inputs.sides[FrontLeft].rangeMm <= 250U) hazards |= 2U;
    if ((forcedHazards & 4U) != 0U &&
        sideReadingUsable(inputs.sides[FrontRight], inputs.nowMs) &&
        inputs.sides[FrontRight].rangeMm <= 250U) hazards |= 4U;
  }
  return hazards;
}

bool Controller::wallRecoveryInputsReady(
    const Inputs& inputs, bool allowFrontOpticalNoTarget,
    bool requireRearClearance) const {
  const bool frontObserved = frontReadingUsable(inputs.front, inputs.nowMs) ||
      (allowFrontOpticalNoTarget && inputs.front.online &&
       inputs.front.i2cStatus == 0U && !inputs.front.valid &&
       (inputs.front.rangeStatus == 2U || inputs.front.rangeStatus == 4U ||
        inputs.front.rangeStatus == 7U) &&
       readingFresh(inputs.nowMs, inputs.front.updatedAtMs, config_.sensorStaleMs));
  if (!frontObserved ||
      !inputs.imu.online || !inputs.imu.valid ||
      !isfinite(inputs.imu.yawDegrees) ||
      !readingFresh(inputs.nowMs, inputs.imu.updatedAtMs, config_.sensorStaleMs) ||
      inputs.imu.accuracy < config_.minimumRunImuAccuracy ||
      (config_.requireVisionForMotion && !visionFreshAndValid(inputs))) return false;
  for (const RangeReading& side : inputs.sides) {
    if (!side.online || side.i2cStatus != 0U ||
        !readingFresh(inputs.nowMs, side.updatedAtMs, config_.sensorStaleMs) ||
        (!side.valid && !opticalNoTargetStatus(side.rangeStatus))) return false;
  }
  if (!requireRearClearance) return true;
  const uint8_t rearIndices[] = {BackLeft, BackRight};
  for (const uint8_t index : rearIndices) {
    const RangeReading& rear = inputs.sides[index];
    if (!((sideReadingUsable(rear, inputs.nowMs) && rear.rangeMm > 200U) ||
          sideReadingIsOpen(rear, inputs.nowMs))) return false;
  }
  return true;
}

bool Controller::rearRayAtOrBelow(const Inputs& inputs, uint16_t limitMm) const {
  return (sideReadingUsable(inputs.sides[BackLeft], inputs.nowMs) &&
          inputs.sides[BackLeft].rangeMm <= limitMm) ||
         (sideReadingUsable(inputs.sides[BackRight], inputs.nowMs) &&
          inputs.sides[BackRight].rangeMm <= limitMm);
}

bool Controller::rearSweepAllowsSteering(const Inputs& inputs,
                                        float steering) const {
  if (!config_.guardedObstaclePassEnabled) return true;
  const auto blocked = [&](uint8_t index, uint8_t mask) {
    const RangeReading& rear = inputs.sides[index];
    if (sideReadingUsable(rear, inputs.nowMs)) return rear.rangeMm <= 200U;
    return (rearDepartureConstraintMask_ & mask) != 0U &&
           !sideReadingIsOpen(rear, inputs.nowMs);
  };
  if (steering > 0.0F && blocked(BackLeft, 1U)) return false;
  if (steering < 0.0F && blocked(BackRight, 2U)) return false;
  return true;
}

bool Controller::recoveryPillarRouteUsable(
    const Inputs& inputs, ObstacleTarget target) const {
  if (!visionFreshAndValid(inputs)) return false;
  if (config_.preferForwardObstacleProgress &&
      !config_.stopAfterFirstVerifiedObstaclePass) {
    const bool willSkipAfterBackoff = decision_.wallRecoveryPhase != 0U &&
        wallRecoveryPoweredMs_ > 0U &&
        committedObstacle_ == target;
    const bool skipStillActive = recoverySkippedObstacle_ == target &&
        !elapsedAtLeast(inputs.nowMs, recoverySkippedObstacleAtMs_,
                        kRecoveryObstacleSkipMs);
    if (willSkipAfterBackoff || skipStillActive) return false;
  }
  return blobUsable(target == ObstacleTarget::RedPassRight
      ? inputs.vision.red : inputs.vision.green);
}

bool Controller::skippedPillarStraightDepartureReady(
    const Inputs& inputs, bool continuing) const {
  if (!config_.preferForwardObstacleProgress ||
      !config_.guardedObstaclePassEnabled ||
      config_.stopAfterFirstVerifiedObstaclePass ||
      decision_.state != RunState::Running ||
      decision_.direction == CourseDirection::Unknown) return false;
  const bool willSkip = decision_.wallRecoveryPhase != 0U &&
      wallRecoveryPoweredMs_ > 0U &&
      committedObstacle_ != ObstacleTarget::None;
  const bool recentSkip = recoverySkippedObstacle_ != ObstacleTarget::None &&
      !elapsedAtLeast(inputs.nowMs, recoverySkippedObstacleAtMs_,
                      kRecoveryObstacleSkipMs);
  const bool continuingKnownDeparture = continuing && rearStraightDepartureActive_;
  if ((!willSkip && !recentSkip && !continuingKnownDeparture) ||
      (rearDepartureConstraintMask_ == 0U && !rearRayAtOrBelow(inputs, 350U)) ||
      !visionFreshAndValid(inputs) ||
      recoveryPillarRouteUsable(inputs, ObstacleTarget::RedPassRight) ||
      recoveryPillarRouteUsable(inputs, ObstacleTarget::GreenPassLeft) ||
      absoluteValue(unwrappedHeadingDegrees_ -
                    straightHeadingReferenceDegrees_) > 8.0F ||
      !wallRecoveryInputsReady(inputs, false, false) ||
      inputs.front.rangeMm <= (continuing ? 350U : 400U)) return false;
  const uint8_t leadingIndices[] = {FrontLeft, FrontRight};
  for (uint8_t index : leadingIndices) {
    const RangeReading& ray = inputs.sides[index];
    if (!((sideReadingUsable(ray, inputs.nowMs) && ray.rangeMm > 200U) ||
          sideReadingIsOpen(ray, inputs.nowMs))) return false;
  }
  const uint8_t rearIndices[] = {BackLeft, BackRight};
  for (uint8_t index : rearIndices) {
    if (!sideReadingUsable(inputs.sides[index], inputs.nowMs) &&
        !sideReadingIsOpen(inputs.sides[index], inputs.nowMs)) return false;
  }
  return true;
}

bool Controller::rearLimitedForwardDepartureReady(const Inputs& inputs) const {
  const bool redVisible = recoveryPillarRouteUsable(inputs, ObstacleTarget::RedPassRight);
  const bool greenVisible = recoveryPillarRouteUsable(inputs, ObstacleTarget::GreenPassLeft);
  const bool freshPillarRoute = redVisible || greenVisible;
  const bool skippedStraightRoute = skippedPillarStraightDepartureReady(inputs);
  const uint16_t frontDepartureMm =
      freshPillarRoute || skippedStraightRoute || decision_.state == RunState::Cornering
          ? 400U : 650U;
  if (!wallRecoveryInputsReady(inputs, false, false) ||
      inputs.front.rangeMm < frontDepartureMm) return false;
  const uint8_t rearIndices[] = {BackLeft, BackRight};
  for (const uint8_t index : rearIndices) {
    if (!sideReadingUsable(inputs.sides[index], inputs.nowMs) &&
        !sideReadingIsOpen(inputs.sides[index], inputs.nowMs)) return false;
  }
  const uint8_t frontIndices[] = {FrontLeft, FrontRight};
  for (const uint8_t index : frontIndices) {
    const RangeReading& side = inputs.sides[index];
    if (!((sideReadingUsable(side, inputs.nowMs) && side.rangeMm > 200U) ||
          sideReadingIsOpen(side, inputs.nowMs))) return false;
  }
  float steering = decision_.state == RunState::Cornering
      ? (decision_.direction == CourseDirection::Clockwise ? 1.0F : -1.0F)
      : -(unwrappedHeadingDegrees_ -
          (decision_.state == RunState::DirectionProbe
               ? runStartHeadingDegrees_ : straightHeadingReferenceDegrees_)) *
            static_cast<float>(config_.clockwiseYawSign);
  if (decision_.state != RunState::Cornering && (redVisible || greenVisible)) {
    const bool retainRed = committedObstacle_ == ObstacleTarget::RedPassRight && redVisible;
    const bool retainGreen = committedObstacle_ == ObstacleTarget::GreenPassLeft && greenVisible;
    const bool chooseRed = retainRed || (!retainGreen && redVisible &&
        (!greenVisible || blobPriority(inputs.vision.red) >= blobPriority(inputs.vision.green)));
    // A close target's required passing side determines the departure route.
    // The final command is independently checked against these same rear rays.
    steering = chooseRed ? 1.0F : -1.0F;
  }
  return skippedStraightRoute || rearSweepAllowsSteering(inputs, steering);
}

bool Controller::wallRecoveryHazardsClear(
    const Inputs& inputs, bool requireCenterClearance) const {
  if (requireCenterClearance && (decision_.wallRecoveryHazards & 1U) != 0U &&
      (!frontReadingUsable(inputs.front, inputs.nowMs) ||
       inputs.front.rangeMm < 650U)) return false;
  const uint8_t indices[] = {FrontLeft, FrontRight};
  for (uint8_t i = 0; i < 2U; ++i) {
    if ((decision_.wallRecoveryHazards & (2U << i)) == 0U) continue;
    const RangeReading& side = inputs.sides[indices[i]];
    if (!((sideReadingUsable(side, inputs.nowMs) && side.rangeMm > 300U) ||
          sideReadingIsOpen(side, inputs.nowMs))) return false;
  }
  return true;
}

bool Controller::wallRecoveryPillarClear(const Inputs& inputs) const {
  if (committedObstacle_ != ObstacleTarget::None &&
      obstaclePeakBottomY_ >= config_.obstacleCloseBottomY) return false;
  if (obstacleCenteredEscapeActive_ && !obstacleCenteredEscapeRecovered_) return false;
  return !((blobUsable(inputs.vision.red) &&
            inputs.vision.red.bottomY >= config_.obstacleCloseBottomY) ||
           (blobUsable(inputs.vision.green) &&
            inputs.vision.green.bottomY >= config_.obstacleCloseBottomY));
}

bool Controller::handleWallRecovery(const Inputs& inputs, uint32_t safetyFaults) {
  if (!config_.wallRecoveryEnabled) return false;
  const bool retryHealth = config_.preferForwardObstacleProgress &&
      config_.guardedObstaclePassEnabled &&
      !config_.stopAfterFirstVerifiedObstaclePass;
  const bool cornerRecovery = decision_.state == RunState::Cornering;
  if (decision_.wallRecoveryPhase == 0U) {
    const float cardinalReference = decision_.state == RunState::DirectionProbe
        ? runStartHeadingDegrees_ : straightHeadingReferenceDegrees_;
    const float currentHeading = unwrappedHeadingDegrees_ +
        shortestAngleDeltaDegrees(lastRawHeadingDegrees_, inputs.imu.yawDegrees);
    const bool displacedApproach = !cornerRecovery &&
        (committedObstacle_ != ObstacleTarget::None ||
         !wallRecoveryPillarClear(inputs) ||
         absoluteValue(currentHeading - cardinalReference) > 8.0F);
    const uint16_t triggerMm = displacedApproach ? 350U : 300U;
    const uint8_t sideHazards = leadingObstacleHazards(inputs);
    const bool frontHazard = frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm <= triggerMm;
    if ((decision_.state != RunState::DirectionProbe &&
         decision_.state != RunState::Running &&
         decision_.state != RunState::FinishDelay && !cornerRecovery) ||
        (!frontHazard && sideHazards == 0U)) return false;
    decision_.wallRecoveryHazards = sideHazards | (frontHazard ? 1U : 0U);
    const float reference = cornerRecovery && sideHazards == 0U
        ? cornerStartHeadingDegrees_ :
        currentHeading;
    if ((!retryHealth &&
         !wallRecoveryInputsReady(inputs, sideHazards != 0U, false)) ||
        wallRecoveryAttempts_ >= 3U) {
      latchFault(FaultFrontTooClose);
      return true;
    }
    ++wallRecoveryAttempts_;
    wallRecoveryHeadingReferenceDegrees_ = reference;
    wallRecoveryStartedAtMs_ = inputs.nowMs;
    wallRecoveryPhaseStartedAtMs_ = inputs.nowMs;
    wallRecoveryUpdatedAtMs_ = inputs.nowMs;
    wallRecoveryPoweredMs_ = 0;
    wallRecoveryHealthHoldActive_ = false;
    wallRecoveryClearSinceMs_ = 0;
    wallRecoveryClearLastEvidenceMs_ = 0;
    wallRecoveryClearLeftSampleMs_ = 0;
    wallRecoveryClearRightSampleMs_ = 0;
    wallRecoveryClearSamples_ = 0;
    wallRecoveryForwardSinceMs_ = 0;
    wallRecoveryForwardEvidenceMs_ = 0;
    wallRecoveryForwardFrontSampleMs_ = 0;
    wallRecoveryForwardLeftSampleMs_ = 0;
    wallRecoveryForwardRightSampleMs_ = 0;
    wallRecoveryForwardSamples_ = 0;
    decision_.wallRecoveryExitReason = 0;
    decision_.wallRecoveryPhase = 1;
  }
  const bool sideRecovery = (decision_.wallRecoveryHazards & 6U) != 0U;
  const bool opticalFrontAllowed = sideRecovery ||
      (decision_.wallRecoveryPhase == 3U &&
       decision_.wallRecoveryExitReason >= 2U);
  const uint32_t reverseExemptions = FaultFrontTooClose |
      (opticalFrontAllowed ? uint32_t(FaultFrontTofUnavailable |
                              FaultSideGeometryUnavailable) : 0U);
  const uint32_t elapsed = inputs.nowMs - wallRecoveryUpdatedAtMs_;
  wallRecoveryUpdatedAtMs_ = inputs.nowMs;
  if (wallRecoveryHealthHoldActive_) {
    // The three-second maneuver bound counts usable recovery time. Sensor
    // waiting is stopped and remains bounded by the absolute whole-run limit.
    wallRecoveryStartedAtMs_ += elapsed;
  }
  if (decision_.wallRecoveryPhase == 2U &&
      decision_.driveDirection == DriveDirection::Reverse &&
      (inputs.driveFeedbackAvailable ? inputs.appliedDrivePermille > 0U
                                    : decision_.speedPermille > 0U)) {
    wallRecoveryPoweredMs_ += elapsed;
  }
  if (elapsedAtLeast(inputs.nowMs, runStartedAtMs_, config_.maxRunMs)) {
    latchFault(FaultRunTimeout);
    return true;
  }
  const auto holdForRecoveryHealth = [&](uint32_t faults) {
    wallRecoveryHealthHoldActive_ = true;
    decision_.faultFlags = faults;
    decision_.motionAllowed = false;
    decision_.speedPermille = 0;
    decision_.steeringPermille = 0;
    decision_.driveDirection = driveDirectionInterlock_.update(
        DriveDirection::Stopped, inputs.nowMs, config_.parkingReverseNeutralMs);
    lastCommandedSpeedPermille_ = 0;
    lastSpeedCommandAtMs_ = inputs.nowMs;
    // Unknown measurements break consecutive clearance proof. Reacquire it
    // from distinct fresh samples after the observation stream recovers.
    wallRecoveryClearSamples_ = 0;
    wallRecoveryClearSinceMs_ = 0;
    wallRecoveryClearLastEvidenceMs_ = 0;
    wallRecoveryClearLeftSampleMs_ = 0;
    wallRecoveryClearRightSampleMs_ = 0;
    wallRecoveryForwardSamples_ = 0;
    wallRecoveryForwardSinceMs_ = 0;
    wallRecoveryForwardEvidenceMs_ = 0;
    wallRecoveryForwardFrontSampleMs_ = 0;
    wallRecoveryForwardLeftSampleMs_ = 0;
    wallRecoveryForwardRightSampleMs_ = 0;
    return true;
  };
  const uint32_t transientHealthFaults =
      FaultImuUnavailable | FaultImuStale | FaultImuUncalibrated |
      FaultFrontTofUnavailable | FaultFrontTofStale | FaultSideTofUnavailable |
      FaultSideTofStale | FaultVisionUnavailable | FaultVisionStale |
      FaultSideGeometryUnavailable;
  const auto rearObserved = [&](uint8_t index) {
    const RangeReading& rear = inputs.sides[index];
    return sideReadingUsable(rear, inputs.nowMs) ||
        sideReadingIsOpen(rear, inputs.nowMs);
  };
  const uint32_t nonExemptFaults = safetyFaults & ~reverseExemptions;
  if (nonExemptFaults != 0U ||
      !wallRecoveryInputsReady(inputs, opticalFrontAllowed, false) ||
      (retryHealth && (!rearObserved(BackLeft) || !rearObserved(BackRight)))) {
    if (retryHealth && (nonExemptFaults & ~transientHealthFaults) == 0U) {
      return holdForRecoveryHealth(nonExemptFaults != 0U
          ? nonExemptFaults : uint32_t(FaultSideGeometryUnavailable));
    }
    latchFault((safetyFaults & ~reverseExemptions) != 0U
                   ? safetyFaults & ~reverseExemptions
                   : uint32_t(FaultSideGeometryUnavailable));
    return true;
  }
  wallRecoveryHealthHoldActive_ = false;
  if (!updateHeading(inputs)) {
    latchFault(FaultHeadingJump);
    return true;
  }
  if (absoluteValue(unwrappedHeadingDegrees_ -
                    wallRecoveryHeadingReferenceDegrees_) >
          (cornerRecovery && !sideRecovery ? 125.0F :
           config_.preferForwardObstacleProgress &&
                   !config_.stopAfterFirstVerifiedObstaclePass &&
                   decision_.wallRecoveryPhase == 1U ? 30.0F : 12.0F)) {
    latchFault(FaultFrontTooClose);
    return true;
  }
  // Count the preceding powered interval before braking. A numeric rear
  // boundary is a travel limit, not a missing sensor or permission to coast
  // all the way down to the hard 200 mm limit.
  if ((decision_.wallRecoveryPhase == 2U && rearRayAtOrBelow(inputs, 350U)) ||
      (decision_.wallRecoveryPhase == 1U && rearRayAtOrBelow(inputs, 200U))) {
    decision_.wallRecoveryPhase = 3;
    decision_.wallRecoveryExitReason = 2;
    wallRecoveryPhaseStartedAtMs_ = inputs.nowMs;
  }
  if (decision_.wallRecoveryPhase == 1U &&
      elapsedAtLeast(inputs.nowMs, wallRecoveryPhaseStartedAtMs_, 300U)) {
    if (config_.preferForwardObstacleProgress &&
        !config_.stopAfterFirstVerifiedObstaclePass &&
        (!cornerRecovery || sideRecovery)) {
      // Coast during neutral precedes reverse travel. Measure the straight
      // backout's 12-degree deviation from its actual settled departure yaw.
      wallRecoveryHeadingReferenceDegrees_ = unwrappedHeadingDegrees_;
    }
    decision_.wallRecoveryPhase = 2;
    wallRecoveryPhaseStartedAtMs_ = inputs.nowMs;
  }
  const bool observedClear = wallRecoveryHazardsClear(inputs);
  const bool observedSidesClear = wallRecoveryHazardsClear(inputs, false);
  if (sideRecovery && decision_.wallRecoveryPhase >= 2U) {
    if (!observedSidesClear) {
      wallRecoveryClearSamples_ = 0;
      wallRecoveryClearSinceMs_ = 0;
    } else {
      const bool distinctLeft = (decision_.wallRecoveryHazards & 2U) == 0U ||
          inputs.sides[FrontLeft].updatedAtMs != wallRecoveryClearLeftSampleMs_;
      const bool distinctRight = (decision_.wallRecoveryHazards & 4U) == 0U ||
          inputs.sides[FrontRight].updatedAtMs != wallRecoveryClearRightSampleMs_;
      if (distinctLeft && distinctRight) {
        if (wallRecoveryClearSamples_ == 0U) {
          wallRecoveryClearSinceMs_ = inputs.nowMs;
        }
        wallRecoveryClearLeftSampleMs_ = inputs.sides[FrontLeft].updatedAtMs;
        wallRecoveryClearRightSampleMs_ = inputs.sides[FrontRight].updatedAtMs;
        wallRecoveryClearLastEvidenceMs_ = inputs.nowMs;
        if (wallRecoveryClearSamples_ < 255U) ++wallRecoveryClearSamples_;
      }
    }
  }
  const bool sideProofReady = observedSidesClear &&
      (!sideRecovery || (wallRecoveryClearSamples_ >= 3U &&
        elapsedAtLeast(wallRecoveryClearLastEvidenceMs_,
                       wallRecoveryClearSinceMs_, 100U)));
  const bool recoveryClear = observedClear && sideProofReady &&
      (!sideRecovery || wallRecoveryPoweredMs_ >= 250U);
  if (decision_.wallRecoveryPhase == 2U &&
      (recoveryClear || wallRecoveryPoweredMs_ >= 1500U)) {
    decision_.wallRecoveryPhase = 3;
    decision_.wallRecoveryExitReason = recoveryClear ? 1U : 3U;
    wallRecoveryPhaseStartedAtMs_ = inputs.nowMs;
  }
  const bool rearLimited = decision_.wallRecoveryExitReason == 2U;
  const bool plannedTimeLimitedRetry = decision_.wallRecoveryExitReason == 3U &&
      (cornerRecovery ||
       skippedPillarStraightDepartureReady(inputs) ||
       recoveryPillarRouteUsable(inputs, ObstacleTarget::RedPassRight) ||
       recoveryPillarRouteUsable(inputs, ObstacleTarget::GreenPassLeft));
  const bool forwardRouteReady = rearLimitedForwardDepartureReady(inputs);
  if (rearLimited && decision_.wallRecoveryPhase == 3U) {
    if (!forwardRouteReady) {
      wallRecoveryForwardSamples_ = 0;
      wallRecoveryForwardSinceMs_ = 0;
    } else if (inputs.front.updatedAtMs != wallRecoveryForwardFrontSampleMs_ &&
               inputs.sides[FrontLeft].updatedAtMs != wallRecoveryForwardLeftSampleMs_ &&
               inputs.sides[FrontRight].updatedAtMs != wallRecoveryForwardRightSampleMs_) {
      if (wallRecoveryForwardSamples_ == 0U) wallRecoveryForwardSinceMs_ = inputs.nowMs;
      wallRecoveryForwardFrontSampleMs_ = inputs.front.updatedAtMs;
      wallRecoveryForwardLeftSampleMs_ = inputs.sides[FrontLeft].updatedAtMs;
      wallRecoveryForwardRightSampleMs_ = inputs.sides[FrontRight].updatedAtMs;
      wallRecoveryForwardEvidenceMs_ = inputs.nowMs;
      if (wallRecoveryForwardSamples_ < 255U) ++wallRecoveryForwardSamples_;
    }
  }
  const bool forwardRouteConfirmed = forwardRouteReady &&
      wallRecoveryForwardSamples_ >= 3U &&
      elapsedAtLeast(wallRecoveryForwardEvidenceMs_, wallRecoveryForwardSinceMs_, 100U);
  const bool partialDepartureReady = sideProofReady &&
      ((rearLimited && forwardRouteConfirmed) ||
       (plannedTimeLimitedRetry && forwardRouteReady && wallRecoveryPoweredMs_ >= 250U));
  const bool hardRearDepartureReady = !rearRayAtOrBelow(inputs, 200U) ||
      rearLimitedForwardDepartureReady(inputs);
  const bool mayExit = (rearLimited ? partialDepartureReady
                                  : recoveryClear || partialDepartureReady) &&
      hardRearDepartureReady;
  if (decision_.wallRecoveryPhase == 3U &&
      elapsedAtLeast(inputs.nowMs, wallRecoveryPhaseStartedAtMs_, 300U) &&
      !mayExit && !rearLimited && !plannedTimeLimitedRetry) {
      latchFault(FaultFrontTooClose);
      return true;
  }
  if (decision_.wallRecoveryPhase == 3U &&
      elapsedAtLeast(inputs.nowMs, wallRecoveryPhaseStartedAtMs_, 300U) && mayExit) {
    rearDepartureConstraintMask_ = 0;
    rearStraightDepartureActive_ = false;
    rearStraightDepartureExhausted_ = false;
    rearStraightDeparturePoweredMs_ = 0;
    if (sideReadingUsable(inputs.sides[BackLeft], inputs.nowMs) &&
        inputs.sides[BackLeft].rangeMm <= 350U) rearDepartureConstraintMask_ |= 1U;
    if (sideReadingUsable(inputs.sides[BackRight], inputs.nowMs) &&
        inputs.sides[BackRight].rangeMm <= 350U) rearDepartureConstraintMask_ |= 2U;
    decision_.wallRecoveryPhase = 0;
    decision_.wallRecoveryHazards = 0;
    decision_.wallRecoveryExitReason = 0;
    // Reverse invalidates the old pass sequence. Progress mode skips the known
    // failed target briefly after actual backoff; strict mode retries its fresh
    // box. Neither path awards passage or changes the corner/lap counters.
    if (committedObstacle_ != ObstacleTarget::None &&
        visionFreshAndValid(inputs)) {
      const ObstacleTarget previousTarget = committedObstacle_;
      const BlobReading& retainedBlob =
          previousTarget == ObstacleTarget::RedPassRight
              ? inputs.vision.red : inputs.vision.green;
      const bool skipRecoveredTarget = config_.preferForwardObstacleProgress &&
          !config_.stopAfterFirstVerifiedObstaclePass &&
          wallRecoveryPoweredMs_ > 0U;
      if (skipRecoveredTarget) {
        clearCommittedObstacle(inputs.nowMs, false);
        recoverySkippedObstacle_ = previousTarget;
        recoverySkippedObstacleAtMs_ = inputs.nowMs;
        obstaclePassCornerRecoveryPending_ = true;
      } else if (blobUsable(retainedBlob)) {
        commitObstacle(previousTarget,
                       obstacleOffsetForBlob(previousTarget, retainedBlob),
                       retainedBlob.centerX, retainedBlob.confidence,
                       retainedBlob.bottomY, inputs.nowMs);
      } else {
        clearCommittedObstacle(inputs.nowMs, false);
        obstaclePassCornerRecoveryPending_ = false;
      }
      decision_.obstacle = committedObstacle_;
      decision_.targetLateralOffsetMm = committedObstacleOffsetMm_;
    }
    directionProbeTimerStartedAtMs_ = inputs.nowMs;
    openingConfirmed(CourseDirection::Unknown, inputs);
    directionDecisionHoldActive_ = false;
    directionDecisionHoldSinceMs_ = 0;
    if (decision_.state == RunState::Running) {
      cornerDetectionArmed_ = committedObstacle_ == ObstacleTarget::None;
      decision_.cornerDetectionArmed = cornerDetectionArmed_;
    } else if (cornerRecovery) {
      // Backing out is authorized reverse progress, not a new corner. Keep
      // the cardinal target/count while restarting the physical-turn bound.
      cornerStartHeadingDegrees_ = unwrappedHeadingDegrees_;
      cornerStartedAtMs_ = inputs.nowMs;
      cornerReadySinceMs_ = 0;
    } else if (decision_.state == RunState::FinishDelay) {
      finishEnteredAtMs_ = inputs.nowMs;
      finishReadySinceMs_ = 0;
    }
  }
  if (decision_.wallRecoveryPhase != 0U &&
      elapsedAtLeast(inputs.nowMs, wallRecoveryStartedAtMs_, 3000U)) {
    latchFault(FaultFrontTooClose);
    return true;
  }
  if (decision_.wallRecoveryPhase == 2U &&
      !wallRecoveryInputsReady(inputs, opticalFrontAllowed, true)) {
    if (retryHealth) return holdForRecoveryHealth(FaultSideGeometryUnavailable);
    latchFault(FaultSideGeometryUnavailable);
    return true;
  }
  const DriveDirection requested = decision_.wallRecoveryPhase == 2U
                                       ? DriveDirection::Reverse
                                       : DriveDirection::Stopped;
  decision_.driveDirection = driveDirectionInterlock_.update(
      requested, inputs.nowMs, config_.parkingReverseNeutralMs);
  decision_.speedPermille = decision_.driveDirection == DriveDirection::Reverse
                                ? 300U : 0U;
  decision_.motionAllowed = decision_.speedPermille > 0U;
  decision_.steeringPermille = 0;
  if (cornerRecovery && !sideRecovery && (decision_.wallRecoveryPhase == 1U ||
                         decision_.wallRecoveryPhase == 2U)) {
    const float yawSign = decision_.direction == CourseDirection::Clockwise
        ? static_cast<float>(config_.clockwiseYawSign)
        : -static_cast<float>(config_.clockwiseYawSign);
    const float progress = (unwrappedHeadingDegrees_ -
        wallRecoveryHeadingReferenceDegrees_) * yawSign;
    if (progress > 4.0F) {
      decision_.steeringPermille =
          decision_.direction == CourseDirection::Clockwise
              ? config_.cornerSteeringPermille : -config_.cornerSteeringPermille;
    }
  }
  decision_.faultFlags = FaultNone;
  decision_.unwrappedHeadingDegrees = unwrappedHeadingDegrees_;
  lastCommandedSpeedPermille_ = decision_.speedPermille;
  lastSpeedCommandAtMs_ = inputs.nowMs;
  return true;
}

Decision Controller::update(const Inputs& inputs) {
  updateCenteredEscapeDriveTime(inputs);
  const bool startEdge = updateButton(inputs);

  WallGuidance wall;
  const uint32_t currentSafetyFaults = evaluateSafety(inputs, wall);
  // A legal Open start may have no numeric side target or a target beyond the
  // opening-reference range. Learn a missing close-wall baseline while the
  // direction probe is still safely outside its stop boundary. This prevents
  // a permissive start from turning into a direction timeout without ever
  // treating a 5 m sample as a local wall reference.
  const bool probeFrontAllowsBaseline =
      (frontReadingUsable(inputs.front, inputs.nowMs) &&
       inputs.front.rangeMm > config_.directionProbeFrontMaximumMm) ||
      (sideReadingIsOpen(inputs.front, inputs.nowMs) && wall.bothPairs);
  if (decision_.state == RunState::DirectionProbe &&
      currentSafetyFaults == FaultNone &&
      committedObstacle_ == ObstacleTarget::None &&
      (!config_.obstacleAvoidanceEnabled ||
       !visionFreshAndValid(inputs) ||
       (!blobUsable(inputs.vision.red) &&
        !blobUsable(inputs.vision.green))) &&
      probeFrontAllowsBaseline) {
    captureOpeningBaselines(inputs, false);
  }
  // A corner may be accepted with only one usable side while the opposite
  // forward ray legitimately reports optical no-target. Refill only while
  // corner detection is disarmed and the front-clear re-arm is settling.
  // Once armed, the first new side change may already be the next opening.
  if (decision_.state == RunState::Running &&
      currentSafetyFaults == FaultNone &&
      committedObstacle_ == ObstacleTarget::None &&
      obstacleRecenterStartOffsetMm_ == 0 &&
      !cornerDetectionArmed_ &&
      !(obstaclePassCornerRecoveryPending_ &&
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm < config_.cornerRearmFrontClearMm)) {
    // Corner-exit rays are often still looking obliquely through the previous
    // opening. Refresh on every disarmed straight frame, then freeze the final
    // settled values on the exact frame the clear-front rearm becomes stable.
    captureOpeningBaselines(inputs, true);
  }
  const OpeningEvidence opening = computeOpeningEvidence(inputs);
  decision_.wallLateralOffsetMm = wall.lateralOffsetMm;
  decision_.wallAngleErrorMm = wall.angleErrorMm;
  decision_.outerWallDistanceMm = wall.outerWallDistanceMm;
  decision_.outerWallUsable = wall.outerWallUsable;
  decision_.outerWallPair = wall.outerWallPair;
  decision_.cornerDetectionArmed = cornerDetectionArmed_;
  decision_.leftOpeningMm = opening.leftDeltaMm;
  decision_.rightOpeningMm = opening.rightDeltaMm;

  if (decision_.state == RunState::FaultLatched) {
    decision_.faultFlags = latchedFaultFlags_;
    makeMotionDecision(inputs, wall);
    return decision_;
  }
  if (decision_.state == RunState::Finished) {
    decision_.faultFlags = FaultNone;
    makeMotionDecision(inputs, wall);
    return decision_;
  }

  if (decision_.state == RunState::WaitingForHealth) {
    decision_.faultFlags = currentSafetyFaults;
    const bool reverseStartReady = config_.wallRecoveryEnabled &&
        (currentSafetyFaults & ~uint32_t(FaultFrontTooClose)) == 0U &&
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm <= 300U && wallRecoveryInputsReady(inputs) &&
        wallRecoveryPillarClear(inputs);
    if (currentSafetyFaults == FaultNone || reverseStartReady) {
      decision_.state = RunState::WaitingForStart;
    }
    if (reverseStartReady && startEdge) {
      startRun(inputs);
      handleWallRecovery(inputs, currentSafetyFaults);
      return decision_;
    }
    makeMotionDecision(inputs, wall);
    return decision_;
  }

  if (decision_.state == RunState::WaitingForStart) {
    decision_.faultFlags = currentSafetyFaults;
    const bool reverseStartReady = config_.wallRecoveryEnabled &&
        (currentSafetyFaults & ~uint32_t(FaultFrontTooClose)) == 0U &&
        frontReadingUsable(inputs.front, inputs.nowMs) &&
        inputs.front.rangeMm <= 300U && wallRecoveryInputsReady(inputs) &&
        wallRecoveryPillarClear(inputs);
    if (currentSafetyFaults != FaultNone && !reverseStartReady) {
      decision_.state = RunState::WaitingForHealth;
    } else if (startEdge) {
      startRun(inputs);
      decision_.faultFlags = FaultNone;
      if (reverseStartReady) {
        handleWallRecovery(inputs, currentSafetyFaults);
        return decision_;
      }
    }
    makeMotionDecision(inputs, wall);
    return decision_;
  }

  // Recovery-enabled runs wait at zero output for transient sensing loss.
  // Terminal faults still require reset; the button cannot restart them.
  if (config_.wallRecoveryEnabled &&
      elapsedAtLeast(inputs.nowMs, runStartedAtMs_, config_.maxRunMs)) {
    latchFault(FaultRunTimeout);
    return decision_;
  }
  const uint32_t transientHealthFaults =
      FaultImuUnavailable | FaultImuStale | FaultImuUncalibrated |
      FaultFrontTofUnavailable | FaultFrontTofStale | FaultSideTofUnavailable |
      FaultSideTofStale | FaultVisionUnavailable | FaultVisionStale |
      FaultSideGeometryUnavailable;
  // A side-triggered reverse has independent rear guidance. Handle it before
  // the forward-only optical-front grace hold, while retaining real sensor
  // freshness/electrical checks inside the recovery input gate.
  const uint8_t leadingHazards = leadingObstacleHazards(inputs);
  if (handleForwardSideEscape(inputs, currentSafetyFaults)) return decision_;
  if (config_.wallRecoveryEnabled &&
      (leadingHazards != 0U || (decision_.wallRecoveryHazards & 6U) != 0U ||
       (config_.preferForwardObstacleProgress &&
        config_.guardedObstaclePassEnabled &&
        !config_.stopAfterFirstVerifiedObstaclePass &&
        decision_.wallRecoveryPhase != 0U) ||
       (decision_.wallRecoveryPhase == 3U &&
        decision_.wallRecoveryExitReason >= 2U)) &&
      handleWallRecovery(inputs, currentSafetyFaults)) return decision_;
  if (leadingHazards != 0U) {
    makeMotionDecision(inputs, wall);
    return decision_;
  }
  if (config_.wallRecoveryEnabled && currentSafetyFaults != FaultNone &&
      (currentSafetyFaults & ~transientHealthFaults) == 0U) {
    if (decision_.wallRecoveryPhase != 0U &&
        elapsedAtLeast(inputs.nowMs, wallRecoveryStartedAtMs_, 3000U)) {
      latchFault(FaultFrontTooClose);
      return decision_;
    }
    decision_.faultFlags = currentSafetyFaults;
    decision_.motionAllowed = false;
    decision_.speedPermille = 0;
    decision_.steeringPermille = 0;
    decision_.driveDirection = driveDirectionInterlock_.update(
        DriveDirection::Stopped, inputs.nowMs, config_.parkingReverseNeutralMs);
    lastCommandedSpeedPermille_ = 0;
    lastSpeedCommandAtMs_ = inputs.nowMs;
    wallRecoveryUpdatedAtMs_ = inputs.nowMs;
    obstacleAbsentSinceMs_ = 0;
    obstacleClearSinceMs_ = 0;
    obstacleBodyClearStartedAtMs_ = 0;
    return decision_;
  }
  if (handleWallRecovery(inputs, currentSafetyFaults)) return decision_;
  if (currentSafetyFaults != FaultNone) {
    latchFault(currentSafetyFaults);
    return decision_;
  }
  decision_.faultFlags = FaultNone;
  if (elapsedAtLeast(inputs.nowMs, runStartedAtMs_, config_.maxRunMs)) {
    latchFault(FaultRunTimeout);
    return decision_;
  }
  if (!updateHeading(inputs)) {
    latchFault(FaultHeadingJump);
    return decision_;
  }
  if (sideElectricalLossActive_) {
    // Hold the current navigation state as well as the motor during the short
    // electrical recovery window. A missing ray must not count a corner or
    // change obstacle commitment while the vehicle is stopped.
    obstacleAbsentSinceMs_ = 0;
    obstacleClearSinceMs_ = 0;
    obstacleBodyClearStartedAtMs_ = 0;
    cornerReadySinceMs_ = 0;
    finishReadySinceMs_ = 0;
    cornerRearmClearSinceMs_ = 0;
    openingCandidate_ = CourseDirection::Unknown;
    openingCandidateSinceMs_ = 0;
    directionDecisionHoldSinceMs_ = 0;
    directionDecisionHoldActive_ = false;
    if (decision_.state == RunState::DirectionProbe) {
      directionProbeTimerStartedAtMs_ = inputs.nowMs;
    }
    decision_.unwrappedHeadingDegrees = unwrappedHeadingDegrees_;
    makeMotionDecision(inputs, wall);
    return decision_;
  }
  if (config_.requireVisionForMotion && visionLossActive_) {
    // Required-camera grace is a stopped recovery window, not permission to
    // advance direction, corner, or obstacle state using missing frames.
    obstacleAbsentSinceMs_ = 0;
    obstacleClearSinceMs_ = 0;
    obstacleBodyClearStartedAtMs_ = 0;
    cornerReadySinceMs_ = 0;
    finishReadySinceMs_ = 0;
    cornerRearmClearSinceMs_ = 0;
    openingCandidate_ = CourseDirection::Unknown;
    openingCandidateSinceMs_ = 0;
    directionDecisionHoldSinceMs_ = 0;
    directionDecisionHoldActive_ = false;
    if (decision_.state == RunState::DirectionProbe) {
      directionProbeTimerStartedAtMs_ = inputs.nowMs;
    }
    decision_.unwrappedHeadingDegrees = unwrappedHeadingDegrees_;
    makeMotionDecision(inputs, wall);
    return decision_;
  }

  // Pillar seats are in straight sections. While a corner is still active,
  // retain any pass already in progress but do not accept a new target. On
  // the exact update that verifies the corner exit, navigation first clears
  // the old straight's target and changes state to Running; the same fresh
  // camera observation can then be committed to the new straight without
  // being immediately erased.
  if (decision_.state == RunState::Cornering) {
    updateNavigation(inputs, wall, opening);
    if (decision_.state != RunState::Cornering &&
        decision_.state != RunState::FaultLatched) {
      updateObstacle(inputs);
    }
  } else {
    updateObstacle(inputs);
    updateNavigation(inputs, wall, opening);
  }
  decision_.unwrappedHeadingDegrees = unwrappedHeadingDegrees_;
  decision_.cornerTargetHeadingDegrees = cornerTargetHeadingDegrees_;
  const bool reportHeadingOnlyProbeError =
      !config_.sideSensorPairsAreParallel &&
      decision_.state == RunState::DirectionProbe;
  if (decision_.direction == CourseDirection::Unknown &&
      !reportHeadingOnlyProbeError) {
    decision_.headingErrorDegrees = 0.0F;
  } else {
    const float reference = decision_.state == RunState::Cornering
                                ? cornerTargetHeadingDegrees_
                                : straightHeadingReferenceDegrees_;
    decision_.headingErrorDegrees = unwrappedHeadingDegrees_ - reference;
  }
  if (decision_.direction == CourseDirection::Unknown) {
    decision_.directedProgressDegrees = 0.0F;
  }

  if (decision_.state == RunState::FaultLatched) return decision_;
  makeMotionDecision(inputs, wall);
  return decision_;
}

}  // namespace autonomy

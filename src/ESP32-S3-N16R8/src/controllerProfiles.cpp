#include "controllerProfiles.h"

namespace autonomy {

const char* controllerSourceIdentity(ChallengeProfile profile) {
  return profile == ChallengeProfile::Obstacle
             ? "BUBBLEGUM AUTONOMOUS OBSTACLE 20260905-R42 FULL COURSE - PARKING DISABLED"
             : "BUBBLEGUM AUTONOMOUS OPEN 20260905-R18 - MOTION CAPABLE";
}

Config makeControllerConfig(ChallengeProfile profile) {
  Config config;
  config.sensorStaleMs = 300;
  config.visionStaleMs = 300;
  config.directionTimeoutMs = 12000;
  config.directionDecisionHoldMs = 300;
  config.straightFrontOpticalGraceMs = 1200;
  config.cornerFrontOpticalGraceMs = 2500;
  config.cornerMinimumDurationMs = 350;
  config.cornerTimeoutMs = 5000;
  config.cornerExitStableMs = 200;
  config.cornerRefractoryMs = 250;
  config.cornerRearmStableMs = 100;
  config.openPostCornerCounterSteerMs = 150;
  config.postCornerHeadingOnlyMs = 150;
  config.postCornerWallBlendMs = 250;
  config.straightSideGeometryGraceMs = 600;
  config.cornerSideGeometryGraceMs = 2500;
  config.imuAccuracyGraceMs = 500;
  config.finishDriveMs = 600;
  config.finishStableMs = 250;
  config.finishTimeoutMs = 1200;
  config.obstacleHoldMs = 700;
  config.maxRunMs = 170000;

  config.cornerOpeningDeltaMm = 180.0F;
  config.directionOpeningWinnerMarginMm = 100.0F;
  config.cornerCompletionDegrees = 85.0F;
  config.openCornerExitFallbackDegrees = 88.0F;
  config.openCornerExitMaximumDegrees = 100.0F;
  config.openCornerTaperStartDegrees = 70.0F;
  config.openCornerCounterSteerStartDegrees = 92.0F;
  config.openCornerCounterSteerRampDegrees = 8.0F;
  config.cornerMaximumDegrees = 125.0F;
  config.cornerExitMaximumWallAngleErrorMm = 70.0F;
  config.finishAlignmentDegrees = 12.0F;
  config.finishMaximumWallAngleErrorMm = 60.0F;
  config.maxHeadingStepDegrees = 45.0F;
  config.maxReverseProgressDegrees = 25.0F;
  // Measured on the assembled car on 2026-09-01: a physical clockwise turn
  // changed the logged XMS-A5 yaw by approximately +92 degrees.
  config.clockwiseYawSign = 1;
  config.minimumRunImuAccuracy = 1;

  const bool obstacle = profile == ChallengeProfile::Obstacle;
  config.obstacleAvoidanceEnabled = obstacle;
  config.requireVisionForMotion = obstacle;
  config.acceptFrontWrapTargetRange = !obstacle;
  // The assembled chassis determines the first direction from the symmetric
  // FL/FR forward-space view, not from a same-ray front/rear delta.
  config.directionProbeOpeningDeltaMm = 35.0F;
  config.openingMinimumDistinctSamples = 2;
  config.directionMinimumDistinctSamples = 3;
  // The assembled car has a narrow FL/FR front pair and a wider BL/BR rear
  // pair. All four rays point outward at roughly 45 degrees. Therefore a raw
  // FL-BL or FR-BR difference contains mounting offset and is not wall angle.
  // This is physical chassis geometry and cannot change with challenge mode.
  config.sideSensorPairsAreParallel = false;
  config.requireAllSideSensorsFresh = false;
  // R20 only carries parking data and safety interfaces. Both production
  // profiles must retain the demonstrated R19/R18 course behavior.
  config.parkingEnabled = false;
  config.parkingCalibrationValidated = false;
  config.parkingRearClearanceValidated = false;
  config.parkingBaySide = ParkingBaySide::Unknown;
  config.parkingReverseNeutralMs = kMinimumDriveReversalNeutralMs;

  config.frontEmergencyStopMm = 180;
  // R16 did not commit until front=638 mm. Begin evaluating at 1.0 m, stop as
  // soon as a candidate appears, and never authorize a late first turn once
  // only 600 mm remains.
  config.directionProbeFrontMaximumMm = 1000;
  config.directionProbeAbortFrontMm = 600;
  // At full Open speed the assembled car covered about 160 mm during the
  // 100 ms corner confirmation.  Starting that confirmation at 650 mm made
  // corner two begin at only 476 mm, even though the wall-following geometry
  // was centred and square.  Keep the confirmation (it rejects one-sample
  // noise), but give Open enough physical braking/steering margin.  Obstacle
  // keeps a slightly tighter trigger and still requires the expected side
  // opening plus no active pillar pass, so a pillar cannot become a corner.
  config.frontCornerTriggerMm = obstacle ? 900 : 1000;
  config.cornerRearmFrontClearMm = obstacle ? 1000 : 1200;
  config.openingReferenceMaximumMm = 1200;
  config.frontSlowDistanceMm = 1100;
  config.frontStoppingProxyMm = 920;
  config.nominalSideDistanceMm = 300;
  // R3 field evidence showed that chasing a 200 mm outside-wall offset on a
  // short full-speed straight pulled the nose into the wall before corner 2.
  // The first completed turn naturally exited near 350 mm; 300 mm keeps the
  // car in the safe outer half without commanding an aggressive lane change.
  config.openOuterWallTargetMm = 300;
  config.openOuterWallDeadbandMm = 20;
  config.openInnerWallMinimumMm = 100;
  config.obstacleMinimumConfidence = 400;
  config.obstacleMinimumBottomY = 3000;
  config.obstacleCloseBottomY = 4500;
  config.obstacleFullSteeringBottomY = 5500;
  config.obstacleSideClearanceGuardMm = 140;
  config.obstacleSideClearanceSpeedPermille = 300;

  config.lateralSteeringPerMm = obstacle ? 3.0F : 2.0F;
  config.wallAngleSteeringPerMm = 2.5F;
  config.obstacleImageSteeringPerUnit = 0.10F;
  config.obstacleCornerRearmHeadingDegrees = 8.0F;
  config.headingSteeringPerDegree = obstacle ? 8.0F : 14.0F;
  config.maximumHeadingCorrectionPermille = obstacle ? 260 : 520;
  // Open R16 is deliberately IMU-heading-only until the diagonal mounting
  // offsets are measured. Keep a hard zero authority here as a second guard.
  config.maximumWallCorrectionPermille = obstacle ? 500 : 0;
  config.openMaximumWallAngleCorrectionPermille = 120;
  config.openCornerMinimumApproachSteerPermille = 250;
  config.openCornerCounterSteerPermille = 450;
  config.obstacleDesiredImageOffset = 3500;
  // 190 mm car plus 50 mm pillar needs at least 120 mm centre separation even
  // with zero margin. Use 160 mm minimum and retain an absolute 300 mm lane
  // envelope for the official 1.0 m corridor.
  config.obstacleMinimumOffsetMm = 160;
  config.obstacleOffsetMm = 180;
  config.obstacleMaximumOffsetMm = 300;
  config.obstacleMaximumImageSteeringPermille = 500;
  config.obstacleSideClearanceSteeringPermille = 650;
  config.obstacleMinimumCommitMs = 300;
  config.obstacleMaximumCommitMs = 8000;
  config.obstacleBlobLossMs = 150;
  config.obstaclePreCloseLossClearMs = 500;
  config.obstaclePassClearConfirmationMs = 180;
  config.obstacleRecommitCooldownMs = 300;
  config.obstacleRecenterMs = 300;
  config.obstaclePassClearDeltaMm = 120;
  config.maximumSteeringPermille = 1000;

  if (obstacle) {
    // Fast enough to stay above the steering-load breakaway floor, but below
    // an empty Open straight while a pillar is localized and passed.
    config.directionProbeSpeedPermille = 400;
    config.cruiseSpeedPermille = 600;
    // Use the same motion-capable corner pair physically proven by Open R18.
    // Pillars exist only on straights and an Obstacle corner still requires
    // the expected side opening, so this does not turn a pillar into a corner.
    config.cornerSpeedPermille = 450;
    config.cornerDriveDelayMs = 0;
    config.cornerBlindSpeedPermille = 450;
    config.finishSpeedPermille = 320;
    config.accelerationPermillePerSecond = 1200;
    config.decelerationPermillePerSecond = 1800;
    config.cornerSteeringPermille = 760;
  } else {
    // R17 proved the first-direction decision and steering sign, but its
    // 300-permille corner request was reduced to 200 by the front envelope.
    // The assembled drivetrain stalled at full lock. Four earlier field turns
    // completed 90 degrees in 2.38-2.58 s at 450 with +/-760 steering, so use
    // that physically demonstrated pair and energize drive with steering.
    config.directionProbeSpeedPermille = 600;
    config.cruiseSpeedPermille = 1000;
    config.cornerSpeedPermille = 450;
    config.cornerDriveDelayMs = 0;
    config.cornerBlindSpeedPermille = 200;
    config.finishSpeedPermille = 450;
    config.accelerationPermillePerSecond = 1600;
    config.decelerationPermillePerSecond = 3000;
    // Keep the physically demonstrated steering command.
    config.cornerSteeringPermille = 760;
  }
  config.obstacleSpeedPermille = 400;
  config.visionUncertainSpeedPermille = 250;
  config.visionOverrunSpeedPermille = 250;
  config.fullSpeedFrameQuality = 45;
  config.obstacleUncertainSpeedPermille = 300;
  config.learnedObstacleSectionSpeedPermille = 400;
  // Obstacle steering can carry substantial load even before a corner. Keep
  // its non-emergency command above the measured low-speed stall region.
  config.minimumMovingSpeedPermille = obstacle ? 300 : 200;
  return config;
}

Config makeGuardedObstacleConfig() {
  Config config = makeControllerConfig(ChallengeProfile::Obstacle);
  config.guardedObstaclePassEnabled = true;
  config.wallRecoveryEnabled = true;
  config.preferForwardObstacleProgress = true;
  config.rememberObstacleSections = false;
  // R39 accelerated from the proven 450 corner command to 595 while no
  // pillar was accepted. Keep obstacle straights at the existing moving
  // approach/pass command, including the interval between detections.
  // Retain 450 at full corner lock and the 300 moving/reverse floor.
  config.cruiseSpeedPermille = 400;
  config.accelerationPermillePerSecond = 600;
  // R33 tracked a far red box across the upcoming corner and drove toward
  // the wall. Keep distant boxes out of the active straight-pillar pass.
  config.obstacleMinimumBottomY = 6000;
  config.obstacleCloseBottomY = 6500;
  config.obstacleFullSteeringBottomY = 7500;
  config.obstacleMaximumCommitMs = 12000;
  config.stopAfterFirstVerifiedObstaclePass = false;
  config.maxRunMs = 170000;
  config.acceptFrontWrapTargetRange = true;
  config.straightSideGeometryGraceMs = 2000;
  config.obstacleSideClearanceGuardMm = 200;
  config.obstacleDesiredImageOffset = 7000;
  config.obstacleCenteredEscapeInitialAbsX = 4000;
  config.obstacleCenteredEscapeHeadingDegrees = 22.0F;
  // The first live R31 turn needed about 2.8 s from camera commitment,
  // including startup servo settling. Leave room for the measured response.
  config.obstacleCenteredEscapeDeadlineMs = 4000;
  config.obstacleCenteredEscapeHeadingStableMs = 80;
  config.recoverCenteredEscapeBeforePass = true;
  config.obstaclePassProofMinimumBottomY = 7500;
  config.obstacleBodyClearOppositeSideGuardMm = 140;
  config.obstacleBodyClearFrontGuardMm = 320;
  config.obstacleSinglePassBodyClearMs = 500;
  config.parkingEnabled = false;
  return config;
}

}  // namespace autonomy

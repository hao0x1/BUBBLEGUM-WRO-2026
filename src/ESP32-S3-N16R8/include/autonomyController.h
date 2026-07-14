#pragma once

#include <stdint.h>

// This module contains decisions only.  It has no Arduino, GPIO, PWM, I2C, or
// UART dependencies, so the exact same code can run in host tests and on the
// ESP32.  Steering signs are explicit throughout: negative is left and
// positive is right.
namespace autonomy {

enum class RunState : uint8_t {
  WaitingForHealth = 0,
  WaitingForStart,
  DirectionProbe,
  Running,
  Cornering,
  FinishDelay,
  Finished,
  FaultLatched,
  // Reserved for the fail-closed parking state machine. Production profiles
  // keep parking disabled until its mounted-car calibration is complete.
  Parking,
};

enum class ParkingPhase : uint8_t {
  Disabled = 0,
  ExitAcquire,
  ExitArc,
  ExitCounterArc,
  Course,
  Search,
  Stage,
  NeutralBeforeReverse,
  ReverseEnter,
  ReverseAlign,
  Verify,
};

enum class ParkingBaySide : uint8_t {
  Unknown = 0,
  Left,
  Right,
};

enum class DriveDirection : int8_t {
  Reverse = -1,
  Stopped = 0,
  Forward = 1,
};

// VehicleActuators independently enforces the same minimum. Keeping this
// guard in the pure controller layer makes the neutral interval host-testable
// before any future parking state is allowed to request reverse torque.
constexpr uint32_t kMinimumDriveReversalNeutralMs = 300U;

class DriveDirectionInterlock {
 public:
  DriveDirection update(DriveDirection requested, uint32_t nowMs,
                        uint32_t neutralMs);
  void reset();

 private:
  DriveDirection lastNonzeroDirection_ = DriveDirection::Stopped;
  bool neutralActive_ = false;
  uint32_t neutralStartedAtMs_ = 0;
};

// The course direction is learned from which side wall opens at the first
// corner. The BNO yaw sign is an explicit mounted-car configuration: it must
// never be inferred from a turn that could have the servo direction reversed.
enum class CourseDirection : int8_t {
  Unknown = 0,
  Clockwise = -1,
  CounterClockwise = 1,
};

enum class ObstacleTarget : uint8_t {
  None = 0,
  RedPassRight,
  GreenPassLeft,
};

enum FaultFlag : uint32_t {
  FaultNone = 0,
  FaultImuUnavailable = 1UL << 0,
  FaultImuStale = 1UL << 1,
  FaultFrontTofUnavailable = 1UL << 2,
  FaultFrontTofStale = 1UL << 3,
  FaultSideTofUnavailable = 1UL << 4,
  FaultSideTofStale = 1UL << 5,
  FaultSideGeometryUnavailable = 1UL << 6,
  FaultFrontTooClose = 1UL << 7,
  FaultVisionUnavailable = 1UL << 8,
  FaultVisionStale = 1UL << 9,
  FaultDirectionTimeout = 1UL << 10,
  FaultHeadingJump = 1UL << 11,
  FaultWrongWay = 1UL << 12,
  FaultRunTimeout = 1UL << 13,
  FaultCornerTimeout = 1UL << 14,
  FaultCornerOvershoot = 1UL << 15,
  FaultDirectionAmbiguous = 1UL << 16,
  FaultConfiguration = 1UL << 17,
  FaultFinishTimeout = 1UL << 18,
  FaultImuUncalibrated = 1UL << 19,
  FaultObstaclePassTimeout = 1UL << 20,
};

enum SideSensorIndex : uint8_t {
  FrontLeft = 0,
  BackLeft = 1,
  FrontRight = 2,
  BackRight = 3,
  SideSensorCount = 4,
};

struct RangeReading {
  bool online = false;
  bool valid = false;
  uint16_t rangeMm = 0;
  uint32_t updatedAtMs = 0;
  // Keep electrical/bus health separate from optical validity. A healthy ToF
  // can legitimately report no target while looking through a corner.
  uint8_t rangeStatus = 0;
  uint8_t i2cStatus = 0;
};

struct ImuReading {
  bool online = false;
  bool valid = false;
  float yawDegrees = 0.0F;
  uint32_t updatedAtMs = 0;
  uint8_t accuracy = 0;  // sensor-reported gyro calibration, 0..3
};

struct BlobReading {
  bool present = false;
  int16_t centerX = 0;      // -10000 left .. +10000 right
  uint16_t bottomY = 0;     // 0 top .. 10000 bottom
  uint16_t width = 0;       // normalized to 0..10000
  uint16_t height = 0;      // normalized to 0..10000
  uint16_t confidence = 0;  // normalized to 0..1000
};

struct VisionReading {
  bool online = false;
  bool valid = false;
  bool pipelineOverrun = false;
  uint8_t frameQuality = 100;
  uint32_t updatedAtMs = 0;
  BlobReading red;
  BlobReading green;
  // These are one atomic image-ordered pair. "Left" and "right" do not
  // identify which physical side of the parking bay the car occupies.
  BlobReading magentaLeft;
  BlobReading magentaRight;
};

struct Inputs {
  uint32_t nowMs = 0;
  bool buttonPressed = false;
  bool driveFeedbackAvailable = false;
  uint16_t appliedDrivePermille = 0;
  // Board-level startup filtering may carry a recently confirmed clear front
  // reading across one brief optical no-target sample. This evidence is only
  // consumed by the stopped, zero-authority cross-pair arming gate.
  bool startupFrontClearRecent = false;
  ImuReading imu;
  RangeReading front;
  RangeReading sides[SideSensorCount];
  VisionReading vision;
};

struct Config {
  uint32_t buttonDebounceMs = 40;
  uint32_t sensorStaleMs = 200;
  uint32_t visionStaleMs = 150;
  // In Obstacle Challenge a missing required camera must stop motion
  // immediately, but a short UART hiccup should be allowed to recover instead
  // of permanently ending the round. Persistent loss still latches a fault.
  uint32_t visionLossGraceMs = 500;
  uint32_t directionTimeoutMs = 15000;
  // At the close-front direction boundary, hold the motor stopped while an
  // opening is confirmed. Persistent ambiguity then faults instead of guessing.
  uint32_t directionDecisionHoldMs = 300;
  uint32_t openingConfirmationMs = 100;
  // A confirmation must span real ToF acquisitions. Fast controller loops
  // are not allowed to count one unchanged range sample several times.
  uint8_t openingMinimumDistinctSamples = 2;
  // First-direction choice is higher consequence than a known-direction wall
  // trigger, so production Open can demand an extra physical acquisition.
  uint8_t directionMinimumDistinctSamples = 2;
  // A fresh optical no-target is not an electrical failure. Open
  // DirectionProbe permits strong status 2/4 while the IMU and at least one
  // numeric side ray remain live, until the separately bounded
  // directionTimeoutMs. Once direction is known, the front rangefinder may
  // not remain blind indefinitely while driving straight.
  // A strong fresh no-target may persist through an Open corner while healthy
  // IMU and at least one numeric outer-wall ray independently guide the turn;
  // cornerTimeoutMs stays the hard bound. Weak optical outcomes, an absent
  // outer ray, and Obstacle corners use this grace.
  uint32_t straightFrontOpticalGraceMs = 1200;
  uint32_t cornerFrontOpticalGraceMs = 2500;
  uint32_t cornerMinimumDurationMs = 350;
  uint32_t cornerTimeoutMs = 5000;
  // Allow the steering linkage to reach the commanded angle before the motor
  // starts pushing into a corner. Zero retains the legacy simultaneous start.
  uint32_t cornerDriveDelayMs = 0;
  uint32_t cornerExitStableMs = 200;
  // Minimum debounce after a completed corner. Corner detection is re-armed
  // by a stable clear front view, not by this timer alone, so a short fast
  // straight can never be hidden for a fixed 1.2 seconds.
  uint32_t cornerRefractoryMs = 250;
  uint32_t cornerRearmStableMs = 100;
  // Open Challenge steering is progressively unwound before the exact
  // 90-degree heading, then briefly counter-steered to cancel chassis yaw.
  // This feed-forward phase compensates for servo and vehicle inertia; IMU
  // heading feedback remains active after it decays.
  uint32_t openPostCornerCounterSteerMs = 300;
  // Give the chassis a short yaw-settle interval, then restore the selected
  // outer-wall correction quickly enough to protect the next straight.
  uint32_t postCornerHeadingOnlyMs = 150;
  uint32_t postCornerWallBlendMs = 250;
  // A fresh optical no-target is allowed separately below. A genuine side-ToF
  // bus/staleness failure stops commanded drive immediately, but receives this
  // short recovery window before the run is permanently fault-latched.
  uint32_t sideElectricalLossGraceMs = 250;
  // Outside a turn, a bump or oblique wall can briefly remove enough side
  // returns to compute safe geometry. Continue slowly for this interval while
  // the modules remain electrically healthy; persistent loss still stops.
  uint32_t straightSideGeometryGraceMs = 600;
  // During a turn, an opening can temporarily make a side pair invalid. The
  // car may continue slowly for this bounded interval, then must stop if no
  // complete left or right pair returns.
  uint32_t cornerSideGeometryGraceMs = 1500;
  uint32_t imuAccuracyGraceMs = 500;
  uint32_t finishDriveMs = 600;
  uint32_t finishStableMs = 200;
  uint32_t finishTimeoutMs = 1200;
  // Retained for source compatibility with existing board configuration. Pass
  // completion now uses the side-ToF geometry confirmation below instead.
  uint32_t obstacleHoldMs = 0;
  // An accepted pillar remains the pass target until the appropriate side
  // ToF pair says it has moved from the front of the car to behind it.  This
  // deliberately does not depend on the next camera frame.
  uint32_t obstacleMinimumCommitMs = 250;
  uint32_t obstacleMaximumCommitMs = 8000;
  // A committed target clears only after it was visibly close, then absent,
  // and the pass-side pair made a relative front-to-back transition.
  uint32_t obstacleBlobLossMs = 150;
  // A detection that disappears before ever reaching the close threshold is
  // not allowed to steer blindly until the hard pass timeout. Brief loss keeps
  // the commitment; longer confirmed loss stops first, then abandons it.
  uint32_t obstaclePreCloseLossClearMs = 500;
  uint32_t obstaclePassClearConfirmationMs = 180;
  // Optional single-pass calibration tail. After the normal close/loss/side
  // transition has been strictly verified, retain the committed pass side
  // for this additional clear-front interval before the terminal stop. Normal
  // production profiles leave this zero.
  uint32_t obstacleSinglePassBodyClearMs = 0;
  // Calibration-only deterministic escape for a pillar initially near the
  // image centre. All four values must be non-zero to enable it. The car
  // holds the mirrored pass steering until BNO yaw proves the requested turn;
  // missing the deadline stops the attempt instead of driving on a stale
  // image trajectory. Production and Open leave these zero.
  uint32_t obstacleCenteredEscapeDeadlineMs = 0;
  uint32_t obstacleCenteredEscapeHeadingStableMs = 0;
  uint32_t obstacleRecommitCooldownMs = 300;
  uint32_t obstacleRecenterMs = 300;
  uint32_t maxRunMs = 170000;

  float cornerOpeningDeltaMm = 180.0F;
  // Direction learning gets its own threshold because a legal starting zone
  // can expose only a short portion of the first opening. Later corners retain
  // the larger cornerOpeningDeltaMm guard.
  float directionProbeOpeningDeltaMm = 180.0F;
  float directionOpeningWinnerMarginMm = 100.0F;
  float cornerCompletionDegrees = 72.0F;
  // Open Challenge has no pillars. If low walls leave every side ray optically
  // blind, a measured near-90-degree IMU turn may finish the corner without
  // waiting forever for wall reacquisition. Obstacle Challenge never uses this
  // fallback because it needs the stricter pillar/wall geometry.
  float openCornerExitFallbackDegrees = 85.0F;
  // The heading must remain inside this bounded window for the full exit
  // stability interval. Merely passing the minimum angle while still
  // rotating is not a completed corner.
  float openCornerExitMaximumDegrees = 100.0F;
  float openCornerTaperStartDegrees = 55.0F;
  float openCornerCounterSteerStartDegrees = 92.0F;
  // Counter-steer grows with measured overshoot instead of snapping to its
  // configured maximum as soon as the start angle is crossed.
  float openCornerCounterSteerRampDegrees = 8.0F;
  float cornerMaximumDegrees = 125.0F;
  float cornerExitMaximumWallAngleErrorMm = 80.0F;
  float finishAlignmentDegrees = 10.0F;
  float finishMaximumWallAngleErrorMm = 60.0F;
  float maxHeadingStepDegrees = 50.0F;
  float maxReverseProgressDegrees = 20.0F;

  // +1 means increasing BNO yaw is a clockwise/right turn; -1 means
  // decreasing yaw is clockwise/right. The conservative starting value is
  // the usual mathematical convention. Confirm it by rotating the mounted,
  // wheels-lifted car by hand before enabling autonomous motor output.
  int8_t clockwiseYawSign = -1;
  uint8_t minimumRunImuAccuracy = 1;

  // Open Challenge has no pillars. Disabling obstacle avoidance keeps camera
  // telemetry available for diagnostics but prevents a coloured wall, mat, or
  // reflection from steering the vehicle. Obstacle Challenge enables this.
  bool obstacleAvoidanceEnabled = true;
  // Calibration-only containment: stop permanently after the same strict
  // close/loss/side-transition proof used for an ordinary completed pass.
  // Production profiles leave this false.
  bool stopAfterFirstVerifiedObstaclePass = false;
  // Production may use guarded obstacle phases without the calibration's
  // terminal stop after its first completed pass.
  bool guardedObstaclePassEnabled = false;
  // Full-course progress may abandon unproven pillar passes and use a bounded
  // forward escape into measured free space. Single-pass calibration stays strict.
  bool preferForwardObstacleProgress = false;
  // Optional coarse caution map across laps. False keeps both learned masks
  // empty and prevents earlier pillar locations from changing later speed.
  bool rememberObstacleSections = true;
  bool wallRecoveryEnabled = false;
  bool requireVisionForMotion = false;
  // The assembled Open car's VL53L1X repeatedly reports WrapTargetFail (7)
  // while its range still tracks a measured white wall/card. Open may use
  // that numeric range for clearance and corner timing; Obstacle keeps the
  // stricter default because pillars can create ambiguous multi-path returns.
  bool acceptFrontWrapTargetRange = false;
  // Set true only after confirming that FL/BL face directly left and FR/BR
  // face directly right. A false setting never treats pair differences as
  // wall angle.
  bool sideSensorPairsAreParallel = true;
  // Every side sensor must always stay online and fresh. When this stricter
  // switch is true, every individual range must also be valid outside a turn;
  // when false, the fused wall geometry may tolerate one invalid ray.
  bool requireAllSideSensorsFresh = true;

  // Parking is scaffolding only in R20. Enabling it must remain impossible
  // until both the state-machine implementation and mounted-car calibration
  // are deliberately completed. No motion constants are guessed here.
  bool parkingEnabled = false;
  bool parkingCalibrationValidated = false;
  bool parkingRearClearanceValidated = false;
  ParkingBaySide parkingBaySide = ParkingBaySide::Unknown;
  uint32_t parkingReverseNeutralMs = kMinimumDriveReversalNeutralMs;

  uint16_t frontEmergencyStopMm = 220;
  uint16_t directionProbeFrontMaximumMm = 700;
  uint16_t directionProbeAbortFrontMm = 320;
  uint16_t frontCornerTriggerMm = 360;
  uint16_t cornerRearmFrontClearMm = 700;
  uint16_t openingReferenceMaximumMm = 650;
  uint16_t frontSlowDistanceMm = 500;
  // This is a controller-only stopping-distance proxy, not a claim about the
  // vehicle's measured braking distance. Tune it from a recorded stop test.
  uint16_t frontStoppingProxyMm = 500;
  uint16_t nominalSideDistanceMm = 300;
  // Open Challenge follows the outside boundary (left when clockwise, right
  // when counter-clockwise). This is raw sensor-to-wall distance, not body
  // clearance, and must be calibrated on the actual car.
  uint16_t openOuterWallTargetMm = 200;
  uint16_t openOuterWallDeadbandMm = 20;
  uint16_t openInnerWallMinimumMm = 100;
  uint16_t obstacleMinimumConfidence = 400;
  uint16_t obstacleMinimumBottomY = 3500;
  uint16_t obstacleCloseBottomY = 4500;
  // Camera steering grows from zero at obstacleMinimumBottomY to full
  // authority here. A newly acquired far pillar therefore cannot snap the
  // steering away from a clear cardinal straight.
  uint16_t obstacleFullSteeringBottomY = 5500;
  uint16_t obstaclePassClearDeltaMm = 120;
  // The four side sensors are independently angled. These raw thresholds are
  // emergency proximity guards only; they are not treated as wall-normal
  // distances or as a front/rear wall-angle pair.
  uint16_t obstacleSideClearanceGuardMm = 0;
  // Single-pass calibration may use a smaller hard-stop threshold for the
  // free-side wall during its short body-clear tail. The pillar side keeps the
  // ordinary, larger avoidance guard. Production and Open leave this zero.
  uint16_t obstacleBodyClearOppositeSideGuardMm = 0;
  // A proven, bounded body-clear tail needs forward stopping clearance,
  // not the larger distance reserved for acquiring the first 90-degree turn.
  // Zero preserves the direction-probe threshold for existing profiles.
  uint16_t obstacleBodyClearFrontGuardMm = 0;
  // Opt-in single-pillar lane change: return to the entry heading before
  // collecting any evidence that the pillar has actually been passed.
  bool recoverCenteredEscapeBeforePass = false;
  uint16_t obstaclePassProofMinimumBottomY = 0;
  uint16_t obstacleSideClearanceSpeedPermille = 300;

  int16_t obstacleCenteredEscapeInitialAbsX = 0;

  float lateralSteeringPerMm = 3.0F;
  float wallAngleSteeringPerMm = 3.0F;
  // Normalized image X spans -10000..+10000. During an active pillar pass this
  // camera term replaces uncalibrated FL/FR lane error on the diagonal chassis.
  float obstacleImageSteeringPerUnit = 0.10F;
  float obstacleCornerRearmHeadingDegrees = 8.0F;
  float obstacleCenteredEscapeHeadingDegrees = 0.0F;
  // Straight sections use both wall geometry and the next 90-degree IMU
  // heading. This keeps one noisy side range from becoming the only steering
  // authority. The correction is bounded independently from wall guidance.
  float headingSteeringPerDegree = 8.0F;
  int16_t maximumHeadingCorrectionPermille = 260;
  // Wall geometry is a separate, lower-authority steering source. This keeps
  // one oblique ToF return from overpowering the IMU heading correction.
  int16_t maximumWallCorrectionPermille = 1000;
  int16_t openMaximumWallAngleCorrectionPermille = 120;
  // Keep enough steering authority to finish the last few degrees of an Open
  // corner. The old hard-coded 70/1000 approach command could leave the
  // physical linkage short of the cardinal heading after the early taper.
  int16_t openCornerMinimumApproachSteerPermille = 180;
  int16_t openCornerCounterSteerPermille = 450;
  // A red pillar should settle left of this image offset while the car passes
  // on its right; green uses the mirrored target.
  int16_t obstacleDesiredImageOffset = 3500;
  int16_t obstacleMinimumOffsetMm = 80;
  int16_t obstacleOffsetMm = 170;
  int16_t obstacleMaximumOffsetMm = 300;
  int16_t obstacleMaximumImageSteeringPermille = 500;
  int16_t obstacleSideClearanceSteeringPermille = 650;
  int16_t maximumSteeringPermille = 1000;

  uint16_t directionProbeSpeedPermille = 220;
  uint16_t cruiseSpeedPermille = 260;
  uint16_t cornerSpeedPermille = 220;
  uint16_t cornerBlindSpeedPermille = 140;
  uint16_t obstacleSpeedPermille = 200;
  // If vision is optional, unknown or stale vision caps motion instead of
  // granting cruise speed. Required vision still faults before this cap runs.
  uint16_t visionUncertainSpeedPermille = 160;
  uint16_t visionOverrunSpeedPermille = 150;
  uint8_t fullSpeedFrameQuality = 45;
  uint16_t obstacleUncertainSpeedPermille = 160;
  // Without an axle encoder the controller deliberately remembers only which
  // straight sections contained pillars, not a fake centimetre coordinate.
  // Later laps approach those sections below full open-course speed until
  // current camera/ToF evidence takes over.
  uint16_t learnedObstacleSectionSpeedPermille = 220;
  uint16_t finishSpeedPermille = 180;
  uint16_t minimumMovingSpeedPermille = 140;
  // Command slew rates are controller limits, not measured vehicle dynamics.
  uint16_t accelerationPermillePerSecond = 500;
  uint16_t decelerationPermillePerSecond = 900;
  int16_t cornerSteeringPermille = 760;
};

struct Decision {
  RunState state = RunState::WaitingForHealth;
  CourseDirection direction = CourseDirection::Unknown;
  ObstacleTarget obstacle = ObstacleTarget::None;
  ParkingPhase parkingPhase = ParkingPhase::Disabled;
  DriveDirection driveDirection = DriveDirection::Stopped;
  uint32_t faultFlags = FaultNone;
  uint8_t wallRecoveryPhase = 0;  // 0 normal, 1 neutral, 2 reverse, 3 neutral
  uint8_t wallRecoveryHazards = 0;  // 1 centre front, 2 front-left, 4 front-right
  uint8_t wallRecoveryExitReason = 0;  // 1 cleared, 2 rear limit, 3 reverse time limit

  bool startLatched = false;
  bool motionAllowed = false;
  uint16_t speedPermille = 0;
  int16_t steeringPermille = 0;  // negative left, positive right

  float unwrappedHeadingDegrees = 0.0F;
  float directedProgressDegrees = 0.0F;
  // Obstacle/unknown-direction: corridor offset (negative left, positive
  // right). Open with known direction: signed outside-wall distance error
  // mapped to the same steering convention.
  float wallLateralOffsetMm = 0.0F;
  float wallAngleErrorMm = 0.0F;
  float leftOpeningMm = 0.0F;
  float rightOpeningMm = 0.0F;
  float cornerProgressDegrees = 0.0F;
  int16_t targetLateralOffsetMm = 0;  // negative left, positive right
  float cornerTargetHeadingDegrees = 0.0F;
  float headingErrorDegrees = 0.0F;
  float outerWallDistanceMm = 0.0F;
  bool outerWallUsable = false;
  bool outerWallPair = false;
  bool cornerDetectionArmed = false;

  int8_t configuredClockwiseYawSign = -1;

  uint8_t cornerCount = 0;
  uint8_t lapCount = 0;
  uint8_t learnedRedSections = 0;    // bit 0..3: straight after corner n
  uint8_t learnedGreenSections = 0;  // bit 0..3: straight after corner n
};

class Controller {
 public:
  explicit Controller(const Config& config = Config());

  Decision update(const Inputs& inputs);
  void reset();

  const Config& config() const { return config_; }
  const Decision& decision() const { return decision_; }

  // Returns the signed shortest rotation from `fromDegrees` to `toDegrees` in
  // (-180, 180].  Example: +179 -> -179 is +2 degrees.
  static float shortestAngleDeltaDegrees(float fromDegrees, float toDegrees);

 private:
  struct WallGuidance {
    bool usable = false;
    bool completePair = false;
    bool bothPairs = false;
    bool outerWallUsable = false;
    bool outerWallPair = false;
    bool innerGuardActive = false;
    float lateralOffsetMm = 0.0F;
    float angleErrorMm = 0.0F;
    float outerWallDistanceMm = 0.0F;
  };

  struct OpeningEvidence {
    bool leftUsable = false;
    bool rightUsable = false;
    float leftDeltaMm = 0.0F;
    float rightDeltaMm = 0.0F;
    uint16_t leftReferenceMm = 0;
    uint16_t rightReferenceMm = 0;
  };

  static bool elapsedAtLeast(uint32_t nowMs, uint32_t sinceMs,
                             uint32_t intervalMs);
  static bool readingFresh(uint32_t nowMs, uint32_t updatedAtMs,
                           uint32_t maximumAgeMs);
  static int16_t clampSteering(float value, int16_t limit);
  static uint16_t clampSpeed(uint16_t value);
  static float absoluteValue(float value);
  static float headingRemainderDegrees(float unwrappedDegrees);

  bool updateButton(const Inputs& inputs);
  uint32_t evaluateSafety(const Inputs& inputs, WallGuidance& wall);
  WallGuidance computeWallGuidance(const Inputs& inputs) const;
  OpeningEvidence computeOpeningEvidence(const Inputs& inputs) const;
  CourseDirection forwardSpaceDirectionCandidate(
      const Inputs& inputs) const;
  bool sideReadingUsable(const RangeReading& reading,
                         uint32_t nowMs) const;
  bool frontReadingUsable(const RangeReading& reading,
                          uint32_t nowMs) const;
  bool sideReadingIsOpen(const RangeReading& reading,
                         uint32_t nowMs) const;
  bool sideReadingIsSigmaFail(const RangeReading& reading,
                              uint32_t nowMs) const;
  bool straightSideGeometryUsable(const Inputs& inputs,
                                  const WallGuidance& wall) const;
  bool anyNumericSideReadingUsable(const Inputs& inputs) const;
  bool anyLocalSideReadingUsable(const Inputs& inputs) const;
  void captureOpeningBaselines(const Inputs& inputs, bool resetFirst);
  void startRun(const Inputs& inputs);
  bool updateHeading(const Inputs& inputs);
  void updateNavigation(const Inputs& inputs, const WallGuidance& wall,
                        const OpeningEvidence& opening);
  void updateCornerDetectionRearm(const Inputs& inputs,
                                  const OpeningEvidence& opening);
  void enterCorner(const Inputs& inputs);
  void updateCorner(const Inputs& inputs, const WallGuidance& wall,
                    const OpeningEvidence& opening);
  bool expectedOpeningPresent(const OpeningEvidence& opening) const;
  bool openingConfirmed(CourseDirection candidate, const Inputs& inputs);
  float courseProgressDegrees() const;
  void updateObstacle(const Inputs& inputs);
  void refreshCommittedObstacle(const BlobReading& blob);
  int16_t obstacleOffsetForBlob(ObstacleTarget target,
                                const BlobReading& blob) const;
  float obstacleImageSteeringCorrection() const;
  float obstacleCenteredEscapeProgressDegrees() const;
  bool centeredEscapeRecoveryActive() const;
  bool centeredEscapePostTurnActive() const;
  bool guardedPassEnabled() const;
  uint32_t centeredEscapeElapsedMs(uint32_t nowMs) const;
  void updateCenteredEscapeDriveTime(const Inputs& inputs);
  uint8_t leadingObstacleHazards(const Inputs& inputs) const;
  bool handleForwardSideEscape(const Inputs& inputs, uint32_t safetyFaults);
  bool wallRecoveryInputsReady(const Inputs& inputs,
                               bool allowFrontOpticalNoTarget = false,
                               bool requireRearClearance = true) const;
  bool wallRecoveryHazardsClear(const Inputs& inputs,
                                bool requireCenterClearance = true) const;
  bool rearRayAtOrBelow(const Inputs& inputs, uint16_t limitMm) const;
  bool rearSweepAllowsSteering(const Inputs& inputs, float steering) const;
  bool rearLimitedForwardDepartureReady(const Inputs& inputs) const;
  bool recoveryPillarRouteUsable(const Inputs& inputs, ObstacleTarget target) const;
  bool skippedPillarStraightDepartureReady(const Inputs& inputs,
                                           bool continuing = false) const;
  bool wallRecoveryPillarClear(const Inputs& inputs) const;
  bool handleWallRecovery(const Inputs& inputs, uint32_t safetyFaults);
  void clearCommittedObstacle(uint32_t nowMs, bool beginRecenter);
  bool obstaclePassSidePairDelta(const Inputs& inputs, float& deltaMm) const;
  void commitObstacle(ObstacleTarget target, int16_t lateralOffset,
                      int16_t centerX, uint16_t confidence, uint16_t bottomY,
                      uint32_t nowMs);
  uint16_t applySpeedSlew(uint16_t requestedSpeed, uint32_t nowMs,
                          bool enforceImmediateCap);
  void makeMotionDecision(const Inputs& inputs, const WallGuidance& wall);
  bool visionFreshAndValid(const Inputs& inputs) const;
  bool blobUsable(const BlobReading& blob) const;
  static uint64_t blobPriority(const BlobReading& blob);
  void latchFault(uint32_t flags);

  Config config_;
  Decision decision_;
  bool configurationValid_ = true;
  DriveDirectionInterlock driveDirectionInterlock_;

  bool rawButtonPressed_ = false;
  bool stableButtonPressed_ = false;
  bool buttonArmed_ = false;
  uint32_t rawButtonChangedAtMs_ = 0;
  uint32_t resetAtMs_ = 0;

  bool headingInitialized_ = false;
  float lastRawHeadingDegrees_ = 0.0F;
  float unwrappedHeadingDegrees_ = 0.0F;
  float runStartHeadingDegrees_ = 0.0F;
  float cornerStartHeadingDegrees_ = 0.0F;
  float cornerTargetHeadingDegrees_ = 0.0F;
  float straightHeadingReferenceDegrees_ = 0.0F;
  CourseDirection openingCandidate_ = CourseDirection::Unknown;
  CourseDirection opticalProbeCandidate_ = CourseDirection::Unknown;
  CourseDirection opticalProbeDirection_ = CourseDirection::Unknown;
  uint32_t opticalProbeSinceMs_ = 0;
  uint32_t opticalProbeConfirmedAtMs_ = 0;
  uint32_t opticalProbeLastLeftAtMs_ = 0;
  uint32_t opticalProbeLastRightAtMs_ = 0;
  uint8_t opticalProbeSamples_ = 0;
  bool leftOpeningBaselineValid_ = false;
  bool rightOpeningBaselineValid_ = false;
  uint16_t leftOpeningBaselineMm_ = 0;
  uint16_t rightOpeningBaselineMm_ = 0;

  uint32_t runStartedAtMs_ = 0;
  uint32_t directionProbeTimerStartedAtMs_ = 0;
  uint32_t lastCornerAtMs_ = 0;
  uint32_t cornerRearmClearSinceMs_ = 0;
  bool cornerDetectionArmed_ = false;
  uint32_t cornerStartedAtMs_ = 0;
  uint32_t cornerReadySinceMs_ = 0;
  uint32_t openingCandidateSinceMs_ = 0;
  uint32_t openingCandidateLastLeftSampleAtMs_ = 0;
  uint32_t openingCandidateLastRightSampleAtMs_ = 0;
  uint32_t openingCandidateLastFrontSampleAtMs_ = 0;
  uint32_t openingCandidateLastEvidenceAtMs_ = 0;
  uint8_t openingCandidateDistinctSamples_ = 0;
  uint32_t directionDecisionHoldSinceMs_ = 0;
  bool directionDecisionHoldActive_ = false;
  uint32_t finishEnteredAtMs_ = 0;
  uint32_t finishReadySinceMs_ = 0;
  uint32_t straightSideGeometryLostSinceMs_ = 0;
  uint32_t cornerSideGeometryLostSinceMs_ = 0;
  uint32_t sideElectricalLostSinceMs_ = 0;
  uint32_t frontOpticalLostSinceMs_ = 0;
  bool straightSideGeometryLossActive_ = false;
  bool cornerSideGeometryLossActive_ = false;
  bool sideElectricalLossActive_ = false;
  bool frontOpticalLossActive_ = false;
  bool frontOpticallyDegraded_ = false;
  bool sideGeometryDegraded_ = false;
  bool calibrationCrossGuardActive_ = false;
  uint32_t imuAccuracyLostSinceMs_ = 0;
  uint32_t visionLostSinceMs_ = 0;
  bool visionLossActive_ = false;
  ObstacleTarget committedObstacle_ = ObstacleTarget::None;
  int16_t committedObstacleOffsetMm_ = 0;
  int16_t committedObstacleCenterX_ = 0;
  uint16_t committedObstacleConfidence_ = 0;
  uint32_t obstacleCommittedAtMs_ = 0;
  uint32_t obstacleClearSinceMs_ = 0;
  uint32_t obstacleBodyClearStartedAtMs_ = 0;
  bool obstacleBodyClearActive_ = false;
  bool obstacleCenteredEscapeActive_ = false;
  bool obstacleCenteredEscapeTurnComplete_ = false;
  uint32_t obstacleCenteredEscapeDrivenMs_ = 0;
  uint32_t obstacleCenteredEscapeDriveUpdatedAtMs_ = 0;
  bool obstacleCenteredEscapeRecovered_ = false;
  bool obstacleCenteredEscapeRecoveryCandidateActive_ = false;
  uint32_t obstacleCenteredEscapeRecoveryCandidateSampleAtMs_ = 0;
  uint32_t obstacleCenteredEscapeRecoveredVisionSampleAtMs_ = 0;
  bool obstacleCenteredEscapePostRecoveryObserved_ = false;
  bool obstacleSuccessorCandidateActive_ = false;
  uint32_t obstacleSuccessorCandidateSampleAtMs_ = 0;
  bool obstacleSuccessorRollover_ = false;
  bool obstacleCenteredEscapeClearanceViolated_ = false;
  bool obstacleCenteredEscapeHeadingCandidateActive_ = false;
  uint32_t obstacleCenteredEscapeHeadingCandidateSampleAtMs_ = 0;
  float obstacleCenteredEscapeHeadingReferenceDegrees_ = 0.0F;
  uint32_t obstacleAbsentSinceMs_ = 0;
  uint16_t obstaclePeakBottomY_ = 0;
  bool obstaclePairDeltaInitialized_ = false;
  float obstacleMinimumPairDeltaMm_ = 0.0F;
  ObstacleTarget lastClearedObstacle_ = ObstacleTarget::None;
  uint32_t obstacleRecommitBlockedSinceMs_ = 0;
  ObstacleTarget recoverySkippedObstacle_ = ObstacleTarget::None;
  uint32_t recoverySkippedObstacleAtMs_ = 0;
  uint32_t obstacleRecenterStartedAtMs_ = 0;
  int16_t obstacleRecenterStartOffsetMm_ = 0;
  bool obstaclePassCornerRecoveryPending_ = false;
  uint16_t lastCommandedSpeedPermille_ = 0;
  uint32_t lastSpeedCommandAtMs_ = 0;
  uint32_t latchedFaultFlags_ = FaultNone;
  uint8_t wallRecoveryAttempts_ = 0;
  uint8_t rearDepartureConstraintMask_ = 0;
  bool rearStraightDepartureActive_ = false;
  bool rearStraightDepartureExhausted_ = false;
  uint32_t rearStraightDepartureStartedAtMs_ = 0;
  uint32_t rearStraightDepartureUpdatedAtMs_ = 0;
  uint32_t rearStraightDeparturePoweredMs_ = 0;
  bool forwardSideEscapeActive_ = false;
  bool forwardSideEscapeSuppressed_ = false;
  uint8_t forwardSideEscapeHazards_ = 0;
  uint8_t forwardSideEscapeBlockedHazards_ = 0;
  uint32_t forwardSideEscapeStartedAtMs_ = 0;
  uint32_t forwardSideEscapeUpdatedAtMs_ = 0;
  uint32_t forwardSideEscapePoweredMs_ = 0;
  float forwardSideEscapeHeadingDegrees_ = 0.0F;
  uint32_t wallRecoveryStartedAtMs_ = 0;
  uint32_t wallRecoveryPhaseStartedAtMs_ = 0;
  uint32_t wallRecoveryUpdatedAtMs_ = 0;
  uint32_t wallRecoveryPoweredMs_ = 0;
  bool wallRecoveryHealthHoldActive_ = false;
  uint32_t wallRecoveryClearSinceMs_ = 0;
  uint32_t wallRecoveryClearLastEvidenceMs_ = 0;
  uint32_t wallRecoveryClearLeftSampleMs_ = 0;
  uint32_t wallRecoveryClearRightSampleMs_ = 0;
  uint8_t wallRecoveryClearSamples_ = 0;
  uint32_t wallRecoveryForwardSinceMs_ = 0;
  uint32_t wallRecoveryForwardEvidenceMs_ = 0;
  uint32_t wallRecoveryForwardFrontSampleMs_ = 0;
  uint32_t wallRecoveryForwardLeftSampleMs_ = 0;
  uint32_t wallRecoveryForwardRightSampleMs_ = 0;
  uint8_t wallRecoveryForwardSamples_ = 0;
  float wallRecoveryHeadingReferenceDegrees_ = 0.0F;
};

}  // namespace autonomy

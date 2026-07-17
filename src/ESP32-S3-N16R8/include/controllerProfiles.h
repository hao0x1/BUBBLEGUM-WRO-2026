#pragma once

#include "autonomyController.h"

namespace autonomy {

enum class ChallengeProfile : uint8_t {
  Open = 1,
  Obstacle = 2,
};

// Pure, host-testable source of the exact configurations used by the board.
// Keeping these values out of Arduino main prevents production tuning from
// silently drifting away from the regression suite.
Config makeControllerConfig(ChallengeProfile profile);
// Current full-course Obstacle profile. The base profile above remains the
// recorded R23 reference for regression comparisons.
Config makeGuardedObstacleConfig();

// Kept beside the profile values so the board banner and host regression
// identify the exact same motion source.
const char* controllerSourceIdentity(ChallengeProfile profile);

}  // namespace autonomy

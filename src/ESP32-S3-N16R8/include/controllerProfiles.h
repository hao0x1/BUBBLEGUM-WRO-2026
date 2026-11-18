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
// Separately selectable Open fallback; ordinary Open remains R20.
Config makeOpenFallbackConfig();
// Separate competition backups; false keeps required camera, true permits
// measured-route progress when camera observations are unavailable.
Config makeObstacleFallbackConfig(bool progressPriority);
// Last-resort field profile for weakly reflecting walls. Direction is selected
// from the direction announced for the round; fresh I2C optical no-target
// samples remain motion-capable instead of becoming WAIT_HEALTH.
Config makeObstacleContinueConfig(CourseDirection direction);

// Kept beside the profile values so the board banner and host regression
// identify the exact same motion source.
const char* controllerSourceIdentity(ChallengeProfile profile);
const char* openFallbackSourceIdentity();
const char* obstacleFallbackSourceIdentity(bool progressPriority);
const char* obstacleContinueSourceIdentity(CourseDirection direction);

}  // namespace autonomy

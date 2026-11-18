// Deterministic hardware boundary tests. Compile the actual production
// vehicleActuators.cpp; only GPIO, LEDC, clock, and timer transport are stubbed.
#include "vehicleActuators.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "boardConfig.h"
#include "driver/gpio.h"

struct FakeEspTimer {
  esp_timer_create_args_t args;
  uint64_t period = 0;
  int64_t nextFire = 0;
  bool active = false;
  bool deleted = false;
};

namespace fake {
int64_t nowUs = 1000000;
std::array<unsigned int, 64> gpio{};
std::array<uint32_t, 16> pwm{};
std::vector<std::unique_ptr<FakeEspTimer>> timers;
unsigned int enableHighWrites = 0;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void reset(int64_t startUs = 1000000) {
  nowUs = startUs;
  gpio.fill(0);
  pwm.fill(0);
  timers.clear();
  enableHighWrites = 0;
}

void advanceTo(int64_t timeUs) {
  require(timeUs >= nowUs, "fake clock must be monotonic");
  // Run actual production callbacks at the timer's physical deadlines.
  for (;;) {
    FakeEspTimer* next = nullptr;
    for (const auto& timer : timers) {
      if (timer->active && !timer->deleted && timer->nextFire <= timeUs &&
          (next == nullptr || timer->nextFire < next->nextFire)) {
        next = timer.get();
      }
    }
    if (next == nullptr) break;
    nowUs = next->nextFire;
    next->nextFire += static_cast<int64_t>(next->period);
    next->args.callback(next->args.arg);
  }
  nowUs = timeUs;
}

void motorStopped(const VehicleActuators& actuators) {
  require(gpio[board::DRIVE_ENABLE_PIN] == 0, "drive enable must be LOW");
  require(pwm[2] == 0 && pwm[3] == 0, "both motor duties must be zero");
  require(actuators.driveCommand() == 0.0F, "reported drive must be zero");
}

void motorRunning(const VehicleActuators& actuators, float command) {
  require(gpio[board::DRIVE_ENABLE_PIN] == 1, "drive enable must be HIGH");
  require((pwm[2] != 0) != (pwm[3] != 0), "exactly one motor duty is active");
  require((command > 0 ? pwm[2] : pwm[3]) != 0,
          "correct motor direction channel must be active");
  require(std::fabs(actuators.driveCommand() - command) < 0.00001F,
          "applied drive command must match request");
}
}  // namespace fake

void digitalWrite(uint8_t pin, uint8_t value) { fake::gpio.at(pin) = value; }
void pinMode(uint8_t, uint8_t) {}
double ledcSetup(uint8_t, double frequency, uint8_t) { return frequency; }
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t channel, uint32_t duty) {
  fake::pwm.at(channel) = duty;
  fake::require(!(fake::pwm[2] != 0 && fake::pwm[3] != 0),
                "production must never energize both bridge PWM inputs");
}
int gpio_set_level(gpio_num_t pin, unsigned int value) {
  fake::gpio.at(static_cast<std::size_t>(pin)) = value;
  if (pin == board::DRIVE_ENABLE_PIN && value != 0) {
    ++fake::enableHighWrites;
    fake::require((fake::pwm[2] != 0) != (fake::pwm[3] != 0),
                  "enable HIGH requires exactly one configured motor duty");
  }
  return ESP_OK;
}
int64_t esp_timer_get_time() { return fake::nowUs; }
esp_err_t esp_timer_create(const esp_timer_create_args_t* args,
                           esp_timer_handle_t* handle) {
  auto timer = std::make_unique<FakeEspTimer>();
  timer->args = *args;
  *handle = timer.get();
  fake::timers.push_back(std::move(timer));
  return ESP_OK;
}
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period) {
  timer->period = period;
  timer->nextFire = fake::nowUs + static_cast<int64_t>(period);
  timer->active = true;
  return ESP_OK;
}
esp_err_t esp_timer_stop(esp_timer_handle_t timer) {
  timer->active = false;
  return ESP_OK;
}
esp_err_t esp_timer_delete(esp_timer_handle_t timer) {
  timer->active = false;
  timer->deleted = true;
  return ESP_OK;
}

namespace {
using Result = VehicleActuators::DriveResult;

void mirroredNeutralBoundaries(float initialDirection, int64_t startUs) {
  fake::reset(startUs);
  VehicleActuators actuators;
  fake::require(actuators.beginSafe(), "beginSafe must succeed");
  const float initial = initialDirection * 0.3F;
  const float opposite = -initial;
  fake::require(actuators.setDriveWithResult(initial) == Result::Applied,
                "initial direction must apply");
  fake::motorRunning(actuators, initial);
  fake::advanceTo(startUs + 20000);
  actuators.stop();
  const int64_t stoppedUs = fake::nowUs;
  const auto highWrites = fake::enableHighWrites;
  for (const int64_t elapsedUs : {0LL, 1000LL, 149999LL, 299999LL}) {
    fake::advanceTo(stoppedUs + elapsedUs);
    fake::require(actuators.setDriveWithResult(opposite) == Result::WaitingForNeutral,
                  "opposite direction must wait before 300000 us");
    fake::motorStopped(actuators);
    fake::require(!actuators.faulted() && !actuators.leaseExpired(),
                  "neutral waiting must not latch a fault or a lease expiry");
  }
  fake::require(!actuators.setDrive(opposite), "legacy bool wrapper must reject wait");
  fake::require(fake::enableHighWrites == highWrites,
                "neutral retries must never pulse enable HIGH");
  fake::advanceTo(stoppedUs + 300000);
  fake::require(actuators.setDriveWithResult(opposite) == Result::Applied,
                "exactly 300000 us must apply; retries must not reset stop clock");
  fake::motorRunning(actuators, opposite);
  fake::require(actuators.setDrive(opposite), "legacy bool wrapper accepts applied drive");
}

void controllerMillisLeadsPhysicalStop(float initialDirection) {
  fake::reset();
  VehicleActuators actuators;
  fake::require(actuators.beginSafe(), "beginSafe must succeed");
  fake::require(actuators.setDriveWithResult(initialDirection * 0.3F) == Result::Applied,
                "initial drive applies");
  const int64_t controllerNeutralStartedUs = fake::nowUs;
  fake::advanceTo(controllerNeutralStartedUs + 700);
  actuators.stop();
  fake::advanceTo(controllerNeutralStartedUs + 300000);
  fake::require(actuators.setDriveWithResult(-initialDirection * 0.3F) ==
                    Result::WaitingForNeutral,
                "300 ms controller elapsed but 299300 us physical neutral must wait");
  fake::motorStopped(actuators);
  fake::advanceTo(controllerNeutralStartedUs + 300700);
  fake::require(actuators.setDriveWithResult(-initialDirection * 0.3F) == Result::Applied,
                "remaining 700 us must make retry eligible without restart");
  fake::motorRunning(actuators, -initialDirection * 0.3F);
}

void energizedSignChangesRemainRejected(float initialDirection) {
  fake::reset();
  VehicleActuators actuators;
  fake::require(actuators.beginSafe(), "beginSafe must succeed");
  fake::require(actuators.setDriveWithResult(initialDirection * 0.3F) == Result::Applied,
                "initial drive applies");
  fake::advanceTo(fake::nowUs + 1000);
  fake::require(actuators.setDriveWithResult(-initialDirection * 0.3F) == Result::Rejected,
                "energized sign change is a rejected command, not a neutral retry");
  fake::motorStopped(actuators);
}

void notReadyAndFaultedRemainRejected() {
  fake::reset();
  VehicleActuators actuators;
  fake::require(actuators.setDriveWithResult(0.3F) == Result::Rejected,
                "not-ready drive stays rejected");
  fake::motorStopped(actuators);
  fake::require(actuators.beginSafe(), "beginSafe must succeed");
  actuators.faultStop();
  fake::require(actuators.setDriveWithResult(-0.3F) == Result::Rejected,
                "faulted drive stays rejected");
  fake::require(!actuators.setDrive(0.3F), "legacy bool wrapper rejects fault");
  fake::motorStopped(actuators);
  actuators.clearFault();
  fake::require(actuators.setDriveWithResult(-0.3F) == Result::Applied,
                "ordinary cleared fault may resume drive");
}

void expiredLeaseCannotBeCleared() {
  fake::reset();
  VehicleActuators actuators;
  fake::require(actuators.beginSafe(), "beginSafe must succeed");
  fake::require(actuators.setDriveWithResult(0.3F) == Result::Applied,
                "initial drive applies");
  const int64_t startedUs = fake::nowUs;
  fake::advanceTo(startedUs + 149999);
  fake::require(!actuators.leaseExpired(), "lease must not expire prematurely");
  fake::advanceTo(startedUs + 150000);
  fake::require(actuators.leaseExpired(), "actual timer callback must expire lease");
  fake::require(fake::gpio[board::DRIVE_ENABLE_PIN] == 0,
                "timer callback independently drops drive enable");
  fake::require(actuators.setDriveWithResult(0.3F) == Result::Rejected,
                "expired lease cannot resume drive");
  fake::motorStopped(actuators);
  actuators.clearFault();
  fake::require(actuators.faulted() && actuators.leaseExpired(),
                "clearFault cannot reset expired lease");
  fake::require(actuators.setDriveWithResult(-0.3F) == Result::Rejected,
                "expired lease must not be misclassified as neutral waiting");
  fake::require(!actuators.beginSafe(), "beginSafe cannot reset expired lease");
  fake::require(actuators.leaseExpired(), "expired lease stays latched after beginSafe");
  fake::motorStopped(actuators);
}
}  // namespace

int main() {
  try {
    for (const float direction : {1.0F, -1.0F}) {
      mirroredNeutralBoundaries(direction, 1000000);
      mirroredNeutralBoundaries(direction,
                               static_cast<int64_t>(std::numeric_limits<uint32_t>::max()) - 100000);
      controllerMillisLeadsPhysicalStop(direction);
      energizedSignChangesRemainRejected(direction);
    }
    notReadyAndFaultedRemainRejected();
    expiredLeaseCannotBeCleared();
    std::cout << "PASS: actual actuator implementation: mirrored 300000 us interlocks, "
                 "uint32 wrap, 700 us clock skew retries, bridge outputs, rejection paths, "
                 "and permanent lease expiry\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}

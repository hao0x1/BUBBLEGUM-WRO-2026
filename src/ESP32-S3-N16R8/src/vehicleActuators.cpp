#include "vehicleActuators.h"

#include <cmath>

#include "boardConfig.h"
#include "driver/gpio.h"

namespace {

float clampFloat(float value, float minimum, float maximum) {
  if (!std::isfinite(value)) return 0.0F;
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

}  // namespace

VehicleActuators::VehicleActuators() = default;

VehicleActuators::VehicleActuators(const Config& config) : config_(config) {}

bool VehicleActuators::leaseExpired() const {
  portENTER_CRITICAL(&driveLeaseMux_);
  const bool expired = driveLeaseExpired_;
  portEXIT_CRITICAL(&driveLeaseMux_);
  return expired;
}

void VehicleActuators::driveLeaseTimerCallback(void* argument) {
  auto* actuators = static_cast<VehicleActuators*>(argument);
  bool expiredNow = false;

  portENTER_CRITICAL(&actuators->driveLeaseMux_);
  const uint32_t nowUs = static_cast<uint32_t>(esp_timer_get_time());
  if (actuators->driveLeaseArmed_ && !actuators->driveLeaseExpired_ &&
      static_cast<uint32_t>(nowUs - actuators->lastDriveLeaseRenewUs_) >=
          DRIVE_LEASE_TIMEOUT_US) {
    actuators->driveLeaseArmed_ = false;
    // This latch deliberately cannot be cleared without restarting the MCU.
    actuators->driveLeaseExpired_ = true;
    expiredNow = true;
  }
  portEXIT_CRITICAL(&actuators->driveLeaseMux_);

  if (expiredNow) {
    // The timer callback touches no PWM, servo, logging, or heap APIs. Dropping
    // the tied IBT-2 enables independently stops drive even if loop() is stuck.
    gpio_set_level(static_cast<gpio_num_t>(board::DRIVE_ENABLE_PIN), 0);
  }
}

bool VehicleActuators::startDriveLeaseTimer() {
  if (driveLeaseTimer_ != nullptr) return true;

  esp_timer_create_args_t timerArguments = {};
  timerArguments.callback = &VehicleActuators::driveLeaseTimerCallback;
  timerArguments.arg = this;
  timerArguments.dispatch_method = ESP_TIMER_TASK;
  timerArguments.name = "driveLease";
  timerArguments.skip_unhandled_events = true;

  if (esp_timer_create(&timerArguments, &driveLeaseTimer_) != ESP_OK) {
    driveLeaseTimer_ = nullptr;
    return false;
  }
  if (esp_timer_start_periodic(driveLeaseTimer_, DRIVE_LEASE_CHECK_US) !=
      ESP_OK) {
    esp_timer_delete(driveLeaseTimer_);
    driveLeaseTimer_ = nullptr;
    return false;
  }
  return true;
}

void VehicleActuators::stopDriveLeaseTimer() {
  if (driveLeaseTimer_ == nullptr) return;
  esp_timer_stop(driveLeaseTimer_);
  esp_timer_delete(driveLeaseTimer_);
  driveLeaseTimer_ = nullptr;
}

bool VehicleActuators::configIsValid() const {
  // The broad limits reject obvious unit/configuration mistakes. The defaults
  // deliberately use a much narrower range until the linkage is calibrated.
  constexpr uint16_t MIN_ALLOWED_PULSE_US = 500;
  constexpr uint16_t MAX_ALLOWED_PULSE_US = 2500;
  return config_.steeringLeftPulseUs >= MIN_ALLOWED_PULSE_US &&
         config_.steeringLeftPulseUs < config_.steeringCenterPulseUs &&
         config_.steeringCenterPulseUs < config_.steeringRightPulseUs &&
         config_.steeringRightPulseUs <= MAX_ALLOWED_PULSE_US;
}

void VehicleActuators::forcePinsLowBeforeAttach() {
  // Load a LOW output value before enabling each output driver. This prevents
  // a brief HIGH pulse while the ESP32 changes a pin from input to output.
  digitalWrite(board::DRIVE_ENABLE_PIN, LOW);
  pinMode(board::DRIVE_ENABLE_PIN, OUTPUT);
  digitalWrite(board::DRIVE_ENABLE_PIN, LOW);

  digitalWrite(board::DRIVE_RPWM_PIN, LOW);
  pinMode(board::DRIVE_RPWM_PIN, OUTPUT);
  digitalWrite(board::DRIVE_RPWM_PIN, LOW);

  digitalWrite(board::DRIVE_LPWM_PIN, LOW);
  pinMode(board::DRIVE_LPWM_PIN, OUTPUT);
  digitalWrite(board::DRIVE_LPWM_PIN, LOW);

  digitalWrite(board::STEERING_SERVO_PIN, LOW);
  pinMode(board::STEERING_SERVO_PIN, OUTPUT);
  digitalWrite(board::STEERING_SERVO_PIN, LOW);
}

void VehicleActuators::leavePinsLowAfterSetupFailure() {
  // No pin is attached until every timer setup has succeeded, so ordinary GPIO
  // writes are sufficient on this failure path.
  digitalWrite(board::DRIVE_ENABLE_PIN, LOW);
  digitalWrite(board::DRIVE_RPWM_PIN, LOW);
  digitalWrite(board::DRIVE_LPWM_PIN, LOW);
  digitalWrite(board::STEERING_SERVO_PIN, LOW);
  stopDriveLeaseTimer();
  ready_ = false;
  faulted_ = true;
  driveCommand_ = 0.0F;
  lastNonzeroDirection_ = 0;
  lastStoppedUs_ = static_cast<uint32_t>(esp_timer_get_time());
  steeringCommand_ = 0.0F;
}

bool VehicleActuators::beginSafe() {
  if (leaseExpired()) {
    forcePinsLowBeforeAttach();
    leavePinsLowAfterSetupFailure();
    return false;
  }

  ready_ = false;
  faulted_ = false;
  driveCommand_ = 0.0F;
  lastNonzeroDirection_ = 0;
  lastStoppedUs_ = static_cast<uint32_t>(esp_timer_get_time());
  steeringCommand_ = 0.0F;
  forcePinsLowBeforeAttach();

  portENTER_CRITICAL(&driveLeaseMux_);
  driveLeaseArmed_ = false;
  lastDriveLeaseRenewUs_ = 0;
  portEXIT_CRITICAL(&driveLeaseMux_);

  if (!configIsValid() || !startDriveLeaseTimer()) {
    leavePinsLowAfterSetupFailure();
    return false;
  }

  // On Arduino-ESP32 2.x, channels in each pair share one timer. Keep the
  // servo on channel 0 and the two equal-frequency motor signals on 2 and 3.
  const uint32_t actualServoHz =
      ledcSetup(SERVO_LEDC_CHANNEL, SERVO_FREQUENCY_HZ,
                SERVO_RESOLUTION_BITS);
  const uint32_t actualRpwmHz =
      ledcSetup(RPWM_LEDC_CHANNEL, MOTOR_FREQUENCY_HZ,
                MOTOR_RESOLUTION_BITS);
  const uint32_t actualLpwmHz =
      ledcSetup(LPWM_LEDC_CHANNEL, MOTOR_FREQUENCY_HZ,
                MOTOR_RESOLUTION_BITS);
  if (actualServoHz == 0U || actualRpwmHz == 0U || actualLpwmHz == 0U) {
    leavePinsLowAfterSetupFailure();
    return false;
  }

  // Every channel's initial duty is zero before it is routed to a physical pin.
  ledcWrite(SERVO_LEDC_CHANNEL, 0U);
  ledcWrite(RPWM_LEDC_CHANNEL, 0U);
  ledcWrite(LPWM_LEDC_CHANNEL, 0U);
  ledcAttachPin(board::STEERING_SERVO_PIN, SERVO_LEDC_CHANNEL);
  ledcAttachPin(board::DRIVE_RPWM_PIN, RPWM_LEDC_CHANNEL);
  ledcAttachPin(board::DRIVE_LPWM_PIN, LPWM_LEDC_CHANNEL);

  ready_ = true;
  stop();
  setSteering(0.0F);
  return true;
}

void VehicleActuators::writeSteeringPulseUs(uint16_t pulseUs) {
  if (!ready_) return;

  constexpr uint32_t SERVO_DUTY_SCALE = 1UL << SERVO_RESOLUTION_BITS;
  const uint64_t scaled =
      static_cast<uint64_t>(pulseUs) * SERVO_FREQUENCY_HZ *
      SERVO_DUTY_SCALE;
  const uint32_t duty = static_cast<uint32_t>((scaled + 500000ULL) / 1000000ULL);
  ledcWrite(SERVO_LEDC_CHANNEL, duty);
}

void VehicleActuators::setSteering(float command) {
  if (!ready_) return;

  steeringCommand_ = clampFloat(command, -1.0F, 1.0F);
  const float hardwareCommand =
      config_.steeringReversed ? -steeringCommand_ : steeringCommand_;

  float pulseUs = static_cast<float>(config_.steeringCenterPulseUs);
  if (hardwareCommand < 0.0F) {
    pulseUs += hardwareCommand *
               static_cast<float>(config_.steeringCenterPulseUs -
                                  config_.steeringLeftPulseUs);
  } else {
    pulseUs += hardwareCommand *
               static_cast<float>(config_.steeringRightPulseUs -
                                  config_.steeringCenterPulseUs);
  }
  writeSteeringPulseUs(static_cast<uint16_t>(std::lround(pulseUs)));
}

bool VehicleActuators::setForward(float command) {
  return setDrive(clampFloat(command, 0.0F, 1.0F));
}

bool VehicleActuators::setDrive(float command) {
  return setDriveWithResult(command) == DriveResult::Applied;
}

VehicleActuators::DriveResult VehicleActuators::setDriveWithResult(float command) {
  const float requestedCommand = clampFloat(command, -1.0F, 1.0F);
  if (requestedCommand == 0.0F) {
    stop();
    return DriveResult::Applied;
  }

  if (!ready_ || faulted_ || leaseExpired()) {
    stop();
    return DriveResult::Rejected;
  }

  // Never reverse an already energized bridge in one call. The caller must
  // explicitly stop first, allowing it to enforce a vehicle-appropriate
  // neutral delay before commanding the opposite direction.
  if ((driveCommand_ > 0.0F && requestedCommand < 0.0F) ||
      (driveCommand_ < 0.0F && requestedCommand > 0.0F)) {
    stop();
    return DriveResult::Rejected;
  }

  const float magnitude = std::fabs(requestedCommand);
  const int8_t requestedDirection = requestedCommand > 0.0F ? 1 : -1;
  const uint32_t nowUs = static_cast<uint32_t>(esp_timer_get_time());
  if (lastNonzeroDirection_ != 0 &&
      requestedDirection != lastNonzeroDirection_ &&
      static_cast<uint32_t>(nowUs - lastStoppedUs_) <
          DRIVE_REVERSAL_NEUTRAL_US) {
    stop();
    return DriveResult::WaitingForNeutral;
  }
  const uint32_t duty = static_cast<uint32_t>(
      std::lround(magnitude * static_cast<float>(MOTOR_MAX_DUTY)));
  const bool useRpwm = requestedCommand > 0.0F
                           ? config_.forwardUsesRpwm
                           : !config_.forwardUsesRpwm;

  // Program the bridge while its previous enable state is protected by the
  // previous lease. The implementation never drives RPWM and LPWM together.
  if (useRpwm) {
    ledcWrite(LPWM_LEDC_CHANNEL, 0U);
    ledcWrite(RPWM_LEDC_CHANNEL, duty);
  } else {
    ledcWrite(RPWM_LEDC_CHANNEL, 0U);
    ledcWrite(LPWM_LEDC_CHANNEL, duty);
  }

  bool accepted = false;
  portENTER_CRITICAL(&driveLeaseMux_);
  if (!driveLeaseExpired_) {
    lastDriveLeaseRenewUs_ = static_cast<uint32_t>(esp_timer_get_time());
    driveLeaseArmed_ = true;
    // Keep the lease renewal and enable edge atomic relative to the callback;
    // an expiration can therefore never race with a later HIGH write.
    gpio_set_level(static_cast<gpio_num_t>(board::DRIVE_ENABLE_PIN), 1);
    accepted = true;
  }
  portEXIT_CRITICAL(&driveLeaseMux_);

  if (!accepted) {
    stop();
    return DriveResult::Rejected;
  }
  driveCommand_ = requestedCommand;
  lastNonzeroDirection_ = requestedDirection;
  return DriveResult::Applied;
}

void VehicleActuators::stop() {
  // Disable the bridge first, then clear both PWM signals. This remains safe
  // even if stop() interrupts a command update.
  gpio_set_level(static_cast<gpio_num_t>(board::DRIVE_ENABLE_PIN), 0);
  portENTER_CRITICAL(&driveLeaseMux_);
  driveLeaseArmed_ = false;
  portEXIT_CRITICAL(&driveLeaseMux_);
  if (ready_) {
    ledcWrite(RPWM_LEDC_CHANNEL, 0U);
    ledcWrite(LPWM_LEDC_CHANNEL, 0U);
  } else {
    digitalWrite(board::DRIVE_RPWM_PIN, LOW);
    digitalWrite(board::DRIVE_LPWM_PIN, LOW);
  }
  if (driveCommand_ != 0.0F) {
    lastStoppedUs_ = static_cast<uint32_t>(esp_timer_get_time());
  }
  driveCommand_ = 0.0F;
}

void VehicleActuators::faultStop() {
  faulted_ = true;
  stop();
  setSteering(0.0F);
}

void VehicleActuators::clearFault() {
  stop();
  // A lease expiration means loop() failed to renew drive in time. It remains
  // a permanent actuator fault until the MCU is deliberately restarted.
  faulted_ = leaseExpired();
}

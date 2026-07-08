#pragma once

#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

// Owns the steering-servo and IBT-2 PWM outputs used by an explicitly enabled
// autonomous build. Constructing this class has no hardware side effects;
// beginSafe() must be called before commands are accepted.
class VehicleActuators {
 public:
  enum class DriveResult : uint8_t {
    Applied,
    WaitingForNeutral,
    Rejected,
  };

  struct Config {
    // Measured on the assembled Ackermann linkage on 2026-09-01. The physical
    // limits were 900/2100 us; production stays 20 us inside both endpoints.
    // A command of -1 is left, 0 is centre, and +1 is right.
    uint16_t steeringLeftPulseUs = 920;
    uint16_t steeringCenterPulseUs = 1500;
    uint16_t steeringRightPulseUs = 2080;
    bool steeringReversed = false;

    // On most IBT-2 installations forward uses RPWM while LPWM stays LOW. Flip
    // this setting if the motor wiring makes that direction run backward.
    bool forwardUsesRpwm = true;
  };

  VehicleActuators();
  explicit VehicleActuators(const Config& config);
  VehicleActuators(const VehicleActuators&) = delete;
  VehicleActuators& operator=(const VehicleActuators&) = delete;

  // Starts with the motor disabled, both motor PWM duties at zero, and the
  // steering servo centred. Returns false and leaves all outputs safe if the
  // configuration or any LEDC timer setup is invalid.
  bool beginSafe();

  // Normalized commands are clamped to their documented ranges.
  void setSteering(float command);  // -1.0 left, 0.0 centre, +1.0 right
  // A nonzero command also renews the independent 150 ms drive lease. False
  // means the command was rejected and drive remains disabled. setDrive()
  // accepts reverse commands for calibrated recovery/parking maneuvers. A
  // direct sign change is rejected; command zero first so the drivetrain gets
  // a neutral interval before reversing.
  bool setForward(float command);  // 0.0 stopped, 1.0 configured maximum
  bool setDrive(float command);    // -1.0 reverse, 0.0 stopped, +1.0 forward
  // WaitingForNeutral leaves the bridge disabled and may be retried on the
  // next control update. It is only returned after an explicit stop while the
  // full electrical neutral interval, measured at that stop, is still pending.
  // All other rejection paths remain non-retryable actuator failures.
  DriveResult setDriveWithResult(float command);

  // stop() disables only drive motion. faultStop() additionally centres the
  // steering and latches out later drive commands until clearFault() is called.
  // An expired drive lease is stronger and requires an MCU restart.
  void stop();
  void faultStop();
  void clearFault();

  bool ready() const { return ready_; }
  bool faulted() const { return faulted_; }
  bool leaseExpired() const;
  float steeringCommand() const { return steeringCommand_; }
  float driveCommand() const { return driveCommand_; }

 private:
  static constexpr uint8_t SERVO_LEDC_CHANNEL = 0;
  // Channels 2 and 3 share an LEDC timer, which is intentional: both IBT-2
  // inputs use the same frequency and resolution. Channel 0 uses another timer.
  static constexpr uint8_t RPWM_LEDC_CHANNEL = 2;
  static constexpr uint8_t LPWM_LEDC_CHANNEL = 3;

  static constexpr uint32_t SERVO_FREQUENCY_HZ = 50;
  // ESP32-S3 LEDC timers support at most 14-bit resolution.
  static constexpr uint8_t SERVO_RESOLUTION_BITS = 14;
  static constexpr uint32_t MOTOR_FREQUENCY_HZ = 20000;
  static constexpr uint8_t MOTOR_RESOLUTION_BITS = 10;
  static constexpr uint32_t MOTOR_MAX_DUTY =
      (1UL << MOTOR_RESOLUTION_BITS) - 1UL;
  static constexpr uint32_t DRIVE_LEASE_CHECK_US = 10000U;
  static constexpr uint32_t DRIVE_LEASE_TIMEOUT_US = 150000U;
  // Match the conservative calibration interlock. Recovery code may only
  // shorten this after measured drivetrain coast-down validation.
  static constexpr uint32_t DRIVE_REVERSAL_NEUTRAL_US = 300000U;

  bool configIsValid() const;
  bool startDriveLeaseTimer();
  void stopDriveLeaseTimer();
  static void driveLeaseTimerCallback(void* argument);
  void forcePinsLowBeforeAttach();
  void leavePinsLowAfterSetupFailure();
  void writeSteeringPulseUs(uint16_t pulseUs);

  Config config_;
  bool ready_ = false;
  bool faulted_ = false;
  float steeringCommand_ = 0.0F;
  float driveCommand_ = 0.0F;

  esp_timer_handle_t driveLeaseTimer_ = nullptr;
  mutable portMUX_TYPE driveLeaseMux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t lastDriveLeaseRenewUs_ = 0;
  volatile bool driveLeaseArmed_ = false;
  volatile bool driveLeaseExpired_ = false;
  int8_t lastNonzeroDirection_ = 0;
  uint32_t lastStoppedUs_ = 0;
};

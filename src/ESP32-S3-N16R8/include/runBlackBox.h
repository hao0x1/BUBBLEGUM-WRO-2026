#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace runblackbox {

constexpr uint32_t kRecordMagic = 0x58424742UL;  // "BGBX" on little endian.
constexpr uint16_t kRecordSchema = 1U;
constexpr uint16_t kNoValidFrontRange = 0U;

enum class TerminalStatus : uint8_t {
  Active = 1,
  Finished = 2,
  FaultLatched = 3,
};

enum HealthBit : uint16_t {
  HealthImuOnline = 1U << 0,
  HealthImuValid = 1U << 1,
  HealthFrontOnline = 1U << 2,
  HealthFrontValid = 1U << 3,
  HealthAllSidesOnline = 1U << 4,
  HealthSideGeometryValid = 1U << 5,
  HealthVisionOnline = 1U << 6,
  HealthVisionValid = 1U << 7,
  HealthStartGateStable = 1U << 8,
  HealthProtocolOkay = 1U << 9,
};

// The explicit reserved bytes keep the CRC boundary identical on the ESP32
// and in host tests. Every field has a fixed-width type so records remain
// self-describing across firmware rebuilds.
struct Record {
  uint32_t magic;
  uint16_t schema;
  uint16_t recordSize;
  uint32_t faultFlags;
  uint32_t elapsedMs;
  uint16_t minimumValidFrontMm;
  uint16_t maximumAbsoluteSteeringPermille;
  uint16_t healthBits;
  uint8_t terminalStatus;
  int8_t direction;
  uint8_t cornerCount;
  uint8_t lapCount;
  uint8_t actuatorFault;
  uint8_t lastObstacle;
  uint8_t learnedRedSections;
  uint8_t learnedGreenSections;
  uint8_t reserved[2];
  uint32_t crc32;
};

static_assert(offsetof(Record, crc32) == 32U,
              "Run black-box CRC boundary changed");
static_assert(sizeof(Record) == 36U, "Run black-box record layout changed");

struct Summary {
  uint32_t faultFlags = 0;
  uint32_t elapsedMs = 0;
  uint16_t minimumValidFrontMm = kNoValidFrontRange;
  uint16_t maximumAbsoluteSteeringPermille = 0;
  uint16_t healthBits = 0;
  int8_t direction = 0;
  uint8_t cornerCount = 0;
  uint8_t lapCount = 0;
  bool actuatorFault = false;
  uint8_t lastObstacle = 0;
  uint8_t learnedRedSections = 0;
  uint8_t learnedGreenSections = 0;
};

uint32_t crc32(const uint8_t* data, size_t length);
Record makeRecord(TerminalStatus status, const Summary& summary);
bool validateRecord(const Record& record);
const char* statusName(TerminalStatus status);
const char* directionName(int8_t direction);
const char* obstacleName(uint8_t obstacle);

#ifdef ARDUINO
// Persistent storage exists only in the autonomous firmware. During a run,
// observe() changes RAM only. persistActive() and persistTerminal() are the
// only methods that may write NVS, and main.cpp calls them only while outputs
// are already stopped.
class Recorder {
 public:
  void begin(Print& log);
  void startRun(uint32_t nowMs, const Summary& initial, Print& log);
  void observe(uint32_t nowMs, const Summary& latest);
  void persistActive(bool outputsStoppedInsideServoSettle, Print& log);
  void persistTerminal(TerminalStatus status, bool outputsStopped, Print& log);

  bool runActive() const { return runActive_; }

 private:
  bool store(TerminalStatus status, Print& log);
  void printRecord(const Record& record, Print& log) const;

  Summary summary_{};
  uint32_t runStartedAtMs_ = 0;
  bool runActive_ = false;
  bool activeAttempted_ = false;
  bool terminalAttempted_ = false;
};
#endif

}  // namespace runblackbox

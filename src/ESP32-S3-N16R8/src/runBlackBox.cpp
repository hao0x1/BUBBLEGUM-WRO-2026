#include "runBlackBox.h"

#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <Preferences.h>
#endif

namespace runblackbox {

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      const uint32_t mask =
          static_cast<uint32_t>(-static_cast<int32_t>(crc & 1U));
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

Record makeRecord(TerminalStatus status, const Summary& summary) {
  Record record;
  memset(&record, 0, sizeof(record));
  record.magic = kRecordMagic;
  record.schema = kRecordSchema;
  record.recordSize = sizeof(Record);
  record.faultFlags = summary.faultFlags;
  record.elapsedMs = summary.elapsedMs;
  record.minimumValidFrontMm = summary.minimumValidFrontMm;
  record.maximumAbsoluteSteeringPermille =
      summary.maximumAbsoluteSteeringPermille;
  record.healthBits = summary.healthBits;
  record.terminalStatus = static_cast<uint8_t>(status);
  record.direction = summary.direction;
  record.cornerCount = summary.cornerCount;
  record.lapCount = summary.lapCount;
  record.actuatorFault = summary.actuatorFault ? 1U : 0U;
  record.lastObstacle = summary.lastObstacle;
  record.learnedRedSections = summary.learnedRedSections;
  record.learnedGreenSections = summary.learnedGreenSections;
  record.crc32 =
      crc32(reinterpret_cast<const uint8_t*>(&record), offsetof(Record, crc32));
  return record;
}

bool validateRecord(const Record& record) {
  if (record.magic != kRecordMagic || record.schema != kRecordSchema ||
      record.recordSize != sizeof(Record)) {
    return false;
  }
  if (record.terminalStatus < static_cast<uint8_t>(TerminalStatus::Active) ||
      record.terminalStatus >
          static_cast<uint8_t>(TerminalStatus::FaultLatched)) {
    return false;
  }
  return record.crc32 ==
         crc32(reinterpret_cast<const uint8_t*>(&record),
               offsetof(Record, crc32));
}

const char* statusName(TerminalStatus status) {
  switch (status) {
    case TerminalStatus::Active:
      return "ACTIVE";
    case TerminalStatus::Finished:
      return "FINISHED";
    case TerminalStatus::FaultLatched:
      return "FAULT_LATCHED";
  }
  return "UNKNOWN";
}

const char* directionName(int8_t direction) {
  if (direction < 0) return "CLOCKWISE";
  if (direction > 0) return "COUNTERCLOCKWISE";
  return "UNKNOWN";
}

const char* obstacleName(uint8_t obstacle) {
  if (obstacle == 1U) return "RED_PASS_RIGHT";
  if (obstacle == 2U) return "GREEN_PASS_LEFT";
  return "NONE";
}

#ifdef ARDUINO
namespace {
constexpr char kNamespace[] = "bubblegumRun";
constexpr char kLastRecordKey[] = "last";
}

void Recorder::begin(Print& log) {
  Preferences preferences;
  if (!preferences.begin(kNamespace, true)) {
    log.println(F("BLACKBOX_LAST,NONE"));
    return;
  }

  const size_t storedSize = preferences.getBytesLength(kLastRecordKey);
  if (storedSize == 0U) {
    preferences.end();
    log.println(F("BLACKBOX_LAST,NONE"));
    return;
  }
  if (storedSize != sizeof(Record)) {
    preferences.end();
    log.println(F("BLACKBOX_LAST,CORRUPT"));
    return;
  }

  Record record;
  const size_t readSize =
      preferences.getBytes(kLastRecordKey, &record, sizeof(record));
  preferences.end();
  if (readSize != sizeof(record) || !validateRecord(record)) {
    log.println(F("BLACKBOX_LAST,CORRUPT"));
    return;
  }
  printRecord(record, log);
}

void Recorder::startRun(uint32_t nowMs, const Summary& initial, Print& log) {
  (void)log;
  if (runActive_) return;
  summary_ = initial;
  summary_.elapsedMs = 0;
  runStartedAtMs_ = nowMs;
  runActive_ = true;
  activeAttempted_ = false;
  terminalAttempted_ = false;
}

void Recorder::observe(uint32_t nowMs, const Summary& latest) {
  if (!runActive_) return;
  summary_.faultFlags = latest.faultFlags;
  summary_.elapsedMs = static_cast<uint32_t>(nowMs - runStartedAtMs_);
  if (latest.minimumValidFrontMm != kNoValidFrontRange &&
      (summary_.minimumValidFrontMm == kNoValidFrontRange ||
       latest.minimumValidFrontMm < summary_.minimumValidFrontMm)) {
    summary_.minimumValidFrontMm = latest.minimumValidFrontMm;
  }
  if (latest.maximumAbsoluteSteeringPermille >
      summary_.maximumAbsoluteSteeringPermille) {
    summary_.maximumAbsoluteSteeringPermille =
        latest.maximumAbsoluteSteeringPermille;
  }
  summary_.healthBits = latest.healthBits;
  summary_.direction = latest.direction;
  summary_.cornerCount = latest.cornerCount;
  summary_.lapCount = latest.lapCount;
  summary_.actuatorFault =
      summary_.actuatorFault || latest.actuatorFault;
  if (latest.lastObstacle != 0U) {
    summary_.lastObstacle = latest.lastObstacle;
  }
  summary_.learnedRedSections = latest.learnedRedSections;
  summary_.learnedGreenSections = latest.learnedGreenSections;
}

void Recorder::persistActive(bool outputsStoppedInsideServoSettle,
                             Print& log) {
  if (!runActive_ || activeAttempted_) return;
  activeAttempted_ = true;
  if (!outputsStoppedInsideServoSettle) {
    log.println(F("[blackbox] WARNING: ACTIVE write skipped outside safe stop"));
    return;
  }
  store(TerminalStatus::Active, log);
}

void Recorder::persistTerminal(TerminalStatus status, bool outputsStopped,
                               Print& log) {
  if (!runActive_ || terminalAttempted_ ||
      status == TerminalStatus::Active) {
    return;
  }
  if (!outputsStopped) {
    log.println(F("[blackbox] WARNING: terminal write deferred until stopped"));
    return;
  }
  terminalAttempted_ = true;
  store(status, log);
  runActive_ = false;
}

bool Recorder::store(TerminalStatus status, Print& log) {
  const Record record = makeRecord(status, summary_);
  Preferences preferences;
  if (!preferences.begin(kNamespace, false)) {
    log.println(F("[blackbox] WARNING: NVS unavailable; driving unaffected"));
    return false;
  }
  const size_t written =
      preferences.putBytes(kLastRecordKey, &record, sizeof(record));
  preferences.end();
  if (written != sizeof(record)) {
    log.println(F("[blackbox] WARNING: NVS write failed; driving unaffected"));
    return false;
  }
  log.print(F("BLACKBOX_SAVED,"));
  log.println(statusName(status));
  return true;
}

void Recorder::printRecord(const Record& record, Print& log) const {
  log.print(F("BLACKBOX_LAST,status="));
  log.print(statusName(static_cast<TerminalStatus>(record.terminalStatus)));
  log.print(F(",direction="));
  log.print(directionName(record.direction));
  log.print(F(",corners="));
  log.print(record.cornerCount);
  log.print(F(",laps="));
  log.print(record.lapCount);
  log.print(F(",fault=0x"));
  log.print(record.faultFlags, HEX);
  log.print(F(",actuator_fault="));
  log.print(record.actuatorFault);
  log.print(F(",last_obstacle="));
  log.print(obstacleName(record.lastObstacle));
  log.print(F(",red_sections=0x"));
  log.print(record.learnedRedSections, HEX);
  log.print(F(",green_sections=0x"));
  log.print(record.learnedGreenSections, HEX);
  log.print(F(",min_front_mm="));
  log.print(record.minimumValidFrontMm);
  log.print(F(",health=0x"));
  log.print(record.healthBits, HEX);
  log.print(F(",elapsed_ms="));
  log.print(record.elapsedMs);
  log.print(F(",max_abs_steer="));
  log.println(record.maximumAbsoluteSteeringPermille);
}

#endif

}  // namespace runblackbox

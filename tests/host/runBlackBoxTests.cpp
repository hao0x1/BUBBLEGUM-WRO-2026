#include "runBlackBox.h"

#include <cstdint>
#include <iostream>

namespace {
int failures = 0;

void check(bool condition, const char* message) {
  if (condition) return;
  ++failures;
  std::cerr << "FAIL: " << message << '\n';
}
}  // namespace

int main() {
  const uint8_t standardVector[] = {'1', '2', '3', '4', '5',
                                    '6', '7', '8', '9'};
  check(runblackbox::crc32(standardVector, sizeof(standardVector)) ==
            0xCBF43926UL,
        "CRC32 must match the standard check vector");

  runblackbox::Summary summary;
  summary.faultFlags = 0x12345678UL;
  summary.elapsedMs = 170001UL;
  summary.minimumValidFrontMm = 181U;
  summary.maximumAbsoluteSteeringPermille = 760U;
  summary.healthBits = 0x03FFU;
  summary.direction = -1;
  summary.cornerCount = 12U;
  summary.lapCount = 3U;
  summary.actuatorFault = true;
  summary.lastObstacle = 2U;
  summary.learnedRedSections = 0x05U;
  summary.learnedGreenSections = 0x0AU;

  const runblackbox::Record record = runblackbox::makeRecord(
      runblackbox::TerminalStatus::FaultLatched, summary);
  check(runblackbox::validateRecord(record),
        "new record must pass magic/schema/size/CRC validation");
  check(record.minimumValidFrontMm == 181U &&
            record.maximumAbsoluteSteeringPermille == 760U,
        "summary metrics must survive record construction");
  check(record.cornerCount == 12U && record.lapCount == 3U &&
            record.learnedRedSections == 0x05U &&
            record.learnedGreenSections == 0x0AU,
        "course summary must survive record construction");

  runblackbox::Record corrupt = record;
  corrupt.elapsedMs ^= 1U;
  check(!runblackbox::validateRecord(corrupt),
        "one corrupted payload bit must fail CRC validation");
  corrupt = record;
  corrupt.magic ^= 1U;
  check(!runblackbox::validateRecord(corrupt),
        "wrong magic must fail validation");
  corrupt = record;
  corrupt.schema += 1U;
  check(!runblackbox::validateRecord(corrupt),
        "unknown schema must fail validation");
  corrupt = record;
  corrupt.terminalStatus = 0U;
  corrupt.crc32 = runblackbox::crc32(
      reinterpret_cast<const uint8_t*>(&corrupt),
      offsetof(runblackbox::Record, crc32));
  check(!runblackbox::validateRecord(corrupt),
        "unknown terminal status must fail validation even with a valid CRC");

  if (failures != 0) return 1;
  std::cout << "run black-box host checks passed\n";
  return 0;
}

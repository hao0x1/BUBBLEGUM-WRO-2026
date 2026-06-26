#pragma once

#include <stddef.h>
#include <stdint.h>

namespace vision {

constexpr uint8_t SYNC_0 = 0xA5;
constexpr uint8_t SYNC_1 = 0x5A;
constexpr uint8_t VERSION = 1;
constexpr size_t FIXED_HEADER_SIZE = 10;  // version..payload length
constexpr size_t MAX_PAYLOAD_SIZE = 64;
constexpr size_t MAX_BODY_SIZE = FIXED_HEADER_SIZE + MAX_PAYLOAD_SIZE + 2;
constexpr size_t HEARTBEAT_PAYLOAD_SIZE = 8;
constexpr size_t OBSERVATION_PAYLOAD_SIZE = 52;

constexpr uint32_t SESSION_REBASE_AFTER_MS = 1500;
constexpr uint32_t MAX_SENDER_STEP_MS = 5000;
constexpr uint16_t MAX_SEQUENCE_GAP = 4096;
constexpr uint16_t MAX_PROCESSING_AGE_MS = 1000;

enum class MessageType : uint8_t {
  Heartbeat = 0x01,
  Observation = 0x10,
};

enum ObservationFlag : uint16_t {
  CameraValid = 1U << 0,
  TrackValid = 1U << 1,
  DirectionValid = 1U << 2,
  RangeValid = 1U << 3,
  RedPresent = 1U << 4,
  GreenPresent = 1U << 5,
  MagentaLeftPresent = 1U << 6,
  MagentaRightPresent = 1U << 7,
  PipelineOverrun = 1U << 8,
  ExposureUnstable = 1U << 9,
};

constexpr uint16_t KNOWN_OBSERVATION_FLAGS = 0x03FF;

struct Blob {
  int16_t centerX = 0;       // -10000 (left) .. +10000 (right)
  uint16_t bottomY = 0;      // 0 (top) .. 10000 (bottom)
  uint16_t width = 0;        // normalized image width
  uint16_t height = 0;       // normalized image height
  uint16_t confidence = 0;   // 1..1000 when present, 0 when absent
};

struct Heartbeat {
  uint8_t role = 0;
  uint8_t state = 0;
  uint16_t errorFlags = 0;
  uint16_t lastRxSequence = 0xFFFF;
  uint16_t loopRateHzX10 = 0;
  uint16_t sequence = 0;
  uint32_t senderTimestampMs = 0;
  uint32_t receivedAtMs = 0;
};

struct Observation {
  uint16_t flags = 0;
  int8_t courseDirection = 0;
  uint8_t frameQuality = 0;
  uint16_t processingAgeMs = 0;
  int16_t trackError = 0;
  int16_t pathHeadingMrad = 0;
  uint16_t nearestRangeMm = 0xFFFF;
  Blob red;
  Blob green;
  Blob magentaLeft;
  Blob magentaRight;
  uint16_t sequence = 0;
  uint32_t senderTimestampMs = 0;
  uint32_t receivedAtMs = 0;

  bool has(ObservationFlag flag) const {
    return (flags & static_cast<uint16_t>(flag)) != 0;
  }
};

struct DecoderStats {
  uint32_t validFrames = 0;
  uint32_t validHeartbeats = 0;
  uint32_t validObservations = 0;
  uint32_t crcErrors = 0;
  uint32_t lengthErrors = 0;
  uint32_t versionErrors = 0;
  uint32_t typeErrors = 0;
  uint32_t payloadErrors = 0;
  uint32_t sequenceErrors = 0;
  uint32_t timestampErrors = 0;
  uint32_t staleSessionErrors = 0;
  uint32_t droppedFrames = 0;
};

enum class DecodeEvent : uint8_t {
  None,
  HeartbeatAccepted,
  ObservationAccepted,
  FrameRejected,
};

uint16_t crc16CcittFalse(const uint8_t* data, size_t length);

class Decoder {
 public:
  DecodeEvent consume(uint8_t byte, uint32_t nowMs);
  void clear();

  bool haveHeartbeat() const { return haveHeartbeat_; }
  bool haveObservation() const { return haveObservation_; }
  const Heartbeat& heartbeat() const { return heartbeat_; }
  const Observation& observation() const { return observation_; }
  const DecoderStats& stats() const { return stats_; }
  bool heartbeatReadyAndFresh(uint32_t nowMs, uint32_t maxReceiptAgeMs) const;
  bool observationFresh(uint32_t nowMs, uint32_t maxReceiptAgeMs) const;

 private:
  enum class ParseState : uint8_t { SeekSync0, SeekSync1, CollectBody };

  DecodeEvent validateAndDispatch(uint32_t nowMs);
  bool validateSession(uint16_t sequence, uint32_t senderTimestampMs,
                       MessageType type, bool bootHeartbeat, uint32_t nowMs);
  bool decodeHeartbeat(const uint8_t* payload, Heartbeat& output) const;
  bool decodeObservation(const uint8_t* payload, Observation& output) const;
  static bool validateBlob(const uint8_t* raw, const Blob& blob, bool present);
  void resetParser(uint8_t lastByte = 0);
  void reject(uint32_t& counter);

  ParseState parseState_ = ParseState::SeekSync0;
  uint8_t body_[MAX_BODY_SIZE] = {};
  size_t bodyIndex_ = 0;
  size_t expectedBodySize_ = 0;

  bool haveSession_ = false;
  uint16_t lastSequence_ = 0;
  uint32_t lastSenderTimestampMs_ = 0;
  uint32_t lastValidReceiptMs_ = 0;
  bool haveHeartbeat_ = false;
  bool haveObservation_ = false;
  Heartbeat heartbeat_;
  Observation observation_;
  DecoderStats stats_;
};

// Runs the standard CRC check, all three shared golden vectors, corruption,
// duplicate-sequence, and out-of-range payload checks without any hardware.
bool protocolSelfTest();

}  // namespace vision

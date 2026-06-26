#include "visionProtocol.h"

#include <string.h>

namespace vision {
namespace {

uint16_t readU16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8U);
}

int16_t readI16(const uint8_t* data) {
  return static_cast<int16_t>(readU16(data));
}

uint32_t readU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

void writeU16(uint8_t* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFU);
  data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void writeU32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFU);
  data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

bool allZero(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    if (data[i] != 0U) return false;
  }
  return true;
}

}  // namespace

uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

void Decoder::clear() {
  parseState_ = ParseState::SeekSync0;
  bodyIndex_ = 0;
  expectedBodySize_ = 0;
  haveSession_ = false;
  lastSequence_ = 0;
  lastSenderTimestampMs_ = 0;
  lastValidReceiptMs_ = 0;
  haveHeartbeat_ = false;
  haveObservation_ = false;
  heartbeat_ = {};
  observation_ = {};
  stats_ = {};
}

DecodeEvent Decoder::consume(uint8_t byte, uint32_t nowMs) {
  switch (parseState_) {
    case ParseState::SeekSync0:
      if (byte == SYNC_0) parseState_ = ParseState::SeekSync1;
      return DecodeEvent::None;

    case ParseState::SeekSync1:
      if (byte == SYNC_1) {
        parseState_ = ParseState::CollectBody;
        bodyIndex_ = 0;
        expectedBodySize_ = 0;
      } else {
        parseState_ = byte == SYNC_0 ? ParseState::SeekSync1
                                     : ParseState::SeekSync0;
      }
      return DecodeEvent::None;

    case ParseState::CollectBody:
      if (bodyIndex_ >= sizeof(body_)) {
        reject(stats_.lengthErrors);
        resetParser(byte);
        return DecodeEvent::FrameRejected;
      }
      body_[bodyIndex_++] = byte;
      if (bodyIndex_ == FIXED_HEADER_SIZE) {
        const uint16_t payloadLength = readU16(&body_[8]);
        if (payloadLength > MAX_PAYLOAD_SIZE) {
          reject(stats_.lengthErrors);
          resetParser(byte);
          return DecodeEvent::FrameRejected;
        }
        expectedBodySize_ = FIXED_HEADER_SIZE + payloadLength + 2U;
      }
      if (expectedBodySize_ != 0U && bodyIndex_ == expectedBodySize_) {
        const DecodeEvent event = validateAndDispatch(nowMs);
        resetParser();
        return event;
      }
      return DecodeEvent::None;
  }
  return DecodeEvent::None;
}

void Decoder::resetParser(uint8_t lastByte) {
  parseState_ = lastByte == SYNC_0 ? ParseState::SeekSync1
                                   : ParseState::SeekSync0;
  bodyIndex_ = 0;
  expectedBodySize_ = 0;
}

void Decoder::reject(uint32_t& counter) { ++counter; }

bool Decoder::decodeHeartbeat(const uint8_t* payload, Heartbeat& output) const {
  output.role = payload[0];
  output.state = payload[1];
  output.errorFlags = readU16(&payload[2]);
  output.lastRxSequence = readU16(&payload[4]);
  output.loopRateHzX10 = readU16(&payload[6]);
  return output.role == 1U && output.state <= 3U &&
         (output.errorFlags & 0xFF00U) == 0U;
}

bool Decoder::validateBlob(const uint8_t* raw, const Blob& blob,
                           bool present) {
  if (!present) return allZero(raw, 10U);
  return blob.centerX >= -10000 && blob.centerX <= 10000 &&
         blob.bottomY <= 10000U && blob.width > 0U && blob.width <= 10000U &&
         blob.height > 0U && blob.height <= 10000U &&
         blob.confidence > 0U && blob.confidence <= 1000U;
}

bool Decoder::decodeObservation(const uint8_t* payload,
                                Observation& output) const {
  output.flags = readU16(&payload[0]);
  output.courseDirection = static_cast<int8_t>(payload[2]);
  output.frameQuality = payload[3];
  output.processingAgeMs = readU16(&payload[4]);
  output.trackError = readI16(&payload[6]);
  output.pathHeadingMrad = readI16(&payload[8]);
  output.nearestRangeMm = readU16(&payload[10]);

  if ((output.flags & ~KNOWN_OBSERVATION_FLAGS) != 0U ||
      output.courseDirection < -1 || output.courseDirection > 1 ||
      output.frameQuality > 100U ||
      output.processingAgeMs > MAX_PROCESSING_AGE_MS ||
      output.trackError < -10000 || output.trackError > 10000 ||
      output.pathHeadingMrad < -6284 || output.pathHeadingMrad > 6284) {
    return false;
  }

  if ((!output.has(DirectionValid) && output.courseDirection != 0) ||
      (output.has(DirectionValid) && output.courseDirection == 0)) {
    return false;
  }
  if (!output.has(TrackValid) &&
      (output.trackError != 0 || output.pathHeadingMrad != 0)) {
    return false;
  }
  if ((output.has(RangeValid) && output.nearestRangeMm == 0xFFFFU) ||
      (!output.has(RangeValid) && output.nearestRangeMm != 0xFFFFU)) {
    return false;
  }

  constexpr uint16_t navigationAndTargetFlags =
      TrackValid | DirectionValid | RangeValid | RedPresent | GreenPresent |
      MagentaLeftPresent | MagentaRightPresent;
  if (!output.has(CameraValid) &&
      (((output.flags & navigationAndTargetFlags) != 0U) ||
       output.frameQuality != 0U)) {
    return false;
  }

  Blob* blobs[] = {&output.red, &output.green, &output.magentaLeft,
                   &output.magentaRight};
  constexpr ObservationFlag presenceFlags[] = {
      RedPresent, GreenPresent, MagentaLeftPresent, MagentaRightPresent};
  size_t offset = 12U;
  for (size_t i = 0; i < 4U; ++i, offset += 10U) {
    blobs[i]->centerX = readI16(&payload[offset]);
    blobs[i]->bottomY = readU16(&payload[offset + 2U]);
    blobs[i]->width = readU16(&payload[offset + 4U]);
    blobs[i]->height = readU16(&payload[offset + 6U]);
    blobs[i]->confidence = readU16(&payload[offset + 8U]);
    if (!validateBlob(&payload[offset], *blobs[i],
                      output.has(presenceFlags[i]))) {
      return false;
    }
  }
  return true;
}

bool Decoder::validateSession(uint16_t sequence, uint32_t senderTimestampMs,
                              MessageType type, bool bootHeartbeat,
                              uint32_t nowMs) {
  const bool locallyTimedOut =
      haveSession_ &&
      static_cast<uint32_t>(nowMs - lastValidReceiptMs_) >
          SESSION_REBASE_AFTER_MS;

  if (bootHeartbeat || (type == MessageType::Heartbeat && locallyTimedOut)) {
    haveSession_ = false;
    haveHeartbeat_ = false;
    haveObservation_ = false;
    heartbeat_ = {};
    observation_ = {};
  } else if (locallyTimedOut && type == MessageType::Observation) {
    reject(stats_.staleSessionErrors);
    return false;
  }

  if (haveSession_) {
    const uint16_t sequenceStep =
        static_cast<uint16_t>(sequence - lastSequence_);
    if (sequenceStep == 0U || sequenceStep > MAX_SEQUENCE_GAP) {
      reject(stats_.sequenceErrors);
      return false;
    }
    const int32_t senderStep =
        static_cast<int32_t>(senderTimestampMs - lastSenderTimestampMs_);
    // Sequence protects against replay. A 1 ms sender clock may legitimately
    // stamp an observation and heartbeat with the same millisecond.
    if (senderStep < 0 ||
        static_cast<uint32_t>(senderStep) > MAX_SENDER_STEP_MS) {
      reject(stats_.timestampErrors);
      return false;
    }
  }
  return true;
}

DecodeEvent Decoder::validateAndDispatch(uint32_t nowMs) {
  const uint16_t payloadLength = readU16(&body_[8]);
  const size_t crcOffset = FIXED_HEADER_SIZE + payloadLength;
  if (crc16CcittFalse(body_, crcOffset) != readU16(&body_[crcOffset])) {
    reject(stats_.crcErrors);
    return DecodeEvent::FrameRejected;
  }
  if (body_[0] != VERSION) {
    reject(stats_.versionErrors);
    return DecodeEvent::FrameRejected;
  }

  const auto type = static_cast<MessageType>(body_[1]);
  if (type != MessageType::Heartbeat && type != MessageType::Observation) {
    reject(stats_.typeErrors);
    return DecodeEvent::FrameRejected;
  }
  if ((type == MessageType::Heartbeat &&
       payloadLength != HEARTBEAT_PAYLOAD_SIZE) ||
      (type == MessageType::Observation &&
       payloadLength != OBSERVATION_PAYLOAD_SIZE)) {
    reject(stats_.lengthErrors);
    return DecodeEvent::FrameRejected;
  }

  const uint8_t* payload = &body_[FIXED_HEADER_SIZE];
  Heartbeat decodedHeartbeat;
  Observation decodedObservation;
  bool bootHeartbeat = false;
  if (type == MessageType::Heartbeat) {
    if (!decodeHeartbeat(payload, decodedHeartbeat)) {
      reject(stats_.payloadErrors);
      return DecodeEvent::FrameRejected;
    }
    bootHeartbeat = decodedHeartbeat.state == 0U;
  } else if (!decodeObservation(payload, decodedObservation)) {
    reject(stats_.payloadErrors);
    return DecodeEvent::FrameRejected;
  }

  // Never establish or use a camera stream before a heartbeat has identified
  // the sender/session. This makes a lost BOOT/READY frame fail closed.
  if (type == MessageType::Observation && !haveHeartbeat_) {
    reject(stats_.staleSessionErrors);
    return DecodeEvent::FrameRejected;
  }

  const uint16_t sequence = readU16(&body_[2]);
  const uint32_t senderTimestampMs = readU32(&body_[4]);
  if (!validateSession(sequence, senderTimestampMs, type, bootHeartbeat,
                       nowMs)) {
    return DecodeEvent::FrameRejected;
  }

  if (haveSession_) {
    const uint16_t sequenceStep =
        static_cast<uint16_t>(sequence - lastSequence_);
    if (sequenceStep > 1U) {
      stats_.droppedFrames += static_cast<uint32_t>(sequenceStep - 1U);
    }
  }

  lastSequence_ = sequence;
  lastSenderTimestampMs_ = senderTimestampMs;
  lastValidReceiptMs_ = nowMs;
  haveSession_ = true;
  ++stats_.validFrames;

  if (type == MessageType::Heartbeat) {
    decodedHeartbeat.sequence = sequence;
    decodedHeartbeat.senderTimestampMs = senderTimestampMs;
    decodedHeartbeat.receivedAtMs = nowMs;
    heartbeat_ = decodedHeartbeat;
    haveHeartbeat_ = true;
    ++stats_.validHeartbeats;
    return DecodeEvent::HeartbeatAccepted;
  }

  decodedObservation.sequence = sequence;
  decodedObservation.senderTimestampMs = senderTimestampMs;
  decodedObservation.receivedAtMs = nowMs;
  observation_ = decodedObservation;
  haveObservation_ = true;
  ++stats_.validObservations;
  return DecodeEvent::ObservationAccepted;
}

bool Decoder::heartbeatReadyAndFresh(uint32_t nowMs,
                                     uint32_t maxReceiptAgeMs) const {
  return haveHeartbeat_ && heartbeat_.state == 1U &&
         static_cast<uint32_t>(nowMs - heartbeat_.receivedAtMs) <=
             maxReceiptAgeMs;
}

bool Decoder::observationFresh(uint32_t nowMs,
                               uint32_t maxReceiptAgeMs) const {
  if (!heartbeatReadyAndFresh(nowMs, maxReceiptAgeMs)) return false;
  if (!haveObservation_) return false;
  const uint32_t receiptAgeMs = nowMs - observation_.receivedAtMs;
  if (receiptAgeMs > maxReceiptAgeMs) return false;
  // End-to-end age is camera processing age plus time spent waiting after the
  // UART receipt.  Checking each term independently could admit nearly twice
  // the configured freshness limit.
  return static_cast<uint32_t>(observation_.processingAgeMs) <=
         maxReceiptAgeMs - receiptAgeMs;
}

namespace {

template <size_t N>
DecodeEvent feedFrame(Decoder& decoder, const uint8_t (&frame)[N],
                      uint32_t nowMs) {
  DecodeEvent event = DecodeEvent::None;
  for (size_t i = 0; i < N; ++i) event = decoder.consume(frame[i], nowMs);
  return event;
}

constexpr uint8_t GOLDEN_HEARTBEAT[] = {
    0xA5, 0x5A, 0x01, 0x01, 0x34, 0x12, 0x04, 0x03, 0x02, 0x01, 0x08,
    0x00, 0x01, 0x01, 0x02, 0x00, 0xEF, 0xBE, 0x2C, 0x01, 0xAD, 0x77};

constexpr uint8_t GOLDEN_RED[] = {
    0xA5, 0x5A, 0x01, 0x10, 0x07, 0x00, 0x40, 0xE2, 0x01, 0x00, 0x34,
    0x00, 0x11, 0x00, 0x00, 0x58, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0x05, 0x11, 0x95, 0x1D, 0x46, 0x04, 0xE7, 0x11, 0x2A,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C,
    0x3C};

constexpr uint8_t GOLDEN_BOTH[] = {
    0xA5, 0x5A, 0x01, 0x10, 0x08, 0x00, 0x56, 0xE2, 0x01, 0x00, 0x34,
    0x00, 0x31, 0x00, 0x00, 0x5B, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0x5E, 0x12, 0x95, 0x1D, 0xAA, 0x03, 0xA0, 0x0F, 0xF8,
    0x02, 0x88, 0xE6, 0xC4, 0x1C, 0x6B, 0x03, 0xC4, 0x10, 0x48, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB,
    0xA7};

static_assert(sizeof(GOLDEN_HEARTBEAT) == 22U, "heartbeat vector length");
static_assert(sizeof(GOLDEN_RED) == 66U, "observation vector length");
static_assert(sizeof(GOLDEN_BOTH) == 66U, "observation vector length");

bool feedReadyHeartbeat(Decoder& decoder, uint16_t sequence,
                        uint32_t senderTimestampMs, uint32_t nowMs) {
  uint8_t frame[sizeof(GOLDEN_HEARTBEAT)];
  memcpy(frame, GOLDEN_HEARTBEAT, sizeof(frame));
  writeU16(&frame[4], sequence);
  writeU32(&frame[6], senderTimestampMs);
  writeU16(&frame[20], crc16CcittFalse(&frame[2], 18U));

  DecodeEvent event = DecodeEvent::None;
  for (size_t i = 0; i < sizeof(frame); ++i) {
    event = decoder.consume(frame[i], nowMs);
  }
  return event == DecodeEvent::HeartbeatAccepted &&
         decoder.heartbeatReadyAndFresh(nowMs, 0U);
}

}  // namespace

bool protocolSelfTest() {
  static constexpr uint8_t checkText[] = {'1', '2', '3', '4', '5',
                                          '6', '7', '8', '9'};
  if (crc16CcittFalse(checkText, sizeof(checkText)) != 0x29B1U) return false;

  Decoder heartbeatDecoder;
  if (feedFrame(heartbeatDecoder, GOLDEN_HEARTBEAT, 100U) !=
          DecodeEvent::HeartbeatAccepted ||
      !heartbeatDecoder.haveHeartbeat() ||
      heartbeatDecoder.heartbeat().sequence != 0x1234U ||
      heartbeatDecoder.heartbeat().state != 1U) {
    return false;
  }
  // Same sequence and timestamp must be rejected even though the CRC is valid.
  if (feedFrame(heartbeatDecoder, GOLDEN_HEARTBEAT, 110U) !=
          DecodeEvent::FrameRejected ||
      heartbeatDecoder.stats().sequenceErrors != 1U) {
    return false;
  }

  // Two different message sequences may share a millisecond timestamp. The
  // sequence still prevents replay, so a coarse sender clock is not rejected.
  uint8_t sameMillisecond[sizeof(GOLDEN_HEARTBEAT)];
  memcpy(sameMillisecond, GOLDEN_HEARTBEAT, sizeof(sameMillisecond));
  writeU16(&sameMillisecond[4], 0x1235U);
  const uint16_t sameMillisecondCrc =
      crc16CcittFalse(&sameMillisecond[2], 18U);
  writeU16(&sameMillisecond[20], sameMillisecondCrc);
  DecodeEvent sameMillisecondEvent = DecodeEvent::None;
  for (size_t i = 0; i < sizeof(sameMillisecond); ++i) {
    sameMillisecondEvent = heartbeatDecoder.consume(sameMillisecond[i], 120U);
  }
  if (sameMillisecondEvent != DecodeEvent::HeartbeatAccepted ||
      heartbeatDecoder.heartbeat().sequence != 0x1235U) {
    return false;
  }

  // An otherwise valid observation must not establish a session by itself.
  // The K230 must first identify a READY session with a heartbeat.
  Decoder redDecoder;
  if (feedFrame(redDecoder, GOLDEN_RED, 180U) !=
          DecodeEvent::FrameRejected ||
      redDecoder.stats().staleSessionErrors != 1U ||
      redDecoder.haveObservation()) {
    return false;
  }
  const uint16_t redSequence = readU16(&GOLDEN_RED[4]);
  const uint32_t redTimestamp = readU32(&GOLDEN_RED[6]);
  if (!feedReadyHeartbeat(redDecoder,
                          static_cast<uint16_t>(redSequence - 1U),
                          redTimestamp - 1U, 190U) ||
      feedFrame(redDecoder, GOLDEN_RED, 200U) !=
          DecodeEvent::ObservationAccepted ||
      !redDecoder.observation().has(RedPresent) ||
      redDecoder.observation().has(GreenPresent) ||
      redDecoder.observation().red.confidence != 810U ||
      !redDecoder.observationFresh(431U, 250U) ||
      redDecoder.observationFresh(432U, 250U)) {
    return false;
  }

  Decoder bothDecoder;
  const uint16_t bothSequence = readU16(&GOLDEN_BOTH[4]);
  const uint32_t bothTimestamp = readU32(&GOLDEN_BOTH[6]);
  if (!feedReadyHeartbeat(bothDecoder,
                          static_cast<uint16_t>(bothSequence - 1U),
                          bothTimestamp - 1U, 290U) ||
      feedFrame(bothDecoder, GOLDEN_BOTH, 300U) !=
          DecodeEvent::ObservationAccepted ||
      !bothDecoder.observation().has(RedPresent) ||
      !bothDecoder.observation().has(GreenPresent)) {
    return false;
  }

  uint8_t corrupt[sizeof(GOLDEN_RED)];
  memcpy(corrupt, GOLDEN_RED, sizeof(corrupt));
  corrupt[25] ^= 0x40U;
  Decoder corruptDecoder;
  DecodeEvent corruptEvent = DecodeEvent::None;
  for (size_t i = 0; i < sizeof(corrupt); ++i) {
    corruptEvent = corruptDecoder.consume(corrupt[i], 400U);
  }
  if (corruptEvent != DecodeEvent::FrameRejected ||
      corruptDecoder.stats().crcErrors != 1U) {
    return false;
  }

  // Give an otherwise valid frame an impossible normalized red centre, then
  // update its CRC.  This proves field-range checks run after CRC validation.
  uint8_t outOfRange[sizeof(GOLDEN_RED)];
  memcpy(outOfRange, GOLDEN_RED, sizeof(outOfRange));
  outOfRange[24] = 0xFF;
  outOfRange[25] = 0x7F;
  const uint16_t crc = crc16CcittFalse(&outOfRange[2], 62U);
  writeU16(&outOfRange[64], crc);
  Decoder rangeDecoder;
  DecodeEvent rangeEvent = DecodeEvent::None;
  for (size_t i = 0; i < sizeof(outOfRange); ++i) {
    rangeEvent = rangeDecoder.consume(outOfRange[i], 500U);
  }
  return rangeEvent == DecodeEvent::FrameRejected &&
         rangeDecoder.stats().payloadErrors == 1U;
}

}  // namespace vision

"""K230 vision program for WRO traffic signs and parking markers.

Run this standalone file from RAM first with the CanMV VS Code extension. It
sends observations to the ESP32 through the pictured 40-pin UART header. The
ESP32 makes the steering and speed decisions. The values sent by this program
describe image geometry, not distance in cm.

The filtering and temporal-tracking code stays importable on a host computer
so its exact board logic can be tested without importing CanMV media modules.
The current production profile uses only red/green pillars; parking telemetry
is experimental and disabled below.
"""

import gc

try:
    import ustruct as struct
except ImportError:
    import struct


FRAME_WIDTH = 320
FRAME_HEIGHT = 240
PREVIEW_FPS = 15
PREVIEW_DIVISOR = 4
# Production default: no USB preview or overlay work in the driving loop.
# Change this one value to True only for a tethered CanMV calibration session;
# perception logic stays the same, but preview load can change timing/overrun
# diagnostics and must not be used for a driving performance claim.
ENABLE_IDE_PREVIEW = False

# Parking control is disabled in the current ESP32 Obstacle profile. Keep the
# extra full-frame colour scan off in production until the pair geometry has
# been validated from the mounted camera. When explicitly enabled, protocol
# "left/right" means image-left/image-right only, not a physical bay side.
ENABLE_EXPERIMENTAL_PARKING_TELEMETRY = False

# K230 -> ESP32 vision link through the pictured 40-pin header. Physical pin
# 11 is K230 GPIO5/UART2 TX, physical pin 13 is GPIO6/UART2 RX, and physical
# pin 14 is GND. Cross TX/RX at the ESP32. Never connect a 40-pin-header power
# pin between the boards; power the K230 separately through USB-C for IDE work
# or its dedicated 8-24 V input for offline use.
ENABLE_UART_LINK = True
UART_BAUDRATE = 115200
UART_TX_GPIO = 5
UART_RX_GPIO = 6
UART_HEARTBEAT_INTERVAL_MS = 100
PIPELINE_OVERRUN_AGE_MS = 80
UART_OBSERVATION_FRAME_BYTES = 66
UART_OBSERVATION_WIRE_TIME_MS = (
    UART_OBSERVATION_FRAME_BYTES * 10 * 1000 + UART_BAUDRATE - 1
) // UART_BAUDRATE

# Minimal embedded copy of protocol V1. The RAM-run tracker deliberately has
# no sibling-file import because CanMV does not upload protocol/ automatically.
UART_SYNC = b"\xA5\x5A"
UART_VERSION = 1
UART_TYPE_HEARTBEAT = 0x01
UART_TYPE_OBSERVATION = 0x10
UART_ROLE_K230 = 1
UART_STATE_BOOT = 0
UART_STATE_READY = 1
UART_STATE_DEGRADED = 2
UART_UNKNOWN_SEQUENCE = 0xFFFF
UART_UNKNOWN_RANGE_MM = 0xFFFF
UART_ERROR_LOOP_OVERRUN = 1 << 1
UART_ERROR_CONFIG_INVALID = 1 << 7
UART_FLAG_CAMERA_VALID = 1 << 0
UART_FLAG_RED_PRESENT = 1 << 4
UART_FLAG_GREEN_PRESENT = 1 << 5
UART_FLAG_MAGENTA_LEFT_PRESENT = 1 << 6
UART_FLAG_MAGENTA_RIGHT_PRESENT = 1 << 7
UART_FLAG_PIPELINE_OVERRUN = 1 << 8
UART_FLAG_EXPOSURE_UNSTABLE = 1 << 9
UART_OBSERVATION_PAYLOAD_SIZE = 52
UART_MAX_WRITE_ERRORS = 0xFFFF

# Starting values from a properly illuminated desk scene. The competition-day
# calibrator stores replacements on the SD card. Production tries the primary
# file, then its last-known-good backup, and finally these compiled defaults.
COLOR_CONFIG_LEGACY_MAGIC = "BUBBLEGUM_COLOR_V1"
COLOR_CONFIG_MAGIC = "BUBBLEGUM_COLOR_V2"
COLOR_CONFIG_PATH = "/sdcard/colorThresholds.cfg"
COLOR_CONFIG_BACKUP_PATH = "/sdcard/colorThresholds.bak"
COLOR_CONFIG_MAX_BYTES = 320
DEFAULT_RED_LAB = (30, 72, 20, 68, 7, 58)
DEFAULT_GREEN_LAB = (30, 75, -48, -14, 15, 68)
# A calibrated threshold must stay clearly on the expected side of neutral
# LAB A.  The margin rejects a white/grey sample that merely happens to be
# separated from the red box, while retaining the latest field calibration
# (red A 24..47, green A -33..-10) and the compiled diagnostic defaults.
MIN_RED_A_LOW = 8
MAX_GREEN_A_HIGH = -8
RED_LAB = DEFAULT_RED_LAB
GREEN_LAB = DEFAULT_GREEN_LAB
MAGENTA_LAB = None
RED_THRESHOLDS = [RED_LAB]
GREEN_THRESHOLDS = [GREEN_LAB]
MAGENTA_THRESHOLDS = []

# The official traffic sign is 50 x 50 x 100 mm, so an unobstructed image is
# expected to be upright and roughly twice as tall as it is wide.
MIN_PIXELS = 24
MIN_AREA = 32
MIN_DENSITY_100 = 32
MIN_BOTTOM_PX = 48
MAX_CANDIDATES = 6
# CanMV materializes find_blobs() before Python can copy a bounded subset. A
# cluttered frame therefore fails closed before scoring/tracking and also caps
# Python-side scan time. The mounted ROI and LAB calibration remain the real
# controls for native allocation pressure.
MAX_RAW_BLOBS_PER_COLOR = 32

# Search only the configurable track region. The default is deliberately the
# full frame: cropping at the old base threshold would cut the top off a real
# pillar whose base is valid. Narrow this only from mounted-camera field images.
# CanMV's ROI is (x, y, width, height).
TRACK_ROI_X = 0
TRACK_ROI_Y = 0
TRACK_ROI_W = FRAME_WIDTH
TRACK_ROI_H = FRAME_HEIGHT - TRACK_ROI_Y
TRACK_ROI = (TRACK_ROI_X, TRACK_ROI_Y, TRACK_ROI_W, TRACK_ROI_H)

# A valid sign must end on the searched track/ground region. Keeping this
# separate from TRACK_ROI makes an intentional track calibration visible and
# preserves fail-closed behavior if the ROI is later widened for debugging.
GROUND_BASE_MIN_Y = MIN_BOTTOM_PX
GROUND_BASE_MAX_Y = TRACK_ROI_Y + TRACK_ROI_H - 1

# Edge clipping relaxes aspect ratio only for plausible close pillars. These
# hard limits still reject a wall/card that reaches an image edge.
HARD_MAX_WIDTH_PERCENT = 60
HARD_MAX_AREA_PERCENT = 35
RELAXED_MAX_WIDTH_PERCENT = 50
RELAXED_MAX_AREA_PERCENT = 22
RELAXED_MIN_DENSITY_100 = 55

# This dimensionless score combines upright geometry, fill, bounded size, and
# ground position below. It is intentionally not a LAB calibration value.
# Raise it only after reviewing raw-candidate overlays on the mounted camera.
MIN_CANDIDATE_QUALITY = 45

# Association limits are deliberately hard-capped. A large nearby box must
# not make the gate wide enough to jump to another same-color pillar.
MAX_ASSOC_X_PX = 48
MAX_ASSOC_BASE_PX = 72
MAX_ASSOC_AREA_RATIO = 3

# A report requires consecutive associated detections. Briefly retain a
# confirmed target for overlays through an occlusion, but stale memory is
# never put on UART and therefore never controls steering.
TRACK_CONFIRM_HITS = 3
TRACK_OCCLUSION_MEMORY_FRAMES = 2
UNCONFIRMED_MISS_LIMIT = 1

# This is a fail-closed sanity check, not a final track calibration. The dark
# desk failure had L upper-quartile near 0-3 and L max near 12. A valid frame
# needs both usable midtones and at least one properly exposed highlight. At
# the measured 46 FPS, checking every three frames bounds a newly detected
# exposure failure to roughly 65 ms without doing a full statistics pass on
# every image.
VISION_CHECK_INTERVAL = 3
MIN_VISION_L_UQ = 8
MIN_VISION_L_MAX = 25

# Candidate tuple indexes. Only primitive values survive beyond a snapshot;
# CanMV Blob objects are never retained.
C_X = 0
C_Y = 1
C_W = 2
C_H = 3
C_PIXELS = 4
C_DENSITY = 5
C_CX = 6
C_BASE = 7
C_Q = 8
C_FLAGS = 9
C_FLAG_RELAXED_SHAPE = 1 << 0
C_FLAG_SIDE_CLIPPED = 1 << 1
C_FLAG_NO_GROUND_SUPPORT = 1 << 2

# New targets must visibly terminate on the bright, mostly neutral track mat.
# This prevents an upright red/green item beyond the 100 mm black wall (for
# example clothing) from becoming a steering instruction.  The 3 x 3 fan is
# deliberately wider than a small far pillar and tolerates one complete row
# landing on an official blue/orange/yellow marking.
GROUND_SUPPORT_ROW_OFFSETS = (2, 5, 9)
GROUND_SUPPORT_MIN_SAMPLES = 6


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def validate_lab_threshold(threshold):
    """Return a normalized six-value LAB threshold or raise ValueError."""

    if not isinstance(threshold, (tuple, list)) or len(threshold) != 6:
        raise ValueError("LAB threshold must contain six integers")

    normalized = []
    for value in threshold:
        # bool is an int on both CPython and MicroPython, but accepting it in a
        # persisted calibration would hide a damaged or hand-edited file.
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("LAB threshold values must be integers")
        normalized.append(value)

    limits = ((0, 100), (0, 100)) + ((-128, 127),) * 4
    for value, bounds in zip(normalized, limits):
        if not bounds[0] <= value <= bounds[1]:
            raise ValueError("LAB threshold value outside sensor range")
    for low_index in (0, 2, 4):
        if normalized[low_index] >= normalized[low_index + 1]:
            raise ValueError("LAB threshold needs a non-zero range")
    return tuple(normalized)


def validate_color_threshold_pair(red_lab, green_lab):
    """Reject structurally valid but unsafe red/green configurations."""

    red_lab = validate_lab_threshold(red_lab)
    green_lab = validate_lab_threshold(green_lab)
    for threshold in (red_lab, green_lab):
        if threshold[1] - threshold[0] > 90:
            raise ValueError("LAB luminance threshold is implausibly wide")
        if threshold[3] - threshold[2] > 110:
            raise ValueError("LAB A threshold is implausibly wide")
        if threshold[5] - threshold[4] > 110:
            raise ValueError("LAB B threshold is implausibly wide")
    if red_lab[2] < MIN_RED_A_LOW:
        raise ValueError("red LAB A range is not clearly positive")
    if green_lab[3] > MAX_GREEN_A_HIGH:
        raise ValueError("green LAB A range is not clearly negative")
    # These two competition colours separate primarily on LAB A. Overlapping
    # bands let the same pixels become both a red and a green target.
    if red_lab[2] <= green_lab[3]:
        raise ValueError("red and green LAB A thresholds overlap")
    return red_lab, green_lab


def thresholds_overlap(first, second):
    return all(
        not (first[index + 1] < second[index] or second[index + 1] < first[index])
        for index in (0, 2, 4)
    )


def validate_color_thresholds(red_lab, green_lab, magenta_lab=None):
    red_lab, green_lab = validate_color_threshold_pair(red_lab, green_lab)
    if magenta_lab is None:
        return red_lab, green_lab, None
    magenta_lab = validate_lab_threshold(magenta_lab)
    if magenta_lab[1] - magenta_lab[0] > 90:
        raise ValueError("magenta LAB luminance threshold is implausibly wide")
    if magenta_lab[3] - magenta_lab[2] > 110:
        raise ValueError("magenta LAB A threshold is implausibly wide")
    if magenta_lab[5] - magenta_lab[4] > 110:
        raise ValueError("magenta LAB B threshold is implausibly wide")
    if thresholds_overlap(red_lab, magenta_lab):
        raise ValueError("red and magenta LAB thresholds overlap")
    if thresholds_overlap(green_lab, magenta_lab):
        raise ValueError("green and magenta LAB thresholds overlap")
    return red_lab, green_lab, magenta_lab


def magenta_detection_thresholds(red_lab, green_lab, magenta_lab):
    """Build a bounded darker-magenta companion without changing calibration.

    The saved V2 sample separates red from magenta mainly by luminance, so
    blindly lowering magenta L would also classify calibrated red pixels as
    magenta. Instead, any darker companion keeps only a calibrated chroma tail
    that lies wholly outside both pillar thresholds. Its L extension is at
    most one calibrated magenta L-band width. If no such tail exists, the
    original threshold is returned unchanged rather than guessing.

    This affects production detection only. It deliberately does not weaken
    calibration validation or rewrite the persisted file.
    """

    red_lab, green_lab, magenta_lab = validate_color_thresholds(
        red_lab,
        green_lab,
        magenta_lab,
    )
    if magenta_lab is None:
        return []

    thresholds = [magenta_lab]
    shadow_l_high = magenta_lab[0] - 1
    calibrated_l_span = magenta_lab[1] - magenta_lab[0]
    shadow_l_low = max(0, magenta_lab[0] - calibrated_l_span)
    if shadow_l_low >= shadow_l_high:
        return thresholds

    best = None
    best_span = -1
    # Look for an upper or lower tail on A and then B. Requiring one channel
    # to be wholly outside both pillar bands guarantees three-dimensional LAB
    # boxes cannot overlap, even though their luminance bands now may.
    for channel_index in (2, 4):
        upper_low = max(
            magenta_lab[channel_index],
            red_lab[channel_index + 1] + 1,
            green_lab[channel_index + 1] + 1,
        )
        upper_high = magenta_lab[channel_index + 1]
        if upper_low < upper_high:
            candidate = list(magenta_lab)
            candidate[0] = shadow_l_low
            candidate[1] = shadow_l_high
            candidate[channel_index] = upper_low
            candidate[channel_index + 1] = upper_high
            span = upper_high - upper_low
            if span > best_span:
                best = tuple(candidate)
                best_span = span

        lower_low = magenta_lab[channel_index]
        lower_high = min(
            magenta_lab[channel_index + 1],
            red_lab[channel_index] - 1,
            green_lab[channel_index] - 1,
        )
        if lower_low < lower_high:
            candidate = list(magenta_lab)
            candidate[0] = shadow_l_low
            candidate[1] = shadow_l_high
            candidate[channel_index] = lower_low
            candidate[channel_index + 1] = lower_high
            span = lower_high - lower_low
            if span > best_span:
                best = tuple(candidate)
                best_span = span

    if best is None:
        return thresholds
    best = validate_lab_threshold(best)
    if thresholds_overlap(best, red_lab) or thresholds_overlap(
        best,
        green_lab,
    ):
        # Defensive guard for future edits to the tail construction.
        return thresholds
    thresholds.append(best)
    return thresholds


def format_color_thresholds(red_lab, green_lab, magenta_lab=None):
    """Return the small, deterministic on-card calibration format."""

    red_lab, green_lab, magenta_lab = validate_color_thresholds(
        red_lab, green_lab, magenta_lab
    )
    if magenta_lab is None:
        return "%s\nRED=%s\nGREEN=%s\n" % (
            COLOR_CONFIG_LEGACY_MAGIC,
            ",".join(str(value) for value in red_lab),
            ",".join(str(value) for value in green_lab),
        )
    return "%s\nRED=%s\nGREEN=%s\nMAGENTA=%s\n" % (
        COLOR_CONFIG_MAGIC,
        ",".join(str(value) for value in red_lab),
        ",".join(str(value) for value in green_lab),
        ",".join(str(value) for value in magenta_lab),
    )


def parse_color_thresholds(text):
    """Strictly parse one red/green calibration file."""

    if not isinstance(text, str):
        raise ValueError("colour config must be text")
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if not lines or lines[0] not in (
        COLOR_CONFIG_LEGACY_MAGIC,
        COLOR_CONFIG_MAGIC,
    ):
        raise ValueError("wrong colour config header or line count")
    expected_keys = (
        ("RED", "GREEN")
        if lines[0] == COLOR_CONFIG_LEGACY_MAGIC
        else ("RED", "GREEN", "MAGENTA")
    )
    if len(lines) != len(expected_keys) + 1:
        raise ValueError("wrong colour config header or line count")

    values = {}
    for line in lines[1:]:
        if "=" not in line:
            raise ValueError("malformed colour config line")
        key, encoded = line.split("=", 1)
        if key not in expected_keys or key in values:
            raise ValueError("unknown or duplicate colour config key")
        parts = encoded.split(",")
        if len(parts) != 6:
            raise ValueError("colour config threshold needs six values")
        try:
            threshold = tuple(int(part) for part in parts)
        except (TypeError, ValueError):
            raise ValueError("colour config contains a non-integer")
        values[key] = validate_lab_threshold(threshold)

    if any(key not in values for key in expected_keys):
        raise ValueError("colour config is missing a required threshold")
    return validate_color_thresholds(
        values["RED"], values["GREEN"], values.get("MAGENTA")
    )


def read_color_thresholds(path):
    with open(path, "r") as handle:
        text = handle.read(COLOR_CONFIG_MAX_BYTES + 1)
    if len(text) > COLOR_CONFIG_MAX_BYTES:
        raise ValueError("colour config is unexpectedly large")
    return parse_color_thresholds(text)


def load_color_thresholds(
    primary_path=COLOR_CONFIG_PATH,
    backup_path=COLOR_CONFIG_BACKUP_PATH,
):
    """Load primary, backup, or diagnostic compiled defaults in that order."""

    for label, path in (("SD primary", primary_path), ("SD backup", backup_path)):
        try:
            red_lab, green_lab, magenta_lab = read_color_thresholds(path)
            return red_lab, green_lab, magenta_lab, "%s %s" % (label, path)
        except (OSError, ValueError) as error:
            print("colour config unavailable:", path, error)
    return DEFAULT_RED_LAB, DEFAULT_GREEN_LAB, None, "built-in defaults"


def persisted_pillar_config_ready(source):
    """Return True only for a structurally valid persisted red/green config.

    The built-in desk thresholds remain useful for displaying a diagnostic
    preview, but they are never authority for obstacle steering.  Magenta is
    optional because the current obstacle controller is pillar-only; a V2
    file enables parking-pair observations without changing this readiness
    gate.
    """

    if not isinstance(source, str):
        return False
    return source.startswith("SD primary ") or source.startswith("SD backup ")


def uart_crc16_ccitt_false(data):
    """Return protocol V1 CRC-16/CCITT-FALSE."""

    crc = 0xFFFF
    for byte in data:
        crc ^= int(byte) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def uart_build_frame(message_type, sequence, timestamp_ms, payload=b""):
    """Build one self-delimiting V1 frame, byte-identical to protocol/."""

    payload = bytes(payload)
    if not 0 <= int(message_type) <= 0xFF:
        raise ValueError("UART message type outside uint8")
    if not 0 <= int(sequence) <= 0xFFFF:
        raise ValueError("UART sequence outside uint16")
    if not 0 <= int(timestamp_ms) <= 0xFFFFFFFF:
        raise ValueError("UART timestamp outside uint32")
    if len(payload) > 64:
        raise ValueError("UART payload exceeds 64 bytes")
    body = struct.pack(
        "<BBHIH",
        UART_VERSION,
        int(message_type),
        int(sequence),
        int(timestamp_ms),
        len(payload),
    ) + payload
    return UART_SYNC + body + struct.pack(
        "<H",
        uart_crc16_ccitt_false(body),
    )


def uart_pack_heartbeat(
    state,
    error_flags=0,
    last_rx_sequence=UART_UNKNOWN_SEQUENCE,
    loop_rate_hz_x10=0,
):
    """Pack the fixed eight-byte K230 heartbeat payload."""

    if state < 0 or state > 3:
        raise ValueError("UART heartbeat state outside 0..3")
    if error_flags < 0 or error_flags > 0xFF:
        raise ValueError("UART heartbeat error flags outside uint8")
    if last_rx_sequence < 0 or last_rx_sequence > 0xFFFF:
        raise ValueError("UART heartbeat RX sequence outside uint16")
    if loop_rate_hz_x10 < 0 or loop_rate_hz_x10 > 0xFFFF:
        raise ValueError("UART heartbeat rate outside uint16")
    return struct.pack(
        "<BBHHH",
        UART_ROLE_K230,
        int(state),
        int(error_flags),
        int(last_rx_sequence),
        int(loop_rate_hz_x10),
    )


def _uart_round_ratio(numerator, denominator):
    if denominator <= 0:
        raise ValueError("UART normalization denominator must be positive")
    if numerator < 0:
        return -((-numerator + denominator // 2) // denominator)
    return (numerator + denominator // 2) // denominator


def uart_blob_from_pixels(x, y, width, height, quality):
    """Normalize one current tracker box into a protocol V1 blob slot."""

    x = int(x)
    y = int(y)
    width = int(width)
    height = int(height)
    quality = int(quality)
    if width <= 0 or height <= 0:
        raise ValueError("UART present blob dimensions must be positive")
    if (
        x < 0
        or y < 0
        or x + width > FRAME_WIDTH
        or y + height > FRAME_HEIGHT
    ):
        raise ValueError("UART blob lies outside the frame")
    if quality < 0 or quality > 100:
        raise ValueError("UART blob quality outside 0..100")

    center_x_twice = 2 * x + width - 1
    center_x = _uart_round_ratio(
        (center_x_twice - (FRAME_WIDTH - 1)) * 10000,
        FRAME_WIDTH - 1,
    )
    bottom_y = _uart_round_ratio(
        (y + height - 1) * 10000,
        FRAME_HEIGHT - 1,
    )
    width_norm = min(
        10000,
        _uart_round_ratio(width * 10000, FRAME_WIDTH),
    )
    height_norm = min(
        10000,
        _uart_round_ratio(height * 10000, FRAME_HEIGHT),
    )
    confidence = 1 + (quality * 999) // 100
    return (center_x, bottom_y, width_norm, height_norm, confidence)


def uart_target_slot(target):
    """Return a slot only for a confirmed target hit in the current frame."""

    if target is None or not target.actionable():
        return None
    return uart_blob_from_pixels(
        target.x,
        target.y,
        target.w,
        target.h,
        target.q,
    )


def uart_parking_pair_slots(left_target, right_target):
    """Return two ordered current slots, or fail closed as an absent pair."""

    if left_target is None or right_target is None:
        return None, None
    if not left_target.actionable() or not right_target.actionable():
        return None, None
    if left_target.center_x() >= right_target.center_x():
        return None, None
    if left_target.x + left_target.w > right_target.x:
        return None, None
    if (
        left_target.y + left_target.h <= right_target.y
        or right_target.y + right_target.h <= left_target.y
    ):
        return None, None
    return uart_target_slot(left_target), uart_target_slot(right_target)


def _uart_pack_blob(slot):
    if slot is None:
        return struct.pack("<hHHHH", 0, 0, 0, 0, 0)
    if len(slot) != 5:
        raise ValueError("UART blob slot must have five fields")
    return struct.pack(
        "<hHHHH",
        int(slot[0]),
        int(slot[1]),
        int(slot[2]),
        int(slot[3]),
        int(slot[4]),
    )


def uart_pack_tracker_observation(
    camera_valid,
    frame_quality,
    processing_age_ms,
    red_target=None,
    green_target=None,
    pipeline_overrun=False,
    magenta_left_target=None,
    magenta_right_target=None,
    exposure_unstable=None,
):
    """Pack current actionable targets; invalid/stale inputs become zeros.

    Magenta slots are intentionally populated only by a jointly confirmed
    parking pair.  Callers must not assign an unpaired magenta blob to either
    side.  ``exposure_unstable`` defaults to the legacy behavior for host API
    compatibility, while the live loop supplies the actual failure reason.
    """

    frame_quality = int(frame_quality)
    processing_age_ms = int(processing_age_ms)
    if frame_quality < 0 or frame_quality > 100:
        raise ValueError("UART frame quality outside 0..100")
    if processing_age_ms < 0 or processing_age_ms > 0xFFFF:
        raise ValueError("UART processing age outside uint16")

    flags = 0
    if pipeline_overrun:
        flags |= UART_FLAG_PIPELINE_OVERRUN
    if exposure_unstable is None:
        exposure_unstable = not camera_valid
    if camera_valid:
        flags |= UART_FLAG_CAMERA_VALID
        red = uart_target_slot(red_target)
        green = uart_target_slot(green_target)
        magenta_left, magenta_right = uart_parking_pair_slots(
            magenta_left_target,
            magenta_right_target,
        )
        if red is not None:
            flags |= UART_FLAG_RED_PRESENT
        if green is not None:
            flags |= UART_FLAG_GREEN_PRESENT
        # A lone magenta target has no safe left/right identity in protocol
        # V1.  Drop both unless the pair tracker supplied two fresh members.
        if magenta_left is None or magenta_right is None:
            magenta_left = None
            magenta_right = None
        else:
            flags |= UART_FLAG_MAGENTA_LEFT_PRESENT
            flags |= UART_FLAG_MAGENTA_RIGHT_PRESENT
    else:
        # A failed camera/config/scan gate never carries a prior target.
        if exposure_unstable:
            flags |= UART_FLAG_EXPOSURE_UNSTABLE
        frame_quality = 0
        red = None
        green = None
        magenta_left = None
        magenta_right = None

    payload = struct.pack(
        "<HbBHhhH",
        flags,
        0,
        frame_quality,
        processing_age_ms,
        0,
        0,
        UART_UNKNOWN_RANGE_MM,
    )
    payload += _uart_pack_blob(red)
    payload += _uart_pack_blob(green)
    payload += _uart_pack_blob(magenta_left)
    payload += _uart_pack_blob(magenta_right)
    if len(payload) != UART_OBSERVATION_PAYLOAD_SIZE:
        raise RuntimeError("UART observation payload size changed")
    return payload


def uart_frame_quality(vision_ok, l_uq, l_max):
    """Map current luminance statistics to the protocol's 0..100 field."""

    if not vision_ok:
        return 0
    return clamp((int(l_uq) + int(l_max)) // 2, 0, 100)


class UartFrameWriter:
    """One bounded, non-throwing writer with a shared V1 sequence counter."""

    def __init__(self, uart, initial_sequence=0):
        if initial_sequence < 0 or initial_sequence > 0xFFFF:
            raise ValueError("initial UART sequence outside uint16")
        self.uart = uart
        self.sequence = int(initial_sequence)
        self.frames_sent = 0
        self.write_errors = 0
        self.write_error_streak = 0

    def _record_error(self):
        self.write_errors = min(
            UART_MAX_WRITE_ERRORS,
            self.write_errors + 1,
        )
        self.write_error_streak = min(255, self.write_error_streak + 1)

    def send(self, message_type, timestamp_ms, payload):
        frame = uart_build_frame(
            message_type,
            self.sequence,
            timestamp_ms,
            payload,
        )
        try:
            written = self.uart.write(frame)
        except Exception:
            self._record_error()
            return False
        if written is None or int(written) != len(frame):
            self._record_error()
            return False

        self.sequence = (self.sequence + 1) & 0xFFFF
        self.frames_sent = min(0xFFFFFFFF, self.frames_sent + 1)
        self.write_error_streak = 0
        return True

    def close(self):
        try:
            self.uart.deinit()
        except Exception:
            pass


def open_uart_link():
    """Configure the LushanPi 40-pin UART2 route and framed writer."""

    from machine import FPIOA, UART

    fpioa = FPIOA()
    fpioa.set_function(UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_GPIO, FPIOA.UART2_RXD)
    uart = UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    return UartFrameWriter(uart)


def uart_send_boot_heartbeat(uart_link, timestamp_ms):
    """Start/rebase one UART session before camera initialization blocks."""

    return uart_link.send(
        UART_TYPE_HEARTBEAT,
        int(timestamp_ms) & 0xFFFFFFFF,
        uart_pack_heartbeat(UART_STATE_BOOT),
    )


def vision_quality(statistics):
    """Return ``(ok, l_uq, l_max)`` for a CanMV statistics object."""

    l_uq = int(statistics.l_uq())
    l_max = int(statistics.l_max())
    ok = l_uq >= MIN_VISION_L_UQ and l_max >= MIN_VISION_L_MAX
    return ok, l_uq, l_max


def uart_health_status(exposure_ok, calibration_ready, scan_ok, pipeline_overrun):
    """Map current perception evidence to heartbeat state and error flags."""

    error_flags = 0
    if not calibration_ready:
        error_flags |= UART_ERROR_CONFIG_INVALID
    if pipeline_overrun:
        error_flags |= UART_ERROR_LOOP_OVERRUN
    ready = (
        exposure_ok
        and calibration_ready
        and scan_ok
        and not pipeline_overrun
    )
    return UART_STATE_READY if ready else UART_STATE_DEGRADED, error_flags


def candidate_rank(candidate):
    # Ground contact is the depth proxy available from one camera. Make it the
    # primary key so a clean far pillar cannot hide a nearer same-colour one;
    # bounded 0..100 quality only breaks ties at the same base row.
    return candidate[C_BASE] * 101 + clamp(candidate[C_Q], 0, 100)


def track_roi_is_valid():
    """Reject a bad hand-edited ROI before passing it to CanMV."""

    if TRACK_ROI_W <= 0 or TRACK_ROI_H <= 0:
        return False
    if TRACK_ROI_X < 0 or TRACK_ROI_Y < 0:
        return False
    if TRACK_ROI_X + TRACK_ROI_W > FRAME_WIDTH:
        return False
    if TRACK_ROI_Y + TRACK_ROI_H > FRAME_HEIGHT:
        return False
    return (
        TRACK_ROI_Y <= GROUND_BASE_MIN_Y
        and GROUND_BASE_MIN_Y <= GROUND_BASE_MAX_Y
        and GROUND_BASE_MAX_Y < TRACK_ROI_Y + TRACK_ROI_H
    )


def base_on_ground(center_x, base_y):
    """Return True only when a blob base lies in the configured track ROI."""

    if not track_roi_is_valid():
        return False
    roi_right = TRACK_ROI_X + TRACK_ROI_W - 1
    if center_x < TRACK_ROI_X or center_x > roi_right:
        return False
    return GROUND_BASE_MIN_Y <= base_y <= GROUND_BASE_MAX_Y


def track_floor_pixel(pixel):
    """Return True for a bright, mostly neutral RGB track-mat pixel."""

    if not isinstance(pixel, (tuple, list)) or len(pixel) < 3:
        return False
    try:
        red = int(pixel[0])
        green = int(pixel[1])
        blue = int(pixel[2])
    except (TypeError, ValueError):
        return False
    if min(red, green, blue) < 0 or max(red, green, blue) > 255:
        return False
    darkest = min(red, green, blue)
    brightest = max(red, green, blue)
    return (
        red + green + blue >= 180
        and brightest > 0
        and 5 * darkest >= 3 * brightest
    )


def candidate_has_ground_support(image, candidate):
    """Check a bounded 3 x 3 fan of pixels just below a pillar candidate."""

    if image is None or candidate is None:
        return False
    center_x = candidate[C_CX]
    base_y = candidate[C_BASE]
    span = clamp(max(6, candidate[C_W]), 6, 32)
    floor_samples = 0
    total_samples = 0
    for row_offset in GROUND_SUPPORT_ROW_OFFSETS:
        sample_y = base_y + row_offset
        if sample_y < 0 or sample_y >= FRAME_HEIGHT:
            continue
        for sample_x in (center_x - span, center_x, center_x + span):
            if sample_x < 0 or sample_x >= FRAME_WIDTH:
                continue
            try:
                pixel = image.get_pixel(sample_x, sample_y)
            except (AttributeError, OSError, TypeError, ValueError):
                return False
            total_samples += 1
            if track_floor_pixel(pixel):
                floor_samples += 1
    return (
        total_samples >= GROUND_SUPPORT_MIN_SAMPLES
        and 3 * floor_samples >= 2 * total_samples
    )


def copy_blob(blob):
    """Copy and validate one blob, returning a fixed primitive tuple."""

    x = int(blob.x())
    y = int(blob.y())
    width = int(blob.w())
    height = int(blob.h())
    pixels = int(blob.pixels())
    density_100 = int(blob.density() * 100)

    if width < 4 or height < 8 or pixels < MIN_PIXELS:
        return None
    if density_100 < MIN_DENSITY_100:
        return None

    right = x + width - 1
    base = y + height - 1
    if x < 0 or y < 0 or right >= FRAME_WIDTH or base >= FRAME_HEIGHT:
        return None
    center_x = x + (width - 1) // 2
    if not base_on_ground(center_x, base):
        return None

    touches_top = y <= 3
    touches_bottom = base >= FRAME_HEIGHT - 4
    touches_side = x <= 3 or right >= FRAME_WIDTH - 4
    candidate_flags = C_FLAG_SIDE_CLIPPED if touches_side else 0
    box_area = width * height
    frame_area = FRAME_WIDTH * FRAME_HEIGHT

    if width * 100 > HARD_MAX_WIDTH_PERCENT * FRAME_WIDTH:
        return None
    if box_area * 100 > HARD_MAX_AREA_PERCENT * frame_area:
        return None

    # Normal view: about 2:1, with generous perspective tolerance. A close
    # pillar clipped by an image edge is allowed a wider apparent shape only
    # when it is dense and still bounded in width/area.
    aspect_100 = (100 * height) // width
    min_aspect_100 = 115
    large_edge_candidate = (
        width >= FRAME_WIDTH // 12 and height >= FRAME_HEIGHT // 8
    )
    if touches_bottom and large_edge_candidate:
        min_aspect_100 = 55
    elif touches_top and large_edge_candidate:
        min_aspect_100 = 75
    max_aspect_100 = 1200 if touches_side else 600
    if aspect_100 < min_aspect_100 or aspect_100 > max_aspect_100:
        return None

    if aspect_100 < 115:
        candidate_flags |= C_FLAG_RELAXED_SHAPE
        if density_100 < RELAXED_MIN_DENSITY_100:
            return None
        if width * 100 > RELAXED_MAX_WIDTH_PERCENT * FRAME_WIDTH:
            return None
        if box_area * 100 > RELAXED_MAX_AREA_PERCENT * frame_area:
            return None

    oversized = (
        width > (48 * FRAME_WIDTH) // 100
        or box_area > (28 * frame_area) // 100
    )
    if oversized and not (touches_top or touches_bottom):
        return None

    # Scores saturate. A huge colored background cannot win by pixels alone.
    aspect_error = abs(aspect_100 - 200)
    aspect_q = 100 - min(100, (100 * aspect_error) // 200)
    density_q = clamp(
        ((density_100 - MIN_DENSITY_100) * 100) // 48,
        0,
        100,
    )
    size_q = min(100, (pixels * 100) // 2500)
    bottom_q = (base * 100) // (FRAME_HEIGHT - 1)
    quality = (
        45 * aspect_q
        + 25 * density_q
        + 15 * size_q
        + 15 * bottom_q
    ) // 100
    if quality < MIN_CANDIDATE_QUALITY:
        return None

    return (
        x,
        y,
        width,
        height,
        pixels,
        density_100,
        center_x,
        base,
        quality,
        candidate_flags,
    )


def copy_parking_blob(blob):
    """Copy one magenta region without assuming pillar aspect ratio.

    The two official parking limits are 200 x 20 x 100 mm and can present
    either a broad side or a narrow end to the camera.  Until mounted-camera
    measurements exist, no made-up aspect, scale, or pixel-gap limits are
    used.  Safety instead comes from calibrated colour, existing hard frame
    bounds, and joint three-frame pair tracking below.
    """

    x = int(blob.x())
    y = int(blob.y())
    width = int(blob.w())
    height = int(blob.h())
    pixels = int(blob.pixels())
    density_100 = int(blob.density() * 100)

    if width < 4 or height < 4 or pixels < MIN_PIXELS:
        return None
    if density_100 < MIN_DENSITY_100:
        return None

    right = x + width - 1
    base = y + height - 1
    if x < 0 or y < 0 or right >= FRAME_WIDTH or base >= FRAME_HEIGHT:
        return None
    center_x = x + (width - 1) // 2
    if not base_on_ground(center_x, base):
        return None

    box_area = width * height
    frame_area = FRAME_WIDTH * FRAME_HEIGHT
    if box_area < MIN_AREA:
        return None
    if width * 100 > HARD_MAX_WIDTH_PERCENT * FRAME_WIDTH:
        return None
    if box_area * 100 > HARD_MAX_AREA_PERCENT * frame_area:
        return None

    # Density is the least assumptive per-object confidence available here;
    # temporal and pair geometry decide presence, not this value by itself.
    quality = clamp(density_100, 0, 100)
    candidate_flags = (
        C_FLAG_SIDE_CLIPPED
        if x <= 3 or right >= FRAME_WIDTH - 4
        else 0
    )
    return (
        x,
        y,
        width,
        height,
        pixels,
        density_100,
        center_x,
        base,
        quality,
        candidate_flags,
    )


def candidate_seed_safe(candidate):
    """Reject clipped/relaxed shapes when establishing a new identity."""

    return candidate is not None and candidate[C_FLAGS] == 0


def candidate_rectangles_overlap(first, second):
    """Return True when two candidate rectangles share at least one pixel."""

    return not (
        first[C_X] + first[C_W] <= second[C_X]
        or second[C_X] + second[C_W] <= first[C_X]
        or first[C_Y] + first[C_H] <= second[C_Y]
        or second[C_Y] + second[C_H] <= first[C_Y]
    )


def _candidate_contains_ground_point(candidate, other):
    """Return True when ``other`` ends inside ``candidate``'s silhouette."""

    return (
        candidate[C_X]
        <= other[C_CX]
        < candidate[C_X] + candidate[C_W]
        and candidate[C_Y]
        <= other[C_BASE]
        < candidate[C_Y] + candidate[C_H]
    )


def reject_cross_color_ambiguity(red_candidates, green_candidates):
    """Drop regions whose apparent ground contact has two colour identities.

    Persisted thresholds are disjoint in LAB space, but adjacent fragmented
    patches can still form overlapping red/green boxes around one scene
    object.  A real steering instruction is not inferable in that geometry,
    so both candidates fail closed for the current frame.
    """

    red_conflict = [False] * len(red_candidates)
    green_conflict = [False] * len(green_candidates)
    for red_index in range(len(red_candidates)):
        red = red_candidates[red_index]
        for green_index in range(len(green_candidates)):
            green = green_candidates[green_index]
            if _candidate_contains_ground_point(
                red, green
            ) or _candidate_contains_ground_point(green, red):
                red_conflict[red_index] = True
                green_conflict[green_index] = True

    return (
        [
            red_candidates[index]
            for index in range(len(red_candidates))
            if not red_conflict[index]
        ],
        [
            green_candidates[index]
            for index in range(len(green_candidates))
            if not green_conflict[index]
        ],
    )


def keep_best_bounded(candidates, candidate):
    """Keep at most MAX_CANDIDATES without an unbounded per-frame list."""

    if len(candidates) < MAX_CANDIDATES:
        candidates.append(candidate)
        return

    worst_index = 0
    worst_rank = candidate_rank(candidates[0])
    for index in range(1, len(candidates)):
        rank = candidate_rank(candidates[index])
        if rank < worst_rank:
            worst_rank = rank
            worst_index = index
    if candidate_rank(candidate) > worst_rank:
        candidates[worst_index] = candidate


def detect_candidates_checked(image, thresholds, copier=copy_blob):
    """Return ``(candidates, scan_ok)`` and expose overload as a fault."""

    candidates = []
    if not track_roi_is_valid():
        return candidates, False
    blobs = image.find_blobs(
        thresholds,
        roi=TRACK_ROI,
        x_stride=2,
        y_stride=2,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=False,
    )
    if len(blobs) > MAX_RAW_BLOBS_PER_COLOR:
        blobs = None
        return candidates, False
    require_ground_support = copier is copy_blob
    for blob in blobs:
        candidate = copier(blob)
        if candidate is not None:
            if require_ground_support and not candidate_has_ground_support(
                image,
                candidate,
            ):
                flagged = list(candidate)
                flagged[C_FLAGS] |= C_FLAG_NO_GROUND_SUPPORT
                candidate = tuple(flagged)
            keep_best_bounded(candidates, candidate)
    blobs = None
    return candidates, True


def detect_candidates(image, thresholds):
    """Compatibility wrapper for host tools that only need pillar boxes."""

    return detect_candidates_checked(image, thresholds)[0]


def detect_parking_candidates(image, thresholds):
    return detect_candidates_checked(image, thresholds, copy_parking_blob)


def detect_frame_candidates(
    image,
    red_thresholds,
    green_thresholds,
    magenta_thresholds,
):
    """Run the two required scans and an explicitly enabled optional third."""

    red_candidates, red_scan_ok = detect_candidates_checked(
        image,
        red_thresholds,
    )
    green_candidates, green_scan_ok = detect_candidates_checked(
        image,
        green_thresholds,
    )
    if magenta_thresholds:
        magenta_candidates, magenta_scan_ok = detect_parking_candidates(
            image,
            magenta_thresholds,
        )
    else:
        magenta_candidates = []
        magenta_scan_ok = True
    return (
        red_candidates,
        green_candidates,
        magenta_candidates,
        red_scan_ok,
        green_scan_ok,
        magenta_scan_ok,
    )


def update_candidate_trackers(
    red_target,
    green_target,
    parking_pair,
    red_candidates,
    green_candidates,
    magenta_candidates,
    red_scan_ok,
    green_scan_ok,
    magenta_scan_ok,
):
    """Apply one frame while keeping optional parking health isolated.

    Red and green are the required sensors for the current Obstacle profile.
    A failed experimental magenta scan therefore clears only the parking pair;
    it must never suppress an otherwise valid pillar observation.
    """

    pillar_scan_ok = red_scan_ok and green_scan_ok
    if not pillar_scan_ok:
        red_target.reset()
        green_target.reset()
        parking_pair.reset()
        return [], [], [], False

    red_candidates, green_candidates = reject_cross_color_ambiguity(
        red_candidates,
        green_candidates,
    )
    if magenta_scan_ok:
        # When the optional scan is enabled, a mixed red/magenta silhouette
        # is not trustworthy as a red traffic sign. This catches a shadowed
        # pink marker whose darker pixels enter the red band while its more
        # saturated pixels remain in the safe magenta companion band.
        original_magenta_candidates = magenta_candidates
        red_candidates, magenta_without_red = reject_cross_color_ambiguity(
            red_candidates,
            original_magenta_candidates,
        )
        green_candidates, magenta_without_green = reject_cross_color_ambiguity(
            green_candidates,
            original_magenta_candidates,
        )
        magenta_candidates = [
            candidate
            for candidate in original_magenta_candidates
            if candidate in magenta_without_red
            and candidate in magenta_without_green
        ]
    red_target.update(red_candidates)
    green_target.update(green_candidates)

    if magenta_scan_ok:
        parking_pair.update(magenta_candidates)
    else:
        magenta_candidates = []
        parking_pair.reset()
    return (
        red_candidates,
        green_candidates,
        magenta_candidates,
        True,
    )


def best_global(candidates):
    best = None
    best_rank = -1
    for candidate in candidates:
        rank = candidate_rank(candidate)
        if rank > best_rank:
            best = candidate
            best_rank = rank
    return best


def best_seed_candidate(candidates):
    """Choose the best complete, pillar-shaped candidate for a new track."""

    best = None
    best_rank = -1
    for candidate in candidates:
        if not candidate_seed_safe(candidate):
            continue
        rank = candidate_rank(candidate)
        if rank > best_rank:
            best = candidate
            best_rank = rank
    return best


class StableTarget:
    """Fixed-size confirmed tracker with fail-closed occlusion memory."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.alive = False
        self.confirmed = False
        self.hits = 0
        self.misses = 0
        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        self.q = 0
        self.flags = 0

    def clip_to_frame(self):
        """Keep independently smoothed position and size inside the image."""

        self.x = clamp(int(self.x), 0, FRAME_WIDTH - 1)
        self.y = clamp(int(self.y), 0, FRAME_HEIGHT - 1)
        self.w = clamp(int(self.w), 1, FRAME_WIDTH - self.x)
        self.h = clamp(int(self.h), 1, FRAME_HEIGHT - self.y)

    def seed(self, candidate):
        self.alive = True
        self.confirmed = TRACK_CONFIRM_HITS <= 1
        self.hits = 1
        self.misses = 0
        self.x = candidate[C_X]
        self.y = candidate[C_Y]
        self.w = candidate[C_W]
        self.h = candidate[C_H]
        self.q = candidate[C_Q]
        self.flags = candidate[C_FLAGS]
        self.clip_to_frame()

    def center_x(self):
        return self.x + (self.w - 1) // 2

    def base(self):
        return min(FRAME_HEIGHT - 1, self.y + self.h - 1)

    def associated_candidate(self, candidates):
        if not self.alive:
            return None

        old_cx = self.center_x()
        old_base = self.base()
        old_area = max(1, self.w * self.h)
        best = None
        best_distance = 100000
        best_quality = -1

        for candidate in candidates:
            new_area = max(1, candidate[C_W] * candidate[C_H])
            if max(old_area, new_area) > (
                MAX_ASSOC_AREA_RATIO * min(old_area, new_area)
            ):
                continue

            # A new or reacquired identity must overlap its previous box on
            # every confirmation frame.  This prevents several unrelated
            # small patches from chaining through the generous motion gate.
            if self.hits < TRACK_CONFIRM_HITS:
                if (
                    self.x + self.w <= candidate[C_X]
                    or candidate[C_X] + candidate[C_W] <= self.x
                    or self.y + self.h <= candidate[C_Y]
                    or candidate[C_Y] + candidate[C_H] <= self.y
                ):
                    continue

            dx = abs(candidate[C_CX] - old_cx)
            db = abs(candidate[C_BASE] - old_base)
            x_gate = min(
                MAX_ASSOC_X_PX,
                max(24, (3 * (candidate[C_W] + self.w)) // 4),
            )
            base_gate = min(
                MAX_ASSOC_BASE_PX,
                max(24, (candidate[C_H] + self.h) // 2),
            )
            if dx > x_gate or db > base_gate:
                continue

            # Continuity wins. Quality may break an exact distance tie, but a
            # cleaner neighboring pillar cannot steal an established track.
            distance = 2 * dx + db
            if distance < best_distance or (
                distance == best_distance and candidate[C_Q] > best_quality
            ):
                best = candidate
                best_distance = distance
                best_quality = candidate[C_Q]
        return best

    def update(self, candidates):
        if not self.alive:
            candidate = best_seed_candidate(candidates)
            if candidate is not None:
                self.seed(candidate)
            return

        candidate = self.associated_candidate(candidates)
        if candidate is not None:
            # Integer EMA: 55% previous state, 45% new measurement.
            self.x = (55 * self.x + 45 * candidate[C_X] + 50) // 100
            self.y = (55 * self.y + 45 * candidate[C_Y] + 50) // 100
            self.w = (55 * self.w + 45 * candidate[C_W] + 50) // 100
            self.h = (55 * self.h + 45 * candidate[C_H] + 50) // 100
            self.q = (55 * self.q + 45 * candidate[C_Q] + 50) // 100
            # Geometry is smoothed for stability, but confidence flags always
            # describe the current measurement. In particular, a newly
            # clipped parking edge must not remain actionable through EMA.
            self.flags = candidate[C_FLAGS]
            # Position and size are rounded separately. Two valid boxes that
            # both touch an image edge can otherwise produce a smoothed box
            # one pixel outside the frame.
            self.clip_to_frame()
            self.hits = min(255, self.hits + 1)
            self.misses = 0
            if self.hits >= TRACK_CONFIRM_HITS:
                self.confirmed = True
            return

        self.misses = min(255, self.misses + 1)
        # Any gap requires a complete fresh confirmation chain before this
        # old identity can become actionable again.
        self.hits = 0
        if not self.confirmed:
            if self.misses >= UNCONFIRMED_MISS_LIMIT:
                replacement = best_seed_candidate(candidates)
                self.reset()
                if replacement is not None:
                    self.seed(replacement)
            return

        if self.misses > TRACK_OCCLUSION_MEMORY_FRAMES:
            replacement = best_seed_candidate(candidates)
            self.reset()
            if replacement is not None:
                self.seed(replacement)

    def visible(self):
        return (
            self.alive
            and self.confirmed
            and self.misses <= TRACK_OCCLUSION_MEMORY_FRAMES
        )

    def actionable(self):
        return (
            self.visible()
            and self.misses == 0
            and self.hits >= TRACK_CONFIRM_HITS
        )


def ordered_parking_pair(candidates):
    """Return the only unambiguous two-marker image pair, or ``None``.

    Protocol V1 has named left/right slots but no generic-magenta slot.  More
    or fewer than two regions therefore cannot be assigned honestly.  The two
    boxes must also be distinct horizontally and overlap vertically; otherwise
    they do not visibly delimit one parking opening in this frame. Left/right
    here is screen-X ordering only, not a claim about the physical bay sides.
    This remains experimental telemetry, not a parking-control decision.
    """

    if len(candidates) != 2:
        return None
    first = candidates[0]
    second = candidates[1]
    if first[C_CX] == second[C_CX]:
        return None
    if first[C_CX] < second[C_CX]:
        left, right = first, second
    else:
        left, right = second, first

    if left[C_X] + left[C_W] > right[C_X]:
        return None
    if (
        left[C_Y] + left[C_H] <= right[C_Y]
        or right[C_Y] + right[C_H] <= left[C_Y]
    ):
        return None
    return left, right


class ParkingPairTracker:
    """Two fixed tracks that only expose a fresh, jointly confirmed pair."""

    def __init__(self):
        self.left = StableTarget()
        self.right = StableTarget()

    def reset(self):
        self.left.reset()
        self.right.reset()

    def _miss_pair(self):
        self.left.update([])
        self.right.update([])
        # Keep the two identities atomic.  If either expires, the other must
        # not survive and later combine with an unrelated marker.
        if self.left.alive != self.right.alive:
            self.reset()

    def update(self, candidates):
        pair = ordered_parking_pair(candidates)
        if pair is None:
            self._miss_pair()
            return
        left_candidate, right_candidate = pair

        if not self.left.alive and not self.right.alive:
            if candidate_seed_safe(left_candidate) and candidate_seed_safe(
                right_candidate
            ):
                self.left.seed(left_candidate)
                self.right.seed(right_candidate)
            return
        if not self.left.alive or not self.right.alive:
            self.reset()
            return

        # Screen ordering makes assignment one-to-one and prevents both
        # tracks from selecting the same blob.  Each child still applies its
        # bounded area/motion and confirmation-overlap gates.
        left_match = self.left.associated_candidate([left_candidate])
        right_match = self.right.associated_candidate([right_candidate])
        if left_match is None or right_match is None:
            self._miss_pair()
            return
        self.left.update([left_match])
        self.right.update([right_match])

    def visible(self):
        return self.left.visible() and self.right.visible()

    def actionable(self):
        if not self.left.actionable() or not self.right.actionable():
            return False
        if self.left.flags != 0 or self.right.flags != 0:
            return False
        if self.left.center_x() >= self.right.center_x():
            return False
        if self.left.x + self.left.w > self.right.x:
            return False
        return not (
            self.left.y + self.left.h <= self.right.y
            or self.right.y + self.right.h <= self.left.y
        )

    def targets(self):
        if not self.actionable():
            return None, None
        return self.left, self.right


def draw_raw_candidates(image, candidates, color):
    for candidate in candidates:
        image.draw_rectangle(
            candidate[C_X],
            candidate[C_Y],
            candidate[C_W],
            candidate[C_H],
            color=color,
            thickness=1,
        )
        image.draw_cross(
            candidate[C_CX],
            candidate[C_BASE],
            color=color,
            size=3,
            thickness=1,
        )


def draw_target(image, name, target, color, top_y, instruction):
    if not target.visible():
        return

    fresh = target.actionable()
    draw_color = color if fresh else (255, 255, 0)
    center_x = target.center_x()
    base = target.base()
    x_milli = ((2 * center_x - (FRAME_WIDTH - 1)) * 1000) // (
        FRAME_WIDTH - 1
    )

    image.draw_rectangle(
        target.x,
        target.y,
        target.w,
        target.h,
        color=draw_color,
        thickness=3 if fresh else 1,
    )
    image.draw_cross(
        center_x,
        base,
        color=draw_color,
        size=6 if fresh else 3,
        thickness=2 if fresh else 1,
    )

    label_y = target.y - 11
    if label_y < 0:
        label_y = min(FRAME_HEIGHT - 10, target.y + 2)
    image.draw_string_advanced(
        clamp(target.x, 0, FRAME_WIDTH - 120),
        label_y,
        8,
        "%s x=%d b=%d e=%+d" % (name[0], center_x, base, x_milli),
        color=draw_color,
    )
    image.draw_string_advanced(
        2,
        top_y,
        8,
        "%s %s" % (name, instruction if fresh else "STALE - HOLD"),
        color=draw_color,
    )


def draw_vision_status(image, vision_ok, l_uq, l_max):
    if vision_ok:
        image.draw_string_advanced(
            2,
            22,
            8,
            "VISION OK Lq=%d Lmax=%d" % (l_uq, l_max),
            color=(255, 255, 255),
        )
    else:
        image.draw_string_advanced(
            2,
            22,
            12,
            "VISION BAD - ADD WHITE LIGHT",
            color=(255, 255, 0),
        )
        image.draw_string_advanced(
            2,
            36,
            8,
            "Lq=%d Lmax=%d - NO TARGET OUTPUT" % (l_uq, l_max),
            color=(255, 255, 0),
        )


def target_telemetry(name, target):
    if not target.visible():
        return "%s ok=0" % name[0]

    center_x = target.center_x()
    base = target.base()
    x_milli = ((2 * center_x - (FRAME_WIDTH - 1)) * 1000) // (
        FRAME_WIDTH - 1
    )
    base_milli = (base * 1000) // (FRAME_HEIGHT - 1)
    return (
        "%s ok=1 act=%d fresh=%d box=%d,%d,%d,%d aim=%d,%d "
        "x_m=%+d base_m=%d q=%d"
        % (
            name[0],
            1 if target.actionable() else 0,
            1 if target.misses == 0 else 0,
            target.x,
            target.y,
            target.w,
            target.h,
            center_x,
            base,
            x_milli,
            base_milli,
            target.q,
        )
    )


def main():
    global RED_LAB, GREEN_LAB, MAGENTA_LAB
    global RED_THRESHOLDS, GREEN_THRESHOLDS, MAGENTA_THRESHOLDS

    import os
    import time

    from media.display import Display
    from media.media import MediaManager
    from media.sensor import Sensor

    sensor = None
    sensor_running = False
    display_ready = False
    media_ready = False
    image = None
    statistics = None
    red_candidates = None
    green_candidates = None
    magenta_candidates = None
    uart_link = None
    last_heartbeat_tick = None

    red_target = StableTarget()
    green_target = StableTarget()
    parking_pair = ParkingPairTracker()
    vision_ok = False
    vision_l_uq = 0
    vision_l_max = 0

    os.exitpoint(os.EXITPOINT_ENABLE)

    RED_LAB, GREEN_LAB, MAGENTA_LAB, color_config_source = load_color_thresholds()
    RED_THRESHOLDS = [RED_LAB]
    GREEN_THRESHOLDS = [GREEN_LAB]
    MAGENTA_THRESHOLDS = (
        magenta_detection_thresholds(RED_LAB, GREEN_LAB, MAGENTA_LAB)
        if ENABLE_EXPERIMENTAL_PARKING_TELEMETRY
        and MAGENTA_LAB is not None
        else []
    )
    calibration_ready = persisted_pillar_config_ready(
        color_config_source,
    ) and track_roi_is_valid()
    print("colour thresholds loaded from:", color_config_source)
    print("magenta LAB:", MAGENTA_LAB if MAGENTA_LAB is not None else "not calibrated")
    if calibration_ready:
        print("PERSISTED COLOR CONFIG LOADED: field detection not yet proven")
        print(
            "EXPERIMENTAL PARKING TELEMETRY: %s"
            % (
                "enabled (image-left/image-right; unvalidated)"
                if MAGENTA_THRESHOLDS
                else "disabled"
            )
        )
    else:
        print(
            "OBSTACLE CALIBRATION INVALID: preview only; target UART "
            "slots disabled"
        )

    try:
        if ENABLE_UART_LINK:
            try:
                uart_link = open_uart_link()
                boot_tick = time.ticks_ms()
                boot_ok = uart_send_boot_heartbeat(uart_link, boot_tick)
                last_heartbeat_tick = boot_tick
                print(
                    "UART2 telemetry enabled: GPIO%d TX / GPIO%d RX, "
                    "115200 8N1"
                    % (UART_TX_GPIO, UART_RX_GPIO)
                )
                if not boot_ok:
                    print(
                        "UART BOOT heartbeat incomplete; ESP32 must ignore "
                        "the link until a valid heartbeat"
                    )
            except Exception as error:
                # Camera bench testing remains available if an explicitly
                # enabled link cannot initialize. No target leaves the board.
                uart_link = None
                print("UART2 telemetry unavailable:", error)

        sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=30)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)

        if ENABLE_IDE_PREVIEW:
            Display.init(
                Display.VIRT,
                width=FRAME_WIDTH,
                height=FRAME_HEIGHT,
                fps=PREVIEW_FPS,
                to_ide=True,
                quality=50,
                osd_num=1,
            )
            display_ready = True
        MediaManager.init()
        media_ready = True
        sensor.run()
        sensor_running = True

        # Let automatic exposure and white balance settle before reporting.
        for _ in range(25):
            image = sensor.snapshot()
            image = None
            time.sleep_ms(20)

        print("BUBBLEGUM fail-closed obstacle vision started")
        print("sensor type:", sensor.get_type())
        print("red LAB:", RED_LAB)
        print("green LAB:", GREEN_LAB)
        print("position units: pixels plus normalized thousandths; not cm")

        clock = time.clock()
        frame_number = 0
        previous_cycle_age_ms = 0
        scan_ok = True

        while True:
            os.exitpoint()
            clock.tick()
            frame_number += 1
            capture_tick = time.ticks_ms()
            image = sensor.snapshot()

            if frame_number == 1 or (
                frame_number % VISION_CHECK_INTERVAL == 0
            ):
                statistics = image.get_statistics()
                vision_ok, vision_l_uq, vision_l_max = vision_quality(
                    statistics
                )
                statistics = None
                if not vision_ok:
                    red_target.reset()
                    green_target.reset()
                    parking_pair.reset()

            scan_ok = True
            detection_requested = calibration_ready or ENABLE_IDE_PREVIEW
            if vision_ok and detection_requested:
                # All scans happen before any debug overlays are drawn.  An
                # invalid/overloaded pillar scan is different from a healthy
                # frame containing no target, so it degrades the observation.
                # Optional magenta failure is isolated by the update helper.
                (
                    red_candidates,
                    green_candidates,
                    magenta_candidates,
                    red_scan_ok,
                    green_scan_ok,
                    magenta_scan_ok,
                ) = detect_frame_candidates(
                    image,
                    RED_THRESHOLDS,
                    GREEN_THRESHOLDS,
                    MAGENTA_THRESHOLDS,
                )
                (
                    red_candidates,
                    green_candidates,
                    magenta_candidates,
                    scan_ok,
                ) = update_candidate_trackers(
                    red_target,
                    green_target,
                    parking_pair,
                    red_candidates,
                    green_candidates,
                    magenta_candidates,
                    red_scan_ok,
                    green_scan_ok,
                    magenta_scan_ok,
                )
            else:
                red_candidates = []
                green_candidates = []
                magenta_candidates = []
                red_target.reset()
                green_target.reset()
                parking_pair.reset()

            if uart_link is not None:
                # The frame header carries capture time; receipt time remains
                # the ESP32's freshness authority because clocks are separate.
                uart_now_tick = time.ticks_ms()
                # Include the full-frame 8N1 serialization budget so age at
                # the ESP32 is conservative even though the two clocks are
                # unrelated and UART.write() may return after queueing bytes.
                processing_age_ms = clamp(
                    time.ticks_diff(uart_now_tick, capture_tick)
                    + UART_OBSERVATION_WIRE_TIME_MS,
                    0,
                    0xFFFF,
                )
                cycle_overrun = (
                    processing_age_ms > PIPELINE_OVERRUN_AGE_MS
                    or previous_cycle_age_ms > PIPELINE_OVERRUN_AGE_MS
                )
                heartbeat_state, heartbeat_error_flags = uart_health_status(
                    vision_ok,
                    calibration_ready,
                    scan_ok,
                    cycle_overrun,
                )
                pipeline_valid = heartbeat_state == UART_STATE_READY
                magenta_left_target, magenta_right_target = (
                    parking_pair.targets()
                )
                observation = uart_pack_tracker_observation(
                    pipeline_valid,
                    uart_frame_quality(
                        pipeline_valid,
                        vision_l_uq,
                        vision_l_max,
                    ),
                    processing_age_ms,
                    red_target,
                    green_target,
                    # Current processing age cannot include work performed
                    # after this packet is formed (notably IDE preview). Carry
                    # the complete previous-cycle duration forward so a USB
                    # display stall still lowers the ESP32 speed envelope on
                    # the following observation.
                    pipeline_overrun=cycle_overrun,
                    magenta_left_target=magenta_left_target,
                    magenta_right_target=magenta_right_target,
                    exposure_unstable=not vision_ok,
                )
                observation_ok = uart_link.send(
                    UART_TYPE_OBSERVATION,
                    capture_tick & 0xFFFFFFFF,
                    observation,
                )

                if last_heartbeat_tick is None or time.ticks_diff(
                    uart_now_tick,
                    last_heartbeat_tick,
                ) >= UART_HEARTBEAT_INTERVAL_MS:
                    last_heartbeat_tick = uart_now_tick
                    heartbeat = uart_pack_heartbeat(
                        heartbeat_state,
                        error_flags=heartbeat_error_flags,
                        loop_rate_hz_x10=clamp(
                            int(clock.fps() * 10),
                            0,
                            0xFFFF,
                        ),
                    )
                    heartbeat_ok = uart_link.send(
                        UART_TYPE_HEARTBEAT,
                        uart_now_tick & 0xFFFFFFFF,
                        heartbeat,
                    )
                else:
                    heartbeat_ok = True

                if not observation_ok or not heartbeat_ok:
                    # UART errors are absorbed and counters saturate. Log only
                    # the first few and sparse milestones to avoid a print
                    # storm making the camera loop worse.
                    error_count = uart_link.write_errors
                    if error_count <= 3 or error_count % 100 == 0:
                        print(
                            "UART write incomplete; link data ignored, "
                            "errors=%d streak=%d"
                            % (error_count, uart_link.write_error_streak)
                        )

            if ENABLE_IDE_PREVIEW:
                # Thin boxes are valid current-frame objects. Thick boxes are
                # fresh, stable targets; stale grace frames are yellow and
                # never actionable for future control.
                draw_raw_candidates(image, red_candidates, (255, 0, 0))
                draw_raw_candidates(image, green_candidates, (0, 255, 0))
                draw_raw_candidates(
                    image,
                    magenta_candidates,
                    (255, 0, 255),
                )
                draw_target(
                    image,
                    "RED",
                    red_target,
                    (255, 0, 0),
                    2,
                    "KEEP RIGHT",
                )
                draw_target(
                    image,
                    "GREEN",
                    green_target,
                    (0, 255, 0),
                    12,
                    "KEEP LEFT",
                )
                draw_target(
                    image,
                    "MAGENTA-IMG-L",
                    parking_pair.left,
                    (255, 0, 255),
                    46,
                    "EXPERIMENTAL",
                )
                draw_target(
                    image,
                    "MAGENTA-IMG-R",
                    parking_pair.right,
                    (255, 0, 255),
                    56,
                    "EXPERIMENTAL",
                )
                draw_vision_status(
                    image,
                    vision_ok,
                    vision_l_uq,
                    vision_l_max,
                )

                # Detection runs every frame; debug preview is rate-limited to
                # avoid USB/webview backpressure.
                if frame_number % PREVIEW_DIVISOR == 0:
                    Display.show_image(image)

            if frame_number % 60 == 0:
                print(
                    "fps=%.1f vision=%d cfg=%d scan=%d Lq=%d Lmax=%d "
                    "mem=%d raw=%d/%d/%d | %s | %s | pair=%d"
                    % (
                        clock.fps(),
                        1 if vision_ok else 0,
                        1 if calibration_ready else 0,
                        1 if scan_ok else 0,
                        vision_l_uq,
                        vision_l_max,
                        gc.mem_free(),
                        len(red_candidates),
                        len(green_candidates),
                        len(magenta_candidates),
                        target_telemetry("RED", red_target),
                        target_telemetry("GREEN", green_target),
                        1 if parking_pair.actionable() else 0,
                    )
                )

            if frame_number % 120 == 0:
                gc.collect()

            previous_cycle_age_ms = clamp(
                time.ticks_diff(time.ticks_ms(), capture_tick),
                0,
                0xFFFF,
            )

    except KeyboardInterrupt:
        print("pillar tracker stopped by user")
    except BaseException as error:
        if str(error) == "IDE interrupt":
            print("pillar tracker stopped by IDE")
        else:
            print("pillar tracker error:", error)
            raise
    finally:
        # Drop per-frame references before releasing the MPP/VB pipeline.
        statistics = None
        red_candidates = None
        green_candidates = None
        magenta_candidates = None
        image = None
        cleanup_failed = False

        if uart_link is not None:
            uart_link.close()
            uart_link = None

        if sensor_running:
            try:
                sensor.stop()
            except BaseException as error:
                cleanup_failed = True
                print("sensor cleanup failed:", error)
        if display_ready:
            try:
                Display.deinit()
            except BaseException as error:
                cleanup_failed = True
                print("display cleanup failed:", error)
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if media_ready:
            try:
                MediaManager.deinit()
            except BaseException as error:
                cleanup_failed = True
                print("media cleanup failed:", error)
        gc.collect()
        if cleanup_failed:
            print("cleanup incomplete: restart the K230 before retrying")
        else:
            print(
                "camera pipeline stopped; if the next init reports error 18, "
                "restart the K230"
            )


if __name__ == "__main__":
    main()
